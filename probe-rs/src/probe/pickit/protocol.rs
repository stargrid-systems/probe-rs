//! Message framing and the USB transport.
//!
//! Every command is one buffer built from a 16-byte payload header, an 8-byte
//! script header, the parameter block, and the script bytes. The tool answers
//! on the command IN endpoint and moves bulk data over a second pipe.

use std::{io, time::Duration};

use nusb::{
    Endpoint, Interface,
    transfer::{Bulk, In, Out},
};

use super::PickitError;
use crate::probe::usb_util::{BulkReadExt, BulkWriteExt};

const EP_CMD_OUT: u8 = 0x02;
const EP_CMD_IN: u8 = 0x81;
const EP_DATA_OUT: u8 = 0x04;
const EP_DATA_IN: u8 = 0x83;

/// The largest message the tool accepts, header and script included.
pub const MAX_MESSAGE_LEN: usize = 2048;

/// Size of the payload header plus the script header.
const HEADER_LEN: usize = 24;

/// Every response fits in one 512-byte packet.
const RESPONSE_LEN: usize = 512;

/// The tool sets this in the first word of every response it understood.
const STATUS_OK: u32 = 0x0d;

/// The status value the tool expects between a data phase and `script done`.
/// It is also the key a cold connection is probed with.
pub(crate) const ERROR_STATUS_KEY: &str = "ERROR_STATUS_KEY";

const TIMEOUT: Duration = Duration::from_secs(3);

/// How long a data phase is allowed to take, for a transfer of `len` bytes.
///
/// The wire sets the pace here, not USB. The memory scripts do a fully
/// addressed UPDI access per element, which measures at about 0.9 ms a byte on
/// an AVR128DA64, so the time a read takes is set by how much was asked for.
///
/// A fixed timeout therefore becomes a self-inflicted hang as soon as a read is
/// big enough. Reading 3072 bytes takes 2.70 s and passes. Asking for 4096
/// takes about 3.6 s, which used to trip the 3 s ceiling, and [`Transport`]
/// then latched a tool that was still busy answering correctly.
///
/// The allowance here is four times the measured rate, so a part clocked well
/// below the default still finishes in time. Callers also split large accesses
/// up, so this is the second line of defence rather than the first.
fn data_timeout(len: usize) -> Duration {
    TIMEOUT + Duration::from_millis(4 * len as u64)
}

/// Reads that are meant to come back empty use a short timeout.
const DRAIN_TIMEOUT: Duration = Duration::from_millis(250);

/// Error codes the tool returns in the first word of a script response.
mod error_code {
    pub const SUCCESS: u32 = 0x00;
    pub const TARGET_LOCKED: u32 = 0x44;
    pub const NO_TARGET: u32 = 0x51;
    pub const DEBUG_MODE_NO_TARGET: u32 = 0xe19;
}

#[derive(Clone, Copy)]
#[repr(u32)]
enum MessageType {
    /// Run the script, no bulk data.
    Command = 0x0000_0100,
    /// Run the script, then the host sends bulk data.
    ///
    /// avrdude writes this constant with a leading digit too many, so the value
    /// on the wire is the low 32 bits of it. Treat it as an opaque tag.
    Download = 0xc000_0101,
    /// Run the script, then the host reads bulk data.
    Upload = 0x8000_0102,
    /// End the data stream.
    ScriptDone = 0x0000_0103,
    /// Query a named status value.
    Status = 0x0000_0105,
}

/// A parameter block for a script.
///
/// A script consumes its parameters through a prologue of load instructions.
/// There are two loader opcodes and they take different widths, so the block is
/// not always an array of words. `0x91` loads a four-byte parameter and `0x99`
/// loads a single byte. The control and status register scripts use `0x99`, so
/// passing words to them would set every second value to zero.
///
/// Sending no parameter block at all to a script that expects one hangs the
/// tool.
///
/// # Examples
///
/// ```
/// use probe_rs::probe::pickit::Params;
///
/// // ReadMem8 takes an address and a length as words.
/// let read = Params::Words(&[0x4000, 16]);
/// // WriteCSreg takes a register address and a value as single bytes.
/// let write_cs = Params::Bytes(&[0x09, 0x01]);
/// # let _ = (read, write_cs);
/// ```
#[derive(Clone, Copy, Debug)]
pub enum Params<'a> {
    /// Four-byte parameters, which is the common case.
    Words(&'a [u32]),
    /// Single-byte parameters.
    Bytes(&'a [u8]),
}

impl Params<'_> {
    fn encode(&self) -> Vec<u8> {
        match self {
            Params::Words(words) => words.iter().flat_map(|w| w.to_le_bytes()).collect(),
            Params::Bytes(bytes) => bytes.to_vec(),
        }
    }
}

/// A decoded answer from the command endpoint.
///
/// The 16-byte header is stripped and the payload is cut to the length the tool
/// reported.
#[derive(Clone, Debug)]
pub struct Response {
    payload: Vec<u8>,
}

impl Response {
    fn parse(raw: &[u8]) -> Result<Self, PickitError> {
        if raw.len() < 16 {
            return Err(PickitError::ShortResponse);
        }

        let status = word(raw, 0);
        if status != STATUS_OK {
            return Err(PickitError::BadStatus(status));
        }

        let length = (word(raw, 8) as usize).clamp(16, raw.len());

        Ok(Self {
            payload: raw[16..length].to_vec(),
        })
    }

    /// Everything after the 16-byte header.
    ///
    /// A status query puts a NUL-terminated ASCII string here. A script
    /// response puts its error code here as a little-endian word.
    pub fn payload(&self) -> &[u8] {
        &self.payload
    }

    /// The result a script returned inside the response.
    ///
    /// Short results do not use the data endpoint. `GetDeviceID` returns its
    /// four bytes this way. The slice is empty when the response carries no
    /// inline result.
    ///
    /// The length word at payload offset 4 is what says whether a result is
    /// there. The word before it is not a marker, whatever it looks like most
    /// of the time, so nothing here may key on it.
    pub fn inline_data(&self) -> &[u8] {
        // Clamp before adding, or a device that claims a huge length
        // overflows the arithmetic.
        let length = (word(&self.payload, 4) as usize).min(self.payload.len());
        let start = 8;
        let end = (start + length).min(self.payload.len());

        self.payload.get(start..end).unwrap_or_default()
    }

    /// Turns the error code of a script response into an error.
    ///
    /// Call this only on script responses. The payload of a status query is
    /// text, not an error code.
    pub fn check(&self) -> Result<(), PickitError> {
        match word(&self.payload, 0) {
            error_code::SUCCESS => Ok(()),
            error_code::TARGET_LOCKED => Err(PickitError::TargetLocked),
            error_code::NO_TARGET => Err(PickitError::NoTarget),
            error_code::DEBUG_MODE_NO_TARGET => Err(PickitError::DebugModeNoTarget),
            other => Err(PickitError::Script(other)),
        }
    }
}

/// Reads a little-endian word, returning zero when the slice is too short.
fn word(buf: &[u8], offset: usize) -> u32 {
    buf.get(offset..offset + 4)
        .map(|b| u32::from_le_bytes(b.try_into().expect("slice is four bytes")))
        .unwrap_or(0)
}

fn header(message_type: MessageType, message_len: usize, transfer_len: usize) -> [u8; 16] {
    let mut header = [0; 16];
    header[0..4].copy_from_slice(&(message_type as u32).to_le_bytes());
    header[8..12].copy_from_slice(&(message_len as u32).to_le_bytes());
    header[12..16].copy_from_slice(&(transfer_len as u32).to_le_bytes());
    header
}

/// Builds one command message.
///
/// The buffer is a 16-byte payload header, an 8-byte script header, the
/// parameter block, and the script bytes, in that order.
fn message(
    message_type: MessageType,
    script: &[u8],
    params: Params<'_>,
    transfer_len: usize,
) -> Result<Vec<u8>, PickitError> {
    let params = params.encode();
    let message_len = HEADER_LEN + params.len() + script.len();
    if message_len > MAX_MESSAGE_LEN {
        return Err(PickitError::MessageTooLong(message_len));
    }

    let mut message = Vec::with_capacity(message_len);
    message.extend_from_slice(&header(message_type, message_len, transfer_len));
    message.extend_from_slice(&(params.len() as u32).to_le_bytes());
    message.extend_from_slice(&(script.len() as u32).to_le_bytes());
    message.extend_from_slice(&params);
    message.extend_from_slice(script);

    Ok(message)
}

fn read_packet(
    endpoint: &mut Endpoint<Bulk, In>,
    len: usize,
    timeout: Duration,
) -> io::Result<Vec<u8>> {
    let packet_size = endpoint.max_packet_size().max(1);
    let mut buffer = vec![0; len.max(1).next_multiple_of(packet_size)];

    let read = endpoint.read_bulk(&mut buffer, timeout)?;
    buffer.truncate(read);

    Ok(buffer)
}

/// The USB transport for one PICkit.
///
/// This owns the four bulk endpoints and the poisoned latch. The data
/// endpoints are private to this module so that the mandatory status query on
/// the write path cannot be bypassed.
pub struct Transport {
    /// Set on the first transfer timeout and never cleared.
    ///
    /// A tool that stops answering is hung, and every later operation would
    /// time out as well. Latching keeps the first real error visible and stops
    /// the driver from making things worse.
    poisoned: bool,
    cmd_out: Endpoint<Bulk, Out>,
    cmd_in: Endpoint<Bulk, In>,
    data_out: Endpoint<Bulk, Out>,
    data_in: Endpoint<Bulk, In>,
}

impl Transport {
    /// Claims the four bulk endpoints and puts the tool into a known state.
    pub fn new(interface: &Interface) -> io::Result<Self> {
        let mut transport = Self {
            poisoned: false,
            cmd_out: interface.endpoint::<Bulk, Out>(EP_CMD_OUT)?,
            cmd_in: interface.endpoint::<Bulk, In>(EP_CMD_IN)?,
            data_out: interface.endpoint::<Bulk, Out>(EP_DATA_OUT)?,
            data_in: interface.endpoint::<Bulk, In>(EP_DATA_IN)?,
        };
        transport.recover();

        Ok(transport)
    }

    /// True once a transfer has timed out.
    pub fn is_poisoned(&self) -> bool {
        self.poisoned
    }

    /// Drops whatever a previous session left behind.
    ///
    /// Responses survive across host processes. Opening a tool that was left
    /// mid-conversation can need several kilobytes drained before anything
    /// lines up again, so this reads until a read actually times out rather
    /// than until the first empty packet.
    ///
    /// The timeouts here are expected, so they must not poison the transport.
    /// Note that `clear_halt` is deliberately absent. On an endpoint that is
    /// not really halted it resets the data toggle on the host side only, which
    /// desynchronises the pipe and makes every later transfer time out.
    fn recover(&mut self) {
        let mut drained = 0;
        for _ in 0..40 {
            match read_packet(&mut self.cmd_in, RESPONSE_LEN, DRAIN_TIMEOUT) {
                Ok(packet) => drained += packet.len(),
                Err(_) => break,
            }
        }
        for _ in 0..20 {
            match read_packet(&mut self.data_in, RESPONSE_LEN, DRAIN_TIMEOUT) {
                Ok(packet) => drained += packet.len(),
                Err(_) => break,
            }
        }

        // Terminate a half-open stream from a previous run, then drop its reply.
        let done = header(MessageType::ScriptDone, 16, 0);
        let _ = self.cmd_out.write_bulk(&done, DRAIN_TIMEOUT);
        let _ = read_packet(&mut self.cmd_in, RESPONSE_LEN, DRAIN_TIMEOUT);

        if drained > 0 {
            tracing::debug!("dropped {drained} stale bytes from the PICkit");
        }
    }

    /// Runs a script that moves no bulk data.
    pub fn command(&mut self, script: &[u8], params: Params<'_>) -> Result<Response, PickitError> {
        self.send(MessageType::Command, script, params, 0)?;
        self.response()
    }

    /// Runs a script and reads the bytes it produces from the data endpoint.
    pub fn upload(
        &mut self,
        script: &[u8],
        params: Params<'_>,
        len: usize,
    ) -> Result<(Response, Vec<u8>), PickitError> {
        let mut stream = Stream::open(self);
        let result = stream.upload(script, params, len);

        if result.is_err() {
            // Only on the error path. Draining after a successful read costs a
            // full timeout on every operation and destroys throughput.
            stream.drain_data_in(len);
        }

        result
    }

    /// Runs a script and writes bytes to it over the data endpoint.
    pub fn download(
        &mut self,
        script: &[u8],
        params: Params<'_>,
        data: &[u8],
    ) -> Result<Response, PickitError> {
        let mut stream = Stream::open(self);
        stream.download(script, params, data)
    }

    /// Asks the tool for a named status value.
    pub fn status_query(&mut self, key: &str) -> Result<Response, PickitError> {
        let mut message = vec![0; 16];
        message.extend_from_slice(key.as_bytes());
        message.push(0);

        let length = message.len();
        message[0..16].copy_from_slice(&header(MessageType::Status, length, 0));

        self.write_cmd(&message)?;
        self.response()
    }

    fn send(
        &mut self,
        message_type: MessageType,
        script: &[u8],
        params: Params<'_>,
        transfer_len: usize,
    ) -> Result<(), PickitError> {
        let message = message(message_type, script, params, transfer_len)?;

        self.write_cmd(&message)
    }

    /// Reads the answer to a command, skipping the packets that are not one.
    ///
    /// The tool often sends a zero-length packet before the real answer, and it
    /// also sends full 512-byte all-zero packets. Treating either as the answer
    /// shifts every later response by one, and the symptom is that commands
    /// appear to return the previous command's result. A real answer always
    /// carries the status word.
    fn response(&mut self) -> Result<Response, PickitError> {
        self.check_poisoned()?;

        for _ in 0..6 {
            let packet = read_packet(&mut self.cmd_in, RESPONSE_LEN, TIMEOUT);
            let packet = self.poison(packet)?;

            if packet.iter().any(|&byte| byte != 0) {
                return Response::parse(&packet);
            }
        }

        Err(PickitError::NoResponse)
    }

    fn write_cmd(&mut self, message: &[u8]) -> Result<(), PickitError> {
        self.check_poisoned()?;

        let written = self.cmd_out.write_bulk(message, TIMEOUT);
        self.poison(written)?;

        Ok(())
    }

    fn check_poisoned(&self) -> Result<(), PickitError> {
        if self.poisoned {
            return Err(PickitError::Hung);
        }

        Ok(())
    }

    /// Latches the poisoned state when a transfer times out.
    fn poison<T>(&mut self, result: io::Result<T>) -> Result<T, PickitError> {
        if let Err(err) = &result
            && err.kind() == io::ErrorKind::TimedOut
        {
            self.poisoned = true;
            tracing::error!("the PICkit stopped answering, it needs to be unplugged and replugged");
            return Err(PickitError::Hung);
        }

        result.map_err(PickitError::Usb)
    }
}

/// A script stream that is torn down when it goes out of scope.
///
/// Abandoning a stream wedges the tool firmware beyond software recovery, so
/// the `script done` message has to go out on every path, including every early
/// return. Holding the transport inside a guard makes that automatic rather
/// than a call somebody has to remember.
struct Stream<'a> {
    transport: &'a mut Transport,
}

impl<'a> Stream<'a> {
    fn open(transport: &'a mut Transport) -> Self {
        Self { transport }
    }

    fn upload(
        &mut self,
        script: &[u8],
        params: Params<'_>,
        len: usize,
    ) -> Result<(Response, Vec<u8>), PickitError> {
        self.transport
            .send(MessageType::Upload, script, params, len)?;
        let response = self.transport.response()?;

        // The error code rides in this response. A script that produced
        // nothing never writes to the data endpoint, so reading it anyway
        // would time out and latch a healthy tool as hung.
        response.check()?;

        let data = read_packet(&mut self.transport.data_in, len, data_timeout(len));
        let mut data = self.transport.poison(data)?;
        data.truncate(len);

        Ok((response, data))
    }

    fn download(
        &mut self,
        script: &[u8],
        params: Params<'_>,
        data: &[u8],
    ) -> Result<Response, PickitError> {
        self.transport
            .send(MessageType::Download, script, params, data.len())?;
        let response = self.transport.response()?;

        // Fail before any data moves, for the same reason as in `upload`.
        response.check()?;

        if !data.is_empty() {
            self.send_data(data)?;
        }

        Ok(response)
    }

    /// Sends the data phase of a write and then the status query it requires.
    ///
    /// The status query between the data and the `script done` message is not
    /// optional. Without it the tool never considers the operation finished,
    /// the `script done` that follows gets no reply, and the tool wedges. The
    /// two steps live in one function, and the data endpoint is reachable
    /// nowhere else, so they cannot be separated.
    fn send_data(&mut self, data: &[u8]) -> Result<(), PickitError> {
        self.transport.check_poisoned()?;

        let written = self
            .transport
            .data_out
            .write_bulk(data, data_timeout(data.len()));
        self.transport.poison(written)?;

        self.transport.status_query(ERROR_STATUS_KEY)?;

        Ok(())
    }

    /// Throws away data the tool queued for a read that failed.
    fn drain_data_in(&mut self, len: usize) {
        let _ = read_packet(&mut self.transport.data_in, len, Duration::from_millis(400));
    }
}

impl Drop for Stream<'_> {
    fn drop(&mut self) {
        // A poisoned transport is hung, so the teardown would only time out as
        // well and bury the error that got us here.
        if self.transport.poisoned {
            return;
        }

        let done = header(MessageType::ScriptDone, 16, 0);
        let result = self
            .transport
            .write_cmd(&done)
            .and_then(|()| self.transport.response());

        if let Err(err) = result {
            tracing::error!(
                error = &err as &dyn std::error::Error,
                "the PICkit did not answer the script done message, it is now hung and needs to be unplugged and replugged"
            );
        }
    }
}

#[cfg(test)]
mod test {
    use super::*;

    /// A status query answer, as a PICkit Basic sent it.
    const STATUS_RESPONSE: &[u8] = &[
        0x0d, 0x00, 0x00, 0x00, // status
        0x00, 0x00, 0x00, 0x00, //
        0x15, 0x00, 0x00, 0x00, // length 21 = 16 + len("NONE\0")
        0x00, 0x00, 0x00, 0x00, //
        b'N', b'O', b'N', b'E', 0x00,
    ];

    fn script_response(error: u32) -> Vec<u8> {
        let mut raw = vec![0; 24];
        raw[0] = 0x0d;
        raw[8] = 24;
        raw[16..20].copy_from_slice(&error.to_le_bytes());
        raw
    }

    #[test]
    fn message_layout() {
        let message = message(
            MessageType::Upload,
            &[0xaa, 0xbb],
            Params::Words(&[0x1100, 3]),
            3,
        )
        .unwrap();

        assert_eq!(word(&message, 0), MessageType::Upload as u32);
        assert_eq!(word(&message, 8), 24 + 8 + 2);
        assert_eq!(word(&message, 12), 3);
        assert_eq!(word(&message, 16), 8);
        assert_eq!(word(&message, 20), 2);
        assert_eq!(
            &message[24..],
            &[0x00, 0x11, 0x00, 0x00, 3, 0, 0, 0, 0xaa, 0xbb]
        );
    }

    #[test]
    fn byte_parameters_are_not_padded() {
        let message = message(MessageType::Download, &[0x99], Params::Bytes(&[9, 1]), 0).unwrap();

        assert_eq!(word(&message, 16), 2);
        assert_eq!(&message[24..], &[9, 1, 0x99]);
    }

    #[test]
    fn message_length_is_capped() {
        let script = vec![0; MAX_MESSAGE_LEN];
        let err = message(MessageType::Command, &script, Params::Words(&[]), 0).unwrap_err();

        assert!(matches!(err, PickitError::MessageTooLong(_)));
    }

    #[test]
    fn status_query_payload_is_text() {
        let response = Response::parse(STATUS_RESPONSE).unwrap();

        assert_eq!(response.payload(), b"NONE\0");
        assert!(response.inline_data().is_empty());
    }

    /// Every figure here was measured on an AVR128DA64. The point of the
    /// allowance is that a healthy transfer must never trip it, because the
    /// transport treats a timeout as a hung tool and there is no way back.
    #[test]
    fn the_data_timeout_outlasts_a_healthy_transfer() {
        // 3072 bytes took 2.70 s, and 4096 used to fail against a flat 3 s.
        assert!(data_timeout(3072) > Duration::from_millis(2_700));
        assert!(data_timeout(4096) > Duration::from_millis(3_600));
        // The whole 16 KiB of SRAM took 14.46 s.
        assert!(data_timeout(16 * 1024) > Duration::from_millis(14_460));
    }

    /// A flat timeout is what caused the fault, so it has to grow.
    #[test]
    fn the_data_timeout_grows_with_the_transfer() {
        assert!(data_timeout(4096) > data_timeout(512));
        assert!(data_timeout(0) >= TIMEOUT);
    }

    #[test]
    fn error_codes_are_named() {
        assert!(
            Response::parse(&script_response(0x00))
                .unwrap()
                .check()
                .is_ok()
        );
        assert!(matches!(
            Response::parse(&script_response(0x44)).unwrap().check(),
            Err(PickitError::TargetLocked)
        ));
        assert!(matches!(
            Response::parse(&script_response(0x51)).unwrap().check(),
            Err(PickitError::NoTarget)
        ));
        assert!(matches!(
            Response::parse(&script_response(0xe19)).unwrap().check(),
            Err(PickitError::DebugModeNoTarget)
        ));
        assert!(matches!(
            Response::parse(&script_response(0x99)).unwrap().check(),
            Err(PickitError::Script(0x99))
        ));
    }

    /// Builds a response carrying four inline bytes, with `filler` in the word
    /// the tool leaves at offset 12.
    fn inline_response(filler: [u8; 4]) -> Vec<u8> {
        let mut raw = vec![0; 28];
        raw[0] = 0x0d;
        raw[8] = 28;
        raw[12..16].copy_from_slice(&filler);
        raw[20] = 4;
        raw[24..28].copy_from_slice(&[0x1e, 0x97, 0x07, 0x18]);

        raw
    }

    #[test]
    fn inline_data_is_found() {
        let response = Response::parse(&inline_response([0xa5; 4])).unwrap();

        assert!(response.check().is_ok());
        assert_eq!(response.inline_data(), &[0x1e, 0x97, 0x07, 0x18]);
    }

    /// The word at offset 12 reads `a5a5a5a5` most of the time, which made it
    /// look like a marker for inline results. It is not. A DA64 answered
    /// `GetHaltStatus` with `0xc0000101` there after a few dozen calls, and
    /// treating that as "no inline result" lost the answer.
    #[test]
    fn inline_data_does_not_depend_on_the_word_before_it() {
        let response = Response::parse(&inline_response([0x01, 0x01, 0x00, 0xc0])).unwrap();

        assert_eq!(response.inline_data(), &[0x1e, 0x97, 0x07, 0x18]);
    }

    #[test]
    fn a_response_without_inline_data_is_empty() {
        let mut raw = vec![0; 24];
        raw[0] = 0x0d;
        raw[8] = 24;
        raw[12..16].copy_from_slice(&[0xa5; 4]);

        let response = Response::parse(&raw).unwrap();

        assert!(response.inline_data().is_empty());
    }

    /// A claimed inline length far past the payload must not overflow the
    /// arithmetic. The result is capped at what the payload actually holds.
    #[test]
    fn an_over_long_inline_length_is_clamped() {
        let mut raw = vec![0; 32];
        raw[0] = 0x0d;
        raw[8] = 32;
        raw[20..24].copy_from_slice(&u32::MAX.to_le_bytes());
        raw[24..32].copy_from_slice(&[1, 2, 3, 4, 5, 6, 7, 8]);

        let response = Response::parse(&raw).unwrap();

        assert_eq!(response.inline_data(), &[1, 2, 3, 4, 5, 6, 7, 8]);
    }

    #[test]
    fn a_foreign_status_is_rejected() {
        let mut raw = script_response(0);
        raw[0] = 0x0e;

        assert!(matches!(
            Response::parse(&raw),
            Err(PickitError::BadStatus(0x0e))
        ));
        assert!(matches!(
            Response::parse(&[0x0d, 0x00]),
            Err(PickitError::ShortResponse)
        ));
    }
}

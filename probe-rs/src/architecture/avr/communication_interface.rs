//! Communication with an AVR target over UPDI.
//!
//! This is the layer between probe-rs and the PICkit transport. It opens and
//! closes the UPDI session, identifies the part, and moves bytes to and from
//! the two AVR address spaces.
//!
//! # Address convention
//!
//! The two sides of this module disagree about where memory lives, and this is
//! the only place that translates.
//!
//! probe-rs uses the avr-gcc convention everywhere above this module, because
//! that is what ELF and DWARF files for AVR use. Flash starts at `0x0` and the
//! data space starts at [`DATA_SPACE_OFFSET`], so SRAM on an AVR128DA64 is at
//! `0x804000`.
//!
//! The PICkit scripts use the inverse. The data space is at its native chip
//! address, so the same SRAM is at `0x4000`, and flash carries a base offset
//! instead.
//!
//! So both directions move an address, and both move it the other way round
//! from what the name suggests. [`to_chip_data_address`] subtracts
//! [`DATA_SPACE_OFFSET`] and [`to_tool_flash_address`] adds the flash base of
//! the part family.
//!
//! The data space offset is the same everywhere. The flash base is not, which
//! is why it comes from [`AvrFamily::flash_base`] rather than from a constant
//! here.

use std::fmt;

use crate::error::Error;
use crate::memory::MemoryInterface;
use crate::probe::DebugProbeError;
use crate::probe::pickit::{AvrFamily, Params, Pickit, ScriptName, SessionState};

/// Where the avr-gcc convention puts the AVR data space.
///
/// Flash sits below this address and the data space sits above it. The chip
/// itself puts the data space at zero, so this is also the constant that has to
/// come off before an address goes to the probe.
pub const DATA_SPACE_OFFSET: u64 = 0x0080_0000;

/// The highest data-space address the tool can be given.
///
/// Script parameters are 32-bit, so this bounds the translated address rather
/// than the AVR address space, which is far smaller.
const MAX_TOOL_ADDRESS: u64 = u32::MAX as u64;

/// The highest address the data space has.
///
/// The AVR data space is 64 KB on every part this supports. The low half holds
/// the registers, the IO space, the EEPROM and the SRAM, and the high half is
/// the window part of the flash is mapped into.
///
/// Handing the tool an address past this hangs it, so it is checked rather than
/// trusted. Wedge rule 4 is the same idea: never issue a memory access that the
/// part cannot answer.
const MAX_DATA_ADDRESS: u64 = 0xFFFF;

/// The largest single access handed to one script run.
///
/// The memory scripts do a fully addressed UPDI access per element, so the time
/// an access takes is set by how many bytes were asked for, not by USB. That
/// makes an unbounded access a liability: ask for enough and it outlives the
/// transport timeout, which latches the tool as hung when it was still working.
///
/// Splitting costs nothing measurable. The rate is flat against size, at
/// 1.1 KiB/s for everything from 256 bytes to 3072 on an AVR128DA64, so the
/// per-access overhead is already lost in the wire time.
///
/// This has to stay even, because a flash access is word organised. See
/// [`AvrCommunicationInterface::read_flash`].
const MAX_TRANSFER: usize = 512;

/// How many hardware breakpoint units the debug block has.
///
/// There are exactly two and there is no way around it.
pub const HW_BREAKPOINT_UNITS: usize = 2;

/// What `GetHaltStatus` answers for a stopped core.
///
/// An earlier note in this repository had the two values the other way round.
/// This is the polarity a live part showed, and it agrees with the script,
/// which branches to this value when `ASI_OCD_STATUS.STOPPED` is set.
const HALTED: u32 = 0xAAAA_AAAA;

/// What `GetHaltStatus` answers for a running core.
const RUNNING: u32 = 0x5555_5555;

/// Converts a probe-rs address in the data space to the address the chip uses.
///
/// probe-rs places the data space at [`DATA_SPACE_OFFSET`] and the chip places
/// it at zero, so this subtracts the offset.
///
/// # Examples
///
/// ```
/// use probe_rs::architecture::avr::communication_interface::to_chip_data_address;
///
/// // SRAM of an AVR128DA64.
/// assert_eq!(to_chip_data_address(0x80_4000).unwrap(), 0x4000);
/// // An address below the data space is flash, which this cannot translate.
/// assert!(to_chip_data_address(0x1000).is_err());
/// ```
pub fn to_chip_data_address(address: u64) -> Result<u32, AvrError> {
    let chip = address
        .checked_sub(DATA_SPACE_OFFSET)
        .ok_or(AvrError::NotInDataSpace(address))?;

    if chip > MAX_DATA_ADDRESS {
        return Err(AvrError::NotInDataSpace(address));
    }

    Ok(chip as u32)
}

/// Converts a probe-rs flash address to the address the tool scripts use.
///
/// probe-rs places flash at zero and the flash scripts place it at
/// [`AvrFamily::flash_base`], so this adds that base.
///
/// Both bases are verified on hardware, `0x800000` on an AVR128DA64 and
/// `0x8000` on an ATtiny406.
///
/// # Examples
///
/// ```
/// use probe_rs::architecture::avr::communication_interface::to_tool_flash_address;
/// use probe_rs::probe::pickit::AvrFamily;
///
/// assert_eq!(to_tool_flash_address(AvrFamily::Dx, 0).unwrap(), 0x80_0000);
/// assert_eq!(to_tool_flash_address(AvrFamily::Dx, 0x1_0000).unwrap(), 0x81_0000);
/// assert_eq!(to_tool_flash_address(AvrFamily::Tiny0, 0).unwrap(), 0x8000);
/// // An address at or above the data space offset is not in flash.
/// assert!(to_tool_flash_address(AvrFamily::Dx, 0x80_4000).is_err());
/// ```
pub fn to_tool_flash_address(family: AvrFamily, address: u64) -> Result<u32, AvrError> {
    if address >= DATA_SPACE_OFFSET {
        return Err(AvrError::NotInFlash(address));
    }

    let tool = address + family.flash_base();

    if tool > MAX_TOOL_ADDRESS {
        return Err(AvrError::NotInFlash(address));
    }

    Ok(tool as u32)
}

/// An error that happened while talking to an AVR target.
#[derive(thiserror::Error, Debug, docsplay::Display)]
pub enum AvrError {
    /// An error originating from the debug probe occurred.
    DebugProbe(#[from] DebugProbeError),

    /// The connected target is not an AVR device.
    NoAvrTarget,

    /// Address {0:#x} is not in the AVR data space.
    ///
    /// probe-rs uses the avr-gcc convention, which puts the data space at
    /// `0x800000`. Anything below that is flash.
    #[ignore_extra_doc_attributes]
    NotInDataSpace(u64),

    /// Address {0:#x} is not in AVR flash.
    ///
    /// probe-rs uses the avr-gcc convention, which puts flash below `0x800000`.
    /// Anything at or above that is the data space.
    #[ignore_extra_doc_attributes]
    NotInFlash(u64),

    /// The device signature reply was {0} bytes, which is too short to decode.
    ShortDeviceId(usize),

    /// The {script} script answered with {length} bytes, but a word was expected.
    ShortScriptReply {
        /// The script that answered.
        script: ScriptName,
        /// How many bytes it answered with.
        length: usize,
    },

    /// The halt status reply was {0:#010x}, which is neither halted nor running.
    UnknownHaltStatus(u32),

    /// Breakpoint unit {0} does not exist. An AVR has two.
    NoSuchBreakpointUnit(usize),

    /// Flash is word organised, so a write needs an even address and an even length, but got address {address:#x} and length {length}.
    UnalignedFlashWrite {
        /// The address of the access.
        address: u64,
        /// The length of the access in bytes.
        length: usize,
    },
}

impl From<AvrError> for crate::Error {
    fn from(err: AvrError) -> Self {
        match err {
            AvrError::DebugProbe(err) => err.into(),
            other => crate::Error::Avr(other),
        }
    }
}

/// The identity a part reports through `GetDeviceId`.
///
/// # Examples
///
/// ```
/// use probe_rs::architecture::avr::communication_interface::DeviceId;
///
/// // What an AVR128DA64 answers.
/// let id = DeviceId::from_bytes(&[0x1e, 0x97, 0x07, 0x18]).unwrap();
/// assert_eq!(id.signature(), 0x1e_9707);
/// assert_eq!(id.revision, 0x18);
/// ```
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct DeviceId {
    /// The three signature bytes, most significant first.
    pub signature: [u8; 3],
    /// The device revision byte.
    pub revision: u8,
}

impl DeviceId {
    /// Decodes the four bytes the tool returns.
    pub fn from_bytes(bytes: &[u8]) -> Result<Self, AvrError> {
        if bytes.len() < 4 {
            return Err(AvrError::ShortDeviceId(bytes.len()));
        }

        Ok(Self {
            signature: [bytes[0], bytes[1], bytes[2]],
            revision: bytes[3],
        })
    }

    /// The signature bytes packed into one number, as data sheets print it.
    pub fn signature(&self) -> u32 {
        u32::from_be_bytes([0, self.signature[0], self.signature[1], self.signature[2]])
    }
}

impl fmt::Display for DeviceId {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "{:02x}:{:02x}:{:02x} rev {:#04x}",
            self.signature[0], self.signature[1], self.signature[2], self.revision
        )
    }
}

/// The state of an AVR debug interface that outlives a single attach.
///
/// The caller owns this, the same way it owns the Xtensa and RISC-V interface
/// state. It carries the choice of script table, which has to be made before
/// the first script runs, and the identity once it has been read.
#[derive(Debug)]
pub struct AvrDebugInterfaceState {
    family: AvrFamily,
    speed_khz: Option<u32>,
    device_id: Option<DeviceId>,
}

impl AvrDebugInterfaceState {
    /// Creates the state for a target of the given family.
    ///
    /// # Examples
    ///
    /// ```
    /// use probe_rs::architecture::avr::communication_interface::AvrDebugInterfaceState;
    /// use probe_rs::probe::pickit::AvrFamily;
    ///
    /// let state = AvrDebugInterfaceState::new(AvrFamily::Dx);
    /// assert_eq!(state.family(), AvrFamily::Dx);
    /// ```
    pub fn new(family: AvrFamily) -> Self {
        Self {
            family,
            speed_khz: None,
            device_id: None,
        }
    }

    /// The script table this interface runs.
    pub fn family(&self) -> AvrFamily {
        self.family
    }

    /// The identity read from the part, if it has been read already.
    pub fn device_id(&self) -> Option<DeviceId> {
        self.device_id
    }

    /// Requests a UPDI clock, applied when the session opens.
    ///
    /// The tool only accepts a clock while a session exists, so a value set
    /// before that is remembered rather than sent.
    pub fn set_speed_khz(&mut self, speed_khz: u32) {
        self.speed_khz = Some(speed_khz);
    }
}

/// Talks to an AVR target through a PICkit.
///
/// The interface borrows the tool for as long as it lives, and translates
/// between the probe-rs address convention and the one the tool scripts use.
/// See the module documentation for that translation.
///
/// A fresh tool has no UPDI link. [`AvrCommunicationInterface::enter_programming_mode`]
/// is the only operation that is safe first, and everything else fails until it
/// has run.
pub struct AvrCommunicationInterface<'probe> {
    probe: &'probe mut Pickit,
    state: &'probe mut AvrDebugInterfaceState,
}

impl fmt::Debug for AvrCommunicationInterface<'_> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("AvrCommunicationInterface")
            .field("probe", &self.probe)
            .field("state", &self.state)
            .finish_non_exhaustive()
    }
}

impl<'probe> AvrCommunicationInterface<'probe> {
    /// Builds the interface and installs the script table for the target.
    pub fn new(probe: &'probe mut Pickit, state: &'probe mut AvrDebugInterfaceState) -> Self {
        probe.set_scripts(Box::new(state.family));

        Self { probe, state }
    }

    /// Where the tool is in its session lifecycle.
    pub fn session_state(&self) -> SessionState {
        self.probe.state()
    }

    /// The part family this interface was built for.
    pub fn family(&self) -> AvrFamily {
        self.state.family
    }

    /// Opens a programming session, which is the only safe first operation.
    ///
    /// Any requested UPDI clock is applied afterwards, because the tool only
    /// accepts one once a session exists.
    pub fn enter_programming_mode(&mut self) -> Result<(), AvrError> {
        self.probe.enter_prog_mode().map_err(probe_error)?;

        if let Some(speed_khz) = self.state.speed_khz {
            self.probe.set_speed_khz(speed_khz).map_err(probe_error)?;
        }

        Ok(())
    }

    /// Opens a debug session on a running part without restarting it.
    ///
    /// [`AvrCommunicationInterface::enter_programming_mode`] asserts
    /// `ASI_RESET_REQ`, so the ordinary way in restarts the target and destroys
    /// whatever state you attached to look at. The `EnterDebugMode` script
    /// writes nothing to that register, so this reaches a part mid-flight and
    /// leaves it running. Confirmed on an AVR128DA64: the halt status still
    /// reads running afterwards and the program counter keeps moving.
    ///
    /// The core is left running. Halt it if you want it stopped.
    ///
    /// # Why this is not the default
    ///
    /// Only `EnterProgMode` reports whether the part is locked, so a session
    /// opened this way never learns it. A locked part hangs the tool on the
    /// first memory access, and only a replug recovers it. The two properties
    /// are mutually exclusive with these scripts: a session either learns the
    /// lock state or leaves the part running, never both.
    ///
    /// Use this only on a part known to be unlocked.
    pub fn attach_without_reset(&mut self) -> Result<(), AvrError> {
        self.probe.enter_debug_mode_hot().map_err(probe_error)?;

        if let Some(speed_khz) = self.state.speed_khz {
            self.probe.set_speed_khz(speed_khz).map_err(probe_error)?;
        }

        Ok(())
    }

    /// Reads the device signature and revision.
    ///
    /// The answer is cached in the interface state, so repeated calls cost
    /// nothing on the wire.
    pub fn device_id(&mut self) -> Result<DeviceId, AvrError> {
        if let Some(device_id) = self.state.device_id {
            return Ok(device_id);
        }

        // GetDeviceId answers inside the response rather than on the data
        // pipe, so this is a plain command and not a read.
        let response = self
            .probe
            .run(ScriptName::GetDeviceId, Params::Words(&[]))
            .map_err(probe_error)?;

        let device_id = DeviceId::from_bytes(response.inline_data())?;
        self.state.device_id = Some(device_id);

        Ok(device_id)
    }

    /// Reads the data space one byte at a time.
    ///
    /// `address` is a probe-rs address, so it is at or above
    /// [`DATA_SPACE_OFFSET`].
    pub fn read_data_8(&mut self, address: u64, data: &mut [u8]) -> Result<(), AvrError> {
        self.read(ScriptName::ReadMem8, to_chip_data_address(address)?, data)
    }

    /// Writes the data space one byte at a time.
    ///
    /// Split into [`MAX_TRANSFER`] blocks. Flash is deliberately not split this
    /// way, because a flash write erases whole pages and the caller aligns it.
    pub fn write_data_8(&mut self, address: u64, data: &[u8]) -> Result<(), AvrError> {
        let chip = to_chip_data_address(address)?;

        for (index, block) in data.chunks(MAX_TRANSFER).enumerate() {
            let offset = (index * MAX_TRANSFER) as u32;
            self.write(ScriptName::WriteMem8, chip + offset, block)?;
        }

        Ok(())
    }

    /// Reads flash.
    ///
    /// `address` is a probe-rs address, so flash starts at zero. Addressing is
    /// linear across the whole device, so this also reaches the part of flash
    /// that is not mapped into the data space.
    ///
    /// Any address and any length work here. The script underneath accepts
    /// neither, so this widens the request out to whole words and hands back
    /// the slice that was asked for.
    ///
    /// # Why the request is widened
    ///
    /// `ReadProgmem` reads `length / 2` words, rounded down, starting at the
    /// address it is given. Flash is word organised and the script does not
    /// check either end of the request, so both go wrong on their own:
    ///
    /// - An odd address reads misaligned words, and each one comes back as the
    ///   byte at that address twice. On an ATtiny406, four bytes at `0x41` read
    ///   as `d0 d0 c0 c0` where the flash really holds `d0 13 c0 dd`.
    /// - An odd length returns one byte less than was asked for, and a length
    ///   of one returns nothing at all. Nothing is the dangerous case: the tool
    ///   never writes to the data pipe, the host waits for a byte that is not
    ///   coming, and the timeout hangs the tool. Only a replug clears it.
    ///
    /// A one-byte read is exactly what asking the debugger for a `u8` in flash
    /// produces, so this is reachable from ordinary use.
    pub fn read_flash(&mut self, address: u64, data: &mut [u8]) -> Result<(), AvrError> {
        if data.is_empty() {
            return Ok(());
        }

        let (start, skip, widened) = word_span(address, data.len());
        let tool_address = to_tool_flash_address(self.state.family, start)?;

        let mut words = vec![0; widened];
        self.read(ScriptName::ReadProgmem, tool_address, &mut words)?;
        data.copy_from_slice(&words[skip..skip + data.len()]);

        Ok(())
    }

    /// Writes flash.
    ///
    /// The script erases and programs whole pages, and the page size is baked
    /// into the bytecode rather than passed in. A write that does not cover a
    /// whole page leaves the rest of that page in an undefined state, so the
    /// caller has to align its writes itself.
    ///
    /// Both ends have to be even, for the reasons in
    /// [`AvrCommunicationInterface::read_flash`]. A read can widen the request
    /// and trim the answer, but a write cannot, because widening it would put
    /// bytes into flash that the caller never asked to write. So this rejects
    /// the access instead.
    pub fn write_flash(&mut self, address: u64, data: &[u8]) -> Result<(), AvrError> {
        if !address.is_multiple_of(2) || !data.len().is_multiple_of(2) {
            return Err(AvrError::UnalignedFlashWrite {
                address,
                length: data.len(),
            });
        }

        let tool_address = to_tool_flash_address(self.state.family, address)?;

        self.write(ScriptName::WriteProgmem, tool_address, data)
    }

    /// Erases flash, EEPROM, and the lock bits.
    ///
    /// The lock state changes underneath the session, so the tool ends the
    /// session. A fresh programming session is opened here, which leaves the
    /// interface usable and the core held in reset. A caller that was debugging
    /// has to enter debug mode again.
    pub fn erase_chip(&mut self) -> Result<(), AvrError> {
        self.probe.erase_chip().map_err(probe_error)?;
        self.state.device_id = None;

        self.enter_programming_mode()
    }

    /// Switches the open programming session over to debugging.
    ///
    /// This sends the on-chip debug key and then lets the part out of reset, so
    /// it leaves the core running. A caller that wants a halted core has to halt
    /// it afterwards.
    ///
    /// It does nothing when the session is already a debug session, which is
    /// what happens when a second core handle is taken from the same session.
    pub fn enter_debug_mode(&mut self) -> Result<(), AvrError> {
        if self.probe.state() == SessionState::Debugging {
            return Ok(());
        }

        self.probe.enter_debug_mode().map_err(probe_error)?;

        // Opening the session asserted reset and the part is still sitting in
        // it. Until it is let go the core executes nothing, which makes `Halt`
        // fail rather than stop anything. A debug reset is what releases it,
        // and it leaves the core stopped on the reset vector.
        self.debug_reset()
    }

    /// Reports whether the core is stopped.
    pub fn is_halted(&mut self) -> Result<bool, AvrError> {
        match self.inline_word(ScriptName::GetHaltStatus, Params::Words(&[]))? {
            HALTED => Ok(true),
            RUNNING => Ok(false),
            other => Err(AvrError::UnknownHaltStatus(other)),
        }
    }

    /// Stops the core.
    pub fn halt(&mut self) -> Result<(), AvrError> {
        self.command(ScriptName::Halt, Params::Words(&[]))
    }

    /// Starts the core.
    pub fn run(&mut self) -> Result<(), AvrError> {
        self.command(ScriptName::Run, Params::Words(&[]))
    }

    /// Executes one instruction and stops again.
    ///
    /// The script sets the step trap, resumes, and waits for the halt, so the
    /// core is stopped again when this returns.
    pub fn step(&mut self) -> Result<(), AvrError> {
        self.command(ScriptName::SingleStep, Params::Words(&[]))
    }

    /// Resets the core and leaves it stopped at the reset vector.
    ///
    /// This is reset and halt in one operation, so nothing has to catch the
    /// core on its way out of reset.
    pub fn debug_reset(&mut self) -> Result<(), AvrError> {
        self.command(ScriptName::DebugReset, Params::Words(&[]))
    }

    /// Reads the program counter as a word address.
    ///
    /// The debug block holds the program counter plus one, and in different
    /// units on the two debug revisions. The script undoes both, so what comes
    /// back here is the plain word address of the next instruction.
    pub fn program_counter(&mut self) -> Result<u32, AvrError> {
        self.inline_word(ScriptName::GetPc, Params::Words(&[]))
    }

    /// Writes the program counter, as a word address.
    pub fn set_program_counter(&mut self, word_address: u32) -> Result<(), AvrError> {
        self.command(ScriptName::SetPc, Params::Words(&[word_address]))
    }

    /// Arms a hardware breakpoint at a word address.
    ///
    /// The script writes the address register and the per-unit enable bit, but
    /// not the global one, so the caller still has to set `HWBP` in `TRAPEN`.
    pub fn set_hw_breakpoint(&mut self, unit: usize, word_address: u32) -> Result<(), AvrError> {
        self.command(
            ScriptName::SetHwBp,
            Params::Words(&[check_breakpoint_unit(unit)?, word_address]),
        )
    }

    /// Disarms a hardware breakpoint.
    pub fn clear_hw_breakpoint(&mut self, unit: usize) -> Result<(), AvrError> {
        self.command(
            ScriptName::ClearHwBp,
            Params::Words(&[check_breakpoint_unit(unit)?]),
        )
    }

    /// Closes the session and leaves the tool ready for a new one.
    pub fn close(&mut self) -> Result<(), AvrError> {
        self.probe.exit().map_err(probe_error)?;
        self.state.device_id = None;

        Ok(())
    }

    /// Runs a script that answers with nothing.
    fn command(&mut self, name: ScriptName, params: Params<'_>) -> Result<(), AvrError> {
        self.probe.run(name, params).map_err(probe_error)?;

        Ok(())
    }

    /// Runs a script that answers with one word inside the response.
    fn inline_word(&mut self, name: ScriptName, params: Params<'_>) -> Result<u32, AvrError> {
        let response = self.probe.run(name, params).map_err(probe_error)?;
        let data = response.inline_data();

        let bytes = data
            .get(..4)
            .ok_or(AvrError::ShortScriptReply {
                script: name,
                length: data.len(),
            })?
            .try_into()
            .expect("slice is four bytes");

        Ok(u32::from_le_bytes(bytes))
    }

    fn read(
        &mut self,
        name: ScriptName,
        tool_address: u32,
        data: &mut [u8],
    ) -> Result<(), AvrError> {
        if data.is_empty() {
            return Ok(());
        }

        for (index, block) in data.chunks_mut(MAX_TRANSFER).enumerate() {
            let address = tool_address + (index * MAX_TRANSFER) as u32;

            tracing::trace!(
                script = ?name,
                address = format_args!("{address:#010x}"),
                len = block.len(),
                "reading target memory"
            );

            let params = [address, block.len() as u32];
            let read = self
                .probe
                .read(name, Params::Words(&params), block.len())
                .map_err(probe_error)?;

            let len = read.len().min(block.len());
            block[..len].copy_from_slice(&read[..len]);
        }

        Ok(())
    }

    fn write(&mut self, name: ScriptName, tool_address: u32, data: &[u8]) -> Result<(), AvrError> {
        if data.is_empty() {
            return Ok(());
        }

        tracing::trace!(
            script = ?name,
            address = format_args!("{tool_address:#010x}"),
            len = data.len(),
            "writing target memory"
        );

        let params = [tool_address, data.len() as u32];

        self.probe
            .write(name, Params::Words(&params), data)
            .map_err(probe_error)
    }
}

/// An AVR has two breakpoint units, and the script takes the index as a word.
fn check_breakpoint_unit(unit: usize) -> Result<u32, AvrError> {
    if unit >= HW_BREAKPOINT_UNITS {
        return Err(AvrError::NoSuchBreakpointUnit(unit));
    }

    Ok(unit as u32)
}

/// The whole-word span that covers a flash request.
///
/// Returns the address to ask the script for, how many bytes of the answer to
/// drop at the front, and how many bytes to ask for. See
/// [`AvrCommunicationInterface::read_flash`] for why a flash read has to be
/// widened at all.
///
/// # Examples
///
/// ```
/// use probe_rs::architecture::avr::communication_interface::word_span;
///
/// // An aligned request is already a whole number of words.
/// assert_eq!(word_span(0x40, 8), (0x40, 0, 8));
/// // An odd address moves the request back a byte and drops that byte.
/// assert_eq!(word_span(0x41, 2), (0x40, 1, 4));
/// // An odd length rounds up, which is what stops a one-byte read hanging.
/// assert_eq!(word_span(0x40, 1), (0x40, 0, 2));
/// ```
pub fn word_span(address: u64, len: usize) -> (u64, usize, usize) {
    let skip = usize::from(!address.is_multiple_of(2));

    (address - skip as u64, skip, (skip + len).next_multiple_of(2))
}

/// Wraps a probe-specific error so this module does not name the probe driver.
fn probe_error(err: impl crate::probe::ProbeError) -> AvrError {
    AvrError::DebugProbe(DebugProbeError::ProbeSpecific(err.into()))
}

/// Which of the two AVR address spaces an address falls in.
///
/// AVR is a Harvard machine, so flash and the data space are separate memories
/// that both start at zero on the chip. probe-rs tells them apart by address,
/// following the avr-gcc convention, and so does this.
///
/// # Examples
///
/// ```
/// use probe_rs::architecture::avr::communication_interface::AddressSpace;
///
/// // The reset vector.
/// assert_eq!(AddressSpace::of(0x0), AddressSpace::Flash);
/// // SRAM of an AVR128DA64.
/// assert_eq!(AddressSpace::of(0x80_4000), AddressSpace::Data);
/// ```
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum AddressSpace {
    /// Flash, which probe-rs places below [`DATA_SPACE_OFFSET`].
    Flash,
    /// The data space, which probe-rs places at [`DATA_SPACE_OFFSET`] and above.
    ///
    /// This one memory holds the IO registers, the fuses, the EEPROM, SRAM, and
    /// the window that part of flash is mapped into.
    Data,
}

impl AddressSpace {
    /// The space a probe-rs address belongs to.
    pub fn of(address: u64) -> Self {
        if address < DATA_SPACE_OFFSET {
            AddressSpace::Flash
        } else {
            AddressSpace::Data
        }
    }
}

impl AvrCommunicationInterface<'_> {
    /// Reads bytes from whichever space the address belongs to.
    fn read_bytes(&mut self, address: u64, data: &mut [u8]) -> Result<(), AvrError> {
        match AddressSpace::of(address) {
            AddressSpace::Flash => self.read_flash(address, data),
            AddressSpace::Data => self.read_data_8(address, data),
        }
    }

    /// Writes bytes to whichever space the address belongs to.
    fn write_bytes(&mut self, address: u64, data: &[u8]) -> Result<(), AvrError> {
        match AddressSpace::of(address) {
            AddressSpace::Flash => self.write_flash(address, data),
            AddressSpace::Data => self.write_data_8(address, data),
        }
    }

}

/// Memory access on an 8-bit Harvard machine.
///
/// Every access routes to flash or to the data space by address, and every
/// width is built out of the byte-wide scripts.
///
/// # Why the word-wide scripts are not used
///
/// The tool pack ships `ReadMem16` and `WriteMem16`, and they are wrong on
/// byte-organised memory. On an ATtiny406 a word read of SRAM returns the byte
/// at the even address twice, and a word write stores only the low byte and
/// leaves the odd byte alone. Both fail silently.
///
/// They work on word-organised memory at an even address, which is flash and
/// the signature row, and nowhere else. Reading `0x1100` gives the true bytes
/// while `0x1103`, the same row one byte along, duplicates. SRAM, the fuses and
/// the EEPROM are byte memories, so every access to them duplicates.
///
/// Nothing is lost by dropping them. The scripts do a fully addressed access
/// per element either way, so a word-wide read is not faster than two
/// byte-wide ones.
impl MemoryInterface<Error> for AvrCommunicationInterface<'_> {
    fn supports_native_64bit_access(&mut self) -> bool {
        false
    }

    fn supports_8bit_transfers(&self) -> Result<bool, Error> {
        Ok(true)
    }

    fn read_8(&mut self, address: u64, data: &mut [u8]) -> Result<(), Error> {
        Ok(self.read_bytes(address, data)?)
    }

    /// Reads 16-bit values a byte at a time. See the note on this impl.
    fn read_16(&mut self, address: u64, data: &mut [u16]) -> Result<(), Error> {
        let mut bytes = vec![0; data.len() * 2];
        self.read_bytes(address, &mut bytes)?;

        let (chunks, _rest) = bytes.as_chunks::<2>();
        for (word, chunk) in data.iter_mut().zip(chunks) {
            *word = u16::from_le_bytes(*chunk);
        }

        Ok(())
    }

    fn read_32(&mut self, address: u64, data: &mut [u32]) -> Result<(), Error> {
        let mut bytes = vec![0; data.len() * 4];
        self.read_bytes(address, &mut bytes)?;

        let (chunks, _rest) = bytes.as_chunks::<4>();
        for (word, chunk) in data.iter_mut().zip(chunks) {
            *word = u32::from_le_bytes(*chunk);
        }

        Ok(())
    }

    fn read_64(&mut self, address: u64, data: &mut [u64]) -> Result<(), Error> {
        let mut bytes = vec![0; data.len() * 8];
        self.read_bytes(address, &mut bytes)?;

        let (chunks, _rest) = bytes.as_chunks::<8>();
        for (word, chunk) in data.iter_mut().zip(chunks) {
            *word = u64::from_le_bytes(*chunk);
        }

        Ok(())
    }

    /// Reads without widening the access.
    ///
    /// The default implementation rounds the access out to 32-bit boundaries.
    /// The bottom of the AVR data space is the IO registers, where reading a
    /// byte nobody asked for can have a side effect, so this reads exactly what
    /// was asked for instead.
    fn read(&mut self, address: u64, data: &mut [u8]) -> Result<(), Error> {
        Ok(self.read_bytes(address, data)?)
    }

    fn write_8(&mut self, address: u64, data: &[u8]) -> Result<(), Error> {
        Ok(self.write_bytes(address, data)?)
    }

    /// Writes 16-bit values a byte at a time. See the note on this impl.
    fn write_16(&mut self, address: u64, data: &[u16]) -> Result<(), Error> {
        let bytes: Vec<u8> = data.iter().flat_map(|word| word.to_le_bytes()).collect();

        Ok(self.write_bytes(address, &bytes)?)
    }

    fn write_32(&mut self, address: u64, data: &[u32]) -> Result<(), Error> {
        let bytes: Vec<u8> = data.iter().flat_map(|word| word.to_le_bytes()).collect();

        Ok(self.write_bytes(address, &bytes)?)
    }

    fn write_64(&mut self, address: u64, data: &[u64]) -> Result<(), Error> {
        let bytes: Vec<u8> = data.iter().flat_map(|word| word.to_le_bytes()).collect();

        Ok(self.write_bytes(address, &bytes)?)
    }

    /// Writes without widening the access. See [`MemoryInterface::read`].
    fn write(&mut self, address: u64, data: &[u8]) -> Result<(), Error> {
        Ok(self.write_bytes(address, data)?)
    }

    /// Nothing is buffered, so there is nothing to flush.
    fn flush(&mut self) -> Result<(), Error> {
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The three memories of an AVR128DA64, as the target description and the
    /// chip each see them. See `targets.md`.
    const DATA_SPACE: &[(u64, u32)] = &[
        // IO at the very bottom of the data space.
        (0x80_0000, 0x0000),
        // Fuses.
        (0x80_1050, 0x1050),
        // EEPROM.
        (0x80_1400, 0x1400),
        // SRAM.
        (0x80_4000, 0x4000),
        // The mapped flash window.
        (0x80_8000, 0x8000),
    ];

    #[test]
    fn data_addresses_lose_the_offset() {
        for &(probe_rs, chip) in DATA_SPACE {
            assert_eq!(to_chip_data_address(probe_rs).unwrap(), chip);
        }
    }

    /// The Dx flash base is the one that was read back from a part.
    #[test]
    fn dx_flash_addresses_gain_the_verified_base() {
        let dx = AvrFamily::Dx;

        // Flash is at zero for probe-rs and at the base for the tool.
        assert_eq!(to_tool_flash_address(dx, 0x0).unwrap(), 0x80_0000);
        assert_eq!(to_tool_flash_address(dx, 0x200).unwrap(), 0x80_0200);
        // Above the 32 KiB mapped window, where addressing stays linear.
        assert_eq!(to_tool_flash_address(dx, 0x1_0000).unwrap(), 0x81_0000);
        assert_eq!(to_tool_flash_address(dx, 0x1_F000).unwrap(), 0x81_F000);
        // The last byte of a 128 KiB part.
        assert_eq!(to_tool_flash_address(dx, 0x1_FFFF).unwrap(), 0x81_FFFF);
    }

    /// The tiny base differs from the Dx one, so pin it separately. Both are
    /// read back from a part, an ATtiny406 here and an AVR128DA64 above.
    #[test]
    fn tiny_flash_addresses_gain_the_tiny_base() {
        let tiny = AvrFamily::Tiny0;

        assert_eq!(to_tool_flash_address(tiny, 0x0).unwrap(), 0x8000);
        assert_eq!(to_tool_flash_address(tiny, 0x40).unwrap(), 0x8040);
        // The last byte of the 4 KiB flash of an ATtiny406.
        assert_eq!(to_tool_flash_address(tiny, 0x0FFF).unwrap(), 0x8FFF);
    }

    /// Every widened span has to start on a word, cover a whole number of
    /// words, and still contain the bytes that were asked for.
    #[test]
    fn a_widened_flash_read_covers_the_request() {
        for address in 0..8u64 {
            for len in 1..8usize {
                let (start, skip, widened) = word_span(address, len);

                assert!(start.is_multiple_of(2), "{address:#x} {len}");
                assert!(widened.is_multiple_of(2), "{address:#x} {len}");
                assert_eq!(start + skip as u64, address);
                assert!(skip + len <= widened, "{address:#x} {len}");
            }
        }
    }

    /// A one-byte read is the case that hung a tool, so pin it on its own.
    #[test]
    fn a_one_byte_flash_read_asks_for_a_whole_word() {
        assert_eq!(word_span(0x40, 1), (0x40, 0, 2));
        assert_eq!(word_span(0x41, 1), (0x40, 1, 2));
    }

    /// A split must not land in the middle of a word, or a flash access on
    /// either side of it reads misaligned. See `read_flash`.
    #[test]
    fn the_transfer_limit_is_a_whole_number_of_words() {
        const { assert!(MAX_TRANSFER > 0) };
        const { assert!(MAX_TRANSFER.is_multiple_of(2)) };
    }

    /// The two spaces meet at the offset, and neither may cross into the other.
    #[test]
    fn the_two_spaces_do_not_overlap() {
        for family in [AvrFamily::Dx, AvrFamily::Tiny0] {
            assert!(to_tool_flash_address(family, DATA_SPACE_OFFSET - 1).is_ok());
            assert!(to_tool_flash_address(family, DATA_SPACE_OFFSET).is_err());
        }

        assert!(to_chip_data_address(DATA_SPACE_OFFSET - 1).is_err());
        assert_eq!(to_chip_data_address(DATA_SPACE_OFFSET).unwrap(), 0);
    }

    /// Flash goes up and data goes down, so translating one way and back is not
    /// the identity. This pins the direction of each function.
    #[test]
    fn the_two_translations_move_in_opposite_directions() {
        for family in [AvrFamily::Dx, AvrFamily::Tiny0] {
            let flash = to_tool_flash_address(family, 0x1234).unwrap();
            assert!(u64::from(flash) > 0x1234);
        }

        let data = to_chip_data_address(0x80_1234).unwrap();
        assert!(u64::from(data) < 0x80_1234);
    }

    /// A stale or garbage variable location lands here, and forwarding it to
    /// the tool hangs it rather than returning an error.
    #[test]
    fn data_addresses_past_the_data_space_are_rejected() {
        let too_high = DATA_SPACE_OFFSET + MAX_DATA_ADDRESS + 1;

        assert!(to_chip_data_address(too_high).is_err());
        assert!(to_chip_data_address(too_high - 1).is_ok());
        assert!(to_chip_data_address(DATA_SPACE_OFFSET + 0x1_0000).is_err());
    }

    /// Routing is the whole of the Harvard split, so pin where the boundary is
    /// and that each of the three memories of a part lands on the right side.
    #[test]
    fn addresses_route_to_the_space_that_holds_them() {
        assert_eq!(AddressSpace::of(0x0), AddressSpace::Flash);
        assert_eq!(AddressSpace::of(0x1_FFFF), AddressSpace::Flash);
        assert_eq!(AddressSpace::of(DATA_SPACE_OFFSET - 1), AddressSpace::Flash);

        assert_eq!(AddressSpace::of(DATA_SPACE_OFFSET), AddressSpace::Data);
        for &(probe_rs, _) in DATA_SPACE {
            assert_eq!(AddressSpace::of(probe_rs), AddressSpace::Data);
        }
    }

    /// The debug block is in the data space, so run control reaches it with the
    /// ordinary memory scripts.
    #[test]
    fn the_debug_block_is_in_the_data_space() {
        use crate::architecture::avr::ocd;

        assert_eq!(
            AddressSpace::of(ocd::address(ocd::REGISTER_FILE)),
            AddressSpace::Data
        );
        assert_eq!(
            to_chip_data_address(ocd::address(ocd::REGISTER_FILE)).unwrap(),
            0x0FA0
        );
    }

    #[test]
    fn only_the_two_breakpoint_units_are_accepted() {
        assert_eq!(check_breakpoint_unit(0).unwrap(), 0);
        assert_eq!(check_breakpoint_unit(1).unwrap(), 1);
        assert!(check_breakpoint_unit(2).is_err());
    }

    #[test]
    fn device_id_decodes_the_avr128da64_reply() {
        let id = DeviceId::from_bytes(&[0x1e, 0x97, 0x07, 0x18]).unwrap();

        assert_eq!(id.signature, [0x1e, 0x97, 0x07]);
        assert_eq!(id.signature(), 0x1e_9707);
        assert_eq!(id.revision, 0x18);
        assert_eq!(id.to_string(), "1e:97:07 rev 0x18");
    }

    #[test]
    fn device_id_needs_four_bytes() {
        assert!(DeviceId::from_bytes(&[0x1e, 0x92, 0x25]).is_err());
    }
}

//! The session state machine.
//!
//! Two of the ways to hang a PICkit depend on what happened before the current
//! operation. Talking to the target before a UPDI session exists hangs it, and
//! so does any memory access to a locked part. Both are tracked here, and the
//! methods refuse the unsafe combinations before anything reaches the wire.

use nusb::DeviceInfo;

use super::{
    PickitError,
    protocol::{Params, Response, Transport},
    scripts::{Script, ScriptName, ScriptSource},
};
use crate::probe::{DebugProbeSelector, ProbeCreationError};

/// Where a PICkit is in its session lifecycle.
///
/// The only way out of [`Cold`](SessionState::Cold) is
/// [`Pickit::enter_prog_mode`], because that is the only operation that is safe
/// on a tool that has not talked to the target yet.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SessionState {
    /// No UPDI session exists.
    ///
    /// Only `EnterProgMode` may run. Anything else has no link to talk over and
    /// hangs the tool.
    Cold,
    /// A programming session is open on an unlocked part.
    Programming,
    /// A debug session is open on an unlocked part.
    Debugging,
    /// A session is open but the part is locked.
    ///
    /// Memory access hangs the tool in this state, so it is refused. A chip
    /// erase is the only way forward.
    Locked,
    /// A transfer timed out, so the tool is hung.
    ///
    /// Nothing recovers it except unplugging it. Every operation fails from
    /// here.
    Hung,
}

/// An open PICkit, and the session state that keeps it safe.
///
/// This is the transport. It does not implement [`DebugProbe`] and it knows
/// nothing about AVR cores. It runs scripts, moves bulk data, and refuses the
/// operations that are known to hang the tool.
///
/// A freshly opened tool has no scripts. Call [`Pickit::set_scripts`] with the
/// blobs for the target device before running anything.
///
/// [`DebugProbe`]: crate::probe::DebugProbe
pub struct Pickit {
    transport: Transport,
    scripts: Option<Box<dyn ScriptSource>>,
    state: SessionState,
}

impl std::fmt::Debug for Pickit {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Pickit")
            .field("state", &self.state)
            .field("scripts", &self.scripts)
            .finish_non_exhaustive()
    }
}

impl Pickit {
    /// Opens the first PICkit that matches the selector.
    pub fn open(selector: &DebugProbeSelector) -> Result<Self, ProbeCreationError> {
        let device = super::list_devices()
            .into_iter()
            .find(|device| selector.matches(device))
            .ok_or(ProbeCreationError::NotFound)?;

        Self::open_device(&device)
    }

    /// Opens a PICkit that was already enumerated.
    pub fn open_device(device: &DeviceInfo) -> Result<Self, ProbeCreationError> {
        if !super::is_pickit(device) {
            return Err(ProbeCreationError::NotFound);
        }

        let interface = super::open_interface(device)?;
        let transport = Transport::new(&interface).map_err(ProbeCreationError::Usb)?;

        Ok(Self {
            transport,
            scripts: None,
            state: SessionState::Cold,
        })
    }

    /// Installs the script blobs for the target device.
    pub fn set_scripts(&mut self, scripts: Box<dyn ScriptSource>) {
        self.scripts = Some(scripts);
    }

    /// Where the session is in its lifecycle.
    pub fn state(&self) -> SessionState {
        self.state
    }

    /// Asks the tool for a named status value.
    ///
    /// This is a tool-level query rather than a script, so it is safe on a tool
    /// that has not talked to the target yet. It is the one thing worth doing
    /// before entering programming mode, as a check that the tool answers at
    /// all.
    pub fn status(&mut self, key: &str) -> Result<String, PickitError> {
        let result = self.transport.status_query(key);
        let response = self.finish(result)?;

        let text = response.payload();
        let text = text.split(|&byte| byte == 0).next().unwrap_or_default();

        Ok(String::from_utf8_lossy(text).into_owned())
    }

    /// Opens a programming session, which is the only safe first operation.
    ///
    /// This establishes the UPDI link and reports the lock state of the part in
    /// one step. A part that answers with the locked error moves the session to
    /// [`SessionState::Locked`], where memory access is refused.
    ///
    /// Probing for a target this way is safe. A missing or unpowered target
    /// answers with [`PickitError::NoTarget`] and leaves the tool responsive.
    pub fn enter_prog_mode(&mut self) -> Result<(), PickitError> {
        match self.state {
            SessionState::Cold => {}
            SessionState::Hung => return Err(PickitError::Hung),
            SessionState::Locked => return Err(PickitError::TargetLocked),
            SessionState::Programming | SessionState::Debugging => {
                return Err(PickitError::SessionAlreadyOpen);
            }
        }

        let response = self.command(ScriptName::EnterProgMode)?;

        match response.check() {
            Ok(()) => {
                self.state = SessionState::Programming;
                Ok(())
            }
            Err(err @ PickitError::TargetLocked) => {
                self.state = SessionState::Locked;
                Err(err)
            }
            Err(err) => Err(err),
        }
    }

    /// Switches an open programming session over to debugging.
    ///
    /// A programming session has to come first. Entering debug mode from cold
    /// is not one of the operations known to be safe on a tool that has not
    /// talked to the target yet.
    pub fn enter_debug_mode(&mut self) -> Result<(), PickitError> {
        if self.state != SessionState::Programming {
            return Err(PickitError::SessionNotOpen);
        }

        self.command(ScriptName::EnterDebugMode)?.check()?;
        self.state = SessionState::Debugging;

        Ok(())
    }

    /// Opens a debug session without resetting the part.
    ///
    /// [`Pickit::enter_prog_mode`] is the usual way in and it restarts the
    /// target, because its script asserts `ASI_RESET_REQ`. `EnterDebugMode`
    /// writes nothing to that register at all, so this reaches a part that is
    /// already running and leaves it running. That is the difference between
    /// debugging a fault and destroying the evidence for it.
    ///
    /// Two things are given up. The lock state is never learned, because only
    /// `EnterProgMode` reports it, so the session cannot move itself to
    /// [`SessionState::Locked`] and rule 4 has nothing to act on. And the part
    /// is left wherever it was rather than at the reset vector.
    ///
    /// Only use this on a part known to be unlocked.
    pub fn enter_debug_mode_hot(&mut self) -> Result<(), PickitError> {
        match self.state {
            SessionState::Cold => {}
            SessionState::Hung => return Err(PickitError::Hung),
            SessionState::Locked => return Err(PickitError::TargetLocked),
            SessionState::Programming | SessionState::Debugging => {
                return Err(PickitError::SessionAlreadyOpen);
            }
        }

        self.command(ScriptName::EnterDebugMode)?.check()?;
        self.state = SessionState::Debugging;

        Ok(())
    }

    /// Closes the session and returns the tool to [`SessionState::Cold`].
    ///
    /// This does nothing when no session is open, and nothing when the tool is
    /// hung, because a hung tool cannot answer.
    pub fn exit(&mut self) -> Result<(), PickitError> {
        let name = match self.state {
            SessionState::Cold | SessionState::Hung => return Ok(()),
            SessionState::Programming | SessionState::Locked => ScriptName::ExitProgMode,
            SessionState::Debugging => ScriptName::ExitDebugMode,
        };

        self.command(name)?.check()?;
        self.state = SessionState::Cold;

        Ok(())
    }

    /// Erases flash, EEPROM, and the lock bits.
    ///
    /// This is the only way past a locked part. The lock state changes under
    /// the session, so the session is closed and the caller has to open a new
    /// one.
    pub fn erase_chip(&mut self) -> Result<(), PickitError> {
        match self.state {
            SessionState::Cold => return Err(PickitError::SessionNotOpen),
            SessionState::Hung => return Err(PickitError::Hung),
            SessionState::Programming | SessionState::Debugging | SessionState::Locked => {}
        }

        self.command(ScriptName::EraseChip)?.check()?;
        self.state = SessionState::Cold;

        Ok(())
    }

    /// Sets the UPDI clock of the tool in kHz.
    ///
    /// Speed is not the bottleneck. The scripts do a fully addressed access per
    /// byte, so raising the clock changes nothing measurable.
    pub fn set_speed_khz(&mut self, khz: u32) -> Result<(), PickitError> {
        self.run(ScriptName::SetSpeed, Params::Words(&[khz]))?;

        Ok(())
    }

    /// Runs a script that moves no bulk data.
    ///
    /// Some scripts answer with a short result inside the response. Read it
    /// with [`Response::inline_data`].
    pub fn run(&mut self, name: ScriptName, params: Params<'_>) -> Result<Response, PickitError> {
        self.check_ready()?;

        tracing::debug!(script = ?name, "running script");

        let script = Self::lookup(&self.scripts, name)?;
        let result = self.transport.command(script.bytes(), params);
        let response = self.finish(result)?;

        if let Err(err) = response.check() {
            tracing::debug!(script = ?name, error = &err as &dyn std::error::Error, "script failed");
            return Err(err);
        }

        Ok(response)
    }

    /// Reads `len` bytes from the target.
    ///
    /// Memory scripts take an address and a length, in that order. The address
    /// is the one the tool expects, which puts flash at an offset of `0x800000`
    /// and everything else at its native data-space address.
    ///
    /// A zero-length request is answered by the inline result of the script
    /// instead of the data pipe.
    pub fn read(
        &mut self,
        name: ScriptName,
        params: Params<'_>,
        len: usize,
    ) -> Result<Vec<u8>, PickitError> {
        self.check_ready()?;

        let script = Self::lookup(&self.scripts, name)?;
        let result = self.transport.upload(script.bytes(), params, len);
        let (response, data) = self.finish(result)?;
        response.check()?;

        Self::read_result(response.inline_data(), data, len)
    }

    /// Decides between the inline result and the data phase of a read.
    ///
    /// The tool writes exactly what was asked for, or nothing at all when the
    /// answer fits in the response. A short data phase therefore means the
    /// transfer went wrong, and zero-filling it would hand the caller bytes
    /// that were never read.
    fn read_result(inline: &[u8], data: Vec<u8>, len: usize) -> Result<Vec<u8>, PickitError> {
        if len == 0 {
            return Ok(inline.to_vec());
        }

        if data.len() < len {
            return Err(PickitError::ShortDataPhase(data.len(), len));
        }

        Ok(data)
    }

    /// Writes bytes to the target.
    ///
    /// The address convention is the same as for [`Pickit::read`].
    pub fn write(
        &mut self,
        name: ScriptName,
        params: Params<'_>,
        data: &[u8],
    ) -> Result<(), PickitError> {
        self.check_ready()?;

        let script = Self::lookup(&self.scripts, name)?;
        let result = self.transport.download(script.bytes(), params, data);
        let response = self.finish(result)?;

        response.check()
    }

    fn command(&mut self, name: ScriptName) -> Result<Response, PickitError> {
        let script = Self::lookup(&self.scripts, name)?;
        let result = self.transport.command(script.bytes(), Params::Words(&[]));

        self.finish(result)
    }

    /// Takes a borrow of the script table alone, so the transport stays free.
    fn lookup(
        scripts: &Option<Box<dyn ScriptSource>>,
        name: ScriptName,
    ) -> Result<&Script, PickitError> {
        scripts
            .as_ref()
            .ok_or(PickitError::NoScripts)?
            .script(name)
            .ok_or(PickitError::ScriptMissing(name))
    }

    /// Carries a poisoned transport over into the session state.
    fn finish<T>(&mut self, result: Result<T, PickitError>) -> Result<T, PickitError> {
        if self.transport.is_poisoned() {
            self.state = SessionState::Hung;
        }

        result
    }

    /// Scripts need an open session on an unlocked part.
    ///
    /// A script issued before a UPDI session exists has no link to talk over,
    /// and a memory script issued to a locked part faults inside the tool.
    /// Both hang it, so both are refused here rather than on the wire.
    /// [`Pickit::erase_chip`] and [`Pickit::exit`] have their own rules,
    /// because they are the two things a locked part still allows.
    fn check_ready(&self) -> Result<(), PickitError> {
        match self.state {
            SessionState::Programming | SessionState::Debugging => Ok(()),
            SessionState::Cold => Err(PickitError::SessionNotOpen),
            SessionState::Locked => Err(PickitError::TargetLocked),
            SessionState::Hung => Err(PickitError::Hung),
        }
    }
}

#[cfg(test)]
mod test {
    use super::*;

    /// A read that came back short is an error, not a silent zero-fill. The
    /// tool writes exactly what was asked for or nothing at all.
    #[test]
    fn a_short_data_phase_is_an_error() {
        let err = Pickit::read_result(&[], vec![0; 3], 4).unwrap_err();

        assert!(matches!(err, PickitError::ShortDataPhase(3, 4)));
    }

    #[test]
    fn a_full_data_phase_is_returned_as_it_came() {
        let data = Pickit::read_result(&[], vec![0xaa; 4], 4).unwrap();

        assert_eq!(data, vec![0xaa; 4]);
    }

    /// A zero-length request is answered inline. This is how a script hands
    /// back a short result, such as the four bytes of `GetDeviceId`.
    #[test]
    fn a_zero_length_read_is_answered_inline() {
        let data = Pickit::read_result(&[0x1e, 0x97, 0x07, 0x18], Vec::new(), 0).unwrap();

        assert_eq!(data, &[0x1e, 0x97, 0x07, 0x18]);
    }
}

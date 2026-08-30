//! All the interface bits for AVR targets.
//!
//! An AVR is an 8-bit Harvard machine, which shows up in two places.
//!
//! Flash and the data space are separate memories that both start at zero on
//! the chip, so an address alone does not say which one it means. probe-rs
//! follows the avr-gcc convention and puts flash below `0x800000` and the data
//! space above it. See [`communication_interface`] for the translation.
//!
//! The program counter counts instruction words, while probe-rs and the debug
//! information count bytes. [`Avr`] doubles what it reads and halves what it
//! writes, so nothing above it has to know.
//!
//! The stack pointer and the `Y` frame pointer hold chip data addresses, which
//! are the same numbers probe-rs uses for flash. [`Avr`] adds the data space
//! offset when it reads them and takes it off again when it writes them, so a
//! caller that follows either one reaches the stack. The program counter is a
//! flash address and gets no offset.
//!
//! Run control goes through the tool's own scripts rather than through writes
//! to the debug block. Both routes reach the same hardware, and the scripts are
//! the ones that have been proven on a part.

use std::sync::Arc;
use std::time::{Duration, Instant};

use probe_rs_target::{Architecture, CoreType, InstructionSet};

use crate::core::registers::{CoreRegisters, RegisterId, RegisterValue};
use crate::error::Error;
use crate::{
    BreakpointCause, CoreInformation, CoreInterface, CoreRegister, CoreStatus, HaltReason,
    MemoryInterface,
};

use self::communication_interface::{
    AddressSpace, AvrCommunicationInterface, DATA_SPACE_OFFSET, HW_BREAKPOINT_UNITS,
    to_chip_data_address,
};
use self::ocd::OcdVersion;
use self::registers::{AVR_CORE_REGISTERS, FP, PC, SP, SREG};
use self::sequences::AvrDebugSequence;

pub mod communication_interface;
pub mod ocd;
pub mod registers;
pub mod sequences;

/// How long to wait between polls while waiting for the core to stop.
const HALT_POLL_INTERVAL: Duration = Duration::from_millis(1);

/// How many software breakpoints the driver will plant.
///
/// There is no limit in the part, because a software breakpoint is just a
/// `BREAK` written over an instruction. The limit is the flash: every plant and
/// every removal costs an erase and program cycle of the page it lands in, and
/// some of these parts are rated for as few as 1000. A small number keeps that
/// bounded while still lifting the ceiling well above the two the hardware has.
pub const SW_BREAKPOINT_SLOTS: usize = 4;

/// The flash page containing `address`, and the offset of `address` into it.
///
/// Writing one instruction word means reading its whole page, patching it and
/// writing it back, because `WriteProgmem` erases and programs a page at a
/// time. Getting this wrong does not fail loudly, it writes a page to the wrong
/// place, so it is worth pinning.
///
/// # Examples
///
/// ```
/// use probe_rs::architecture::avr::page_span;
///
/// // A 512 byte page on an AVR-Dx part.
/// assert_eq!(page_span(0x356, 512), (0x200, 0x156));
/// // The first word of a page.
/// assert_eq!(page_span(0x200, 512), (0x200, 0));
/// // A 64 byte page on a tiny.
/// assert_eq!(page_span(0x7a, 64), (0x40, 0x3a));
/// ```
pub fn page_span(address: u64, page_size: u32) -> (u64, usize) {
    let page_size = u64::from(page_size);
    let offset = address % page_size;

    (address - offset, offset as usize)
}

/// A `BREAK` written over an instruction, and the instruction it replaced.
#[derive(Clone, Copy, Debug)]
struct SoftwareBreakpoint {
    /// The byte address of the instruction.
    address: u64,
    /// The first word of the instruction that was there, needed both to put it
    /// back and to step over the breakpoint without touching flash.
    original: u16,
}

/// The state of an AVR core that outlives a single core handle.
#[derive(Debug)]
pub struct AvrCoreState {
    /// The address armed in each breakpoint unit, as a byte address.
    ///
    /// The debug block does not report which units are in use in a form that
    /// survives the address encoding, so this is tracked here instead.
    hw_breakpoints: [Option<u64>; HW_BREAKPOINT_UNITS],

    /// Whether the global hardware breakpoint enable is set.
    breakpoints_enabled: bool,

    /// Set while the last resume was a single step.
    ///
    /// The halt cause shares one bit between breakpoint unit 0 and a finished
    /// step, on both debug revisions, so the two can only be told apart by
    /// remembering which one was asked for.
    expecting_step: bool,

    /// The debug revision, once it has been read off the part.
    ocd_version: Option<OcdVersion>,

    /// The software breakpoints planted in flash.
    sw_breakpoints: [Option<SoftwareBreakpoint>; SW_BREAKPOINT_SLOTS],
}

impl AvrCoreState {
    /// Creates the state for a core that has not been attached to yet.
    pub(crate) fn new() -> Self {
        Self {
            hw_breakpoints: [None; HW_BREAKPOINT_UNITS],
            breakpoints_enabled: false,
            expecting_step: false,
            ocd_version: None,
            sw_breakpoints: [None; SW_BREAKPOINT_SLOTS],
        }
    }

    /// The software breakpoint planted at `address`, if this session planted one.
    fn software_breakpoint_at(&self, address: u64) -> Option<SoftwareBreakpoint> {
        self.sw_breakpoints
            .iter()
            .flatten()
            .find(|breakpoint| breakpoint.address == address)
            .copied()
    }

    /// Takes every planted software breakpoint out of the state.
    ///
    /// The caller restores the instructions itself, from outside the core,
    /// while the image the breakpoints were planted in is still in flash.
    /// Taking the records out leaves no breakpoint behind, so a later removal
    /// cannot write any of them back over a new image.
    pub(crate) fn take_software_breakpoints(&mut self) -> Vec<(u64, u16)> {
        self.sw_breakpoints
            .iter_mut()
            .filter_map(|slot| {
                slot.take()
                    .map(|breakpoint| (breakpoint.address, breakpoint.original))
            })
            .collect()
    }
}

/// An interface to operate an AVR core.
pub struct Avr<'probe> {
    interface: AvrCommunicationInterface<'probe>,
    state: &'probe mut AvrCoreState,
}

impl<'probe> Avr<'probe> {
    /// Attaches to an AVR core over an open UPDI session.
    ///
    /// The session has to be in programming mode already, which is what
    /// `Session` leaves it in, or already debugging, which is what a
    /// reset-free attach leaves it in. Switching over sends the on-chip debug
    /// key and debug-resets the part, which releases the reset a programming
    /// session asserted and leaves the core halted on the reset vector. A
    /// session that was opened without a reset is left exactly as it was: the
    /// core keeps running until someone halts it.
    ///
    /// The debug sequence is taken but not used. [`AvrDebugSequence`] has no
    /// hooks yet, and this signature is here so adding one does not change every
    /// call site.
    pub fn new(
        interface: AvrCommunicationInterface<'probe>,
        state: &'probe mut AvrCoreState,
        _sequence: Arc<dyn AvrDebugSequence>,
    ) -> Result<Self, Error> {
        let mut this = Self { interface, state };
        this.interface.enter_debug_mode()?;

        Ok(this)
    }

    fn core_info(&mut self) -> Result<CoreInformation, Error> {
        let pc = self.read_core_reg(PC.id)?;

        Ok(CoreInformation { pc: pc.try_into()? })
    }

    /// Reads one byte out of the memory-mapped debug block.
    fn read_ocd_8(&mut self, offset: u64) -> Result<u8, Error> {
        self.interface.read_word_8(ocd::address(offset))
    }

    fn write_ocd_8(&mut self, offset: u64, value: u8) -> Result<(), Error> {
        self.interface.write_word_8(ocd::address(offset), value)
    }

    /// Reads one 16-bit debug block field, a byte at a time.
    ///
    /// The fields that span two bytes are byte pairs rather than true words, and
    /// byte access to them is the access that has been read back from a part.
    fn read_ocd_16(&mut self, offset: u64) -> Result<u16, Error> {
        let mut bytes = [0; 2];
        self.interface.read_8(ocd::address(offset), &mut bytes)?;

        Ok(u16::from_le_bytes(bytes))
    }

    fn write_ocd_16(&mut self, offset: u64, value: u16) -> Result<(), Error> {
        self.interface
            .write_8(ocd::address(offset), &value.to_le_bytes())
    }

    /// Gets past a `BREAK` at the program counter without touching flash.
    ///
    /// The instruction the `BREAK` replaced is injected instead, so the core
    /// executes it from [`ocd::INSN0`] and moves on. Without this, resuming
    /// would mean writing flash twice on every hit, and the erase budget on
    /// these parts does not allow that.
    ///
    /// A two-word instruction needs no special handling. Only its first word
    /// was overwritten, so injecting that word alone leaves the part to fetch
    /// the second from flash, where it still is.
    fn step_over_software_breakpoint(&mut self) -> Result<bool, Error> {
        let pc = self.read_core_reg(PC.id)?.try_into()?;
        let Some(breakpoint) = self.state.software_breakpoint_at(pc) else {
            return Ok(false);
        };

        self.interface.inject_instruction(breakpoint.original)?;
        self.interface.step()?;

        Ok(true)
    }

    /// Writes a `BREAK` over the instruction at `address`.
    ///
    /// Trapping on `BREAK` is enabled from the moment debug mode is entered, so
    /// nothing has to be armed. What this costs is an erase and program cycle
    /// of the page the address lands in, both now and again when the
    /// breakpoint is removed.
    fn plant_software_breakpoint(&mut self, slot: usize, address: u64) -> Result<(), Error> {
        Self::check_breakpoint_address(address)?;

        // Setting a breakpoint that this session already planted is a no-op.
        // `Core::set_hw_breakpoint` promises that setting one again keeps it
        // active, and the BREAK in flash is ours, with the instruction it
        // replaced still remembered. Writing the page again would only cost
        // another erase cycle.
        if self.state.software_breakpoint_at(address).is_some() {
            return Ok(());
        }

        let original = self.interface.read_flash_word(address)?;

        if original == ocd::BREAK_INSTRUCTION {
            return Err(Error::Other(format!(
                "There is already a BREAK at {address:#x}. Flash was left in a \
                 state a previous session did not clean up, so the instruction \
                 that belongs there is lost."
            )));
        }

        tracing::warn!(
            "Planting a software breakpoint at {address:#x}. Both hardware units are \
             in use, so this writes flash, and removing it writes flash again. Some \
             AVR parts are rated for as few as 1000 erase cycles."
        );

        self.interface
            .write_flash_word(address, ocd::BREAK_INSTRUCTION)?;
        self.state.sw_breakpoints[slot] = Some(SoftwareBreakpoint { address, original });

        Ok(())
    }

    /// Puts back the instruction a software breakpoint replaced.
    ///
    /// Flash is read back first. The breakpoint may have been overwritten by
    /// something outside this machinery, and then the instruction it remembers
    /// describes an image that is no longer there. Writing it back would leave
    /// two wrong bytes in whatever is in flash now, so the record is dropped
    /// with a warning instead. See [`Avr::restored_word`].
    fn remove_software_breakpoint(&mut self, slot: usize) -> Result<(), Error> {
        let Some(breakpoint) = self.state.sw_breakpoints[slot] else {
            return Ok(());
        };

        let address = breakpoint.address;
        let current = self.interface.read_flash_word(address)?;

        match Avr::restored_word(current, breakpoint.original) {
            Some(word) => self.interface.write_flash_word(address, word)?,
            None => tracing::warn!(
                "The word at {address:#x} is not the BREAK this session planted there, \
                 so flash was rewritten under the breakpoint. Dropping it without \
                 writing back the instruction it remembered, which belongs to the \
                 old image."
            ),
        }

        self.state.sw_breakpoints[slot] = None;

        Ok(())
    }

    /// Decides what removing a software breakpoint does to flash.
    ///
    /// `current` is the word read back from the breakpoint address. Only the
    /// planted `BREAK` says the breakpoint is still in flash and that
    /// `original` still describes the image, so that is the only case where
    /// the instruction goes back. Anything else, including a word that happens
    /// to equal `original`, means flash changed underneath the breakpoint and
    /// restoring would put old-image bytes into a new one. `None` says to drop
    /// the record without writing.
    pub(crate) fn restored_word(current: u16, original: u16) -> Option<u16> {
        (current == ocd::BREAK_INSTRUCTION).then_some(original)
    }

    /// Removes every software breakpoint this session planted.
    ///
    /// A flash rewrite from outside the breakpoint machinery would erase the
    /// planted BREAKs along with the rest of their pages, and the records
    /// would end up describing an image that is gone. Taking them out first
    /// puts the instructions back while they are still the right ones.
    fn remove_all_software_breakpoints(&mut self) -> Result<(), Error> {
        for slot in 0..SW_BREAKPOINT_SLOTS {
            self.remove_software_breakpoint(slot)?;
        }

        Ok(())
    }

    /// Takes the software breakpoints out when a write is headed for flash.
    ///
    /// Only writes routed to flash need this. The data space is a separate
    /// memory, and the breakpoint machinery writes its own words straight
    /// through the interface, so those never run through here and never take
    /// themselves out.
    fn clear_breakpoints_before_flash_write(&mut self, address: u64) -> Result<(), Error> {
        if AddressSpace::of(address) == AddressSpace::Flash {
            self.remove_all_software_breakpoints()?;
        }

        Ok(())
    }

    /// Sets or clears bits in `TRAPEN`, leaving the rest of it alone.
    ///
    /// `TRAPEN` carries the software breakpoint enable, which the part sets by
    /// itself on entering debug mode, so it must never be written whole.
    fn update_trapen(&mut self, set: u16, clear: u16) -> Result<(), Error> {
        let current = self.read_ocd_16(ocd::TRAPEN)?;
        let updated = (current & !clear) | set;

        if updated != current {
            self.write_ocd_16(ocd::TRAPEN, updated)?;
        }

        Ok(())
    }

    /// Reads a byte of the register file, which is mapped into the debug block.
    fn read_register_file(&mut self, index: u64) -> Result<u8, Error> {
        self.read_ocd_8(ocd::REGISTER_FILE + index)
    }

    fn write_register_file(&mut self, index: u64, value: u8) -> Result<(), Error> {
        self.write_ocd_8(ocd::REGISTER_FILE + index, value)
    }

    /// Checks the debug revision of the part against the one its family implies.
    ///
    /// The raw program counter register and the `GetPC` script disagree in a way
    /// that depends only on the revision, so comparing the two identifies it.
    /// That is more reliable than the System Information Block, which this tool
    /// returns truncated.
    ///
    /// Both formulas agree while the core sits at the reset vector, and then
    /// this leaves the revision unknown and tries again at the next halt.
    fn check_ocd_version(&mut self) -> Result<(), Error> {
        if self.state.ocd_version.is_some() {
            return Ok(());
        }

        let expected = OcdVersion::for_family(self.interface.family());
        let raw = u32::from(self.read_ocd_16(ocd::PC)?);
        let word_pc = self.interface.program_counter()?;

        match OcdVersion::detect(raw, word_pc) {
            Some(found) => {
                self.state.ocd_version = Some(found);

                if found != expected {
                    tracing::warn!(
                        "The part reports on-chip debug {found:?} but its family implies \
                         {expected:?}. The target description may name the wrong part."
                    );
                }
            }
            None => tracing::debug!("Cannot tell the on-chip debug revision apart at this address"),
        }

        Ok(())
    }

    /// Turns the `CAUSE` field into the reason probe-rs reports.
    ///
    /// Breakpoint unit 0 and a finished step share one bit, so the caller has to
    /// say which of the two it was expecting.
    fn halt_reason(cause: u16, expecting_step: bool) -> HaltReason {
        use self::ocd::cause;

        if cause & cause::SWBP != 0 {
            HaltReason::Breakpoint(BreakpointCause::Software)
        } else if cause & cause::BP0_OR_STEP != 0 {
            if expecting_step {
                HaltReason::Step
            } else {
                HaltReason::Breakpoint(BreakpointCause::Hardware)
            }
        } else if cause & cause::BP1 != 0 {
            HaltReason::Breakpoint(BreakpointCause::Hardware)
        } else if cause & cause::RESET != 0 {
            // A reset only stops the core because a debug reset was asked for.
            HaltReason::Request
        } else if cause & cause::EXT != 0 {
            HaltReason::Request
        } else if cause & cause::EXTBRK != 0 {
            HaltReason::External
        } else if cause & cause::INT != 0 {
            HaltReason::Exception
        } else {
            HaltReason::Unknown
        }
    }

    /// Turns a word program counter into the byte address probe-rs uses.
    ///
    /// The core counts instruction words. Flash addresses in an ELF file, in the
    /// debug information, and in every probe-rs API count bytes, and AVR
    /// instructions are two or four bytes long, so the two differ by a factor of
    /// two.
    fn word_to_byte_address(word_address: u32) -> u64 {
        u64::from(word_address) * 2
    }

    /// Turns a byte address in flash into the word address the scripts take.
    ///
    /// Instructions are always at even addresses, so an odd address cannot be
    /// the start of one and is rounded down.
    fn byte_to_word_address(byte_address: u64) -> u32 {
        (byte_address / 2) as u32
    }

    /// Turns a raw chip data address into the address probe-rs uses for it.
    ///
    /// A pointer register that keeps the raw value names the same number in
    /// flash, because that is where probe-rs puts flash. Reading through it then
    /// returns program bytes with no error at all, so this translation is what
    /// keeps stack locals readable.
    fn to_probe_rs_data_address(chip_address: u16) -> u32 {
        (DATA_SPACE_OFFSET + u64::from(chip_address)) as u32
    }

    /// Turns a probe-rs data space address back into the raw chip address.
    ///
    /// The inverse of [`Avr::to_probe_rs_data_address`]. An address that is not
    /// in the data space, or that no 16-bit pointer register can hold, is an
    /// error rather than a silent truncation.
    fn to_chip_pointer(address: u32) -> Result<u16, Error> {
        let chip = to_chip_data_address(u64::from(address))?;

        u16::try_from(chip).map_err(|_| {
            Error::Register(format!(
                "{address:#010x} is outside the AVR data space and cannot be a pointer register"
            ))
        })
    }

    /// Checks that `unit_index` names a hardware unit or a software slot.
    ///
    /// [`Core::set_hw_breakpoint`] and [`Core::clear_hw_breakpoint`] take their
    /// index from [`Avr::available_breakpoint_units`], so an out-of-range index
    /// means a caller ignored that count. Letting one through would index a
    /// software slot that does not exist.
    fn check_breakpoint_unit(unit_index: usize) -> Result<(), Error> {
        let unit_count = HW_BREAKPOINT_UNITS + SW_BREAKPOINT_SLOTS;

        if unit_index < unit_count {
            return Ok(());
        }

        Err(Error::Other(format!(
            "There is no breakpoint unit {unit_index}. This core has {unit_count}, \
             and their indexes start at zero."
        )))
    }

    /// The hardware units a debug reset disarmed, with their addresses.
    ///
    /// A debug reset clears both address registers, the per-unit enables, and
    /// the global enable in `TRAPEN`, so every armed unit has to be written
    /// back afterwards. Software breakpoints survive a reset: `SWBP` stays set
    /// and the planted `BREAK` stays in flash, so their slots are not listed.
    fn units_to_rearm(state: &AvrCoreState) -> Vec<(usize, u64)> {
        state
            .hw_breakpoints
            .iter()
            .enumerate()
            .filter_map(|(unit, address)| address.map(|address| (unit, address)))
            .collect()
    }

    /// Writes back the hardware breakpoints a debug reset cleared.
    fn rearm_hw_breakpoints(&mut self) -> Result<(), Error> {
        let units = Self::units_to_rearm(&*self.state);
        let had_units = !units.is_empty();

        for (unit, address) in units {
            let word_address = Self::byte_to_word_address(address);
            self.interface.set_hw_breakpoint(unit, word_address)?;
        }

        // The reset cleared the global enable along with the units.
        if self.state.breakpoints_enabled && had_units {
            self.update_trapen(ocd::trapen::HWBP, 0)?;
        }

        Ok(())
    }

    /// Checks that `address` can be the start of an instruction.
    ///
    /// AVR instructions are word aligned, so an odd address falls inside one
    /// and the word there is half of it. Replacing that word would corrupt the
    /// instruction and lose whatever the breakpoint was meant to keep.
    fn check_breakpoint_address(address: u64) -> Result<(), Error> {
        if address.is_multiple_of(2) {
            return Ok(());
        }

        Err(Error::Other(format!(
            "Cannot plant a software breakpoint at {address:#x}. Breakpoint addresses \
             must be even, because AVR instructions are word aligned."
        )))
    }
}

/// The memory access the core hands out, with the flash guard on writes.
///
/// A data write that lands on flash goes through the same scripts the
/// breakpoint machinery uses, but it erases whole pages and takes every
/// planted software breakpoint in them with it. The guard removes the
/// breakpoints first, while the instructions they replaced are still the
/// right ones to put back. Reads have no such side effect and pass straight
/// through.
impl MemoryInterface<Error> for Avr<'_> {
    fn supports_native_64bit_access(&mut self) -> bool {
        self.interface.supports_native_64bit_access()
    }

    fn supports_8bit_transfers(&self) -> Result<bool, Error> {
        self.interface.supports_8bit_transfers()
    }

    fn read_8(&mut self, address: u64, data: &mut [u8]) -> Result<(), Error> {
        self.interface.read_8(address, data)
    }

    fn read_16(&mut self, address: u64, data: &mut [u16]) -> Result<(), Error> {
        self.interface.read_16(address, data)
    }

    fn read_32(&mut self, address: u64, data: &mut [u32]) -> Result<(), Error> {
        self.interface.read_32(address, data)
    }

    fn read_64(&mut self, address: u64, data: &mut [u64]) -> Result<(), Error> {
        self.interface.read_64(address, data)
    }

    /// Reads without widening the access. The data space starts with the IO
    /// registers, where reading a byte nobody asked for can have a side
    /// effect.
    fn read(&mut self, address: u64, data: &mut [u8]) -> Result<(), Error> {
        self.interface.read(address, data)
    }

    fn write_8(&mut self, address: u64, data: &[u8]) -> Result<(), Error> {
        self.clear_breakpoints_before_flash_write(address)?;
        self.interface.write_8(address, data)
    }

    fn write_16(&mut self, address: u64, data: &[u16]) -> Result<(), Error> {
        self.clear_breakpoints_before_flash_write(address)?;
        self.interface.write_16(address, data)
    }

    fn write_32(&mut self, address: u64, data: &[u32]) -> Result<(), Error> {
        self.clear_breakpoints_before_flash_write(address)?;
        self.interface.write_32(address, data)
    }

    fn write_64(&mut self, address: u64, data: &[u64]) -> Result<(), Error> {
        self.clear_breakpoints_before_flash_write(address)?;
        self.interface.write_64(address, data)
    }

    /// Writes without widening the access.
    fn write(&mut self, address: u64, data: &[u8]) -> Result<(), Error> {
        self.clear_breakpoints_before_flash_write(address)?;
        self.interface.write(address, data)
    }

    /// Nothing is buffered, so there is nothing to flush.
    fn flush(&mut self) -> Result<(), Error> {
        self.interface.flush()
    }
}

impl CoreInterface for Avr<'_> {
    fn wait_for_core_halted(&mut self, timeout: Duration) -> Result<(), Error> {
        let deadline = Instant::now() + timeout;

        loop {
            if self.interface.is_halted()? {
                return Ok(());
            }

            if Instant::now() >= deadline {
                return Err(Error::Timeout);
            }

            std::thread::sleep(HALT_POLL_INTERVAL);
        }
    }

    fn core_halted(&mut self) -> Result<bool, Error> {
        Ok(self.interface.is_halted()?)
    }

    fn status(&mut self) -> Result<CoreStatus, Error> {
        if !self.interface.is_halted()? {
            return Ok(CoreStatus::Running);
        }

        self.check_ocd_version()?;

        let cause = self.read_ocd_16(ocd::CAUSE)?;

        Ok(CoreStatus::Halted(Self::halt_reason(
            cause,
            self.state.expecting_step,
        )))
    }

    fn halt(&mut self, timeout: Duration) -> Result<CoreInformation, Error> {
        self.state.expecting_step = false;
        self.interface.halt()?;
        self.wait_for_core_halted(timeout)?;

        self.core_info()
    }

    fn run(&mut self) -> Result<(), Error> {
        // Stopped on a planted BREAK, the core would hit it again immediately.
        self.step_over_software_breakpoint()?;
        self.state.expecting_step = false;

        Ok(self.interface.run()?)
    }

    fn reset(&mut self) -> Result<(), Error> {
        self.reset_and_halt(Duration::from_millis(500))?;

        self.run()
    }

    fn reset_and_halt(&mut self, timeout: Duration) -> Result<CoreInformation, Error> {
        self.state.expecting_step = false;

        // The debug reset script leaves the core stopped at the reset vector,
        // so reset and halt need no separate steps.
        self.interface.debug_reset()?;
        self.wait_for_core_halted(timeout)?;
        self.rearm_hw_breakpoints()?;

        self.core_info()
    }

    fn step(&mut self) -> Result<CoreInformation, Error> {
        // Stepping off a planted BREAK is the injected instruction itself, so
        // there is nothing left to step once that has run.
        if self.step_over_software_breakpoint()? {
            self.state.expecting_step = true;

            return self.core_info();
        }

        // The script sets the step trap, resumes, and waits for the halt, so the
        // core is stopped again by the time this returns.
        self.interface.step()?;
        self.state.expecting_step = true;

        self.core_info()
    }

    fn read_core_reg(&mut self, address: RegisterId) -> Result<RegisterValue, Error> {
        let value = match address {
            id if id == PC.id => {
                let word_address = self.interface.program_counter()?;
                Self::word_to_byte_address(word_address) as u32
            }
            id if id == SP.id => Self::to_probe_rs_data_address(self.read_ocd_16(ocd::SP)?),
            id if id == SREG.id => u32::from(self.read_ocd_8(ocd::SREG)?),
            id if id == FP.id => {
                // The Y pair is r28 and r29 of the register file.
                let low = self.read_register_file(28)?;
                let high = self.read_register_file(29)?;
                Self::to_probe_rs_data_address(u16::from_le_bytes([low, high]))
            }
            RegisterId(index) if usize::from(index) < ocd::REGISTER_FILE_LEN => {
                u32::from(self.read_register_file(u64::from(index))?)
            }
            other => {
                return Err(Error::Register(format!(
                    "{other:?} is not a register of an AVR core"
                )));
            }
        };

        Ok(RegisterValue::U32(value))
    }

    fn write_core_reg(&mut self, address: RegisterId, value: RegisterValue) -> Result<(), Error> {
        let value: u32 = value.try_into()?;

        match address {
            id if id == PC.id => {
                let word_address = Self::byte_to_word_address(u64::from(value));
                self.interface.set_program_counter(word_address)?;
            }
            id if id == SP.id => self.write_ocd_16(ocd::SP, Self::to_chip_pointer(value)?)?,
            id if id == SREG.id => self.write_ocd_8(ocd::SREG, value as u8)?,
            id if id == FP.id => {
                let [low, high] = Self::to_chip_pointer(value)?.to_le_bytes();
                self.write_register_file(28, low)?;
                self.write_register_file(29, high)?;
            }
            RegisterId(index) if usize::from(index) < ocd::REGISTER_FILE_LEN => {
                self.write_register_file(u64::from(index), value as u8)?
            }
            other => {
                return Err(Error::Register(format!(
                    "{other:?} is not a register of an AVR core"
                )));
            }
        }

        Ok(())
    }

    /// The two hardware units, then the software slots.
    ///
    /// The order matters. `Core::set_hw_breakpoint` takes the first free slot,
    /// so the free hardware units go first and flash is only written once they
    /// are gone.
    fn available_breakpoint_units(&mut self) -> Result<u32, Error> {
        Ok((HW_BREAKPOINT_UNITS + SW_BREAKPOINT_SLOTS) as u32)
    }

    fn hw_breakpoints(&mut self) -> Result<Vec<Option<u64>>, Error> {
        let mut breakpoints = self.state.hw_breakpoints.to_vec();
        breakpoints.extend(
            self.state
                .sw_breakpoints
                .iter()
                .map(|slot| slot.map(|breakpoint| breakpoint.address)),
        );

        Ok(breakpoints)
    }

    fn enable_breakpoints(&mut self, state: bool) -> Result<(), Error> {
        self.state.breakpoints_enabled = state;

        if state {
            self.update_trapen(ocd::trapen::HWBP, 0)
        } else {
            self.update_trapen(0, ocd::trapen::HWBP)
        }
    }

    fn set_hw_breakpoint(&mut self, unit_index: usize, addr: u64) -> Result<(), Error> {
        Self::check_breakpoint_unit(unit_index)?;

        if let Some(slot) = unit_index.checked_sub(HW_BREAKPOINT_UNITS) {
            return self.plant_software_breakpoint(slot, addr);
        }

        let word_address = Self::byte_to_word_address(addr);
        self.interface.set_hw_breakpoint(unit_index, word_address)?;
        self.state.hw_breakpoints[unit_index] = Some(addr);

        // A breakpoint needs the global enable as well as the per-unit one, and
        // the script only sets the per-unit one. Without this the unit is armed
        // and never fires.
        if self.state.breakpoints_enabled {
            self.update_trapen(ocd::trapen::HWBP, 0)?;
        }

        Ok(())
    }

    fn clear_hw_breakpoint(&mut self, unit_index: usize) -> Result<(), Error> {
        Self::check_breakpoint_unit(unit_index)?;

        if let Some(slot) = unit_index.checked_sub(HW_BREAKPOINT_UNITS) {
            return self.remove_software_breakpoint(slot);
        }

        self.interface.clear_hw_breakpoint(unit_index)?;
        self.state.hw_breakpoints[unit_index] = None;

        Ok(())
    }

    fn registers(&self) -> &'static CoreRegisters {
        &AVR_CORE_REGISTERS
    }

    fn program_counter(&self) -> &'static CoreRegister {
        &PC
    }

    fn frame_pointer(&self) -> &'static CoreRegister {
        &FP
    }

    fn stack_pointer(&self) -> &'static CoreRegister {
        &SP
    }

    /// The program counter, because an AVR has no return address register.
    ///
    /// `call` pushes the return address onto the stack, so there is nothing to
    /// return here. No register carries the return address role, which is what
    /// the unwinder looks at, so this only stands in for callers that ask for a
    /// register and get one they cannot use.
    fn return_address(&self) -> &'static CoreRegister {
        &PC
    }

    fn hw_breakpoints_enabled(&self) -> bool {
        self.state.breakpoints_enabled
    }

    fn architecture(&self) -> Architecture {
        Architecture::Avr
    }

    fn core_type(&self) -> CoreType {
        CoreType::Avr
    }

    fn instruction_set(&mut self) -> Result<InstructionSet, Error> {
        Ok(InstructionSet::Avr)
    }

    fn fpu_support(&mut self) -> Result<bool, Error> {
        Ok(false)
    }

    fn floating_point_register_count(&mut self) -> Result<usize, Error> {
        Ok(0)
    }

    /// Not supported, and not needed. `reset_and_halt` stops the core at the
    /// reset vector on its own.
    fn reset_catch_set(&mut self) -> Result<(), Error> {
        Err(Error::NotImplemented("reset catch on AVR"))
    }

    /// Not supported. See [`CoreInterface::reset_catch_set`].
    fn reset_catch_clear(&mut self) -> Result<(), Error> {
        Err(Error::NotImplemented("reset catch on AVR"))
    }

    fn debug_core_stop(&mut self) -> Result<(), Error> {
        Ok(self.interface.close()?)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A patched page must start on a page boundary and the word being patched
    /// must land inside it, or a software breakpoint writes over the wrong
    /// instructions. Both page sizes in use are covered.
    #[test]
    fn a_patched_word_lands_inside_its_own_page() {
        for page_size in [64u32, 512] {
            for address in (0..2048).step_by(2) {
                let (page, offset) = page_span(address, page_size);

                assert_eq!(page % u64::from(page_size), 0, "{address:#x}/{page_size}");
                assert_eq!(page + offset as u64, address);
                // Two bytes are written, so the word may not straddle the end.
                assert!(offset + 2 <= page_size as usize, "{address:#x}/{page_size}");
            }
        }
    }

    /// The addresses of the software breakpoint run on an AVR128DA64.
    #[test]
    fn the_measured_software_breakpoint_page() {
        // The BREAK went to 0x356, which is 0x156 into the page at 0x200.
        assert_eq!(page_span(0x356, 512), (0x200, 0x156));
    }

    /// The core counts instruction words and everything above it counts bytes.
    /// These are the two directions of that, and both were seen on hardware.
    #[test]
    fn program_counter_addresses_convert_between_words_and_bytes() {
        // level2 of the test firmware, at word 0x00c4 and byte 0x188.
        assert_eq!(Avr::word_to_byte_address(0x00c4), 0x188);
        assert_eq!(Avr::byte_to_word_address(0x188), 0x00c4);

        // The reset vector.
        assert_eq!(Avr::word_to_byte_address(0), 0);
        assert_eq!(Avr::byte_to_word_address(0), 0);

        // The last instruction of a 128 KiB part.
        assert_eq!(Avr::word_to_byte_address(0xFFFF), 0x1_FFFE);
        assert_eq!(Avr::byte_to_word_address(0x1_FFFE), 0xFFFF);
    }

    /// Instructions are always at even addresses, so an odd byte address cannot
    /// name one and rounds down to the instruction it falls inside.
    #[test]
    fn odd_byte_addresses_round_down_to_an_instruction() {
        assert_eq!(Avr::byte_to_word_address(0x189), 0x00c4);
    }

    /// A pointer register goes out as a probe-rs address and comes back as the
    /// raw chip value, so writing back what was read leaves the chip alone.
    #[test]
    fn a_pointer_register_survives_a_read_and_a_write_back() {
        for raw in [0u16, 0x4000, 0x3FFF, 0x7F00, 0xFFFF] {
            let reported = Avr::to_probe_rs_data_address(raw);

            assert_eq!(Avr::to_chip_pointer(reported).unwrap(), raw);
        }
    }

    /// The bug this guards against is silent. A stack pointer left in the raw
    /// chip form names flash, so locals get read out of the program image and
    /// nothing reports an error.
    #[test]
    fn a_stack_pointer_lands_in_the_data_space_and_a_program_counter_in_flash() {
        use self::communication_interface::AddressSpace;

        // A full stack on an AVR128DA64, which has 16 KiB of SRAM at 0x804000.
        assert_eq!(Avr::to_probe_rs_data_address(0x3FFF), 0x80_3FFF);
        assert_eq!(
            AddressSpace::of(u64::from(Avr::to_probe_rs_data_address(0x3FFF))),
            AddressSpace::Data
        );
        assert_eq!(
            AddressSpace::of(u64::from(Avr::to_probe_rs_data_address(0x0000))),
            AddressSpace::Data
        );

        // The program counter is a flash address and takes no offset.
        assert_eq!(
            AddressSpace::of(Avr::word_to_byte_address(0xFFFF)),
            AddressSpace::Flash
        );
    }

    /// A pointer register is 16 bits on the chip, so a probe-rs address that no
    /// AVR data space reaches has to be refused rather than truncated.
    #[test]
    fn a_pointer_outside_the_data_space_is_refused() {
        // A flash address, which is what an untranslated value looks like.
        assert!(Avr::to_chip_pointer(0x3FFF).is_err());
        // Past the end of the 16-bit data space.
        assert!(Avr::to_chip_pointer(0x81_0000).is_err());
    }

    /// Every value here was read out of `CAUSE` after triggering the cause on a
    /// part.
    #[test]
    fn the_measured_halt_causes_map_to_a_reason() {
        assert_eq!(Avr::halt_reason(0x0044, false), HaltReason::Request);
        assert_eq!(
            Avr::halt_reason(0x2004, false),
            HaltReason::Breakpoint(BreakpointCause::Software)
        );
        assert_eq!(Avr::halt_reason(0x0084, false), HaltReason::Request);
    }

    /// Breakpoint unit 0 and a finished step set the same bit, so only the
    /// caller's intent tells them apart.
    #[test]
    fn a_step_and_breakpoint_zero_are_told_apart_by_intent() {
        assert_eq!(Avr::halt_reason(0x0104, true), HaltReason::Step);
        assert_eq!(
            Avr::halt_reason(0x0104, false),
            HaltReason::Breakpoint(BreakpointCause::Hardware)
        );
    }

    /// A fresh core has both units free and hardware breakpoints off.
    #[test]
    fn a_fresh_state_has_no_breakpoints() {
        let state = AvrCoreState::new();

        assert_eq!(state.hw_breakpoints, [None, None]);
        assert!(!state.breakpoints_enabled);
        assert!(!state.expecting_step);
    }

    /// A unit index has to name one of the two hardware units or one of the
    /// four software slots. `set_hw_breakpoint` and `clear_hw_breakpoint` both
    /// take this check, and before it was there an index of six or more reached
    /// the software slots and panicked indexing them, with the BREAK already
    /// written into flash.
    #[test]
    fn a_unit_index_past_the_software_slots_is_rejected() {
        for unit_index in 0..HW_BREAKPOINT_UNITS + SW_BREAKPOINT_SLOTS {
            assert!(
                Avr::check_breakpoint_unit(unit_index).is_ok(),
                "unit {unit_index}"
            );
        }

        for unit_index in [
            HW_BREAKPOINT_UNITS + SW_BREAKPOINT_SLOTS,
            HW_BREAKPOINT_UNITS + SW_BREAKPOINT_SLOTS + 1,
            100,
        ] {
            assert!(
                Avr::check_breakpoint_unit(unit_index).is_err(),
                "unit {unit_index}"
            );
        }
    }

    /// An odd address falls inside an instruction instead of starting one, so
    /// planting a breakpoint there has to be refused before any flash is read
    /// or written.
    #[test]
    fn an_odd_breakpoint_address_is_rejected() {
        assert!(Avr::check_breakpoint_address(0).is_ok());
        assert!(Avr::check_breakpoint_address(0x356).is_ok());
        assert!(Avr::check_breakpoint_address(0x357).is_err());
    }

    /// Setting a breakpoint again, at an address this session has already
    /// planted one at, has to find it again so `plant_software_breakpoint` can
    /// return without touching flash. Which slot it sits in does not matter.
    #[test]
    fn a_breakpoint_this_session_planted_is_found_again() {
        let mut state = AvrCoreState::new();

        assert!(state.software_breakpoint_at(0x356).is_none());
        state.sw_breakpoints[3] = Some(SoftwareBreakpoint {
            address: 0x356,
            original: 0x1234,
        });

        let found = state.software_breakpoint_at(0x356).unwrap();

        assert_eq!(found.address, 0x356);
        assert_eq!(found.original, 0x1234);
        // A different address stays free, so it would still be planted.
        assert!(state.software_breakpoint_at(0x358).is_none());
    }

    #[test]
    fn a_debug_reset_rearms_only_armed_hardware_units() {
        let mut state = AvrCoreState::new();
        state.hw_breakpoints[0] = Some(0x100);
        state.hw_breakpoints[1] = Some(0x2fc);
        state.sw_breakpoints[0] = Some(SoftwareBreakpoint {
            address: 0x356,
            original: 0x818a,
        });

        assert_eq!(Avr::units_to_rearm(&state), vec![(0, 0x100), (1, 0x2fc)]);
    }

    #[test]
    fn nothing_is_rearmed_when_no_hardware_unit_is_armed() {
        let mut state = AvrCoreState::new();
        state.sw_breakpoints[2] = Some(SoftwareBreakpoint {
            address: 0x356,
            original: 0x818a,
        });

        assert!(Avr::units_to_rearm(&state).is_empty());
    }

    /// The backstop in `write_flash_word` refuses a word that would run past
    /// the end of its page rather than panicking in `copy_from_slice`. Only the
    /// last byte of a page can do that, and that byte is odd, so the even
    /// address check keeps the backstop out of reach.
    #[test]
    fn a_word_at_the_last_byte_of_a_page_would_not_fit() {
        for page_size in [64u32, 512] {
            let (_, offset) = page_span(u64::from(page_size) - 1, page_size);

            assert!(offset + 2 > page_size as usize, "{page_size}");
        }
    }

    /// A removal writes the instruction back only when the planted BREAK is
    /// still in flash. Anything else at the address belongs to a newer image,
    /// and the remembered instruction describes the old one, so restoring
    /// would corrupt two bytes of it.
    #[test]
    fn a_breakpoint_word_is_restored_only_when_the_break_is_still_there() {
        // The planted BREAK is intact, so the instruction goes back.
        assert_eq!(
            Avr::restored_word(ocd::BREAK_INSTRUCTION, 0x950f),
            Some(0x950f)
        );
        // Erased flash and new code are both not ours.
        assert_eq!(Avr::restored_word(0xFFFF, 0x950f), None);
        assert_eq!(Avr::restored_word(0x0000, 0x950f), None);
        // A word that happens to equal the remembered instruction is still not
        // a planted BREAK, and writing it would cost an erase cycle for nothing.
        assert_eq!(Avr::restored_word(0x950f, 0x950f), None);
    }

    /// Two breakpoints can share one page. A removal patches only the word it
    /// is given, so taking one of them out leaves the other's BREAK in place.
    /// Both words sit at different offsets of the same page and neither write
    /// covers the other.
    #[test]
    fn two_breakpoints_in_one_page_patch_different_words() {
        let page_size = 512;
        let (first_page, first_offset) = page_span(0x200, page_size);
        let (second_page, second_offset) = page_span(0x356, page_size);

        assert_eq!(first_page, second_page);
        assert!(first_offset + 2 <= second_offset);
    }

    /// Taking the breakpoints out of the state leaves no record behind, so a
    /// later removal cannot write any of them back over a new image.
    #[test]
    fn taking_the_breakpoints_out_leaves_no_record_behind() {
        let mut state = AvrCoreState::new();

        state.sw_breakpoints[1] = Some(SoftwareBreakpoint {
            address: 0x356,
            original: 0x1234,
        });
        state.sw_breakpoints[3] = Some(SoftwareBreakpoint {
            address: 0x400,
            original: 0x5678,
        });

        assert_eq!(
            state.take_software_breakpoints(),
            vec![(0x356, 0x1234), (0x400, 0x5678)]
        );
        assert!(state.software_breakpoint_at(0x356).is_none());
        assert!(state.software_breakpoint_at(0x400).is_none());
        // Taking them again finds nothing.
        assert!(state.take_software_breakpoints().is_empty());
    }
}

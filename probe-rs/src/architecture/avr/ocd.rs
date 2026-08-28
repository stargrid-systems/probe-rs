//! The on-chip debug register block of a modern AVR.
//!
//! The block is memory mapped into the data space at [`BASE`], so everything in
//! it is reachable with ordinary byte and word memory access. That includes the
//! register file, which is otherwise not addressable at all.
//!
//! Microchip does not document this. The map was established by the avr-absurd
//! project, confirmed by decoding Microchip's own tool scripts, and read back
//! from an AVR128DA64 and an ATtiny406. See `scratch/avr/ocd.md`.
//!
//! The offsets here are chip addresses, so they match that document directly.
//! [`address`] turns one into the address probe-rs uses.

use crate::architecture::avr::communication_interface::DATA_SPACE_OFFSET;
use crate::probe::pickit::AvrFamily;

/// Where the debug block sits in the chip data space.
pub const BASE: u64 = 0x0F80;

/// Breakpoint 0 address, a byte address in 17 bits over 3 bytes.
pub const BP0A: u64 = 0x00;
/// Breakpoint 1 address, same layout as [`BP0A`].
pub const BP1A: u64 = 0x04;
/// Which events trap into the debugger, one 16-bit field. See [`trapen`].
pub const TRAPEN: u64 = 0x08;
/// Why the core stopped, one 16-bit field, read only. See [`cause`].
pub const CAUSE: u64 = 0x0C;
/// The instruction to inject instead of the one in flash, write only.
pub const INSN0: u64 = 0x10;
/// The program counter. Reads as the program counter plus one, see [`OcdVersion`].
pub const PC: u64 = 0x14;
/// The stack pointer, 16 bit.
pub const SP: u64 = 0x18;
/// The status register.
pub const SREG: u64 = 0x1C;
/// The register file, `r0` to `r31`, 32 bytes.
pub const REGISTER_FILE: u64 = 0x20;

/// The number of bytes the register file occupies.
pub const REGISTER_FILE_LEN: usize = 32;

/// The `BREAK` instruction, which is how a software breakpoint is planted.
///
/// Writing this opcode over an instruction in flash makes the core stop there.
/// Trapping on it is enabled from the moment debug mode is entered, so nothing
/// else has to be armed. It costs a flash erase cycle every time it is planted
/// or removed, and some of these parts are rated for as few as 1000 cycles.
pub const BREAK_INSTRUCTION: u16 = 0x9598;

/// Turns an offset in this module into the address probe-rs uses for it.
///
/// # Examples
///
/// ```
/// use probe_rs::architecture::avr::ocd;
///
/// // The register file, which the chip has at 0x0FA0.
/// assert_eq!(ocd::address(ocd::REGISTER_FILE), 0x80_0FA0);
/// ```
pub const fn address(offset: u64) -> u64 {
    DATA_SPACE_OFFSET + BASE + offset
}

/// The bits of the `TRAPEN` field.
///
/// The field spans the two bytes at `+0x08` and `+0x09`, and treating it as one
/// little-endian 16-bit value is the convenient way to work with it.
pub mod trapen {
    /// Hold the program counter.
    pub const PCHOLD: u16 = 0x0001;
    /// Enable hardware breakpoints at all. Both this and the per-unit bit are
    /// needed for a breakpoint to fire.
    pub const HWBP: u16 = 0x0002;
    /// Stop again after one instruction.
    pub const STEP: u16 = 0x0004;
    /// Enable breakpoint unit 0.
    pub const BP0: u16 = 0x0100;
    /// Enable breakpoint unit 1.
    pub const BP1: u16 = 0x0200;
    /// Trap on an external break request.
    pub const EXTBRK: u16 = 0x1000;
    /// Trap on a `BREAK` instruction. Set from the moment debug mode is entered.
    pub const SWBP: u16 = 0x2000;
    /// Trap on a jump.
    pub const JMP: u16 = 0x4000;
    /// Trap on an interrupt.
    pub const INT: u16 = 0x8000;

    /// The per-unit enable bit of breakpoint unit `index`.
    pub const fn unit(index: usize) -> u16 {
        if index == 0 { BP0 } else { BP1 }
    }
}

/// The bits of the read-only `CAUSE` field.
///
/// Every value below was triggered deliberately on hardware and read back.
pub mod cause {
    /// The core is stopped. Set alongside whatever caused it to stop.
    pub const STOPPED: u16 = 0x0004;
    /// The debugger asked for the halt.
    pub const EXT: u16 = 0x0040;
    /// A reset stopped the core.
    pub const RESET: u16 = 0x0080;
    /// Breakpoint unit 0 fired, or a step finished.
    ///
    /// The two share this bit on both debug versions, so the cause register
    /// cannot tell them apart. The caller has to remember what it asked for.
    pub const BP0_OR_STEP: u16 = 0x0100;
    /// Breakpoint unit 1 fired.
    pub const BP1: u16 = 0x0200;
    /// An external break request stopped the core.
    pub const EXTBRK: u16 = 0x1000;
    /// A `BREAK` instruction stopped the core.
    pub const SWBP: u16 = 0x2000;
    /// A jump stopped the core.
    pub const JMP: u16 = 0x4000;
    /// An interrupt stopped the core.
    pub const INT: u16 = 0x8000;
}

/// Which revision of the debug hardware a part has.
///
/// Only two things differ, and only one of them needs a branch. The units of
/// the [`PC`] register differ, which is what this type exists for. The
/// breakpoint enable also moves, but writing both places is safe on both
/// revisions, so that needs no branch.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum OcdVersion {
    /// tinyAVR 0-series and 1-series. The [`PC`] register holds a byte address.
    V0,
    /// AVR-Dx and later. The [`PC`] register holds a word address.
    V1,
}

impl OcdVersion {
    /// The revision a part family reports.
    pub fn for_family(family: AvrFamily) -> Self {
        match family {
            AvrFamily::Dx => OcdVersion::V1,
            AvrFamily::Tiny0 => OcdVersion::V0,
        }
    }

    /// Turns the raw [`PC`] register into the word program counter.
    ///
    /// The register reads as the program counter plus one, in word units on
    /// [`OcdVersion::V1`] and in byte units on [`OcdVersion::V0`].
    ///
    /// The `GetPC` script already applies this, so a driver that reads the
    /// program counter through the script does not need this function. It is
    /// here for reading the register directly, which is what
    /// [`OcdVersion::detect`] does.
    ///
    /// # Examples
    ///
    /// ```
    /// use probe_rs::architecture::avr::ocd::OcdVersion;
    ///
    /// // Read from an AVR128DA64, where GetPC answered 0x12a4.
    /// assert_eq!(OcdVersion::V1.word_pc(0x12a5), 0x12a4);
    /// // Read from an ATtiny406, where GetPC answered 0x0031.
    /// assert_eq!(OcdVersion::V0.word_pc(0x0064), 0x0031);
    /// ```
    pub fn word_pc(self, raw: u32) -> u32 {
        match self {
            OcdVersion::V1 => raw.saturating_sub(1),
            OcdVersion::V0 => (raw / 2).saturating_sub(1),
        }
    }

    /// Turns a word program counter into the value the raw [`PC`] register holds.
    ///
    /// This is the inverse of [`OcdVersion::word_pc`], except at a word program
    /// counter of zero on [`OcdVersion::V0`], where the raw value cannot be
    /// recovered because the register counts bytes.
    pub fn raw_pc(self, word_pc: u32) -> u32 {
        match self {
            OcdVersion::V1 => word_pc + 1,
            OcdVersion::V0 => (word_pc + 1) * 2,
        }
    }

    /// Works out the revision by comparing the raw register against `GetPC`.
    ///
    /// The System Information Block also carries the revision, but the read of
    /// it comes back truncated on this tool, so this comparison is the reliable
    /// way. Both formulas agree when the core sits at the reset vector, and
    /// then this answers `None` because there is nothing to tell apart.
    ///
    /// # Examples
    ///
    /// ```
    /// use probe_rs::architecture::avr::ocd::OcdVersion;
    ///
    /// assert_eq!(OcdVersion::detect(0x12a5, 0x12a4), Some(OcdVersion::V1));
    /// assert_eq!(OcdVersion::detect(0x0064, 0x0031), Some(OcdVersion::V0));
    /// // At the reset vector both formulas give zero.
    /// assert_eq!(OcdVersion::detect(0x0001, 0x0000), None);
    /// ```
    pub fn detect(raw: u32, script_word_pc: u32) -> Option<Self> {
        let v1 = OcdVersion::V1.word_pc(raw) == script_word_pc;
        let v0 = OcdVersion::V0.word_pc(raw) == script_word_pc;

        match (v1, v0) {
            (true, false) => Some(OcdVersion::V1),
            (false, true) => Some(OcdVersion::V0),
            _ => None,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The offsets have to match the register map, and the translated addresses
    /// have to match what a memory read on a live part used.
    #[test]
    fn the_offsets_land_where_the_register_map_says() {
        assert_eq!(address(BP0A), 0x80_0F80);
        assert_eq!(address(BP1A), 0x80_0F84);
        assert_eq!(address(TRAPEN), 0x80_0F88);
        assert_eq!(address(CAUSE), 0x80_0F8C);
        assert_eq!(address(INSN0), 0x80_0F90);
        assert_eq!(address(PC), 0x80_0F94);
        assert_eq!(address(SP), 0x80_0F98);
        assert_eq!(address(SREG), 0x80_0F9C);
        assert_eq!(address(REGISTER_FILE), 0x80_0FA0);
    }

    /// A hardware breakpoint needs the global bit as well as the per-unit bit.
    /// The `SetHWBP` script only sets the per-unit one, and a breakpoint armed
    /// without the global bit never fires.
    #[test]
    fn arming_a_breakpoint_needs_both_bits() {
        // Measured on an AVR128DA64 with breakpoint 0 armed and firing.
        let armed = trapen::SWBP | trapen::HWBP | trapen::unit(0);
        assert_eq!(armed, 0x2102);

        // What the script leaves behind on its own, which does not fire.
        assert_eq!(trapen::SWBP | trapen::unit(0), 0x2102 & !trapen::HWBP);

        assert_eq!(trapen::unit(0), 0x0100);
        assert_eq!(trapen::unit(1), 0x0200);
    }

    /// Every value here was read back from a part after triggering the cause.
    #[test]
    fn the_measured_halt_causes_decode() {
        assert_eq!(cause::STOPPED | cause::EXT, 0x0044);
        assert_eq!(cause::STOPPED | cause::BP0_OR_STEP, 0x0104);
        assert_eq!(cause::STOPPED | cause::RESET, 0x0084);
        assert_eq!(cause::STOPPED | cause::SWBP, 0x2004);
    }

    /// Both readings were taken from a halted part alongside what `GetPC`
    /// answered, so these pin the plus-one and the word-versus-byte units.
    #[test]
    fn the_measured_program_counters_convert() {
        // AVR128DA64, raw 0x12a5, GetPC 0x12a4.
        assert_eq!(OcdVersion::V1.word_pc(0x12a5), 0x12a4);
        // ATtiny406, raw 0x0064, GetPC 0x0031.
        assert_eq!(OcdVersion::V0.word_pc(0x0064), 0x0031);
        // AVR128DA64 after a debug reset, raw 0x0001 at the reset vector.
        assert_eq!(OcdVersion::V1.word_pc(0x0001), 0x0000);
        // AVR128DA64 after SetPC(0x011c), raw 0x011d.
        assert_eq!(OcdVersion::V1.word_pc(0x011d), 0x011c);
    }

    #[test]
    fn the_program_counter_conversion_round_trips() {
        for word_pc in [0, 1, 0x11c, 0x12a4, 0xffff] {
            for version in [OcdVersion::V0, OcdVersion::V1] {
                assert_eq!(version.word_pc(version.raw_pc(word_pc)), word_pc);
            }
        }
    }

    #[test]
    fn the_version_falls_out_of_the_two_formulas() {
        assert_eq!(OcdVersion::detect(0x12a5, 0x12a4), Some(OcdVersion::V1));
        assert_eq!(OcdVersion::detect(0x0064, 0x0031), Some(OcdVersion::V0));
        // At the reset vector the two formulas agree, so nothing can be told apart.
        assert_eq!(OcdVersion::detect(0x0001, 0x0000), None);
        // A reading that fits neither formula.
        assert_eq!(OcdVersion::detect(0x1000, 0x0500), None);
    }

    #[test]
    fn the_families_map_to_their_debug_version() {
        assert_eq!(OcdVersion::for_family(AvrFamily::Dx), OcdVersion::V1);
        assert_eq!(OcdVersion::for_family(AvrFamily::Tiny0), OcdVersion::V0);
    }
}

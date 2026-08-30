//! AVR register descriptions and their DWARF numbers.
//!
//! The DWARF numbers come from avr-gcc and from the LLVM AVR backend, which
//! agree on everything probe-rs needs. `r0` to `r31` are DWARF 0 to 31, and the
//! stack pointer is DWARF 32.
//!
//! The stack pointer needs a note. avr-gcc splits it into `SPL` at 32 and `SPH`
//! at 33, while probe-rs wants one register per value, so [`SP`] is a single
//! register carrying number 32. Every function in a Rust AVR binary uses
//! `DW_OP_regx: 32` as its `DW_AT_frame_base`, so this is the number that has
//! to resolve for local variables to be readable.
//!
//! [`SP`] and [`FP`] both hold data space addresses in the probe-rs convention,
//! which the core module translates to and from the raw chip values. That is why
//! they are wider than the 16 bits the chip holds.

use std::sync::LazyLock;

use crate::core::{RegisterDataType, UnwindRule};
use crate::{CoreRegister, CoreRegisters, RegisterId, RegisterRole};

// The return address lives in DWARF column 36, measured from the CIE of an
// object built by avr-gcc 14.2.0. An earlier derivation of 37 was wrong. No
// core register holds the return address on AVR, because `call` pushes it onto
// the stack, and gimli reads the column out of the CIE itself, so probe-rs
// never has to supply it.

/// Builds one of the 32 general purpose registers.
macro_rules! gpr {
    ($name:literal, $number:literal) => {
        CoreRegister {
            roles: &[RegisterRole::Core($name)],
            id: RegisterId($number),
            dwarf_id: Some($number),
            data_type: RegisterDataType::UnsignedInteger(8),
            unwind_rule: UnwindRule::Clear,
        }
    };
}

/// The frame pointer, which is the `Y` pointer pair `r28:r29`.
///
/// A function only sets `Y` up as a frame pointer when it needs one. Optimised
/// code often does not, and then the frame base is the stack pointer instead.
///
/// DWARF numbers the two halves separately, as 28 and 29, so this combined
/// register carries no number of its own.
///
/// It is 32 bits wide for the same reason as [`SP`].
pub const FP: CoreRegister = CoreRegister {
    roles: &[RegisterRole::Core("Y"), RegisterRole::FramePointer],
    id: RegisterId(35),
    dwarf_id: None,
    data_type: RegisterDataType::UnsignedInteger(32),
    unwind_rule: UnwindRule::Preserve,
};

/// The stack pointer, as a probe-rs data space address.
///
/// DWARF number 32, which avr-gcc names `SPL`. See the module documentation for
/// why the high half does not get a register of its own.
///
/// The chip holds 16 bits, but the value probe-rs reports carries the data space
/// offset so that it names the stack rather than flash. That needs 24 bits, and
/// the next size a register value comes in is 32.
pub const SP: CoreRegister = CoreRegister {
    roles: &[RegisterRole::Core("SP"), RegisterRole::StackPointer],
    id: RegisterId(32),
    dwarf_id: Some(32),
    data_type: RegisterDataType::UnsignedInteger(32),
    unwind_rule: UnwindRule::SpecialRule,
};

/// The program counter, as a byte address into flash.
///
/// The hardware counts instruction words, but probe-rs and the debug
/// information both count bytes, so the core doubles the value it reads.
///
/// It is 32 bits wide although no AVR has that much flash. avr-gcc sets the
/// DWARF pointer size to 4, and `DebugRegisters::get_address_size_bytes` takes
/// the address size from this register, so 32 is what makes the two agree.
pub const PC: CoreRegister = CoreRegister {
    roles: &[RegisterRole::Core("PC"), RegisterRole::ProgramCounter],
    id: RegisterId(33),
    dwarf_id: None,
    data_type: RegisterDataType::UnsignedInteger(32),
    unwind_rule: UnwindRule::Clear,
};

/// The status register.
pub const SREG: CoreRegister = CoreRegister {
    roles: &[RegisterRole::Core("SREG"), RegisterRole::ProcessorStatus],
    id: RegisterId(34),
    dwarf_id: None,
    data_type: RegisterDataType::UnsignedInteger(8),
    unwind_rule: UnwindRule::Clear,
};

/// The registers of an AVR core.
pub static AVR_CORE_REGISTERS: LazyLock<CoreRegisters> =
    LazyLock::new(|| CoreRegisters::new(AVR_REGISTERS_SET.iter().collect()));

/// `r28` comes before [`FP`] in this list, so a lookup of DWARF 28 finds the
/// 8-bit register that DWARF actually means.
static AVR_REGISTERS_SET: &[CoreRegister] = &[
    gpr!("r0", 0),
    gpr!("r1", 1),
    gpr!("r2", 2),
    gpr!("r3", 3),
    gpr!("r4", 4),
    gpr!("r5", 5),
    gpr!("r6", 6),
    gpr!("r7", 7),
    gpr!("r8", 8),
    gpr!("r9", 9),
    gpr!("r10", 10),
    gpr!("r11", 11),
    gpr!("r12", 12),
    gpr!("r13", 13),
    gpr!("r14", 14),
    gpr!("r15", 15),
    gpr!("r16", 16),
    gpr!("r17", 17),
    gpr!("r18", 18),
    gpr!("r19", 19),
    gpr!("r20", 20),
    gpr!("r21", 21),
    gpr!("r22", 22),
    gpr!("r23", 23),
    gpr!("r24", 24),
    gpr!("r25", 25),
    gpr!("r26", 26),
    gpr!("r27", 27),
    gpr!("r28", 28),
    gpr!("r29", 29),
    gpr!("r30", 30),
    gpr!("r31", 31),
    FP,
    SP,
    PC,
    SREG,
];

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn general_purpose_registers_are_dwarf_0_to_31() {
        for (number, register) in AVR_CORE_REGISTERS.core_registers().take(32).enumerate() {
            assert_eq!(register.name(), format!("r{number}"));
            assert_eq!(register.dwarf_id, Some(number as u16));
            assert_eq!(register.size_in_bits(), 8);
        }
    }

    /// The stack pointer is DWARF 32, which is what every Rust AVR function
    /// uses as its frame base. Getting this wrong loses every stack local.
    #[test]
    fn the_stack_pointer_is_dwarf_32() {
        assert_eq!(SP.dwarf_id, Some(32));
    }

    /// Both pointer registers report probe-rs data space addresses, which do
    /// not fit in the 16 bits the chip holds. A narrower register would make
    /// the debug adapter reject every value a user could write back.
    #[test]
    fn the_pointer_registers_are_wide_enough_for_a_data_space_address() {
        assert_eq!(SP.size_in_bits(), 32);
        assert_eq!(FP.size_in_bits(), 32);
    }

    /// DWARF 28 means the 8-bit `r28`, not the `Y` pair, so the pair must not
    /// claim the number.
    #[test]
    fn only_one_register_claims_each_dwarf_number() {
        let mut numbers: Vec<u16> = AVR_CORE_REGISTERS
            .core_registers()
            .filter_map(|register| register.dwarf_id)
            .collect();
        let count = numbers.len();

        numbers.sort_unstable();
        numbers.dedup();

        assert_eq!(numbers.len(), count);
        assert_eq!(FP.dwarf_id, None);
    }

    /// The program counter width decides the address size the debug info is
    /// formatted with, and avr-gcc puts a pointer size of 4 in the DWARF.
    #[test]
    fn the_program_counter_is_32_bit() {
        assert_eq!(PC.size_in_bits(), 32);
    }

    #[test]
    fn the_roles_resolve_to_the_expected_registers() {
        assert_eq!(AVR_CORE_REGISTERS.pc(), Some(&PC));
        assert_eq!(AVR_CORE_REGISTERS.psr(), Some(&SREG));
        // AVR pushes the return address onto the stack, so no register holds it.
        assert!(
            !AVR_CORE_REGISTERS
                .all_registers()
                .any(|register| register.register_has_role(RegisterRole::ReturnAddress))
        );
    }
}

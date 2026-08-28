//! Xtensa register descriptions.

use std::sync::LazyLock;

use crate::{
    CoreRegister, CoreRegisters, RegisterRole,
    architecture::xtensa::arch::SpecialRegister,
    core::{RegisterDataType, UnwindRule},
};

/// The program counter register.
pub const PC: CoreRegister = CoreRegister {
    roles: &[RegisterRole::Core("pc"), RegisterRole::ProgramCounter],
    id: crate::RegisterId(0xFF00),
    dwarf_id: Some(16),
    data_type: RegisterDataType::UnsignedInteger(32),
    unwind_rule: UnwindRule::Clear,
};

/// The return address register.
pub const RA: CoreRegister = CoreRegister {
    roles: &[RegisterRole::Core("a0"), RegisterRole::ReturnAddress],
    id: crate::RegisterId(0x0000),
    dwarf_id: Some(0),
    data_type: RegisterDataType::UnsignedInteger(32),
    unwind_rule: UnwindRule::Clear,
};

/// The stack pointer register.
pub const SP: CoreRegister = CoreRegister {
    roles: &[
        RegisterRole::Core("sp"),
        RegisterRole::Core("a1"),
        RegisterRole::StackPointer,
    ],
    id: crate::RegisterId(0x0001),
    dwarf_id: Some(1),
    data_type: RegisterDataType::UnsignedInteger(32),
    unwind_rule: UnwindRule::Clear,
};

/// The frame pointer register.
pub const FP: CoreRegister = CoreRegister {
    roles: &[
        RegisterRole::Core("fp"),
        RegisterRole::Core("a7"),
        RegisterRole::FramePointer,
    ],
    id: crate::RegisterId(0x0007),
    dwarf_id: Some(7),
    data_type: RegisterDataType::UnsignedInteger(32),
    unwind_rule: UnwindRule::Clear,
};

/// Processor status register.
pub const PS: CoreRegister = CoreRegister {
    roles: &[RegisterRole::Core("ps"), RegisterRole::ProcessorStatus],
    id: crate::RegisterId(0xFF01),
    dwarf_id: Some(17),
    data_type: RegisterDataType::UnsignedInteger(32),
    unwind_rule: UnwindRule::Clear,
};

/// Xtensa core registers
pub static XTENSA_CORE_REGISTERS: LazyLock<CoreRegisters> = LazyLock::new(|| {
    CoreRegisters::new(
        XTENSA_REGISTERS_SET
            .iter()
            // Occasionally very useful to observe, but way too much to always include:
            //.chain(XTENSA_EXCEPTION_OPTION_REGISTERS.iter())
            //.chain(XTENSA_HP_INTERRUPT_OPTION_REGISTERS.iter())
            .collect(),
    )
});

static XTENSA_REGISTERS_SET: &[CoreRegister] = &[
    RA,
    SP,
    CoreRegister {
        roles: &[
            RegisterRole::Core("a2"),
            RegisterRole::Argument("a2"),
            RegisterRole::Return("a2"),
        ],
        id: crate::RegisterId(0x0002),
        dwarf_id: Some(2),
        data_type: RegisterDataType::UnsignedInteger(32),
        unwind_rule: UnwindRule::Clear,
    },
    CoreRegister {
        roles: &[
            RegisterRole::Core("a3"),
            RegisterRole::Argument("a3"),
            RegisterRole::Return("a3"),
        ],
        id: crate::RegisterId(0x0003),
        dwarf_id: Some(3),
        data_type: RegisterDataType::UnsignedInteger(32),
        unwind_rule: UnwindRule::Clear,
    },
    CoreRegister {
        roles: &[
            RegisterRole::Core("a4"),
            RegisterRole::Argument("a4"),
            RegisterRole::Return("a4"),
        ],
        id: crate::RegisterId(0x0004),
        dwarf_id: Some(4),
        data_type: RegisterDataType::UnsignedInteger(32),
        unwind_rule: UnwindRule::Clear,
    },
    CoreRegister {
        roles: &[
            RegisterRole::Core("a5"),
            RegisterRole::Argument("a5"),
            RegisterRole::Return("a5"),
        ],
        id: crate::RegisterId(0x0005),
        dwarf_id: Some(5),
        data_type: RegisterDataType::UnsignedInteger(32),
        unwind_rule: UnwindRule::Clear,
    },
    CoreRegister {
        roles: &[RegisterRole::Core("a6"), RegisterRole::Argument("a6")],
        id: crate::RegisterId(0x0006),
        dwarf_id: Some(6),
        data_type: RegisterDataType::UnsignedInteger(32),
        unwind_rule: UnwindRule::Clear,
    },
    FP,
    CoreRegister {
        roles: &[RegisterRole::Core("a8")],
        id: crate::RegisterId(0x0008),
        dwarf_id: Some(8),
        data_type: RegisterDataType::UnsignedInteger(32),
        unwind_rule: UnwindRule::Clear,
    },
    CoreRegister {
        roles: &[RegisterRole::Core("a9")],
        id: crate::RegisterId(0x0009),
        dwarf_id: Some(9),
        data_type: RegisterDataType::UnsignedInteger(32),
        unwind_rule: UnwindRule::Clear,
    },
    CoreRegister {
        roles: &[RegisterRole::Core("a10")],
        id: crate::RegisterId(0x000A),
        dwarf_id: Some(10),
        data_type: RegisterDataType::UnsignedInteger(32),
        unwind_rule: UnwindRule::Clear,
    },
    CoreRegister {
        roles: &[RegisterRole::Core("a11")],
        id: crate::RegisterId(0x000B),
        dwarf_id: Some(11),
        data_type: RegisterDataType::UnsignedInteger(32),
        unwind_rule: UnwindRule::Clear,
    },
    CoreRegister {
        roles: &[RegisterRole::Core("a12")],
        id: crate::RegisterId(0x000C),
        dwarf_id: Some(12),
        data_type: RegisterDataType::UnsignedInteger(32),
        unwind_rule: UnwindRule::Clear,
    },
    CoreRegister {
        roles: &[RegisterRole::Core("a13")],
        id: crate::RegisterId(0x000D),
        dwarf_id: Some(13),
        data_type: RegisterDataType::UnsignedInteger(32),
        unwind_rule: UnwindRule::Clear,
    },
    CoreRegister {
        roles: &[RegisterRole::Core("a14")],
        id: crate::RegisterId(0x000E),
        dwarf_id: Some(14),
        data_type: RegisterDataType::UnsignedInteger(32),
        unwind_rule: UnwindRule::Clear,
    },
    CoreRegister {
        roles: &[RegisterRole::Core("a15")],
        id: crate::RegisterId(0x000F),
        dwarf_id: Some(15),
        data_type: RegisterDataType::UnsignedInteger(32),
        unwind_rule: UnwindRule::Clear,
    },
    PC,
    PS,
];

#[allow(unused)]
static XTENSA_EXCEPTION_OPTION_REGISTERS: &[CoreRegister] = &[
    CoreRegister {
        roles: &[RegisterRole::Other("EPC1")],
        id: crate::RegisterId(0x0100 | SpecialRegister::Epc1 as u16),
        dwarf_id: None,
        data_type: RegisterDataType::UnsignedInteger(32),
        unwind_rule: UnwindRule::Preserve,
    },
    CoreRegister {
        roles: &[RegisterRole::Other("EXCCAUSE")],
        id: crate::RegisterId(0x0100 | SpecialRegister::ExcCause as u16),
        dwarf_id: None,
        data_type: RegisterDataType::UnsignedInteger(32),
        unwind_rule: UnwindRule::Preserve,
    },
    CoreRegister {
        roles: &[RegisterRole::Other("EXCSAVE1")],
        id: crate::RegisterId(0x0100 | SpecialRegister::ExcSave1 as u16),
        dwarf_id: None,
        data_type: RegisterDataType::UnsignedInteger(32),
        unwind_rule: UnwindRule::Preserve,
    },
    CoreRegister {
        roles: &[RegisterRole::Other("EXCVADDR")],
        id: crate::RegisterId(0x0100 | SpecialRegister::ExcVaddr as u16),
        dwarf_id: None,
        data_type: RegisterDataType::UnsignedInteger(32),
        unwind_rule: UnwindRule::Preserve,
    },
    CoreRegister {
        roles: &[RegisterRole::Other("DEPC")],
        id: crate::RegisterId(0x0100 | 192),
        dwarf_id: None,
        data_type: RegisterDataType::UnsignedInteger(32),
        unwind_rule: UnwindRule::Preserve,
    },
];
#[allow(unused)]
static XTENSA_HP_INTERRUPT_OPTION_REGISTERS: &[CoreRegister] = &[
    CoreRegister {
        roles: &[RegisterRole::Other("EPC2")],
        id: crate::RegisterId(0x0100 | SpecialRegister::Epc2 as u16),
        dwarf_id: None,
        data_type: RegisterDataType::UnsignedInteger(32),
        unwind_rule: UnwindRule::Preserve,
    },
    CoreRegister {
        roles: &[RegisterRole::Other("EPC3")],
        id: crate::RegisterId(0x0100 | SpecialRegister::Epc3 as u16),
        dwarf_id: None,
        data_type: RegisterDataType::UnsignedInteger(32),
        unwind_rule: UnwindRule::Preserve,
    },
    CoreRegister {
        roles: &[RegisterRole::Other("EPC4")],
        id: crate::RegisterId(0x0100 | SpecialRegister::Epc4 as u16),
        dwarf_id: None,
        data_type: RegisterDataType::UnsignedInteger(32),
        unwind_rule: UnwindRule::Preserve,
    },
    CoreRegister {
        roles: &[RegisterRole::Other("EPC5")],
        id: crate::RegisterId(0x0100 | SpecialRegister::Epc5 as u16),
        dwarf_id: None,
        data_type: RegisterDataType::UnsignedInteger(32),
        unwind_rule: UnwindRule::Preserve,
    },
    CoreRegister {
        roles: &[RegisterRole::Other("EPC6")],
        id: crate::RegisterId(0x0100 | SpecialRegister::Epc6 as u16),
        dwarf_id: None,
        data_type: RegisterDataType::UnsignedInteger(32),
        unwind_rule: UnwindRule::Preserve,
    },
    CoreRegister {
        roles: &[RegisterRole::Other("EPC7")],
        id: crate::RegisterId(0x0100 | SpecialRegister::Epc7 as u16),
        dwarf_id: None,
        data_type: RegisterDataType::UnsignedInteger(32),
        unwind_rule: UnwindRule::Preserve,
    },
    CoreRegister {
        roles: &[RegisterRole::Other("EPS2")],
        id: crate::RegisterId(0x0100 | SpecialRegister::Eps2 as u16),
        dwarf_id: None,
        data_type: RegisterDataType::UnsignedInteger(32),
        unwind_rule: UnwindRule::Preserve,
    },
    CoreRegister {
        roles: &[RegisterRole::Other("EPS3")],
        id: crate::RegisterId(0x0100 | SpecialRegister::Eps3 as u16),
        dwarf_id: None,
        data_type: RegisterDataType::UnsignedInteger(32),
        unwind_rule: UnwindRule::Preserve,
    },
    CoreRegister {
        roles: &[RegisterRole::Other("EPS4")],
        id: crate::RegisterId(0x0100 | SpecialRegister::Eps4 as u16),
        dwarf_id: None,
        data_type: RegisterDataType::UnsignedInteger(32),
        unwind_rule: UnwindRule::Preserve,
    },
    CoreRegister {
        roles: &[RegisterRole::Other("EPS5")],
        id: crate::RegisterId(0x0100 | SpecialRegister::Eps5 as u16),
        dwarf_id: None,
        data_type: RegisterDataType::UnsignedInteger(32),
        unwind_rule: UnwindRule::Preserve,
    },
    CoreRegister {
        roles: &[RegisterRole::Other("EPS6")],
        id: crate::RegisterId(0x0100 | SpecialRegister::Eps6 as u16),
        dwarf_id: None,
        data_type: RegisterDataType::UnsignedInteger(32),
        unwind_rule: UnwindRule::Preserve,
    },
    CoreRegister {
        roles: &[RegisterRole::Other("EPS7")],
        id: crate::RegisterId(0x0100 | SpecialRegister::Eps7 as u16),
        dwarf_id: None,
        data_type: RegisterDataType::UnsignedInteger(32),
        unwind_rule: UnwindRule::Preserve,
    },
    CoreRegister {
        roles: &[RegisterRole::Other("EXCSAVE2")],
        id: crate::RegisterId(0x0100 | SpecialRegister::ExcSave2 as u16),
        dwarf_id: None,
        data_type: RegisterDataType::UnsignedInteger(32),
        unwind_rule: UnwindRule::Preserve,
    },
    CoreRegister {
        roles: &[RegisterRole::Other("EXCSAVE3")],
        id: crate::RegisterId(0x0100 | SpecialRegister::ExcSave3 as u16),
        dwarf_id: None,
        data_type: RegisterDataType::UnsignedInteger(32),
        unwind_rule: UnwindRule::Preserve,
    },
    CoreRegister {
        roles: &[RegisterRole::Other("EXCSAVE4")],
        id: crate::RegisterId(0x0100 | SpecialRegister::ExcSave4 as u16),
        dwarf_id: None,
        data_type: RegisterDataType::UnsignedInteger(32),
        unwind_rule: UnwindRule::Preserve,
    },
    CoreRegister {
        roles: &[RegisterRole::Other("EXCSAVE5")],
        id: crate::RegisterId(0x0100 | SpecialRegister::ExcSave5 as u16),
        dwarf_id: None,
        data_type: RegisterDataType::UnsignedInteger(32),
        unwind_rule: UnwindRule::Preserve,
    },
    CoreRegister {
        roles: &[RegisterRole::Other("EXCSAVE6")],
        id: crate::RegisterId(0x0100 | SpecialRegister::ExcSave6 as u16),
        dwarf_id: None,
        data_type: RegisterDataType::UnsignedInteger(32),
        unwind_rule: UnwindRule::Preserve,
    },
    CoreRegister {
        roles: &[RegisterRole::Other("EXCSAVE7")],
        id: crate::RegisterId(0x0100 | SpecialRegister::ExcSave7 as u16),
        dwarf_id: None,
        data_type: RegisterDataType::UnsignedInteger(32),
        unwind_rule: UnwindRule::Preserve,
    },
];

//! Stack unwinding for AVR, which has no call frame information.
//!
//! The LLVM AVR backend emits neither `.debug_frame` nor `.eh_frame`, so a Rust
//! binary for `avr-none` carries no call frame information at all. The normal
//! probe-rs unwinder is driven by that information, so on AVR it has nothing to
//! read and stops after the innermost frame.
//!
//! This module walks the stack by scanning it instead. That is a heuristic, not
//! a sound algorithm. It can miss frames, and on a program that keeps function
//! pointers on the stack it can report a frame that was never called. Fewer
//! frames are preferred over wrong ones, so a candidate that cannot be verified
//! is dropped and the backtrace ends there.
//!
//! # How the scan works
//!
//! `call` and `rcall` push the return address onto the stack as a *word*
//! address, high byte first. The stack grows down and the stack pointer points
//! at the next free byte, so the live stack starts at `SP + 1`. Reading two
//! bytes big endian and doubling gives a candidate byte address in flash.
//!
//! The stack pointer arrives here as a probe-rs address, which for the AVR data
//! space means the chip value plus
//! [`DATA_SPACE_OFFSET`](probe_rs::architecture::avr::communication_interface::DATA_SPACE_OFFSET).
//! The AVR core module adds that offset when it reads the register. So the
//! addresses this module computes go straight to [`MemoryInterface`] and reach
//! the stack, and the caller stack pointer it hands back stays in the same
//! space.
//!
//! A candidate is accepted only when both of these hold.
//!
//! 1. The debug information has a subprogram covering the call instruction.
//! 2. The instruction in front of the return address really is a call, read
//!    back from flash.
//!
//! Each accepted candidate also moves the stack pointer past itself, so the
//! next frame is searched for further up the stack. Offsets therefore increase
//! with depth and the same value cannot be reported twice.
//!
//! # What was measured
//!
//! The big endian byte order, the word to byte doubling, and both filters come
//! from a halted AVR128DA64 running a purpose built binary with a four level
//! call chain. Scanning the live stack recovered five of five frames with no
//! false positives.
//!
//! Both filters are needed. Filling the scan window with random bytes and
//! counting how often a frame comes out gives 39 percent with only the debug
//! information check, and 1.5 percent with the call site check as well. Neither
//! number is zero. This unwinder can report a frame that was never called.
//!
//! # Known limits
//!
//! - Parts with more than 128 KiB of flash push a three byte return address.
//!   Only the two byte form is implemented, which covers every AVR probe-rs
//!   currently supports.
//! - Interrupts push a return address in the same shape as a call, and leave
//!   nothing on the stack to tell the two apart. So an interrupt frame is
//!   reported as an ordinary call frame, and no exception context is produced.
//! - Registers other than the program counter and the stack pointer are left
//!   holding the callee's values, because without frame information there is no
//!   way to find where the caller's values were saved.
//! - A frame larger than [`MAX_FRAME_SCAN_BYTES`] ends the backtrace, because
//!   its return address sits outside the window the scan looks at.
//! - Nothing marks a scanned frame as less trustworthy than a frame that came
//!   from call frame information. [`crate::StackFrame`] has no field for that.

use std::error::Error;
use std::ops::ControlFlow;

use probe_rs::{InstructionSet, MemoryInterface, RegisterRole};

use crate::{
    DebugError, DebugInfo, DebugRegisters, StackFrame, exception_handling::ExceptionInterface,
};

/// How far above the stack pointer the scan looks for one frame's return address.
///
/// This is the largest frame the unwinder can step over. A wider window finds
/// deeper frames, but it also gives unrelated stack data more chances to look
/// like a return address, so it is deliberately small.
const MAX_FRAME_SCAN_BYTES: usize = 128;

/// How much stack is read at a time.
///
/// The scan stops at the first chunk it cannot read, which is how it notices
/// the top of RAM.
const STACK_READ_CHUNK_BYTES: usize = 16;

/// `call k`, a four byte instruction. The mask clears the target address bits.
const CALL_OPCODE: u16 = 0x940E;
const CALL_MASK: u16 = 0xFE0E;

/// `rcall k`, a two byte instruction. The mask clears the relative offset.
const RCALL_OPCODE: u16 = 0xD000;
const RCALL_MASK: u16 = 0xF000;

/// `icall`, and `eicall` on parts with more than 128 KiB of flash.
const ICALL_OPCODE: u16 = 0x9509;
const EICALL_OPCODE: u16 = 0x9519;

/// Unwinds an AVR stack by scanning it for return addresses.
///
/// See the module documentation for the heuristic and its limits.
pub struct AvrExceptionHandler;

impl ExceptionInterface for AvrExceptionHandler {
    fn unwind_without_debuginfo(
        &self,
        unwind_registers: &mut DebugRegisters,
        _frame_pc: u64,
        _stack_frames: &[StackFrame],
        _instruction_set: Option<InstructionSet>,
        debug_info: &DebugInfo,
        memory: &mut dyn MemoryInterface,
    ) -> ControlFlow<Option<DebugError>> {
        let stack_pointer =
            match unwind_registers.get_register_value_by_role(&RegisterRole::StackPointer) {
                Ok(stack_pointer) => stack_pointer,
                Err(err) => return ControlFlow::Break(Some(err.into())),
            };

        let Some(caller) = scan_for_caller(memory, debug_info, stack_pointer) else {
            // No verifiable return address, so end the backtrace rather than
            // report a frame that may not exist.
            return ControlFlow::Break(None);
        };

        tracing::debug!(
            "UNWIND: AVR stack scan found a return address for a caller at {:#010x}, SP {:#010x}",
            caller.program_counter,
            caller.stack_pointer
        );

        let program_counter = unwind_registers.address_to_register_value(caller.program_counter);
        let stack_pointer = unwind_registers.address_to_register_value(caller.stack_pointer);

        if let Some(register) = unwind_registers.get_program_counter_mut() {
            register.value = Some(program_counter);
        }
        match unwind_registers.get_register_mut_by_role(&RegisterRole::StackPointer) {
            Ok(register) => register.value = Some(stack_pointer),
            Err(err) => return ControlFlow::Break(Some(err.into())),
        }

        ControlFlow::Continue(())
    }
}

/// The register values the scan recovered for one calling frame.
struct ScannedCaller {
    /// The address of the call instruction, not of the instruction after it.
    ///
    /// probe-rs reports the calling frame at the call site, so that the source
    /// location is the line that made the call.
    program_counter: u64,
    /// The stack pointer the caller had, once the return address is popped.
    stack_pointer: u64,
}

/// Searches the live stack above `stack_pointer` for the nearest return address.
fn scan_for_caller(
    memory: &mut dyn MemoryInterface,
    debug_info: &DebugInfo,
    stack_pointer: u64,
) -> Option<ScannedCaller> {
    let stack = read_stack(memory, stack_pointer + 1);

    for offset in 0..stack.len().saturating_sub(1) {
        let word_address = u64::from(u16::from_be_bytes([stack[offset], stack[offset + 1]]));
        let return_address = word_address * 2;

        // The shortest call is two bytes, so nothing below that can have a call
        // in front of it.
        if return_address < 2 {
            continue;
        }

        // Step back into the call instruction. For `rcall` this is the whole
        // instruction, for `call` it is the second half of it. Either way it
        // lands inside the calling function.
        let call_site = return_address - 2;

        if !debug_info.has_function_at(call_site) {
            continue;
        }
        if !preceded_by_call(memory, return_address) {
            continue;
        }

        return Some(ScannedCaller {
            program_counter: call_site,
            // `ret` pops two bytes, which leaves the stack pointer just below
            // the low half of the return address.
            stack_pointer: stack_pointer + offset as u64 + 2,
        });
    }

    None
}

/// Reads the live stack, stopping at the end of readable memory.
fn read_stack(memory: &mut dyn MemoryInterface, start: u64) -> Vec<u8> {
    let mut stack = Vec::with_capacity(MAX_FRAME_SCAN_BYTES);

    while stack.len() < MAX_FRAME_SCAN_BYTES {
        let mut chunk = [0u8; STACK_READ_CHUNK_BYTES];
        if let Err(err) = memory.read_8(start + stack.len() as u64, &mut chunk) {
            tracing::debug!(
                error = &err as &dyn Error,
                "UNWIND: AVR stack scan stopped at the end of readable memory"
            );
            break;
        }
        stack.extend_from_slice(&chunk);
    }

    stack
}

/// Reports whether the instruction in front of `return_address` is a call.
///
/// This is the filter that removed every false positive during the hardware
/// measurement. A candidate whose flash cannot be read counts as unverified and
/// is rejected, because a shorter backtrace is better than a wrong one.
fn preceded_by_call(memory: &mut dyn MemoryInterface, return_address: u64) -> bool {
    let read_opcode = |memory: &mut dyn MemoryInterface, address: u64| {
        let mut opcode = [0u8; 2];
        match memory.read_8(address, &mut opcode) {
            // Flash words are stored little endian.
            Ok(()) => Some(u16::from_le_bytes(opcode)),
            Err(err) => {
                tracing::debug!(
                    error = &err as &dyn Error,
                    "UNWIND: Cannot read the instruction at {address:#010x}, dropping the candidate return address"
                );
                None
            }
        }
    };

    let Some(short) = read_opcode(memory, return_address - 2) else {
        return false;
    };
    if short & RCALL_MASK == RCALL_OPCODE || short == ICALL_OPCODE || short == EICALL_OPCODE {
        return true;
    }

    if return_address < 4 {
        return false;
    }
    read_opcode(memory, return_address - 4).is_some_and(|long| long & CALL_MASK == CALL_OPCODE)
}

#[cfg(test)]
mod test {
    use std::path::PathBuf;

    use probe_rs::architecture::avr::communication_interface::DATA_SPACE_OFFSET;
    use probe_rs::architecture::avr::registers::AVR_CORE_REGISTERS;
    use probe_rs::{MemoryInterface, RegisterRole, RegisterValue, test::MockMemory};

    use super::{AvrExceptionHandler, preceded_by_call};
    use crate::exception_handling::ExceptionInterface;
    use crate::{DebugInfo, DebugRegisters};

    /// Where the stack pointer stood when the real target was halted in `level4`.
    ///
    /// The chip held `0x7F00`. This is the probe-rs address for it, which is what
    /// the AVR core reports.
    const HALTED_STACK_POINTER: u64 = DATA_SPACE_OFFSET + 0x7F00;

    /// The word program counter of the halt, inside `level4`.
    const HALTED_PROGRAM_COUNTER: u64 = 0x248;

    /// The return addresses of the four level call chain, as word addresses, and
    /// where they sat on the live stack relative to `SP + 1`.
    ///
    /// Both columns were measured on hardware. The offsets are the ones the
    /// spike reported, so this exercises the real spacing between frames.
    const MEASURED_STACK: &[(usize, u16)] = &[
        (19, 0x011C), // level3 called level4
        (37, 0x00F0), // level2 called level3
        (50, 0x00C4), // level1 called level2
        (61, 0x015C), // main called level1
        (66, 0x0095), // the C runtime called main
    ];

    fn test_file(name: &str) -> PathBuf {
        let mut path = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
        path.push("tests");
        path.push(name);
        path
    }

    /// Loads the debug info of the AVR binary with the four level call chain.
    fn call_chain_debug_info() -> DebugInfo {
        DebugInfo::from_file(test_file("avr-call-chain")).unwrap()
    }

    /// Builds a target image with the real flash contents and the measured stack.
    fn call_chain_memory() -> MockMemory {
        let mut memory = MockMemory::new();
        memory.add_range(0, std::fs::read(test_file("avr-call-chain.bin")).unwrap());

        let mut stack = vec![0u8; 128];
        for &(offset, word_address) in MEASURED_STACK {
            stack[offset..offset + 2].copy_from_slice(&word_address.to_be_bytes());
        }
        memory.add_range(HALTED_STACK_POINTER + 1, [stack, vec![0u8; 128]].concat());

        memory
    }

    fn halted_registers() -> DebugRegisters {
        DebugRegisters::from_core_registers(&AVR_CORE_REGISTERS, |register_id| {
            let value = match *register_id {
                id if id == probe_rs::architecture::avr::registers::PC.id => HALTED_PROGRAM_COUNTER,
                id if id == probe_rs::architecture::avr::registers::SP.id => HALTED_STACK_POINTER,
                _ => 0,
            };
            Some(RegisterValue::U32(value as u32))
        })
    }

    /// The measured DWARF of the real binary, so the addresses below are the
    /// ones avr-gcc and LLVM actually emitted.
    #[test]
    fn debug_info_knows_the_call_chain_functions() {
        let debug_info = call_chain_debug_info();

        // `level1` through `level4`, taken from `DW_AT_low_pc`.
        for entry in [0x132, 0x188, 0x1E0, 0x238, 0x296] {
            assert!(
                debug_info.has_function_at(entry),
                "no function at {entry:#x}"
            );
        }

        // The C runtime that calls `main` has no debug information, which is
        // why the outermost frame of the chain cannot be recovered.
        assert!(!debug_info.has_function_at(0x128));
    }

    #[test]
    fn call_sites_are_recognised_in_real_flash() {
        let mut memory = call_chain_memory();

        // `234: call 0x238`, so the return address 0x238 is preceded by a call.
        assert!(preceded_by_call(&mut memory, 0x238));
        // `2b4: call 0x132`, the last instruction of `main`.
        assert!(preceded_by_call(&mut memory, 0x2B8));
        // `12a: jmp 0x338` is not a call.
        assert!(!preceded_by_call(&mut memory, 0x12E));
    }

    /// Walks the whole chain the way `DebugInfo::unwind` would.
    #[test]
    fn scanning_recovers_the_call_chain() {
        let debug_info = call_chain_debug_info();
        let mut memory = call_chain_memory();
        let mut registers = halted_registers();
        let handler = AvrExceptionHandler;

        let mut frames = vec![
            registers
                .get_register_value_by_role(&RegisterRole::ProgramCounter)
                .unwrap(),
        ];

        while handler
            .unwind_without_debuginfo(
                &mut registers,
                *frames.last().unwrap(),
                &[],
                None,
                &debug_info,
                &mut memory,
            )
            .is_continue()
        {
            frames.push(
                registers
                    .get_register_value_by_role(&RegisterRole::ProgramCounter)
                    .unwrap(),
            );
        }

        // `level4`, then the call sites in `level3`, `level2`, `level1` and
        // `main`. The fifth measured return address is in the C runtime, which
        // has no debug information, so it is dropped rather than guessed at.
        assert_eq!(frames, vec![0x248, 0x236, 0x1DE, 0x186, 0x2B6]);
    }

    /// The whole path, from an ELF with no call frame information through to
    /// named stack frames.
    ///
    /// This is the test that proves the CFI path degrades into the scan rather
    /// than failing, because `avr-call-chain` really has no `.debug_frame`.
    #[test]
    fn a_full_unwind_names_the_call_chain() {
        let debug_info = call_chain_debug_info();
        let mut memory = call_chain_memory();

        let frames = debug_info
            .unwind(
                &mut memory,
                halted_registers(),
                &AvrExceptionHandler,
                Some(probe_rs::InstructionSet::Avr),
                20,
            )
            .unwrap();

        let names: Vec<&str> = frames
            .iter()
            .map(|frame| frame.function_name.as_str())
            .collect();
        assert_eq!(names, ["level4", "level3", "level2", "level1", "main"]);
    }

    /// A stack full of values that are not return addresses must produce no
    /// frames at all.
    #[test]
    fn a_stack_without_return_addresses_yields_no_frames() {
        let debug_info = call_chain_debug_info();

        let mut memory = MockMemory::new();
        memory.add_range(0, std::fs::read(test_file("avr-call-chain.bin")).unwrap());
        // A counting pattern, which is what uninitialised or data-holding stack
        // tends to look like.
        memory.add_range(HALTED_STACK_POINTER + 1, (0..=255u8).collect());

        let mut registers = halted_registers();
        assert!(
            AvrExceptionHandler
                .unwind_without_debuginfo(
                    &mut registers,
                    HALTED_PROGRAM_COUNTER,
                    &[],
                    None,
                    &debug_info,
                    &mut memory,
                )
                .is_break()
        );
    }

    /// The scan must survive a stack pointer that is close to the top of RAM.
    ///
    /// Reading past the last byte of RAM is the normal way for the scan to run
    /// out of stack on a real part.
    #[test]
    fn a_truncated_stack_read_ends_the_backtrace() {
        let debug_info = call_chain_debug_info();

        let mut memory = BoundedMemory {
            inner: call_chain_memory(),
            end: HALTED_STACK_POINTER + 1 + 16,
        };

        let mut registers = halted_registers();
        assert!(
            AvrExceptionHandler
                .unwind_without_debuginfo(
                    &mut registers,
                    HALTED_PROGRAM_COUNTER,
                    &[],
                    None,
                    &debug_info,
                    &mut memory,
                )
                .is_break()
        );
    }

    /// A target image that refuses reads above `end`, standing in for the top
    /// of RAM. [`MockMemory`] panics on an unmapped read instead of failing, so
    /// it cannot express this on its own.
    struct BoundedMemory {
        inner: MockMemory,
        end: u64,
    }

    impl MemoryInterface for BoundedMemory {
        fn supports_native_64bit_access(&mut self) -> bool {
            false
        }

        fn read_8(&mut self, address: u64, data: &mut [u8]) -> Result<(), probe_rs::Error> {
            if address + data.len() as u64 > self.end {
                return Err(probe_rs::Error::Other("above the top of RAM".to_string()));
            }
            self.inner.read_8(address, data)
        }

        fn read_64(&mut self, _address: u64, _data: &mut [u64]) -> Result<(), probe_rs::Error> {
            unimplemented!()
        }

        fn read_32(&mut self, _address: u64, _data: &mut [u32]) -> Result<(), probe_rs::Error> {
            unimplemented!()
        }

        fn read_16(&mut self, _address: u64, _data: &mut [u16]) -> Result<(), probe_rs::Error> {
            unimplemented!()
        }

        fn write_64(&mut self, _address: u64, _data: &[u64]) -> Result<(), probe_rs::Error> {
            unimplemented!()
        }

        fn write_32(&mut self, _address: u64, _data: &[u32]) -> Result<(), probe_rs::Error> {
            unimplemented!()
        }

        fn write_16(&mut self, _address: u64, _data: &[u16]) -> Result<(), probe_rs::Error> {
            unimplemented!()
        }

        fn write_8(&mut self, _address: u64, _data: &[u8]) -> Result<(), probe_rs::Error> {
            unimplemented!()
        }

        fn supports_8bit_transfers(&self) -> Result<bool, probe_rs::Error> {
            Ok(true)
        }

        fn flush(&mut self) -> Result<(), probe_rs::Error> {
            unimplemented!()
        }
    }

    /// The caller stack pointer has to come back in the same address space it
    /// went in, or the next frame would be scanned out of flash.
    #[test]
    fn the_recovered_stack_pointer_stays_in_the_data_space() {
        let debug_info = call_chain_debug_info();
        let mut memory = call_chain_memory();
        let mut registers = halted_registers();

        assert!(
            AvrExceptionHandler
                .unwind_without_debuginfo(
                    &mut registers,
                    HALTED_PROGRAM_COUNTER,
                    &[],
                    None,
                    &debug_info,
                    &mut memory,
                )
                .is_continue()
        );

        let stack_pointer = registers
            .get_register_value_by_role(&RegisterRole::StackPointer)
            .unwrap();

        // `ret` pops the two byte return address that sat at offset 19.
        assert_eq!(stack_pointer, HALTED_STACK_POINTER + 19 + 2);
        assert!(stack_pointer >= DATA_SPACE_OFFSET);
    }

    /// Guards the assumption that the stack is read, and not the flash that
    /// shares the low addresses.
    #[test]
    fn the_scan_reads_the_data_space() {
        let mut memory = call_chain_memory();
        let mut stack = [0u8; 2];
        memory
            .read_8(HALTED_STACK_POINTER + 1 + 19, &mut stack)
            .unwrap();
        assert_eq!(u16::from_be_bytes(stack), 0x011C);
    }
}

//! The scripts probe-rs runs on the two supported AVR families.
//!
//! Nothing here is copied from Microchip. Every script is written as the
//! sequence of UPDI operations it performs and emitted through
//! [`super::script_emit`], from instruction definitions we recovered by static
//! analysis of the tool firmware. Where a script was proven on hardware, that
//! proof covered the operation sequence, and the sequences below are the same
//! operations written out step by step.
//!
//! The two families differ in five scripts. `ReadProgmem` and `WriteProgmem`
//! follow the flash page size, `EraseChip` follows the NVM controller
//! generation, and the breakpoint scripts follow the flash size, which decides
//! whether a breakpoint address needs more than 16 bits.

use std::collections::HashMap;
use std::sync::OnceLock;

use super::script_emit::{Emitter, Label};
use super::scripts::{AvrFamily, Script, ScriptName};

/// UPDI control and status addresses. See `scratch/avr/ocd.md`.
const CS_OCD_CTRLA: u8 = 0x04;
const CS_OCD_STATUS: u8 = 0x05;
const CS_KEY_STATUS: u8 = 0x07;
const CS_RESET_REQ: u8 = 0x08;
const CS_SYS_STATUS: u8 = 0x0b;

/// A second control and status register the erase script gates on. Bit 2
/// reads as the stopped flag, mirroring `CAUSE` in the debug block.
const CS_CORE_STATUS: u8 = 0x0c;

/// `ASI_OCD_STATUS.STOPPED`.
const STOPPED: u32 = 0x01;
/// `ASI_SYS_STATUS.LOCKSTATUS`.
const LOCKSTATUS: u32 = 0x01;
/// `ASI_SYS_STATUS.SYSRST`.
const SYSRST: u32 = 0x20;

/// Memory-mapped debug block addresses. See `scratch/avr/ocd.md`.
const OCD_BP0A: u32 = 0x0f80;
const OCD_TRAPEN: u32 = 0x0f88;
const OCD_INSN0: u32 = 0x0f90;
const OCD_PC: u32 = 0x0f94;

/// NVM controller register addresses, in the data space.
const NVM_CTRLA: u32 = 0x1000;
const NVM_STATUS: u32 = 0x1002;
const NVM_ADDR_LO: u32 = 0x1008;
const NVM_ADDR_HI: u32 = 0x1009;

/// The signature row sits here in the data space.
const SIGNATURE_ROW: u32 = 0x1100;

/// Where `GetDeviceId` reads the revision byte. Not a documented register:
/// the vendor script reads a word here and sends the low byte, and on
/// hardware that byte matched the revision in the ATDF.
const REVISION_SOURCE: u32 = 0x0f01;

/// The interface id of UPDI inside the tool.
const UPDI_INTERFACE: u8 = 8;

/// The largest burst one pointer setup can move.
const MAX_BURST: u32 = 0x100;

/// The flash size the breakpoint scripts assume for a family. A breakpoint
/// address above 64 KiB needs the 17th bit written separately, which only the
/// 128 KiB Dx parts ever need.
fn breakpoint_address_width_limit(family: AvrFamily) -> u32 {
    match family {
        AvrFamily::Dx => 0x2_0000,
        AvrFamily::Tiny0 => 0x1000,
    }
}

/// Opens the UPDI interface with the given entry mode.
fn open_interface(e: &mut Emitter, mode: u8) {
    e.load_imm8(0, UPDI_INTERFACE);
    e.load_imm8(1, mode);
    e.interface_open(0, 1);
}

/// Asserts or releases `ASI_RESET_REQ`.
fn reset_req(e: &mut Emitter, value: u8) {
    e.load_imm8(0, CS_RESET_REQ);
    e.load_imm8(1, value);
    e.write_cs(0, 1);
}

/// Waits until `ASI_SYS_STATUS.SYSRST` reads as `value`.
fn poll_sysrst(e: &mut Emitter, value: u32) {
    e.load_imm8(1, CS_SYS_STATUS);
    e.tick();
    e.read_cs(1);
    e.poll(SYSRST, value, 10);
}

/// Waits until the NVM controller reports idle.
fn wait_nvm_idle(e: &mut Emitter, timeout: u16) {
    e.load_imm32(0, NVM_STATUS);
    e.tick();
    e.read_word(0);
    e.poll(0x3, 0x0, timeout);
}

/// Writes a command byte to the NVM controller.
fn nvm_command(e: &mut Emitter, command: u32) {
    e.load_imm32(6, NVM_CTRLA);
    e.load_imm32(7, command);
    e.store_byte(6, 7);
}

/// Reads `ASI_SYS_STATUS` into r1 and masks it with `mask`.
fn sys_status_masked(e: &mut Emitter, mask: u32) {
    e.load_imm8(0, CS_SYS_STATUS);
    e.read_cs(0);
    e.load_result(1);
    e.and_imm(1, mask);
}

fn enter_prog_mode() -> Vec<u8> {
    let mut e = Emitter::new();
    e.siglow();
    open_interface(&mut e, 0);
    e.siglow_off();

    // The open leaves 0x19 in the result when no target answers. Anything
    // else, success or not, is worth going on with: the lock state is
    // reported at the end.
    let no_target = e.label();
    e.jump_if_result_eq(0x19, no_target);
    e.delay_ms(50);
    e.delay_ms(64);

    // A programming session that is already up needs no reset dance.
    let done = e.label();
    sys_status_masked(&mut e, 0x08);
    e.jump_if_eq_imm(1, 0x08, done);

    // Assert reset and offer the NVMProg key over UPDI.
    reset_req(&mut e, 0x59);
    e.load_imm8(1, 0);
    e.tag(b" gor");
    e.tag(b"PMVN");
    e.send_key(1);

    // ASI_KEY_STATUS bit 4 confirms the key was taken.
    e.load_imm8(2, CS_KEY_STATUS);
    e.read_cs(2);
    e.load_result(3);
    e.and_imm(3, 0x10);
    let err_100 = e.label();
    e.load_imm8(4, 0x10);
    e.jump_if_ne(3, 4, err_100);

    // A second reset pulse makes the key stick.
    reset_req(&mut e, 0x59);
    poll_sysrst(&mut e, SYSRST);
    reset_req(&mut e, 0x00);
    poll_sysrst(&mut e, 0);

    e.delay_ms(72);

    // The session must be up now.
    sys_status_masked(&mut e, 0x08);
    e.jump_if_eq_imm(1, 0x08, done);

    // Give the part half a second to leave reset, then audit it: still
    // stopped, then locked, in that order.
    e.delay_ms(500);
    e.load_imm8(1, CS_CORE_STATUS);
    e.read_cs(1);
    e.load_result(2);
    e.and_imm(2, 0x04);
    let err_43 = e.label();
    e.jump_if_eq_imm(2, 0x04, err_43);

    sys_status_masked(&mut e, LOCKSTATUS);
    let err_44 = e.label();
    e.jump_if_eq_imm(1, LOCKSTATUS, err_44);
    // The result variable still holds ASI_SYS_STATUS here.
    e.load_result(1);
    e.and_imm(1, 0x02);
    e.jump_if_eq_imm(1, 0x02, err_44);
    e.jump(err_100);

    e.bind(no_target);
    e.load_imm32(1, 0x51);
    e.error(1);
    e.jump(done);

    e.bind(err_44);
    e.load_imm32(1, 0x44);
    e.error(1);
    e.jump(done);

    e.bind(err_43);
    e.load_imm32(1, 0x43);
    e.error(1);
    e.jump(done);

    e.bind(err_100);
    e.load_imm32(1, 0x100);
    e.error(1);

    e.bind(done);
    e.end();
    e.finish()
}

fn exit_prog_mode() -> Vec<u8> {
    let mut e = Emitter::new();
    e.siglow();
    reset_req(&mut e, 0x59);
    reset_req(&mut e, 0x00);
    e.interface_close();
    e.end();
    e.finish()
}

fn enter_debug_mode() -> Vec<u8> {
    let mut e = Emitter::new();
    open_interface(&mut e, 0);
    e.delay_ms(200);

    // A locked part does not get as far as the key.
    sys_status_masked(&mut e, LOCKSTATUS);
    let fail = e.label();
    e.jump_if_eq_imm(1, LOCKSTATUS, fail);

    // Offer the OCD key. UPDI wants it most significant group first.
    e.load_imm8(1, 0);
    e.tag(b"    ");
    e.tag(b" DCO");
    e.send_key(1);

    // ASI_KEY_STATUS bit 1 confirms the key was taken.
    e.load_imm8(2, CS_KEY_STATUS);
    e.read_cs(2);
    e.load_result(3);
    e.and_imm(3, 0x02);
    let done = e.label();
    e.jump_if_eq_imm(3, 0x02, done);

    e.bind(fail);
    e.load_imm32(1, 0x100);
    e.error(1);

    e.bind(done);
    e.end();
    e.finish()
}

fn exit_debug_mode() -> Vec<u8> {
    let mut e = Emitter::new();
    e.interface_close();
    e.end();
    e.finish()
}

fn set_speed() -> Vec<u8> {
    let mut e = Emitter::new();
    e.load_param32(0);
    e.set_speed(0);
    e.end();
    e.finish()
}

fn get_device_id() -> Vec<u8> {
    let mut e = Emitter::new();
    e.txbulk_on();
    e.load_imm32(0, SIGNATURE_ROW);
    e.set_pointer(0);
    e.load_imm16(1, 3);
    e.set_repeat(1);
    e.load_imm8(2, 3);
    e.burst_read_bytes(2);
    e.load_imm32(2, REVISION_SOURCE);
    e.read_word(2);
    e.send_result8();
    e.end();
    e.finish()
}

fn read_mem8() -> Vec<u8> {
    let mut e = Emitter::new();
    e.load_param32(0);
    e.load_param32(1);
    e.txbulk_on();
    let each = e.loop_start(1);
    e.read_word(0);
    e.send_result8();
    e.add_imm(0, 1);
    e.loop_end(each);
    e.end();
    e.finish()
}

fn write_mem8() -> Vec<u8> {
    let mut e = Emitter::new();
    e.load_param32(0);
    e.load_param32(1);
    // One fully addressed store per byte. The bytes themselves ride the
    // download stream behind the two word parameters.
    let each = e.loop_start(1);
    e.load_param8(3);
    e.store_byte(0, 3);
    e.add_imm(0, 1);
    e.loop_end(each);
    e.end();
    e.finish()
}

/// One page of flash per pass, at most one burst of [`MAX_BURST`] bytes per
/// pointer setup.
fn burst_loop(e: &mut Emitter, page_size: u32) {
    e.load_imm32(17, 0);

    let next_page = e.label();
    let capped = e.label();
    let burst = e.label();
    e.bind(next_page);
    e.load_imm32(15, page_size);
    e.jump_if_gt(1, 15, capped);
    e.mov(15, 1);

    e.bind(capped);
    e.load_imm32(16, MAX_BURST);
    e.jump_if_gt(15, 16, burst);
    e.mov(16, 15);

    e.bind(burst);
    e.set_pointer(0);
    e.mov(4, 16);
    e.shr_imm(4, 1);
    e.set_repeat(4);
    e.burst_read_words(4);
    e.sub(15, 16);
    e.add(0, 16);
    e.sub(1, 16);
    e.jump_if_ne(15, 17, burst);
    e.jump_if_ne(1, 17, next_page);
}

fn read_progmem(page_size: u32) -> Vec<u8> {
    let mut e = Emitter::new();
    e.load_param32(0);
    e.load_param32(1);
    e.txbulk_on();
    burst_loop(&mut e, page_size);
    e.end();
    e.finish()
}

/// The Dx NVM controller programs the page the tool loaded with the burst,
/// after a fixed command preamble per page.
fn write_progmem_dx() -> Vec<u8> {
    let mut e = Emitter::new();
    e.load_param32(0);
    e.load_param32(1);
    wait_nvm_idle(&mut e, 20);
    e.load_imm32(17, 0);

    let next_page = e.label();
    let capped = e.label();
    let burst = e.label();
    let failed = e.label();
    let done = e.label();
    e.bind(next_page);
    e.load_imm32(15, 0x200);
    e.jump_if_gt(1, 15, capped);
    e.mov(15, 1);

    e.bind(capped);
    nvm_command(&mut e, 0x08);
    e.load_imm32(6, 0xff);
    e.store_byte(0, 6);
    wait_nvm_idle(&mut e, 20);
    nvm_command(&mut e, 0x00);
    wait_nvm_idle(&mut e, 20);
    nvm_command(&mut e, 0x02);

    e.load_imm32(16, MAX_BURST);
    e.jump_if_gt(15, 16, burst);
    e.mov(16, 15);

    e.bind(burst);
    e.set_pointer(0);
    e.mov(4, 16);
    e.shr_imm(4, 1);
    e.set_repeat(4);
    e.burst_write_words(4);
    e.sub(15, 16);
    e.add(0, 16);
    e.sub(1, 16);
    e.jump_if_ne(15, 17, burst);

    // The burst leaves a nonzero result when the tool could not keep up.
    e.load_result(12);
    e.load_imm32(13, 0);
    e.jump_if_ne(12, 13, failed);
    wait_nvm_idle(&mut e, 20);
    nvm_command(&mut e, 0x00);
    e.jump_if_ne(1, 17, next_page);
    e.jump(done);

    // Leave the controller idle. The busy polls above have already reported
    // the failure through their own timeout.
    e.bind(failed);
    nvm_command(&mut e, 0x00);

    e.bind(done);
    e.end();
    e.finish()
}

/// The tiny NVM controller takes an explicit erase-and-write command after
/// the page has been loaded, and the burst walk disturbs its address
/// register, which gets restored at the end.
fn write_progmem_tiny() -> Vec<u8> {
    let mut e = Emitter::new();
    e.load_param32(0);
    e.load_param32(1);
    e.load_imm32(2, NVM_ADDR_LO);
    e.read_word(2);
    e.load_result(10);
    e.load_imm32(2, NVM_ADDR_HI);
    e.read_word(2);
    e.load_result(11);
    e.load_imm32(17, 0);

    let next_page = e.label();
    let capped = e.label();
    let burst = e.label();
    let restore = e.label();
    e.bind(next_page);
    e.load_imm32(15, 0x40);
    e.jump_if_gt(1, 15, capped);
    e.mov(15, 1);

    e.bind(capped);
    wait_nvm_idle(&mut e, 20);
    nvm_command(&mut e, 0x04);
    wait_nvm_idle(&mut e, 20);

    e.load_imm32(16, MAX_BURST);
    e.jump_if_gt(15, 16, burst);
    e.mov(16, 15);

    e.bind(burst);
    e.set_pointer(0);
    e.mov(4, 16);
    e.shr_imm(4, 1);
    e.set_repeat(4);
    e.burst_write_words(4);
    e.sub(15, 16);
    e.add(0, 16);
    e.sub(1, 16);
    e.jump_if_ne(15, 17, burst);

    nvm_command(&mut e, 0x03);
    e.load_result(12);
    e.load_imm32(13, 0);
    e.jump_if_ne(12, 13, restore);
    wait_nvm_idle(&mut e, 20);
    e.jump_if_ne(1, 17, next_page);

    e.bind(restore);
    e.load_imm32(2, NVM_ADDR_LO);
    e.store_byte(2, 10);
    e.load_imm32(2, NVM_ADDR_HI);
    e.store_byte(2, 11);
    e.end();
    e.finish()
}

fn erase_chip(family: AvrFamily) -> Vec<u8> {
    let mut e = Emitter::new();

    // The erase command only reaches flash when the part is unlocked, out of
    // system reset and running. Otherwise the keys have to be established
    // through a reset dance first.
    let full = e.label();
    sys_status_masked(&mut e, LOCKSTATUS);
    e.jump_if_eq_imm(1, LOCKSTATUS, full);
    e.load_result(1);
    e.and_imm(1, 0x02);
    e.jump_if_eq_imm(1, 0x02, full);
    e.load_imm8(1, CS_CORE_STATUS);
    e.read_cs(1);
    e.load_result(2);
    e.and_imm(2, 0x04);
    e.jump_if_eq_imm(2, 0x04, full);

    // A programming session leaves the NVM key accepted, so the command goes
    // straight to the controller.
    let quick = e.label();
    let done = e.label();
    e.bind(quick);
    wait_nvm_idle(&mut e, 50);
    e.load_imm32(1, NVM_CTRLA);
    e.load_imm32(
        2,
        match family {
            AvrFamily::Dx => 0x20,
            AvrFamily::Tiny0 => 0x05,
        },
    );
    e.store_byte(1, 2);
    wait_nvm_idle(&mut e, 50);
    if family == AvrFamily::Dx {
        nvm_command(&mut e, 0x00);
    }
    e.jump(done);

    // Locked or stopped: offer the NVM keys and cycle reset to apply them.
    e.bind(full);
    e.delay_ms(50);
    e.delay_ms(64);
    if family == AvrFamily::Dx {
        reset_req(&mut e, 0x59);
    }
    e.load_imm32(1, 0);
    e.tag(b"esar");
    e.tag(b"EMVN");
    e.send_key(1);

    // ASI_KEY_STATUS bit 3 confirms the erase key.
    e.load_imm8(2, CS_KEY_STATUS);
    e.read_cs(2);
    e.load_result(3);
    e.and_imm(3, 0x08);
    let err_100 = e.label();
    e.load_imm32(4, 0x08);
    e.jump_if_ne(3, 4, err_100);

    // When the key is not already in effect, offer the programming key too.
    let reset = e.label();
    sys_status_masked(&mut e, 0x08);
    e.jump_if_eq_imm(1, 0x08, reset);
    e.load_imm8(1, 0);
    e.tag(b" gor");
    e.tag(b"PMVN");
    e.send_key(1);
    e.load_imm8(2, CS_KEY_STATUS);
    e.read_cs(2);
    e.load_result(3);
    e.and_imm(3, 0x10);
    e.load_imm8(4, 0x10);
    e.jump_if_ne(3, 4, err_100);

    e.bind(reset);
    reset_req(&mut e, 0x59);
    poll_sysrst(&mut e, SYSRST);
    reset_req(&mut e, 0x00);
    poll_sysrst(&mut e, 0);

    // The lock status has to clear once the erase went through.
    e.load_imm8(1, CS_SYS_STATUS);
    e.tick();
    e.delay_ms(2);
    e.read_cs(1);
    e.poll(LOCKSTATUS, 0, 500);
    e.read_cs(1);
    e.load_result(3);
    e.and_imm(3, 0x40);
    e.jump_if_eq_imm(3, 0x40, err_100);
    e.jump(quick);

    e.bind(err_100);
    e.load_imm32(1, 0x100);
    e.error(1);

    e.bind(done);
    e.end();
    e.finish()
}

/// Reads `ASI_SYS_STATUS` twice and keeps the second value, as the run
/// control scripts do. The first read discards whatever the bus still held.
fn sys_status_settled(e: &mut Emitter) {
    e.load_imm8(0, CS_SYS_STATUS);
    e.read_cs(0);
    e.read_cs(0);
    e.load_result(2);
}

/// A reset request in flight makes the core unreachable, so run control
/// refuses to act and reports it.
fn fail_if_sysrst(e: &mut Emitter, fail: Label) {
    sys_status_settled(e);
    e.and_imm(2, SYSRST);
    e.jump_if_eq_imm(2, SYSRST, fail);
}

fn halt() -> Vec<u8> {
    let mut e = Emitter::new();
    let fail = e.label();
    fail_if_sysrst(&mut e, fail);

    e.load_imm8(0, CS_OCD_CTRLA);
    e.load_imm8(1, 0x01);
    e.write_cs(0, 1);
    e.load_imm8(0, CS_OCD_STATUS);
    e.tick();
    e.read_cs(0);
    e.poll(STOPPED, STOPPED, 100);

    let done = e.label();
    e.jump(done);
    e.bind(fail);
    e.load_imm32(1, 0x100);
    e.error(1);
    e.bind(done);
    e.end();
    e.finish()
}

fn run() -> Vec<u8> {
    let mut e = Emitter::new();
    let fail = e.label();
    fail_if_sysrst(&mut e, fail);

    // Clear the hold and step traps, then resume.
    e.load_imm32(0, OCD_TRAPEN);
    e.load_imm8(1, 0x02);
    e.store_byte(0, 1);
    e.load_imm8(0, CS_OCD_CTRLA);
    e.load_imm8(1, 0x02);
    e.write_cs(0, 1);

    let done = e.label();
    e.jump(done);
    e.bind(fail);
    e.load_imm32(1, 0x100);
    e.error(1);
    e.bind(done);
    e.end();
    e.finish()
}

fn single_step() -> Vec<u8> {
    let mut e = Emitter::new();
    let fail = e.label();
    fail_if_sysrst(&mut e, fail);

    // Set the step trap, resume for one instruction and wait for the halt.
    e.load_imm32(0, OCD_TRAPEN);
    e.load_imm8(1, 0x04);
    e.store_byte(0, 1);
    e.load_imm8(0, CS_OCD_CTRLA);
    e.load_imm8(1, 0x02);
    e.write_cs(0, 1);
    e.load_imm8(0, CS_OCD_STATUS);
    e.tick();
    e.read_cs(0);
    e.poll(STOPPED, STOPPED, 50);

    let done = e.label();
    e.jump(done);
    e.bind(fail);
    e.load_imm32(1, 0x100);
    e.error(1);
    e.bind(done);
    e.end();
    e.finish()
}

fn debug_reset() -> Vec<u8> {
    let mut e = Emitter::new();
    reset_req(&mut e, 0x59);
    poll_sysrst(&mut e, SYSRST);
    reset_req(&mut e, 0x00);
    poll_sysrst(&mut e, 0);
    e.delay_ms(64);
    e.end();
    e.finish()
}

fn get_halt_status() -> Vec<u8> {
    let mut e = Emitter::new();
    e.load_imm8(0, CS_OCD_STATUS);
    e.read_cs(0);
    e.load_result(1);
    e.and_imm(1, STOPPED);

    let halted = e.label();
    let send = e.label();
    e.jump_if_eq_imm(1, STOPPED, halted);
    e.load_imm32(2, 0x5555_5555);
    e.jump(send);
    e.bind(halted);
    e.load_imm32(2, 0xAAAA_AAAA);
    e.bind(send);
    e.send_reg32(2);
    e.end();
    e.finish()
}

fn get_pc() -> Vec<u8> {
    let mut e = Emitter::new();
    e.load_imm32(0, OCD_PC);
    e.read_word(0);
    e.load_result(1);
    e.read_target_info(2, 3);
    e.load_imm32(4, 0);
    let word_units = e.label();
    e.jump_if_ne(3, 4, word_units);
    // Debug version 0 reports a byte address.
    e.shr_imm(1, 1);
    e.bind(word_units);
    // The register reads the program counter plus one.
    e.sub_imm(1, 1);
    e.send_reg32(1);
    e.end();
    e.finish()
}

fn set_pc() -> Vec<u8> {
    let mut e = Emitter::new();
    e.load_param32(0);
    e.read_target_info(2, 3);
    e.load_imm32(4, 0);
    let word_units = e.label();
    e.jump_if_ne(3, 4, word_units);
    e.shl_imm(0, 1);
    e.bind(word_units);

    e.load_imm32(1, OCD_PC);
    e.store_word(1, 0);
    // No instruction may stay injected, or the step would run it instead.
    e.load_imm32(2, OCD_INSN0);
    e.load_imm16(3, 0);
    e.store_word(2, 3);
    e.load_imm32(0, OCD_TRAPEN);
    e.load_imm8(1, 0x04);
    e.store_byte(0, 1);
    e.load_imm8(0, CS_OCD_CTRLA);
    e.load_imm8(1, 0x02);
    e.write_cs(0, 1);
    e.load_imm8(0, CS_OCD_STATUS);
    e.tick();
    e.read_cs(0);
    e.poll(STOPPED, STOPPED, 10);
    e.end();
    e.finish()
}

/// Arms or disarms a hardware breakpoint. `set` decides between writing the
/// given address and writing a zero over it.
fn hw_breakpoint(family: AvrFamily, set: bool) -> Vec<u8> {
    let mut e = Emitter::new();
    e.load_param32(0);
    let done = e.label();
    if set {
        e.load_param32(1);
    } else {
        e.load_imm32(1, 0);
    }
    e.load_imm32(5, 1);
    e.jump_if_gt(0, 5, done);
    if set {
        e.shl_imm(1, 1);
        e.and_imm(1, !1u32);
    }

    let v0 = e.label();
    let v1 = e.label();
    let address = e.label();
    e.read_target_info(2, 3);
    e.jump_if_eq_imm(3, 0, v0);
    e.jump_if_eq_imm(3, 1, v1);
    e.jump(done);

    // Debug version 1 keeps the enable in the high byte of TRAPEN.
    e.bind(v1);
    e.load_imm32(5, OCD_TRAPEN + 1);
    e.read_word(5);
    e.load_result(6);
    e.mov(7, 0);
    e.add_imm(7, 1);
    if set {
        // A zero address is the "no breakpoint" pattern and stays unarmed.
        let keep = e.label();
        e.jump_if_eq_imm(1, 0, keep);
        e.or(6, 7);
        e.bind(keep);
    } else {
        // The bit to clear is the only one that can be set below it, so
        // subtracting removes exactly this unit's enable.
        e.sub(6, 7);
    }
    e.store_byte(5, 6);
    e.jump(address);

    // Debug version 0 keeps the enable in bit 0 of the address register. For
    // a set, the byte address of a word address is even, so adding one sets
    // the enable and nothing else.
    e.bind(v0);
    if set {
        e.add_imm(1, 1);
    }

    e.bind(address);
    e.load_imm32(5, OCD_BP0A);
    e.shl_imm(0, 2);
    e.add(5, 0);
    e.store_word(5, 1);
    e.load_imm32(6, breakpoint_address_width_limit(family));
    e.load_imm32(7, 0x1_0001);
    e.jump_if_lt(6, 7, done);
    e.add_imm(5, 2);
    if set {
        e.mov(6, 1);
        e.shr_imm(6, 16);
    } else {
        e.load_imm32(6, 0);
    }
    e.store_word(5, 6);

    e.bind(done);
    e.end();
    e.finish()
}

fn set_hw_bp(family: AvrFamily) -> Vec<u8> {
    hw_breakpoint(family, true)
}

fn clear_hw_bp(family: AvrFamily) -> Vec<u8> {
    hw_breakpoint(family, false)
}

/// The `ri4command` word of each script.
///
/// The command id is a property of the operation: every part in the tool pack
/// gives the same word to the same name. Its top two bits say which message
/// type moves the data, which the host is free to ignore.
fn ri4command(name: ScriptName) -> u32 {
    match name {
        ScriptName::EnterProgMode => 0x0000_3000,
        ScriptName::ExitProgMode => 0x0000_3100,
        ScriptName::EnterDebugMode => 0x0000_0200,
        ScriptName::ExitDebugMode => 0x0000_0201,
        ScriptName::SetSpeed => 0x0000_1503,
        ScriptName::GetDeviceId => 0x0000_1505,
        ScriptName::EraseChip => 0x0000_1200,
        ScriptName::ReadMem8 => 0x8000_140e,
        ScriptName::WriteMem8 => 0xc000_130f,
        ScriptName::ReadProgmem => 0x8000_1400,
        ScriptName::WriteProgmem => 0xc000_1300,
        ScriptName::Halt => 0x0000_0204,
        ScriptName::Run => 0x0000_0203,
        ScriptName::SingleStep => 0x0000_0206,
        ScriptName::GetHaltStatus => 0x0000_0205,
        ScriptName::GetPc => 0x0000_0207,
        ScriptName::SetPc => 0x0000_0208,
        ScriptName::SetHwBp => 0x0000_4400,
        ScriptName::ClearHwBp => 0x0000_4402,
        ScriptName::DebugReset => 0x0000_0202,
    }
}

/// Builds the script table of one family.
fn build(family: AvrFamily) -> HashMap<ScriptName, Script> {
    let entries = [
        (ScriptName::EnterProgMode, enter_prog_mode()),
        (ScriptName::ExitProgMode, exit_prog_mode()),
        (ScriptName::EnterDebugMode, enter_debug_mode()),
        (ScriptName::ExitDebugMode, exit_debug_mode()),
        (ScriptName::SetSpeed, set_speed()),
        (ScriptName::GetDeviceId, get_device_id()),
        (ScriptName::ReadMem8, read_mem8()),
        (ScriptName::WriteMem8, write_mem8()),
        (
            ScriptName::ReadProgmem,
            read_progmem(family.flash_page_size()),
        ),
        (
            ScriptName::WriteProgmem,
            match family {
                AvrFamily::Dx => write_progmem_dx(),
                AvrFamily::Tiny0 => write_progmem_tiny(),
            },
        ),
        (ScriptName::EraseChip, erase_chip(family)),
        (ScriptName::Halt, halt()),
        (ScriptName::Run, run()),
        (ScriptName::SingleStep, single_step()),
        (ScriptName::GetHaltStatus, get_halt_status()),
        (ScriptName::GetPc, get_pc()),
        (ScriptName::SetPc, set_pc()),
        (ScriptName::SetHwBp, set_hw_bp(family)),
        (ScriptName::ClearHwBp, clear_hw_bp(family)),
        (ScriptName::DebugReset, debug_reset()),
    ];

    entries
        .into_iter()
        .map(|(name, bytes)| (name, Script::new(ri4command(name), bytes)))
        .collect()
}

/// The script table of a family, built once on first use.
pub(super) fn table(family: AvrFamily) -> &'static HashMap<ScriptName, Script> {
    static DX: OnceLock<HashMap<ScriptName, Script>> = OnceLock::new();
    static TINY0: OnceLock<HashMap<ScriptName, Script>> = OnceLock::new();

    match family {
        AvrFamily::Dx => DX.get_or_init(|| build(AvrFamily::Dx)),
        AvrFamily::Tiny0 => TINY0.get_or_init(|| build(AvrFamily::Tiny0)),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::probe::pickit::scripts::ScriptSource;

    /// A minimal reader for the bytecode the emitter above produces. It knows
    /// exactly the instructions that can appear, so a stray byte fails the
    /// decode instead of passing silently.
    #[derive(Clone, Debug, PartialEq, Eq)]
    struct Ins {
        op: u8,
        sub: u8,
        args: Vec<u8>,
    }

    impl Ins {
        fn is(&self, op: u8, sub: u8) -> bool {
            self.op == op && self.sub == sub
        }

        fn imm32(&self, at: usize) -> u32 {
            u32::from_le_bytes(self.args[at..at + 4].try_into().unwrap())
        }
    }

    fn decode(bytes: &[u8]) -> Vec<Ins> {
        fn operand_len(op: u8, sub: u8) -> usize {
            match op {
                0x1e => match sub {
                    0x01 | 0x06 | 0x07 | 0x0f | 0x15 => 2,
                    0x02 => 0,
                    0x03 | 0x09 | 0x0b | 0x0c | 0x0d | 0x0e | 0x10 | 0x11 | 0x14 => 1,
                    other => panic!("decoder does not know subop {other:#04x}"),
                },
                0x50 | 0x51 | 0x5a | 0x95 | 0x9f | 0xa2 | 0xae => 0,
                0x6c | 0x7f | 0x91 | 0x98 | 0x99 | 0xad => 1,
                0x60 | 0x67 | 0x68 | 0x6a | 0x6e | 0x7c | 0x9b | 0x94 | 0xfb => 2,
                0xf9 | 0xfa | 0xfc => 4,
                0x65 | 0x66 | 0x69 | 0x90 | 0x92 => 5,
                0x9c => 3,
                0xa5 => 10,
                0xfd => 6,
                0xfe => 7,
                other => panic!("decoder does not know opcode {other:#04x}"),
            }
        }

        let mut out = Vec::new();
        let mut at = 0;
        while at < bytes.len() {
            let op = bytes[at];
            at += 1;
            let (op, sub) = if op == 0x1e {
                let sub = bytes[at];
                at += 1;
                (op, sub)
            } else {
                (op, 0)
            };
            let len = operand_len(op, sub);
            out.push(Ins {
                op,
                sub,
                args: bytes[at..at + len].to_vec(),
            });
            at += len;
        }
        out
    }

    fn ops(family: AvrFamily, name: ScriptName) -> Vec<Ins> {
        decode(family.script(name).unwrap().bytes())
    }

    #[test]
    fn every_script_ends_with_a_terminator() {
        for name in ScriptName::ALL {
            for family in [AvrFamily::Dx, AvrFamily::Tiny0] {
                let bytes = family.script(name).unwrap().bytes();
                assert_eq!(bytes.last(), Some(&0x5a), "{family:?} {name}");
            }
        }
    }

    /// Measured on hardware: opening a session asserts `ASI_RESET_REQ` and
    /// leaves it asserted, while entering debug mode writes nothing to that
    /// register at all.
    #[test]
    fn enter_prog_mode_touches_reset_but_enter_debug_mode_does_not() {
        // load r0 = ASI_RESET_REQ, load r1 = value, write the control and
        // status register.
        let reset_write = |value: u8| {
            [
                0x9b,
                0x00,
                CS_RESET_REQ,
                0x9b,
                0x01,
                value,
                0x1e,
                0x0f,
                0x00,
                0x01,
            ]
        };

        let prog = AvrFamily::Dx
            .script(ScriptName::EnterProgMode)
            .unwrap()
            .bytes()
            .to_vec();
        assert!(prog.windows(10).any(|w| w == reset_write(0x59).as_slice()));
        assert!(prog.windows(10).any(|w| w == reset_write(0).as_slice()));

        let debug = ops(AvrFamily::Dx, ScriptName::EnterDebugMode);
        assert!(
            !debug.iter().any(|ins| ins.is(0x1e, 0x0f)),
            "entering debug mode must not write a control and status register"
        );
    }

    /// The families agree on everything except the five scripts whose
    /// hardware differs.
    #[test]
    fn the_families_differ_exactly_on_the_nvm_and_breakpoint_scripts() {
        let differing = [
            ScriptName::EraseChip,
            ScriptName::ReadProgmem,
            ScriptName::WriteProgmem,
            ScriptName::SetHwBp,
            ScriptName::ClearHwBp,
        ];

        for name in ScriptName::ALL {
            let dx = AvrFamily::Dx.script(name).unwrap().bytes();
            let tiny = AvrFamily::Tiny0.script(name).unwrap().bytes();
            assert_eq!(
                dx == tiny,
                !differing.contains(&name),
                "{name} disagrees with the expected family split"
            );
        }
    }

    /// The NVM programming command of `WriteProgmem` follows the controller
    /// generation: the Dx controller takes its command before the page data,
    /// the tiny one erases and writes after the page has been loaded.
    #[test]
    fn the_write_progmem_commands_follow_the_family_nvm_controller() {
        let commands = |family| -> Vec<u32> {
            ops(family, ScriptName::WriteProgmem)
                .into_iter()
                .filter(|ins| ins.is(0x90, 0))
                .map(|ins| ins.imm32(1))
                .collect()
        };

        let dx = commands(AvrFamily::Dx);
        let tiny = commands(AvrFamily::Tiny0);

        assert!(dx.contains(&0x200) && !tiny.contains(&0x200), "page size");
        assert!(tiny.contains(&0x40) && !dx.contains(&0x40), "page size");
        assert!(dx.contains(&0x08) && !tiny.contains(&0x08), "buffer clear");
        assert!(dx.contains(&0x02) && !tiny.contains(&0x02), "program");
        assert!(tiny.contains(&0x04) && !dx.contains(&0x04), "buffer clear");
        assert!(
            tiny.contains(&0x03) && !dx.contains(&0x03),
            "erase and write"
        );
    }

    /// Every script has to consume exactly the parameter block its call site
    /// sends, in the same widths, or the cursor drifts and the operation
    /// hangs or corrupts.
    #[test]
    fn scripts_consume_exactly_the_parameters_the_driver_sends() {
        // (word-wide loads, byte-wide loads behind them).
        // The driver sends `Params::Words` per call site: none for the
        // parameter-free scripts, one word for SetSpeed, SetPc and ClearHwBp,
        // two words for the memory and breakpoint scripts. `WriteMem8` is the
        // one script whose data rides the same stream, one byte per loop
        // pass behind the two words.
        let expected: &[(ScriptName, usize, bool)] = &[
            (ScriptName::EnterProgMode, 0, false),
            (ScriptName::ExitProgMode, 0, false),
            (ScriptName::EnterDebugMode, 0, false),
            (ScriptName::ExitDebugMode, 0, false),
            (ScriptName::SetSpeed, 1, false),
            (ScriptName::GetDeviceId, 0, false),
            (ScriptName::EraseChip, 0, false),
            (ScriptName::ReadMem8, 2, false),
            (ScriptName::WriteMem8, 2, true),
            (ScriptName::ReadProgmem, 2, false),
            (ScriptName::WriteProgmem, 2, false),
            (ScriptName::Halt, 0, false),
            (ScriptName::Run, 0, false),
            (ScriptName::SingleStep, 0, false),
            (ScriptName::GetHaltStatus, 0, false),
            (ScriptName::GetPc, 0, false),
            (ScriptName::SetPc, 1, false),
            (ScriptName::SetHwBp, 2, false),
            (ScriptName::ClearHwBp, 1, false),
            (ScriptName::DebugReset, 0, false),
        ];

        for (name, words, bytes_expected) in expected {
            for family in [AvrFamily::Dx, AvrFamily::Tiny0] {
                let ins = ops(family, *name);
                let word_loads = ins.iter().filter(|i| i.is(0x91, 0)).count();
                let byte_loads = ins.iter().filter(|i| i.is(0x99, 0)).count();

                assert_eq!(word_loads, *words, "{family:?} {name}");
                assert_eq!(
                    byte_loads > 0,
                    *bytes_expected,
                    "{family:?} {name} byte parameter"
                );
                assert_eq!(
                    byte_loads,
                    usize::from(*bytes_expected),
                    "{family:?} {name}"
                );
            }
        }
    }
}

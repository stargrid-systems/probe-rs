//! The instruction set of the tool's script machine, and an emitter for it.
//!
//! The tool exposes no named operations. It runs a bytecode program on a small
//! register machine: 32 registers, a result variable that interface reads and
//! comparisons write to, and a parameter cursor that the load instructions
//! consume sequentially. A program ends at the `5a` terminator or at its last
//! byte.
//!
//! The instruction table here was recovered by static analysis of the tool
//! firmware. Only the instructions our scripts use are modelled. The wider
//! machine has around 340 opcodes, most of which serve other target families.
//!
//! The scripts written on top of this live in [`super::builtins`].

/// A jump target that may not be known yet.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Label(usize);

/// A `loop` instruction that is still waiting for its `endl`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct LoopId(u16);

/// Emits script bytecode, one instruction at a time.
///
/// Jump targets are absolute offsets into the program. Reserve one with
/// [`Emitter::label`], hand it to a jump, and bind it later with
/// [`Emitter::bind`], in source order or any other order.
#[derive(Clone, Debug, Default)]
pub struct Emitter {
    bytes: Vec<u8>,
    labels: Vec<Option<u16>>,
    patches: Vec<(usize, Label)>,
    open_loop: Option<u16>,
}

impl Emitter {
    /// Creates an empty program.
    pub fn new() -> Self {
        Self::default()
    }

    /// The offset the next instruction will land at.
    pub fn here(&self) -> u16 {
        u16::try_from(self.bytes.len()).expect("script grew past 64 KiB")
    }

    /// Reserves a jump target, to bind with [`Emitter::bind`].
    pub fn label(&mut self) -> Label {
        self.labels.push(None);
        Label(self.labels.len() - 1)
    }

    /// Binds `label` to the offset of the next instruction.
    pub fn bind(&mut self, label: Label) {
        let here = self.here();
        assert!(self.labels[label.0].is_none(), "label bound twice");
        self.labels[label.0] = Some(here);

        for (at, l) in &self.patches {
            if *l == label {
                self.bytes[*at..*at + 2].copy_from_slice(&here.to_le_bytes());
            }
        }
        self.patches.retain(|(_, l)| *l != label);
    }

    /// Hands back the bytecode once every label is bound.
    pub fn finish(self) -> Vec<u8> {
        assert!(self.patches.is_empty(), "label never bound");
        assert!(self.open_loop.is_none(), "loop without endl");
        self.bytes
    }

    fn byte(&mut self, byte: u8) {
        self.bytes.push(byte);
    }

    fn reg(&mut self, reg: u8) {
        assert!(reg < 32, "register {reg} does not exist");
        self.bytes.push(reg);
    }

    fn imm8(&mut self, imm: u8) {
        self.bytes.push(imm);
    }

    fn imm16(&mut self, imm: u16) {
        self.bytes.extend_from_slice(&imm.to_le_bytes());
    }

    fn imm32(&mut self, imm: u32) {
        self.bytes.extend_from_slice(&imm.to_le_bytes());
    }

    fn target(&mut self, label: Label) {
        match self.labels[label.0] {
            Some(at) => self.imm16(at),
            None => {
                self.patches.push((self.bytes.len(), label));
                self.imm16(0xffff);
            }
        }
    }

    /// `9b`: loads an 8-bit immediate.
    pub fn load_imm8(&mut self, dst: u8, imm: u8) {
        self.byte(0x9b);
        self.reg(dst);
        self.imm8(imm);
    }

    /// `9c`: loads a 16-bit immediate.
    pub fn load_imm16(&mut self, dst: u8, imm: u16) {
        self.byte(0x9c);
        self.reg(dst);
        self.imm16(imm);
    }

    /// `90`: loads a 32-bit immediate.
    pub fn load_imm32(&mut self, dst: u8, imm: u32) {
        self.byte(0x90);
        self.reg(dst);
        self.imm32(imm);
    }

    /// `91`: loads the next 4-byte parameter.
    pub fn load_param32(&mut self, dst: u8) {
        self.byte(0x91);
        self.reg(dst);
    }

    /// `99`: loads the next 1-byte parameter.
    pub fn load_param8(&mut self, dst: u8) {
        self.byte(0x99);
        self.reg(dst);
    }

    /// `6c`: loads the result variable.
    pub fn load_result(&mut self, dst: u8) {
        self.byte(0x6c);
        self.reg(dst);
    }

    /// `60`: copies a register.
    pub fn mov(&mut self, dst: u8, src: u8) {
        self.byte(0x60);
        self.reg(dst);
        self.reg(src);
    }

    /// `92`: adds a 32-bit immediate.
    pub fn add_imm(&mut self, reg: u8, imm: u32) {
        self.byte(0x92);
        self.reg(reg);
        self.imm32(imm);
    }

    /// `6e`: adds a register.
    pub fn add(&mut self, dst: u8, src: u8) {
        self.byte(0x6e);
        self.reg(dst);
        self.reg(src);
    }

    /// `69`: subtracts a 32-bit immediate.
    pub fn sub_imm(&mut self, reg: u8, imm: u32) {
        self.byte(0x69);
        self.reg(reg);
        self.imm32(imm);
    }

    /// `6a`: subtracts a register.
    pub fn sub(&mut self, dst: u8, src: u8) {
        self.byte(0x6a);
        self.reg(dst);
        self.reg(src);
    }

    /// `66`: bitwise-ANDs a 32-bit immediate.
    pub fn and_imm(&mut self, reg: u8, imm: u32) {
        self.byte(0x66);
        self.reg(reg);
        self.imm32(imm);
    }

    /// `7c`: bitwise-ORs a register.
    pub fn or(&mut self, dst: u8, src: u8) {
        self.byte(0x7c);
        self.reg(dst);
        self.reg(src);
    }

    /// `68`: shifts left by an immediate.
    pub fn shl_imm(&mut self, reg: u8, by: u8) {
        self.byte(0x68);
        self.reg(reg);
        self.imm8(by);
    }

    /// `67`: shifts right by an immediate.
    pub fn shr_imm(&mut self, reg: u8, by: u8) {
        self.byte(0x67);
        self.reg(reg);
        self.imm8(by);
    }

    /// `fb`: jumps to a target.
    pub fn jump(&mut self, target: Label) {
        self.byte(0xfb);
        self.target(target);
    }

    /// `fe`: jumps to a target when a register equals a 32-bit immediate.
    pub fn jump_if_eq_imm(&mut self, reg: u8, imm: u32, target: Label) {
        self.byte(0xfe);
        self.reg(reg);
        self.imm32(imm);
        self.target(target);
    }

    /// `fc`: jumps to a target when two registers differ.
    pub fn jump_if_ne(&mut self, a: u8, b: u8, target: Label) {
        self.byte(0xfc);
        self.reg(a);
        self.reg(b);
        self.target(target);
    }

    /// `fa`: jumps to a target when the first register is greater, unsigned.
    pub fn jump_if_gt(&mut self, a: u8, b: u8, target: Label) {
        self.byte(0xfa);
        self.reg(a);
        self.reg(b);
        self.target(target);
    }

    /// `f9`: jumps to a target when the first register is less, unsigned.
    pub fn jump_if_lt(&mut self, a: u8, b: u8, target: Label) {
        self.byte(0xf9);
        self.reg(a);
        self.reg(b);
        self.target(target);
    }

    /// `fd`: jumps to a target when the result variable equals a 32-bit immediate.
    pub fn jump_if_result_eq(&mut self, imm: u32, target: Label) {
        self.byte(0xfd);
        self.imm32(imm);
        self.target(target);
    }

    /// `ad`: starts a loop whose trip count comes from a register.
    ///
    /// Loops do not nest in the programs we write, so there is one open loop
    /// at a time. End it with [`Emitter::loop_end`].
    pub fn loop_start(&mut self, count_reg: u8) -> LoopId {
        assert!(self.open_loop.is_none(), "nested loops are not modelled");
        let back_to = self.here();
        self.byte(0xad);
        self.reg(count_reg);
        self.open_loop = Some(back_to);
        LoopId(back_to)
    }

    /// `ae`: ends a loop: counts down and jumps back while the count is nonzero.
    pub fn loop_end(&mut self, id: LoopId) {
        assert_eq!(self.open_loop, Some(id.0), "loop_end without loop_start");
        self.open_loop = None;
        self.byte(0xae);
    }

    /// `a5`: waits until `(result & mask) == value`, or times out.
    ///
    /// A timed-out poll aborts the script inside the tool; the host sees a
    /// failed operation.
    pub fn poll(&mut self, mask: u32, value: u32, timeout: u16) {
        self.byte(0xa5);
        self.imm32(mask);
        self.imm32(value);
        self.imm16(timeout);
    }

    /// `94`: busy-waits for roughly `ms` milliseconds.
    pub fn delay_ms(&mut self, ms: u16) {
        self.byte(0x94);
        self.imm16(ms);
    }

    /// `a2`: stores a timestamp into the result variable.
    pub fn tick(&mut self) {
        self.byte(0xa2);
    }

    /// `7f`: reports the error code held in a register.
    pub fn error(&mut self, code_reg: u8) {
        self.byte(0x7f);
        self.reg(code_reg);
    }

    /// `5a`: ends the script.
    pub fn end(&mut self) {
        self.byte(0x5a);
    }

    /// `65`: appends four raw bytes to the output.
    pub fn tag(&mut self, bytes: &[u8; 4]) {
        self.byte(0x65);
        self.imm32(u32::from_le_bytes(*bytes));
        self.imm8(4);
    }

    /// `50`: clears the preamble flag, which the open sequence brackets.
    pub fn siglow(&mut self) {
        self.byte(0x50);
    }

    /// `51`: sets the preamble flag again.
    pub fn siglow_off(&mut self) {
        self.byte(0x51);
    }

    /// `95`: turns on sending of the bytes that interface reads produce.
    pub fn txbulk_on(&mut self) {
        self.byte(0x95);
    }

    /// `98`: appends a register as 4 bytes to the output.
    pub fn send_reg32(&mut self, reg: u8) {
        self.byte(0x98);
        self.reg(reg);
    }

    /// `9f`: appends the low byte of the result variable to the output.
    pub fn send_result8(&mut self) {
        self.byte(0x9f);
    }

    /// `1e 01`: opens the UPDI interface. The registers hold the interface id
    /// and the entry mode.
    pub fn interface_open(&mut self, iface_reg: u8, mode_reg: u8) {
        self.byte(0x1e);
        self.byte(0x01);
        self.reg(iface_reg);
        self.reg(mode_reg);
    }

    /// `1e 02`: closes the interface.
    pub fn interface_close(&mut self) {
        self.byte(0x1e);
        self.byte(0x02);
    }

    /// `1e 03`: reads the word at a register-held data-space address into the
    /// result variable.
    pub fn read_word(&mut self, addr_reg: u8) {
        self.byte(0x1e);
        self.byte(0x03);
        self.reg(addr_reg);
    }

    /// `1e 06`: stores the low byte of a register at a register-held address.
    pub fn store_byte(&mut self, addr_reg: u8, value_reg: u8) {
        self.byte(0x1e);
        self.byte(0x06);
        self.reg(addr_reg);
        self.reg(value_reg);
    }

    /// `1e 07`: stores the low word of a register at a register-held address.
    pub fn store_word(&mut self, addr_reg: u8, value_reg: u8) {
        self.byte(0x1e);
        self.byte(0x07);
        self.reg(addr_reg);
        self.reg(value_reg);
    }

    /// `1e 09`: points the burst instructions that follow at a register-held
    /// address.
    pub fn set_pointer(&mut self, addr_reg: u8) {
        self.byte(0x1e);
        self.byte(0x09);
        self.reg(addr_reg);
    }

    /// `1e 10`: sets the repeat count of the burst instruction that follows.
    pub fn set_repeat(&mut self, count_reg: u8) {
        self.byte(0x1e);
        self.byte(0x10);
        self.reg(count_reg);
    }

    /// `1e 0b`: burst-writes words from the download data at the pointer.
    pub fn burst_write_words(&mut self, count_reg: u8) {
        self.byte(0x1e);
        self.byte(0x0b);
        self.reg(count_reg);
    }

    /// `1e 0c`: burst-reads bytes from the pointer into the output.
    pub fn burst_read_bytes(&mut self, count_reg: u8) {
        self.byte(0x1e);
        self.byte(0x0c);
        self.reg(count_reg);
    }

    /// `1e 0d`: burst-reads words from the pointer into the output.
    pub fn burst_read_words(&mut self, count_reg: u8) {
        self.byte(0x1e);
        self.byte(0x0d);
        self.reg(count_reg);
    }

    /// `1e 0e`: reads a UPDI control and status register into the result
    /// variable.
    pub fn read_cs(&mut self, addr_reg: u8) {
        self.byte(0x1e);
        self.byte(0x0e);
        self.reg(addr_reg);
    }

    /// `1e 0f`: writes a UPDI control and status register from the low byte of
    /// a register.
    pub fn write_cs(&mut self, addr_reg: u8, value_reg: u8) {
        self.byte(0x1e);
        self.byte(0x0f);
        self.reg(addr_reg);
        self.reg(value_reg);
    }

    /// `1e 11`: runs the UPDI key handshake. The bytes appended by [`Emitter::tag`]
    /// carry the key, most significant group first.
    pub fn send_key(&mut self, size_reg: u8) {
        self.byte(0x1e);
        self.byte(0x11);
        self.reg(size_reg);
    }

    /// `1e 14`: sets the UPDI clock of the tool from a register.
    pub fn set_speed(&mut self, reg: u8) {
        self.byte(0x1e);
        self.byte(0x14);
        self.reg(reg);
    }

    /// `1e 15`: reads facts about the target. The second register ends up
    /// holding the on-chip debug version, which is what the program counter
    /// and breakpoint scripts branch on.
    pub fn read_target_info(&mut self, scratch_reg: u8, version_reg: u8) {
        self.byte(0x1e);
        self.byte(0x15);
        self.reg(scratch_reg);
        self.reg(version_reg);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Reference bytes are cross-checked against the disassembly in
    /// `scratch/avr/vm/all_scripts.txt`.

    #[test]
    fn immediate_loads_match_the_disassembly() {
        let mut e = Emitter::new();
        // ENTER_DEBUG_MODE offset 0: 9b 00 08.
        e.load_imm8(0, 8);
        // GET_DEVICE_ID offset 10: 9c 01 03 00.
        e.load_imm16(1, 3);
        // GET_DEVICE_ID offset 1: 90 00 00 11 00 00.
        e.load_imm32(0, 0x1100);

        assert_eq!(
            e.finish(),
            [
                0x9b, 0x00, 0x08, //
                0x9c, 0x01, 0x03, 0x00, //
                0x90, 0x00, 0x00, 0x11, 0x00, 0x00,
            ]
        );
    }

    #[test]
    fn a_control_space_write_encodes_as_stcs() {
        let mut e = Emitter::new();
        // ENTER_PROG_MODE offset 47: stcs 0 1 with r0 = 8, r1 = 0x59.
        e.load_imm8(0, 8);
        e.load_imm8(1, 0x59);
        e.write_cs(0, 1);

        assert_eq!(
            e.finish(),
            [0x9b, 0x00, 0x08, 0x9b, 0x01, 0x59, 0x1e, 0x0f, 0x00, 0x01]
        );
    }

    #[test]
    fn a_conditional_jump_carries_an_absolute_target() {
        // GET_HALT_STATUS offset 17 has `fe 01 01 00 00 00 24 00`: the fused
        // compare-and-branch jumps to the absolute offset 0x24.
        let mut e = Emitter::new();
        let target = e.label();
        e.and_imm(1, 1);
        e.jump_if_eq_imm(1, 1, target);
        e.bind(target);
        let bytes = e.finish();

        assert_eq!(
            &bytes[6..14],
            &[0xfe, 0x01, 0x01, 0x00, 0x00, 0x00, 0x0e, 0x00]
        );
    }

    #[test]
    fn a_loop_pair_encodes_as_ad_ae() {
        let mut e = Emitter::new();
        // READ_MEM8: loop 1, body, endl.
        let loop_id = e.loop_start(1);
        e.read_word(0);
        e.send_result8();
        e.loop_end(loop_id);

        assert_eq!(e.finish(), [0xad, 0x01, 0x1e, 0x03, 0x00, 0x9f, 0xae]);
    }

    #[test]
    fn a_poll_encodes_mask_value_timeout() {
        let mut e = Emitter::new();
        // ENTER_PROG_MODE offset 114: poll 0x20 0x20 0xa.
        e.poll(0x20, 0x20, 10);

        assert_eq!(
            e.finish(),
            [
                0xa5, 0x20, 0x00, 0x00, 0x00, 0x20, 0x00, 0x00, 0x00, 0x0a, 0x00
            ]
        );
    }

    #[test]
    #[should_panic(expected = "label never bound")]
    fn an_unbound_label_is_rejected() {
        let mut e = Emitter::new();
        let target = e.label();
        e.jump(target);
        e.finish();
    }
}

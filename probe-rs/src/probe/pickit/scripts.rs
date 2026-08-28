//! The script blobs the tool runs, and where they come from.
//!
//! The tool has no built-in operations. Every operation is a blob of bytecode
//! that Microchip publishes per device in the tool pack, next to a
//! `ri4command` value that hints at the transfer type the blob was written for.
//!
//! probe-rs does not ship those blobs yet. [`ScriptSource`] is the seam where
//! they arrive. An implementation can embed them, index a downloaded pack from
//! a user cache, or hand out fixtures in a test.

use std::collections::HashMap;

/// Bit 31 of `ri4command` marks a script that moves data over the data pipe.
const RI4_DATA_TRANSFER: u32 = 0x8000_0000;

/// One script blob for one operation on one device.
///
/// # Examples
///
/// ```
/// use probe_rs::probe::pickit::Script;
///
/// // ReadMem8_UPDI for an AVR128DA64, whose prologue loads two word parameters.
/// let script = Script::new(0x8000_140e, vec![0x91, 0x00, 0x91, 0x01]);
/// assert!(script.is_data_transfer());
/// ```
#[derive(Clone, Debug)]
pub struct Script {
    ri4command: u32,
    bytes: Vec<u8>,
}

impl Script {
    /// Creates a script from its `ri4command` value and its bytecode.
    pub fn new(ri4command: u32, bytes: Vec<u8>) -> Self {
        Self { ri4command, bytes }
    }

    /// The bytecode the tool runs.
    pub fn bytes(&self) -> &[u8] {
        &self.bytes
    }

    /// The `ri4command` value from the tool pack.
    ///
    /// The host picks the message type itself, so this is a cross-check rather
    /// than something the driver depends on.
    pub fn ri4command(&self) -> u32 {
        self.ri4command
    }

    /// True when `ri4command` marks the script as a data endpoint transfer.
    pub fn is_data_transfer(&self) -> bool {
        self.ri4command & RI4_DATA_TRANSFER != 0
    }
}

/// The scripts this driver uses, named as the tool pack names them.
///
/// The pack holds many more. Most run-control scripts are redundant, because
/// `ReadMem8` and `WriteMem8` reach any data-space address and the on-chip
/// debug block is memory mapped, so run control is driven through memory
/// access instead of through per-operation scripts.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum ScriptName {
    /// Opens a UPDI session for programming. The only safe first operation.
    EnterProgMode,
    /// Closes a programming session.
    ExitProgMode,
    /// Opens a UPDI session for debugging.
    EnterDebugMode,
    /// Closes a debug session.
    ExitDebugMode,
    /// Sets the UPDI clock of the tool in kHz. Takes one word.
    SetSpeed,
    /// Reads the device signature. Returns its result inline.
    GetDeviceId,
    /// Reads the System Information Block.
    ReadSib,
    /// Erases flash, EEPROM, and the lock bits.
    EraseChip,
    /// Reads data space one byte at a time. Takes an address and a length.
    ReadMem8,
    /// Writes data space one byte at a time. Takes an address and a length.
    WriteMem8,
    /// Reads data space one word at a time. Takes an address and a length.
    ReadMem16,
    /// Writes data space one word at a time. Takes an address and a length.
    WriteMem16,
    /// Reads flash. Takes an address and a length.
    ReadProgmem,
    /// Writes flash. Takes an address and a length.
    WriteProgmem,
    /// Reads EEPROM. Takes an address and a length.
    ReadDataEeMem,
    /// Writes EEPROM. Takes an address and a length.
    WriteDataEeMem,
    /// Reads the configuration memory. Takes an address and a length.
    ReadConfigmem,
    /// Writes the configuration memory. Takes an address and a length.
    WriteConfigmem,
    /// Reads the user row. Takes an address and a length.
    ReadIdMem,
    /// Writes the user row. Takes an address and a length.
    WriteIdMem,
    /// Reads a UPDI control and status register. Takes one byte.
    ReadCsReg,
    /// Writes a UPDI control and status register. Takes two bytes.
    WriteCsReg,
}

impl ScriptName {
    /// The name of the script in the tool pack.
    ///
    /// # Examples
    ///
    /// ```
    /// use probe_rs::probe::pickit::ScriptName;
    ///
    /// assert_eq!(ScriptName::EnterProgMode.as_str(), "EnterProgMode_UPDI");
    /// ```
    pub fn as_str(&self) -> &'static str {
        match self {
            ScriptName::EnterProgMode => "EnterProgMode_UPDI",
            ScriptName::ExitProgMode => "ExitProgMode_UPDI",
            ScriptName::EnterDebugMode => "EnterDebugMode_UPDI",
            ScriptName::ExitDebugMode => "ExitDebugMode_UPDI",
            ScriptName::SetSpeed => "SetSpeed_UPDI",
            ScriptName::GetDeviceId => "GetDeviceID_UPDI",
            ScriptName::ReadSib => "ReadSIB_UPDI",
            ScriptName::EraseChip => "EraseChip_UPDI",
            ScriptName::ReadMem8 => "ReadMem8_UPDI",
            ScriptName::WriteMem8 => "WriteMem8_UPDI",
            ScriptName::ReadMem16 => "ReadMem16_UPDI",
            ScriptName::WriteMem16 => "WriteMem16_UPDI",
            ScriptName::ReadProgmem => "ReadProgmem_UPDI",
            ScriptName::WriteProgmem => "WriteProgmem_UPDI",
            ScriptName::ReadDataEeMem => "ReadDataEEmem_UPDI",
            ScriptName::WriteDataEeMem => "WriteDataEEmem_UPDI",
            ScriptName::ReadConfigmem => "ReadConfigmem_UPDI",
            ScriptName::WriteConfigmem => "WriteConfigmem_UPDI",
            ScriptName::ReadIdMem => "ReadIDmem_UPDI",
            ScriptName::WriteIdMem => "WriteIDmem_UPDI",
            ScriptName::ReadCsReg => "ReadCSreg_UPDI",
            ScriptName::WriteCsReg => "WriteCSreg_UPDI",
        }
    }
}

impl std::fmt::Display for ScriptName {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(self.as_str())
    }
}

/// Where the driver looks up the script blobs for one target device.
///
/// probe-rs does not ship Microchip's blobs, so nothing in this crate
/// implements this trait yet. Provide an implementation to give the driver
/// something to run.
pub trait ScriptSource: Send + std::fmt::Debug {
    /// Returns the script for `name`, or `None` when this source does not have it.
    fn script(&self, name: ScriptName) -> Option<&Script>;
}

/// A [`ScriptSource`] backed by a map.
///
/// # Examples
///
/// ```
/// use probe_rs::probe::pickit::{Script, ScriptName, ScriptSource, ScriptTable};
///
/// let mut table = ScriptTable::new();
/// table.insert(ScriptName::GetDeviceId, Script::new(0x0000_1505, vec![0x1e, 0x00]));
///
/// assert!(table.script(ScriptName::GetDeviceId).is_some());
/// assert!(table.script(ScriptName::EraseChip).is_none());
/// ```
#[derive(Clone, Debug, Default)]
pub struct ScriptTable {
    scripts: HashMap<ScriptName, Script>,
}

impl ScriptTable {
    /// Creates an empty table.
    pub fn new() -> Self {
        Self::default()
    }

    /// Adds a script, replacing any script already stored under that name.
    pub fn insert(&mut self, name: ScriptName, script: Script) {
        self.scripts.insert(name, script);
    }
}

impl ScriptSource for ScriptTable {
    fn script(&self, name: ScriptName) -> Option<&Script> {
        self.scripts.get(&name)
    }
}

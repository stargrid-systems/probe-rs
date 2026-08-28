//! The script blobs the tool runs, and where they come from.
//!
//! The tool has no built-in operations. Every operation is a blob of bytecode
//! that Microchip publishes per device in the tool pack, next to a
//! `ri4command` value that hints at the transfer type the blob was written for.
//!
//! [`AvrFamily`] is the built-in source. It carries the blobs for the two AVR
//! families this driver supports. [`ScriptSource`] is the seam, so another
//! implementation can index a downloaded pack from a user cache or hand out
//! fixtures in a test.

use std::borrow::Cow;
use std::collections::HashMap;

use super::blobs;

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
    bytes: Cow<'static, [u8]>,
}

impl Script {
    /// Creates a script from its `ri4command` value and its bytecode.
    pub fn new(ri4command: u32, bytes: Vec<u8>) -> Self {
        Self {
            ri4command,
            bytes: Cow::Owned(bytes),
        }
    }

    /// Creates a script that borrows bytecode compiled into the binary.
    ///
    /// This is what the built-in tables use, so they cost no allocation.
    ///
    /// # Examples
    ///
    /// ```
    /// use probe_rs::probe::pickit::Script;
    ///
    /// static EXIT_DEBUG_MODE: Script = Script::from_static(0x0000_0201, &[0x00, 0x00]);
    /// assert!(!EXIT_DEBUG_MODE.is_data_transfer());
    /// ```
    pub const fn from_static(ri4command: u32, bytes: &'static [u8]) -> Self {
        Self {
            ri4command,
            bytes: Cow::Borrowed(bytes),
        }
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
/// This is every UPDI script the pack ships for the supported parts. Run
/// control has its own scripts here rather than going through raw writes to
/// the memory-mapped debug block, because these are the ones that were proven
/// on hardware.
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
    /// Enters programming mode with a high-voltage pulse.
    EnterProgModeHvSp,
    /// Enters programming mode with a high-voltage pulse and a reset.
    EnterProgModeHvSpRst,
    /// Enters programming mode with a high voltage pulse while the operator
    /// power-cycles the target.
    EnterProgModeHvUpt,
    /// Enters debug mode with a high-voltage pulse.
    EnterDebugModeHvSp,
    /// Enters debug mode with a high-voltage pulse and a reset.
    EnterDebugModeHvSpRst,
    /// Enters debug mode with a high-voltage pulse while the operator
    /// power-cycles the target.
    EnterDebugModeHvUpt,
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
    /// Reads a fuse. Takes an address and a length.
    ReadConfigmemFuse,
    /// Writes a fuse. Takes an address and a length.
    WriteConfigmemFuse,
    /// Reads the lock bits. Takes an address and a length.
    ReadConfigmemLock,
    /// Writes the lock bits. Takes an address and a length.
    WriteConfigmemLock,
    /// Reads the user row. Takes an address and a length.
    ReadIdMem,
    /// Writes the user row. Takes an address and a length.
    WriteIdMem,
    /// Reads a UPDI control and status register. Takes one byte.
    ReadCsReg,
    /// Writes a UPDI control and status register. Takes two bytes.
    WriteCsReg,
    /// Stops the core.
    Halt,
    /// Starts the core.
    Run,
    /// Executes one instruction.
    SingleStep,
    /// Reports whether the core is stopped. Returns its result inline.
    GetHaltStatus,
    /// Reads the program counter. Returns its result inline.
    GetPc,
    /// Writes the program counter. Takes the new value.
    SetPc,
    /// Arms a hardware breakpoint. Takes the address.
    SetHwBp,
    /// Disarms a hardware breakpoint.
    ClearHwBp,
    /// Resets the core and stops it at the reset vector.
    DebugReset,
    /// Asserts reset and keeps it asserted.
    ///
    /// Do not run this on its own. Unlike every script that is known to work,
    /// its bytecode has no `0x5a` terminator, and running it hangs the tool.
    HoldInReset,
    /// Releases a reset asserted by [`ScriptName::HoldInReset`].
    ///
    /// Do not run this on its own, for the same reason as
    /// [`ScriptName::HoldInReset`]. Use [`ScriptName::DebugReset`] to get a
    /// part out of the reset that opening a session asserts.
    ReleaseFromReset,
}

impl ScriptName {
    /// Every script name, in declaration order.
    pub const ALL: [ScriptName; 43] = [
        ScriptName::EnterProgMode,
        ScriptName::ExitProgMode,
        ScriptName::EnterDebugMode,
        ScriptName::ExitDebugMode,
        ScriptName::EnterProgModeHvSp,
        ScriptName::EnterProgModeHvSpRst,
        ScriptName::EnterProgModeHvUpt,
        ScriptName::EnterDebugModeHvSp,
        ScriptName::EnterDebugModeHvSpRst,
        ScriptName::EnterDebugModeHvUpt,
        ScriptName::SetSpeed,
        ScriptName::GetDeviceId,
        ScriptName::ReadSib,
        ScriptName::EraseChip,
        ScriptName::ReadMem8,
        ScriptName::WriteMem8,
        ScriptName::ReadMem16,
        ScriptName::WriteMem16,
        ScriptName::ReadProgmem,
        ScriptName::WriteProgmem,
        ScriptName::ReadDataEeMem,
        ScriptName::WriteDataEeMem,
        ScriptName::ReadConfigmem,
        ScriptName::WriteConfigmem,
        ScriptName::ReadConfigmemFuse,
        ScriptName::WriteConfigmemFuse,
        ScriptName::ReadConfigmemLock,
        ScriptName::WriteConfigmemLock,
        ScriptName::ReadIdMem,
        ScriptName::WriteIdMem,
        ScriptName::ReadCsReg,
        ScriptName::WriteCsReg,
        ScriptName::Halt,
        ScriptName::Run,
        ScriptName::SingleStep,
        ScriptName::GetHaltStatus,
        ScriptName::GetPc,
        ScriptName::SetPc,
        ScriptName::SetHwBp,
        ScriptName::ClearHwBp,
        ScriptName::DebugReset,
        ScriptName::HoldInReset,
        ScriptName::ReleaseFromReset,
    ];

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
            ScriptName::EnterProgModeHvSp => "EnterProgModeHvSp_UPDI",
            ScriptName::EnterProgModeHvSpRst => "EnterProgModeHvSpRst_UPDI",
            ScriptName::EnterProgModeHvUpt => "EnterProgModeHvUpt_UPDI",
            ScriptName::EnterDebugModeHvSp => "EnterDebugModeHvSp_UPDI",
            ScriptName::EnterDebugModeHvSpRst => "EnterDebugModeHvSpRst_UPDI",
            ScriptName::EnterDebugModeHvUpt => "EnterDebugModeHvUpt_UPDI",
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
            ScriptName::ReadConfigmemFuse => "ReadConfigmemFuse_UPDI",
            ScriptName::WriteConfigmemFuse => "WriteConfigmemFuse_UPDI",
            ScriptName::ReadConfigmemLock => "ReadConfigmemLock_UPDI",
            ScriptName::WriteConfigmemLock => "WriteConfigmemLock_UPDI",
            ScriptName::ReadIdMem => "ReadIDmem_UPDI",
            ScriptName::WriteIdMem => "WriteIDmem_UPDI",
            ScriptName::ReadCsReg => "ReadCSreg_UPDI",
            ScriptName::WriteCsReg => "WriteCSreg_UPDI",
            ScriptName::Halt => "Halt_UPDI",
            ScriptName::Run => "Run_UPDI",
            ScriptName::SingleStep => "SingleStep_UPDI",
            ScriptName::GetHaltStatus => "GetHaltStatus_UPDI",
            ScriptName::GetPc => "GetPC_UPDI",
            ScriptName::SetPc => "SetPC_UPDI",
            ScriptName::SetHwBp => "SetHWBP_UPDI",
            ScriptName::ClearHwBp => "ClearHWBP_UPDI",
            ScriptName::DebugReset => "DebugReset_UPDI",
            ScriptName::HoldInReset => "HoldInReset_UPDI",
            ScriptName::ReleaseFromReset => "ReleaseFromReset_UPDI",
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
/// [`AvrFamily`] implements this with the blobs that ship with probe-rs.
/// Implement it yourself to run blobs from somewhere else, such as a tool pack
/// the user downloaded.
pub trait ScriptSource: Send + std::fmt::Debug {
    /// Returns the script for `name`, or `None` when this source does not have it.
    fn script(&self, name: ScriptName) -> Option<&Script>;
}

/// The AVR families probe-rs ships script blobs for.
///
/// Two tables cover every supported part. Eleven of the 43 scripts differ
/// between the families, because the two use a different NVM controller
/// generation and a different on-chip debug version. The rest are identical.
///
/// # Examples
///
/// ```no_run
/// use std::str::FromStr;
///
/// use probe_rs::probe::pickit::{AvrFamily, Pickit};
/// use probe_rs::probe::DebugProbeSelector;
///
/// # fn main() -> Result<(), Box<dyn std::error::Error>> {
/// let selector = DebugProbeSelector::from_str("04d8:9054")?;
/// let mut pickit = Pickit::open(&selector)?;
/// pickit.set_scripts(Box::new(AvrFamily::Dx));
/// # Ok(())
/// # }
/// ```
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum AvrFamily {
    /// AVR Dx series parts, which report on-chip debug version 1.
    Dx,
    /// tiny 0-series and 1-series parts, which report on-chip debug version 0.
    Tiny0,
}

impl AvrFamily {
    /// Picks the script table that covers a device, by its name.
    ///
    /// The two tables differ only in the NVM and debug scripts, and that split
    /// follows the part family. The target description has no field that says
    /// which family a part belongs to, so the name is what there is to go on.
    ///
    /// # Examples
    ///
    /// ```
    /// use probe_rs::probe::pickit::AvrFamily;
    ///
    /// assert_eq!(AvrFamily::for_device("AVR128DA64"), Some(AvrFamily::Dx));
    /// assert_eq!(AvrFamily::for_device("ATtiny406"), Some(AvrFamily::Tiny0));
    /// assert_eq!(AvrFamily::for_device("STM32F103"), None);
    /// ```
    pub fn for_device(name: &str) -> Option<Self> {
        let name = name.to_ascii_lowercase();

        if name.starts_with("attiny") {
            Some(AvrFamily::Tiny0)
        } else if name.starts_with("avr") {
            Some(AvrFamily::Dx)
        } else {
            None
        }
    }

    /// The address the flash scripts give to the first byte of flash.
    ///
    /// The flash scripts do not address flash from zero. They place it above
    /// the data space, so the caller has to add this base to a flash address
    /// before handing it to `ReadProgmem` or `WriteProgmem`.
    ///
    /// The value for [`AvrFamily::Dx`] is `0x800000` and is verified on an
    /// AVR128DA64.
    ///
    /// The value for [`AvrFamily::Tiny0`] is **not verified on hardware**. The
    /// tinyAVR 0-series and 1-series map their whole flash into the data space
    /// at `0x8000`, which is why the mapped window covers all 4 KiB on an
    /// ATtiny406 but only 32 KiB of 128 KiB on a Dx part. A base of `0x8000` is
    /// the plausible consequence, but nobody has read a known byte both ways
    /// and compared. Treat it as a guess until somebody has.
    ///
    /// # Examples
    ///
    /// ```
    /// use probe_rs::probe::pickit::AvrFamily;
    ///
    /// assert_eq!(AvrFamily::Dx.flash_base(), 0x80_0000);
    /// assert_eq!(AvrFamily::Tiny0.flash_base(), 0x8000);
    /// ```
    pub fn flash_base(self) -> u64 {
        match self {
            // Verified on hardware on an AVR128DA64.
            AvrFamily::Dx => 0x0080_0000,
            // Unverified. See the doc comment.
            AvrFamily::Tiny0 => 0x0000_8000,
        }
    }

    /// The flash page size of the parts in this family, in bytes.
    ///
    /// `WriteProgmem` erases and programs whole pages, and `ReadProgmem`
    /// steps a page at a time. Neither takes the page size as a parameter,
    /// because it is baked into the bytecode as an immediate at offset 13.
    /// The values here are the ones read out of those blobs, so the host and
    /// the tool cannot disagree.
    ///
    /// The target description has nowhere to put this. A page size lives in a
    /// flash algorithm, and these parts have none.
    ///
    /// # Examples
    ///
    /// ```
    /// use probe_rs::probe::pickit::AvrFamily;
    ///
    /// assert_eq!(AvrFamily::Dx.flash_page_size(), 512);
    /// assert_eq!(AvrFamily::Tiny0.flash_page_size(), 64);
    /// ```
    pub fn flash_page_size(self) -> u32 {
        match self {
            AvrFamily::Dx => 512,
            AvrFamily::Tiny0 => 64,
        }
    }
}

impl ScriptSource for AvrFamily {
    fn script(&self, name: ScriptName) -> Option<&Script> {
        Some(match self {
            AvrFamily::Dx => blobs::dx(name),
            AvrFamily::Tiny0 => blobs::tiny0(name),
        })
    }
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

#[cfg(test)]
mod tests {
    use super::*;

    /// Both tables must answer for every name, or a script is missing.
    #[test]
    fn every_name_resolves_in_both_families() {
        for name in ScriptName::ALL {
            for family in [AvrFamily::Dx, AvrFamily::Tiny0] {
                let script = family.script(name).unwrap();
                assert!(!script.bytes().is_empty(), "{family:?} {name} is empty");
            }
        }
    }

    #[test]
    fn families_agree_on_ri4command() {
        for name in ScriptName::ALL {
            let dx = AvrFamily::Dx.script(name).unwrap();
            let tiny0 = AvrFamily::Tiny0.script(name).unwrap();

            assert_eq!(dx.ri4command(), tiny0.ri4command(), "{name}");
        }
    }

    /// The top two bits of `ri4command` give the transfer direction. Only the
    /// read and write scripts move bulk data, so only those may set them.
    #[test]
    fn ri4command_direction_matches_the_operation() {
        for name in ScriptName::ALL {
            let script = AvrFamily::Dx.script(name).unwrap();
            let pack_name = name.as_str();

            let expected = if pack_name.starts_with("Read") {
                0x8000_0000
            } else if pack_name.starts_with("Write") {
                0xc000_0000
            } else {
                0
            };

            assert_eq!(script.ri4command() & 0xc000_0000, expected, "{name}");
            assert_eq!(script.is_data_transfer(), expected != 0, "{name}");
        }
    }

    #[test]
    fn the_supported_devices_map_to_a_family() {
        assert_eq!(AvrFamily::for_device("AVR128DA64"), Some(AvrFamily::Dx));
        assert_eq!(AvrFamily::for_device("AVR128DB64"), Some(AvrFamily::Dx));
        assert_eq!(AvrFamily::for_device("ATtiny406"), Some(AvrFamily::Tiny0));
        assert_eq!(AvrFamily::for_device("attiny1614"), Some(AvrFamily::Tiny0));
        assert_eq!(AvrFamily::for_device("nRF52840"), None);
    }

    /// The two families place flash somewhere different, so pin each one.
    #[test]
    fn each_family_pins_its_own_flash_base() {
        assert_eq!(AvrFamily::Dx.flash_base(), 0x0080_0000);
        assert_eq!(AvrFamily::Tiny0.flash_base(), 0x0000_8000);
    }

    /// The page size the host reports has to be the one the tool uses, and the
    /// tool takes it from an immediate in `ReadProgmem`. Read that immediate
    /// back out of the blob so the two cannot drift apart.
    #[test]
    fn the_page_size_matches_the_script_bytecode() {
        for family in [AvrFamily::Dx, AvrFamily::Tiny0] {
            let bytes = family.script(ScriptName::ReadProgmem).unwrap().bytes();

            // `0x90 0x0f <u32 LE>` loads register 15 with the page size.
            assert_eq!(&bytes[11..13], &[0x90, 0x0f], "{family:?}");
            let immediate = u32::from_le_bytes(bytes[13..17].try_into().unwrap());

            assert_eq!(immediate, family.flash_page_size(), "{family:?}");
        }
    }

    /// A page size that is not a power of two would break the page alignment
    /// the flash driver does with a mask.
    #[test]
    fn page_sizes_are_powers_of_two() {
        for family in [AvrFamily::Dx, AvrFamily::Tiny0] {
            assert!(family.flash_page_size().is_power_of_two(), "{family:?}");
        }
    }

    #[test]
    fn names_are_unique() {
        let mut names: Vec<&str> = ScriptName::ALL.iter().map(|n| n.as_str()).collect();
        names.sort_unstable();
        let count = names.len();
        names.dedup();

        assert_eq!(names.len(), count);
    }
}

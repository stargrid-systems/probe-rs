//! Communication with an AVR target over UPDI.
//!
//! This is the layer between probe-rs and the PICkit transport. It opens and
//! closes the UPDI session, identifies the part, and moves bytes to and from
//! the two AVR address spaces.
//!
//! # Address convention
//!
//! The two sides of this module disagree about where memory lives, and this is
//! the only place that translates.
//!
//! probe-rs uses the avr-gcc convention everywhere above this module, because
//! that is what ELF and DWARF files for AVR use. Flash starts at `0x0` and the
//! data space starts at [`DATA_SPACE_OFFSET`], so SRAM on an AVR128DA64 is at
//! `0x804000`.
//!
//! The PICkit scripts use the inverse. The data space is at its native chip
//! address, so the same SRAM is at `0x4000`, and flash carries a base offset
//! instead.
//!
//! So both directions move an address, and both move it the other way round
//! from what the name suggests. [`to_chip_data_address`] subtracts
//! [`DATA_SPACE_OFFSET`] and [`to_tool_flash_address`] adds the flash base of
//! the part family.
//!
//! The data space offset is the same everywhere. The flash base is not, which
//! is why it comes from [`AvrFamily::flash_base`] rather than from a constant
//! here.

use std::fmt;
use std::sync::Once;

use crate::probe::DebugProbeError;
use crate::probe::pickit::{AvrFamily, Params, Pickit, ScriptName, SessionState};

/// Where the avr-gcc convention puts the AVR data space.
///
/// Flash sits below this address and the data space sits above it. The chip
/// itself puts the data space at zero, so this is also the constant that has to
/// come off before an address goes to the probe.
pub const DATA_SPACE_OFFSET: u64 = 0x0080_0000;

/// The highest data-space address the tool can be given.
///
/// Script parameters are 32-bit, so this bounds the translated address rather
/// than the AVR address space, which is far smaller.
const MAX_TOOL_ADDRESS: u64 = u32::MAX as u64;

/// Converts a probe-rs address in the data space to the address the chip uses.
///
/// probe-rs places the data space at [`DATA_SPACE_OFFSET`] and the chip places
/// it at zero, so this subtracts the offset.
///
/// # Examples
///
/// ```
/// use probe_rs::architecture::avr::communication_interface::to_chip_data_address;
///
/// // SRAM of an AVR128DA64.
/// assert_eq!(to_chip_data_address(0x80_4000).unwrap(), 0x4000);
/// // An address below the data space is flash, which this cannot translate.
/// assert!(to_chip_data_address(0x1000).is_err());
/// ```
pub fn to_chip_data_address(address: u64) -> Result<u32, AvrError> {
    let chip = address
        .checked_sub(DATA_SPACE_OFFSET)
        .ok_or(AvrError::NotInDataSpace(address))?;

    if chip > MAX_TOOL_ADDRESS {
        return Err(AvrError::NotInDataSpace(address));
    }

    Ok(chip as u32)
}

/// Warns once that the tiny flash base has never been checked on a part.
static TINY_FLASH_BASE_UNVERIFIED: Once = Once::new();

/// Converts a probe-rs flash address to the address the tool scripts use.
///
/// probe-rs places flash at zero and the flash scripts place it at
/// [`AvrFamily::flash_base`], so this adds that base.
///
/// The base for [`AvrFamily::Tiny0`] has never been checked against a part, so
/// the first call for a tiny target logs a warning.
///
/// # Examples
///
/// ```
/// use probe_rs::architecture::avr::communication_interface::to_tool_flash_address;
/// use probe_rs::probe::pickit::AvrFamily;
///
/// assert_eq!(to_tool_flash_address(AvrFamily::Dx, 0).unwrap(), 0x80_0000);
/// assert_eq!(to_tool_flash_address(AvrFamily::Dx, 0x1_0000).unwrap(), 0x81_0000);
/// assert_eq!(to_tool_flash_address(AvrFamily::Tiny0, 0).unwrap(), 0x8000);
/// // An address at or above the data space offset is not in flash.
/// assert!(to_tool_flash_address(AvrFamily::Dx, 0x80_4000).is_err());
/// ```
pub fn to_tool_flash_address(family: AvrFamily, address: u64) -> Result<u32, AvrError> {
    if address >= DATA_SPACE_OFFSET {
        return Err(AvrError::NotInFlash(address));
    }

    if family == AvrFamily::Tiny0 {
        TINY_FLASH_BASE_UNVERIFIED.call_once(|| {
            tracing::warn!(
                "The flash base offset for tinyAVR 0/1-series parts is a guess. \
                 It has never been read back from a part, so flash access may \
                 land in the wrong place."
            );
        });
    }

    let tool = address + family.flash_base();

    if tool > MAX_TOOL_ADDRESS {
        return Err(AvrError::NotInFlash(address));
    }

    Ok(tool as u32)
}

/// An error that happened while talking to an AVR target.
#[derive(thiserror::Error, Debug, docsplay::Display)]
pub enum AvrError {
    /// An error originating from the debug probe occurred.
    DebugProbe(#[from] DebugProbeError),

    /// The connected target is not an AVR device.
    NoAvrTarget,

    /// Address {0:#x} is not in the AVR data space.
    ///
    /// probe-rs uses the avr-gcc convention, which puts the data space at
    /// `0x800000`. Anything below that is flash.
    #[ignore_extra_doc_attributes]
    NotInDataSpace(u64),

    /// Address {0:#x} is not in AVR flash.
    ///
    /// probe-rs uses the avr-gcc convention, which puts flash below `0x800000`.
    /// Anything at or above that is the data space.
    #[ignore_extra_doc_attributes]
    NotInFlash(u64),

    /// A word access needs an even address and an even length, but got address {address:#x} and length {length}.
    UnalignedWordAccess {
        /// The address of the access.
        address: u64,
        /// The length of the access in bytes.
        length: usize,
    },

    /// The device signature reply was {0} bytes, which is too short to decode.
    ShortDeviceId(usize),
}

impl From<AvrError> for crate::Error {
    fn from(err: AvrError) -> Self {
        match err {
            AvrError::DebugProbe(err) => err.into(),
            other => crate::Error::Avr(other),
        }
    }
}

/// The identity a part reports through `GetDeviceId`.
///
/// # Examples
///
/// ```
/// use probe_rs::architecture::avr::communication_interface::DeviceId;
///
/// // What an AVR128DA64 answers.
/// let id = DeviceId::from_bytes(&[0x1e, 0x97, 0x07, 0x18]).unwrap();
/// assert_eq!(id.signature(), 0x1e_9707);
/// assert_eq!(id.revision, 0x18);
/// ```
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct DeviceId {
    /// The three signature bytes, most significant first.
    pub signature: [u8; 3],
    /// The device revision byte.
    pub revision: u8,
}

impl DeviceId {
    /// Decodes the four bytes the tool returns.
    pub fn from_bytes(bytes: &[u8]) -> Result<Self, AvrError> {
        if bytes.len() < 4 {
            return Err(AvrError::ShortDeviceId(bytes.len()));
        }

        Ok(Self {
            signature: [bytes[0], bytes[1], bytes[2]],
            revision: bytes[3],
        })
    }

    /// The signature bytes packed into one number, as data sheets print it.
    pub fn signature(&self) -> u32 {
        u32::from_be_bytes([0, self.signature[0], self.signature[1], self.signature[2]])
    }
}

impl fmt::Display for DeviceId {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "{:02x}:{:02x}:{:02x} rev {:#04x}",
            self.signature[0], self.signature[1], self.signature[2], self.revision
        )
    }
}

/// The state of an AVR debug interface that outlives a single attach.
///
/// The caller owns this, the same way it owns the Xtensa and RISC-V interface
/// state. It carries the choice of script table, which has to be made before
/// the first script runs, and the identity once it has been read.
#[derive(Debug)]
pub struct AvrDebugInterfaceState {
    family: AvrFamily,
    speed_khz: Option<u32>,
    device_id: Option<DeviceId>,
}

impl AvrDebugInterfaceState {
    /// Creates the state for a target of the given family.
    ///
    /// # Examples
    ///
    /// ```
    /// use probe_rs::architecture::avr::communication_interface::AvrDebugInterfaceState;
    /// use probe_rs::probe::pickit::AvrFamily;
    ///
    /// let state = AvrDebugInterfaceState::new(AvrFamily::Dx);
    /// assert_eq!(state.family(), AvrFamily::Dx);
    /// ```
    pub fn new(family: AvrFamily) -> Self {
        Self {
            family,
            speed_khz: None,
            device_id: None,
        }
    }

    /// The script table this interface runs.
    pub fn family(&self) -> AvrFamily {
        self.family
    }

    /// The identity read from the part, if it has been read already.
    pub fn device_id(&self) -> Option<DeviceId> {
        self.device_id
    }

    /// Requests a UPDI clock, applied when the session opens.
    ///
    /// The tool only accepts a clock while a session exists, so a value set
    /// before that is remembered rather than sent.
    pub fn set_speed_khz(&mut self, speed_khz: u32) {
        self.speed_khz = Some(speed_khz);
    }
}

/// Talks to an AVR target through a PICkit.
///
/// The interface borrows the tool for as long as it lives, and translates
/// between the probe-rs address convention and the one the tool scripts use.
/// See the module documentation for that translation.
///
/// A fresh tool has no UPDI link. [`AvrCommunicationInterface::enter_programming_mode`]
/// is the only operation that is safe first, and everything else fails until it
/// has run.
pub struct AvrCommunicationInterface<'probe> {
    probe: &'probe mut Pickit,
    state: &'probe mut AvrDebugInterfaceState,
}

impl fmt::Debug for AvrCommunicationInterface<'_> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("AvrCommunicationInterface")
            .field("probe", &self.probe)
            .field("state", &self.state)
            .finish_non_exhaustive()
    }
}

impl<'probe> AvrCommunicationInterface<'probe> {
    /// Builds the interface and installs the script table for the target.
    pub fn new(probe: &'probe mut Pickit, state: &'probe mut AvrDebugInterfaceState) -> Self {
        probe.set_scripts(Box::new(state.family));

        Self { probe, state }
    }

    /// Where the tool is in its session lifecycle.
    pub fn session_state(&self) -> SessionState {
        self.probe.state()
    }

    /// Opens a programming session, which is the only safe first operation.
    ///
    /// Any requested UPDI clock is applied afterwards, because the tool only
    /// accepts one once a session exists.
    pub fn enter_programming_mode(&mut self) -> Result<(), AvrError> {
        self.probe.enter_prog_mode().map_err(probe_error)?;

        if let Some(speed_khz) = self.state.speed_khz {
            self.probe.set_speed_khz(speed_khz).map_err(probe_error)?;
        }

        Ok(())
    }

    /// Reads the device signature and revision.
    ///
    /// The answer is cached in the interface state, so repeated calls cost
    /// nothing on the wire.
    pub fn device_id(&mut self) -> Result<DeviceId, AvrError> {
        if let Some(device_id) = self.state.device_id {
            return Ok(device_id);
        }

        // GetDeviceId answers inside the response rather than on the data
        // pipe, so this is a plain command and not a read.
        let response = self
            .probe
            .run(ScriptName::GetDeviceId, Params::Words(&[]))
            .map_err(probe_error)?;

        let device_id = DeviceId::from_bytes(response.inline_data())?;
        self.state.device_id = Some(device_id);

        Ok(device_id)
    }

    /// Reads the data space one byte at a time.
    ///
    /// `address` is a probe-rs address, so it is at or above
    /// [`DATA_SPACE_OFFSET`].
    pub fn read_data_8(&mut self, address: u64, data: &mut [u8]) -> Result<(), AvrError> {
        self.read(ScriptName::ReadMem8, to_chip_data_address(address)?, data)
    }

    /// Writes the data space one byte at a time.
    pub fn write_data_8(&mut self, address: u64, data: &[u8]) -> Result<(), AvrError> {
        self.write(ScriptName::WriteMem8, to_chip_data_address(address)?, data)
    }

    /// Reads the data space one word at a time.
    ///
    /// Both the address and the length have to be even, because the script does
    /// a 16-bit access per step. The length is passed to the script as a byte
    /// count, the same as for the byte-wide script.
    pub fn read_data_16(&mut self, address: u64, data: &mut [u8]) -> Result<(), AvrError> {
        check_word_aligned(address, data.len())?;

        self.read(ScriptName::ReadMem16, to_chip_data_address(address)?, data)
    }

    /// Writes the data space one word at a time.
    ///
    /// The alignment rules of [`AvrCommunicationInterface::read_data_16`] apply.
    pub fn write_data_16(&mut self, address: u64, data: &[u8]) -> Result<(), AvrError> {
        check_word_aligned(address, data.len())?;

        self.write(ScriptName::WriteMem16, to_chip_data_address(address)?, data)
    }

    /// Reads flash.
    ///
    /// `address` is a probe-rs address, so flash starts at zero. Addressing is
    /// linear across the whole device, so this also reaches the part of flash
    /// that is not mapped into the data space.
    pub fn read_flash(&mut self, address: u64, data: &mut [u8]) -> Result<(), AvrError> {
        let tool_address = to_tool_flash_address(self.state.family, address)?;

        self.read(ScriptName::ReadProgmem, tool_address, data)
    }

    /// Writes flash.
    ///
    /// The script erases and programs whole pages, and the page size is baked
    /// into the bytecode rather than passed in. A write that does not cover a
    /// whole page leaves the rest of that page in an undefined state, so the
    /// caller has to align its writes itself.
    pub fn write_flash(&mut self, address: u64, data: &[u8]) -> Result<(), AvrError> {
        let tool_address = to_tool_flash_address(self.state.family, address)?;

        self.write(ScriptName::WriteProgmem, tool_address, data)
    }

    /// Closes the session and leaves the tool ready for a new one.
    pub fn close(&mut self) -> Result<(), AvrError> {
        self.probe.exit().map_err(probe_error)?;
        self.state.device_id = None;

        Ok(())
    }

    fn read(
        &mut self,
        name: ScriptName,
        tool_address: u32,
        data: &mut [u8],
    ) -> Result<(), AvrError> {
        if data.is_empty() {
            return Ok(());
        }

        let params = [tool_address, data.len() as u32];
        let read = self
            .probe
            .read(name, Params::Words(&params), data.len())
            .map_err(probe_error)?;

        let len = read.len().min(data.len());
        data[..len].copy_from_slice(&read[..len]);

        Ok(())
    }

    fn write(&mut self, name: ScriptName, tool_address: u32, data: &[u8]) -> Result<(), AvrError> {
        if data.is_empty() {
            return Ok(());
        }

        let params = [tool_address, data.len() as u32];

        self.probe
            .write(name, Params::Words(&params), data)
            .map_err(probe_error)
    }
}

/// A word-wide script steps two bytes at a time, so both ends have to be even.
fn check_word_aligned(address: u64, length: usize) -> Result<(), AvrError> {
    if !address.is_multiple_of(2) || !length.is_multiple_of(2) {
        return Err(AvrError::UnalignedWordAccess { address, length });
    }

    Ok(())
}

/// Wraps a probe-specific error so this module does not name the probe driver.
fn probe_error(err: impl crate::probe::ProbeError) -> AvrError {
    AvrError::DebugProbe(DebugProbeError::ProbeSpecific(err.into()))
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The three memories of an AVR128DA64, as the target description and the
    /// chip each see them. See `targets.md`.
    const DATA_SPACE: &[(u64, u32)] = &[
        // IO at the very bottom of the data space.
        (0x80_0000, 0x0000),
        // Fuses.
        (0x80_1050, 0x1050),
        // EEPROM.
        (0x80_1400, 0x1400),
        // SRAM.
        (0x80_4000, 0x4000),
        // The mapped flash window.
        (0x80_8000, 0x8000),
    ];

    #[test]
    fn data_addresses_lose_the_offset() {
        for &(probe_rs, chip) in DATA_SPACE {
            assert_eq!(to_chip_data_address(probe_rs).unwrap(), chip);
        }
    }

    /// The Dx flash base is the one that was read back from a part.
    #[test]
    fn dx_flash_addresses_gain_the_verified_base() {
        let dx = AvrFamily::Dx;

        // Flash is at zero for probe-rs and at the base for the tool.
        assert_eq!(to_tool_flash_address(dx, 0x0).unwrap(), 0x80_0000);
        assert_eq!(to_tool_flash_address(dx, 0x200).unwrap(), 0x80_0200);
        // Above the 32 KiB mapped window, where addressing stays linear.
        assert_eq!(to_tool_flash_address(dx, 0x1_0000).unwrap(), 0x81_0000);
        assert_eq!(to_tool_flash_address(dx, 0x1_F000).unwrap(), 0x81_F000);
        // The last byte of a 128 KiB part.
        assert_eq!(to_tool_flash_address(dx, 0x1_FFFF).unwrap(), 0x81_FFFF);
    }

    /// The tiny flash base is a guess, so pin it separately from the Dx one.
    /// If a part ever proves it wrong, only this test and
    /// [`AvrFamily::flash_base`] change.
    #[test]
    fn tiny_flash_addresses_gain_the_unverified_base() {
        let tiny = AvrFamily::Tiny0;

        assert_eq!(to_tool_flash_address(tiny, 0x0).unwrap(), 0x8000);
        assert_eq!(to_tool_flash_address(tiny, 0x40).unwrap(), 0x8040);
        // The last byte of the 4 KiB flash of an ATtiny406.
        assert_eq!(to_tool_flash_address(tiny, 0x0FFF).unwrap(), 0x8FFF);
    }

    /// The two spaces meet at the offset, and neither may cross into the other.
    #[test]
    fn the_two_spaces_do_not_overlap() {
        for family in [AvrFamily::Dx, AvrFamily::Tiny0] {
            assert!(to_tool_flash_address(family, DATA_SPACE_OFFSET - 1).is_ok());
            assert!(to_tool_flash_address(family, DATA_SPACE_OFFSET).is_err());
        }

        assert!(to_chip_data_address(DATA_SPACE_OFFSET - 1).is_err());
        assert_eq!(to_chip_data_address(DATA_SPACE_OFFSET).unwrap(), 0);
    }

    /// Flash goes up and data goes down, so translating one way and back is not
    /// the identity. This pins the direction of each function.
    #[test]
    fn the_two_translations_move_in_opposite_directions() {
        for family in [AvrFamily::Dx, AvrFamily::Tiny0] {
            let flash = to_tool_flash_address(family, 0x1234).unwrap();
            assert!(u64::from(flash) > 0x1234);
        }

        let data = to_chip_data_address(0x80_1234).unwrap();
        assert!(u64::from(data) < 0x80_1234);
    }

    #[test]
    fn data_addresses_beyond_a_word_are_rejected() {
        let too_high = DATA_SPACE_OFFSET + MAX_TOOL_ADDRESS + 1;

        assert!(to_chip_data_address(too_high).is_err());
        assert!(to_chip_data_address(too_high - 1).is_ok());
    }

    #[test]
    fn word_accesses_need_even_address_and_length() {
        assert!(check_word_aligned(0x80_4000, 2).is_ok());
        assert!(check_word_aligned(0x80_4000, 0).is_ok());
        assert!(check_word_aligned(0x80_4001, 2).is_err());
        assert!(check_word_aligned(0x80_4000, 3).is_err());
    }

    #[test]
    fn device_id_decodes_the_avr128da64_reply() {
        let id = DeviceId::from_bytes(&[0x1e, 0x97, 0x07, 0x18]).unwrap();

        assert_eq!(id.signature, [0x1e, 0x97, 0x07]);
        assert_eq!(id.signature(), 0x1e_9707);
        assert_eq!(id.revision, 0x18);
        assert_eq!(id.to_string(), "1e:97:07 rev 0x18");
    }

    #[test]
    fn device_id_needs_four_bytes() {
        assert!(DeviceId::from_bytes(&[0x1e, 0x92, 0x25]).is_err());
    }
}

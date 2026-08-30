use probe_rs_target::{Architecture, NvmRegion, PageInfo, SectorInfo, TransferEncoding};
use std::time::Instant;

use crate::flashing::encoder::FlashEncoder;
use crate::flashing::{FlashError, FlashLayout, FlashProgress, FlashSector, avr};
use crate::session::Session;

/// Flash data
// TODO this is hard to document because this seems like a bad API.
pub enum FlashData {
    /// Raw flash data.
    Raw(FlashLayout),

    /// Encoded flash data.
    Loaded {
        /// The flash encoder.
        encoder: FlashEncoder,

        /// Whether the encoder should ignore fill bytes during processing.
        ignore_fills: bool,
    },
}

impl FlashData {
    /// Returns a reference to the flash layout.
    pub fn layout(&self) -> &FlashLayout {
        match self {
            FlashData::Raw(layout) => layout,
            FlashData::Loaded { encoder, .. } => encoder.flash_layout(),
        }
    }

    /// Returns a reference to the flash layout.
    pub fn layout_mut(&mut self) -> &mut FlashLayout {
        // We're mutating the data, let's invalidate the encoder
        if let FlashData::Loaded { encoder, .. } = self {
            *self = FlashData::Raw(encoder.flash_layout().clone());
        }

        match self {
            FlashData::Raw(layout) => layout,
            FlashData::Loaded { .. } => unreachable!(),
        }
    }

    /// Returns the encoded data.
    pub fn encoder(&mut self, encoding: TransferEncoding, ignore_fills: bool) -> &FlashEncoder {
        if let FlashData::Loaded {
            encoder,
            ignore_fills: was_ignore_fills,
        } = self
            && *was_ignore_fills != ignore_fills
        {
            // Fill handling changed, invalidate the encoder
            *self = FlashData::Raw(encoder.flash_layout().clone());
        }
        if let FlashData::Raw(layout) = self {
            let layout = std::mem::take(layout);
            let encoder = FlashEncoder::new(encoding, layout, ignore_fills);
            *self = FlashData::Loaded {
                encoder,
                ignore_fills,
            };
        }

        match self {
            FlashData::Loaded { encoder, .. } => encoder,
            FlashData::Raw(_) => unreachable!(),
        }
    }
}

/// Represents a piece of data to be flashed.
pub struct LoadedRegion {
    /// The region to flash data to.
    pub region: NvmRegion,

    /// The flash data of the loaded region.
    pub data: FlashData,
}

impl LoadedRegion {
    /// Returns the flash layout of the loaded region.
    pub fn flash_layout(&self) -> &FlashLayout {
        self.data.layout()
    }
}

/// Describes the flash geometry that an [`NvmDriver`] programs.
///
/// The flash builder uses this to split the staged data into sectors and pages. For
/// targets with a flash algorithm the geometry comes from the algorithm's flash
/// properties, so [`FlashAlgorithm`] implements this trait. A driver that does not
/// use an algorithm describes its geometry some other way, which is why the builder
/// asks for this trait and not for `FlashProperties`.
///
/// [`FlashAlgorithm`]: crate::flashing::FlashAlgorithm
pub trait NvmGeometry {
    /// Returns the sector containing `address`, or `None` if the address is outside the flash.
    fn sector_info(&self, address: u64) -> Option<SectorInfo>;

    /// Returns the page containing `address`, or `None` if the address is outside the flash.
    fn page_info(&self, address: u64) -> Option<PageInfo>;

    /// Iterates over every sector of the flash.
    fn sectors(&self) -> Box<dyn Iterator<Item = SectorInfo> + '_>;

    /// Iterates over every page of the flash.
    fn pages(&self) -> Box<dyn Iterator<Item = PageInfo> + '_>;

    /// The value that an erased byte reads back as.
    fn erased_byte_value(&self) -> u8;
}

/// Reads back the non-volatile memory of a target.
///
/// A driver hands one of these to [`NvmDriver::with_reader`]. The reader stays valid
/// for the whole call, so a driver that has to prepare the target for reading only
/// pays that cost once.
pub trait NvmReader {
    /// Reads `data.len()` bytes starting at `address`.
    fn read(&mut self, address: u64, data: &mut [u8]) -> Result<(), FlashError>;
}

/// Moves bytes into the non-volatile memory of a target.
///
/// probe-rs programs most targets by downloading a flash algorithm into target RAM and
/// calling into it. [`Flasher`] implements this trait that way. Targets whose probe or
/// debug sequence programs flash directly implement it themselves, and then they need
/// no flash algorithm at all.
///
/// Everything above the driver is shared. The flash builder plans the sectors and pages,
/// and [`FlashLoader`] runs the phases in order: fill, erase, program, verify.
///
/// The methods take whole regions rather than single sectors or pages. This lets a driver
/// hold on to target state for a whole phase. The flash algorithm driver needs that,
/// because it borrows a [`Core`] from the [`Session`] and keeps the algorithm's init and
/// uninit calls around the phase.
///
/// [`Flasher`]: crate::flashing::Flasher
/// [`FlashLoader`]: crate::flashing::FlashLoader
/// [`Core`]: crate::Core
pub trait NvmDriver {
    /// A short name for this driver. Used to group regions and in diagnostics.
    fn name(&self) -> &str;

    /// The flash geometry this driver programs.
    fn geometry(&self) -> &dyn NvmGeometry;

    /// The encoding this driver expects page data in.
    fn transfer_encoding(&self) -> TransferEncoding {
        TransferEncoding::Raw
    }

    /// Whether programming a page also erases it.
    ///
    /// When this is `true` the loader skips the erase phase. AVR parts erase on write,
    /// for example.
    fn program_erases_page(&self) -> bool {
        false
    }

    /// Whether the driver can overlap the transfer of one page with the programming of
    /// the previous one.
    fn supports_double_buffering(&self) -> bool {
        false
    }

    /// Whether the driver can program `region`.
    ///
    /// A driver can own regions it has no way to write: one driver can cover every
    /// non-volatile region of a part while only some of them are writable through
    /// it. The loader refuses data for a region this returns `false` for, before
    /// it plans pages or touches the target.
    fn programs(&self, _region: &NvmRegion) -> bool {
        true
    }

    /// Whether [`erase_all`](Self::erase_all) can erase the whole device.
    fn is_chip_erase_supported(&self, session: &Session) -> bool;

    /// Erases the whole device.
    fn erase_all(
        &mut self,
        session: &mut Session,
        progress: &mut FlashProgress<'_>,
    ) -> Result<(), FlashError>;

    /// Erases every sector of `regions`.
    fn erase_sectors(
        &mut self,
        session: &mut Session,
        progress: &mut FlashProgress<'_>,
        regions: &mut [LoadedRegion],
    ) -> Result<(), FlashError>;

    /// Erases exactly the sectors listed in `sectors`.
    ///
    /// This backs the standalone erase commands, where the sectors come from the caller's
    /// address range instead of from staged data. The caller takes the sectors from
    /// [`geometry`](Self::geometry), so a driver only ever sees sectors it owns.
    ///
    /// There is no sensible generic implementation, so the default reports that this
    /// driver cannot erase a chosen set of sectors.
    fn erase_selected_sectors(
        &mut self,
        session: &mut Session,
        progress: &mut FlashProgress<'_>,
        sectors: &[FlashSector],
    ) -> Result<(), FlashError> {
        let _ = (session, progress, sectors);
        Err(FlashError::SectorEraseNotSupported {
            name: self.name().to_string(),
        })
    }

    /// Programs every page of `regions`.
    ///
    /// `double_buffering` is the user's preference. A driver that does not support it
    /// ignores the flag.
    fn program_pages(
        &mut self,
        session: &mut Session,
        progress: &mut FlashProgress<'_>,
        regions: &mut [LoadedRegion],
        double_buffering: bool,
    ) -> Result<(), FlashError>;

    /// Calls `f` with a reader for this driver's flash.
    fn with_reader(
        &mut self,
        session: &mut Session,
        f: &mut dyn FnMut(&mut dyn NvmReader) -> Result<(), FlashError>,
    ) -> Result<(), FlashError>;

    /// Verifies that `regions` hold the data staged in them.
    ///
    /// `ignore_filled` masks out the bytes that were only added to pad pages, because
    /// those are not written and may differ.
    ///
    /// The default reads the flash back and compares. Drivers with a faster path
    /// override this.
    fn verify(
        &mut self,
        session: &mut Session,
        progress: &mut FlashProgress<'_>,
        regions: &mut [LoadedRegion],
        ignore_filled: bool,
    ) -> Result<bool, FlashError> {
        progress.started_verifying();

        let mut matches = false;
        let result = self
            .with_reader(session, &mut |reader| {
                matches = compare_flash(regions, progress, ignore_filled, reader)?;
                Ok(())
            })
            .map(|()| matches);

        match result.is_ok() {
            true => progress.finished_verifying(),
            false => progress.failed_verifying(),
        }

        result
    }

    /// Checks that every sector in `sectors` reads back as erased.
    ///
    /// The default reads the sectors through [`with_reader`](Self::with_reader) and compares
    /// every byte against [`NvmGeometry::erased_byte_value`]. Drivers with a dedicated blank
    /// check on the target override this.
    fn blank_check(
        &mut self,
        session: &mut Session,
        progress: &mut FlashProgress<'_>,
        sectors: &[FlashSector],
    ) -> Result<(), FlashError> {
        let _ = progress;
        let erased_byte_value = self.geometry().erased_byte_value();

        self.with_reader(session, &mut |reader| {
            for sector in sectors {
                let mut data = vec![0; sector.size() as usize];
                reader.read(sector.address(), &mut data)?;

                if !data.iter().all(|v| *v == erased_byte_value) {
                    return Err(FlashError::ChipEraseFailed {
                        source: "Not all sectors were erased".into(),
                    });
                }
            }
            Ok(())
        })
    }
}

/// Returns a driver that programs `region` without an on-target flash algorithm.
///
/// probe-rs normally flashes by downloading a flash algorithm into target RAM and
/// calling into it. Some targets cannot do that, because the probe itself performs the
/// non-volatile memory sequencing, or because the core cannot execute from RAM. Those
/// targets get a driver here, and then they need no flash algorithm.
///
/// The architecture decides. AVR is programmed by the probe, so it gets a driver.
/// Every other architecture returns `None` and takes the flash algorithm path.
///
/// An AVR whose target description cannot support the driver is an error rather than
/// a `None`, because falling back to the flash algorithm path would report a missing
/// algorithm when the real problem is the target description.
///
/// The region is not used for AVR. One driver covers every non-volatile region of
/// the part, so the regions share a plan and a chip erase runs once for all of them.
pub(super) fn driver_for_region(
    session: &mut Session,
    _region: &NvmRegion,
    _core_index: usize,
) -> Result<Option<Box<dyn NvmDriver>>, FlashError> {
    match session.architecture() {
        Architecture::Avr => Ok(Some(avr::driver_for(session)?)),
        _ => Ok(None),
    }
}

/// Compares the staged data of `regions` against what the target reports.
pub(super) fn compare_flash(
    regions: &[LoadedRegion],
    progress: &mut FlashProgress<'_>,
    ignore_filled: bool,
    reader: &mut dyn NvmReader,
) -> Result<bool, FlashError> {
    for region in regions {
        let layout = region.data.layout();
        for (idx, page) in layout.pages.iter().enumerate() {
            let start = Instant::now();
            let address = page.address();
            let data = page.data();

            let mut read_back = vec![0; data.len()];
            reader.read(address, &mut read_back)?;

            if ignore_filled {
                // "Unfill" fill regions. These don't get flashed, so their contents are
                // allowed to differ. We mask these bytes with default flash content here,
                // just for the verification process.
                for fill in layout.fills() {
                    if fill.page_index() != idx {
                        continue;
                    }

                    let fill_offset = (fill.address() - address) as usize;
                    let fill_size = fill.size() as usize;

                    let default_bytes = &data[fill_offset..][..fill_size];
                    read_back[fill_offset..][..fill_size].copy_from_slice(default_bytes);
                }
            }
            if data != read_back {
                tracing::debug!("Verification failed for page at address {:#010x}", address);
                return Ok(false);
            }

            progress.page_verified(data.len() as u64, start.elapsed());
        }
    }
    Ok(true)
}

/// Fills the holes in the pages of `regions` with what the target currently holds.
pub(super) fn fill_pages(
    regions: &mut [LoadedRegion],
    progress: &mut FlashProgress<'_>,
    reader: &mut dyn NvmReader,
) -> Result<(), FlashError> {
    for region in regions.iter_mut() {
        let layout = region.data.layout_mut();
        for fill in layout.fills.iter() {
            let t = Instant::now();
            let page = &mut layout.pages[fill.page_index()];

            let page_offset = (fill.address() - page.address()) as usize;
            let page_slice = &mut page.data_mut()[page_offset..][..fill.size() as usize];

            reader.read(fill.address(), page_slice)?;

            progress.page_filled(fill.size(), t.elapsed());
        }
    }

    Ok(())
}

#[cfg(all(test, feature = "builtin-targets"))]
mod tests {
    use probe_rs_target::{MemoryRegion, PageInfo, SectorInfo};

    use super::*;
    use crate::probe::Probe;
    use crate::probe::fake_probe::FakeProbe;
    use crate::{Permissions, Session};

    const SECTOR_SIZE: u64 = 0x400;

    struct TestGeometry;

    impl NvmGeometry for TestGeometry {
        fn sector_info(&self, address: u64) -> Option<SectorInfo> {
            Some(SectorInfo {
                base_address: address & !(SECTOR_SIZE - 1),
                size: SECTOR_SIZE,
            })
        }

        fn page_info(&self, address: u64) -> Option<PageInfo> {
            Some(PageInfo {
                base_address: address & !0xff,
                size: 0x100,
            })
        }

        fn sectors(&self) -> Box<dyn Iterator<Item = SectorInfo> + '_> {
            Box::new(std::iter::once(SectorInfo {
                base_address: 0,
                size: SECTOR_SIZE,
            }))
        }

        fn pages(&self) -> Box<dyn Iterator<Item = PageInfo> + '_> {
            Box::new(std::iter::empty())
        }

        fn erased_byte_value(&self) -> u8 {
            0xff
        }
    }

    /// Serves a fixed image starting at address 0.
    struct ImageReader<'a>(&'a [u8]);

    impl NvmReader for ImageReader<'_> {
        fn read(&mut self, address: u64, data: &mut [u8]) -> Result<(), FlashError> {
            let start = address as usize;
            data.copy_from_slice(&self.0[start..][..data.len()]);
            Ok(())
        }
    }

    /// A driver that only knows how to read back a fixed image.
    ///
    /// Everything else stays on the trait's default implementation, which is what these
    /// tests exercise.
    struct ImageDriver {
        geometry: TestGeometry,
        image: Vec<u8>,
    }

    impl NvmDriver for ImageDriver {
        fn name(&self) -> &str {
            "image"
        }

        fn geometry(&self) -> &dyn NvmGeometry {
            &self.geometry
        }

        fn is_chip_erase_supported(&self, _session: &Session) -> bool {
            false
        }

        fn erase_all(
            &mut self,
            _session: &mut Session,
            _progress: &mut FlashProgress<'_>,
        ) -> Result<(), FlashError> {
            Ok(())
        }

        fn erase_sectors(
            &mut self,
            _session: &mut Session,
            _progress: &mut FlashProgress<'_>,
            _regions: &mut [LoadedRegion],
        ) -> Result<(), FlashError> {
            Ok(())
        }

        fn program_pages(
            &mut self,
            _session: &mut Session,
            _progress: &mut FlashProgress<'_>,
            _regions: &mut [LoadedRegion],
            _double_buffering: bool,
        ) -> Result<(), FlashError> {
            Ok(())
        }

        fn with_reader(
            &mut self,
            _session: &mut Session,
            f: &mut dyn FnMut(&mut dyn NvmReader) -> Result<(), FlashError>,
        ) -> Result<(), FlashError> {
            f(&mut ImageReader(&self.image))
        }
    }

    fn fake_session() -> Session {
        Probe::from_specific_probe(Box::new(FakeProbe::with_mocked_core()))
            .attach("nrf51822_xxAC", Permissions::default())
            .expect("Failed to attach with 'fake' probe.")
    }

    fn driver_with(image: Vec<u8>) -> ImageDriver {
        ImageDriver {
            geometry: TestGeometry,
            image,
        }
    }

    fn one_sector() -> Vec<FlashSector> {
        vec![FlashSector {
            address: 0,
            size: SECTOR_SIZE,
        }]
    }

    /// Every architecture except AVR keeps the flash algorithm path, so nothing
    /// but AVR may get a driver here.
    #[test]
    fn a_non_avr_target_has_no_driver() {
        let mut session = fake_session();
        let region = session
            .target()
            .memory_map
            .iter()
            .find_map(MemoryRegion::as_nvm_region)
            .expect("nrf51822 has flash")
            .clone();

        assert!(
            driver_for_region(&mut session, &region, 0)
                .unwrap()
                .is_none()
        );
    }

    #[test]
    fn default_blank_check_accepts_erased_flash() {
        let mut session = fake_session();
        let mut driver = driver_with(vec![0xff; SECTOR_SIZE as usize]);

        driver
            .blank_check(&mut session, &mut FlashProgress::empty(), &one_sector())
            .unwrap();
    }

    #[test]
    fn default_blank_check_rejects_programmed_flash() {
        let mut session = fake_session();
        let mut image = vec![0xff; SECTOR_SIZE as usize];
        image[0x100] = 0x00;
        let mut driver = driver_with(image);

        let error = driver
            .blank_check(&mut session, &mut FlashProgress::empty(), &one_sector())
            .unwrap_err();

        assert!(matches!(error, FlashError::ChipEraseFailed { .. }));
    }

    #[test]
    fn default_erase_selected_sectors_is_unsupported() {
        let mut session = fake_session();
        let mut driver = driver_with(vec![0xff; SECTOR_SIZE as usize]);

        let error = driver
            .erase_selected_sectors(&mut session, &mut FlashProgress::empty(), &one_sector())
            .unwrap_err();

        assert!(matches!(
            error,
            FlashError::SectorEraseNotSupported { ref name } if name == "image"
        ));
    }
}

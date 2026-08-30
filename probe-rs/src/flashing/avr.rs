//! Flashing an AVR over UPDI, without a flash algorithm.
//!
//! An AVR cannot run a flash algorithm. It is a Harvard machine that cannot
//! execute from RAM, and the smallest supported part has 256 bytes of SRAM. It
//! does not need one either, because the probe already sequences the NVM
//! controller. Programming a page is a single script call.
//!
//! So this is an [`NvmDriver`] that runs on the host and drives the probe. The
//! flash builder, the layout, the fills, the progress reporting, and the verify
//! phase above it are the shared ones.
//!
//! # Erase
//!
//! `WriteProgmem` erases the page it writes. That was measured on hardware with
//! patterns that discriminate: a page written to `0x00` and then written to
//! `0xFF` without an erase reads back as `0xFF`, and flash cannot set bits
//! without an erase. So [`NvmDriver::program_erases_page`] is `true` here and
//! the loader skips the erase phase.
//!
//! `EraseChip` is still needed. It is the only way past a locked part, because
//! it clears the lock bits along with flash and EEPROM.
//!
//! # Whole pages only
//!
//! `WriteProgmem` writes whole pages. A write that covers part of a page leaves
//! the rest of that page undefined, so this driver never issues one. The flash
//! builder pads every page out to the full page size before the driver sees it,
//! which is what makes that safe.
//!
//! Because the write erases, the padding decides what the rest of the page ends
//! up holding. With `keep_unwritten_bytes` off, the builder pads with
//! [`ERASED_BYTE_VALUE`], so the untouched part of the page reads as erased.
//! With it on, the loader reads the current contents into the padding first, so
//! the untouched part survives. Both are correct, and the choice is the same one
//! a user makes on any flash that erases before it writes.
//!
//! # Page size
//!
//! The page size comes from [`AvrFamily::flash_page_size`], which reads it back
//! out of the script bytecode the tool runs. The target description cannot
//! carry it, because a page size lives in a flash algorithm and these parts
//! have none.

use std::ops::Range;
use std::time::Instant;

use probe_rs_target::{MemoryRange, MemoryRegion, NvmRegion, PageInfo, SectorInfo};

use crate::MemoryInterface;
use crate::architecture::avr::communication_interface::{
    AvrCommunicationInterface, AvrError, DATA_SPACE_OFFSET,
};
use crate::flashing::nvm_driver::{LoadedRegion, NvmDriver, NvmGeometry, NvmReader};
use crate::flashing::{FlashError, FlashProgress, FlashSector};
use crate::probe::pickit::{AvrFamily, SessionState};
use crate::session::Session;

/// The name this driver is grouped and logged under.
///
/// One instance covers every non-volatile region of the part, so every region
/// lands in the same plan and a chip erase runs once rather than once per
/// region.
const DRIVER_NAME: &str = "AVR UPDI";

/// Erased AVR flash reads back as all ones.
const ERASED_BYTE_VALUE: u8 = 0xff;

/// Builds the AVR driver for `session`.
///
/// The caller checks the architecture, so the session is an AVR here. The family
/// comes from the interface state the session built at attach time, so the driver
/// and the rest of the session cannot disagree about it.
pub(super) fn driver_for(session: &mut Session) -> Result<Box<dyn NvmDriver>, FlashError> {
    let family = session
        .get_avr_interface()
        .map_err(FlashError::Core)?
        .family();

    let target = session.target();
    let geometry = geometry_for(&target.name, family, &target.memory_map)?;

    Ok(Box::new(AvrNvmDriver { geometry }))
}

/// Works out the flash layout of the part called `name` in `family`.
fn geometry_for(
    name: &str,
    family: AvrFamily,
    memory_map: &[MemoryRegion],
) -> Result<AvrFlashGeometry, FlashError> {
    let Some(range) = flash_range(memory_map) else {
        return Err(FlashError::MissingAvrFlashRegion {
            name: name.to_string(),
        });
    };

    Ok(AvrFlashGeometry {
        range,
        page_size: family.flash_page_size(),
    })
}

/// The address range of the part's flash.
///
/// probe-rs puts flash below [`DATA_SPACE_OFFSET`] and the data space above it,
/// so a region that ends at or below the offset is the flash. The mapped flash
/// window is above the offset and is marked as an alias, so it cannot be
/// mistaken for the real thing.
fn flash_range(memory_map: &[MemoryRegion]) -> Option<Range<u64>> {
    memory_map
        .iter()
        .filter_map(MemoryRegion::as_nvm_region)
        .find(|region| !region.is_alias && region.range.end <= DATA_SPACE_OFFSET)
        .map(|region| region.range.clone())
}

/// The page and sector layout of AVR flash.
///
/// A sector is an erase unit, and on an AVR that is a page, because a page
/// write erases the page. So both are one flash page.
#[derive(Debug)]
struct AvrFlashGeometry {
    range: Range<u64>,
    page_size: u32,
}

impl AvrFlashGeometry {
    /// The base address of every page of the flash, in order.
    fn page_bases(&self) -> impl Iterator<Item = u64> + '_ {
        let page_size = u64::from(self.page_size);
        let first = self.range.start - self.range.start % page_size;

        (first..self.range.end).step_by(self.page_size as usize)
    }
}

impl NvmGeometry for AvrFlashGeometry {
    fn sector_info(&self, address: u64) -> Option<SectorInfo> {
        let page = self.page_info(address)?;

        Some(SectorInfo {
            base_address: page.base_address,
            size: u64::from(page.size),
        })
    }

    fn page_info(&self, address: u64) -> Option<PageInfo> {
        if !self.range.contains(&address) {
            return None;
        }

        let page_size = u64::from(self.page_size);

        Some(PageInfo {
            base_address: address - address % page_size,
            size: self.page_size,
        })
    }

    fn sectors(&self) -> Box<dyn Iterator<Item = SectorInfo> + '_> {
        Box::new(self.page_bases().map(|base| SectorInfo {
            base_address: base,
            size: u64::from(self.page_size),
        }))
    }

    fn pages(&self) -> Box<dyn Iterator<Item = PageInfo> + '_> {
        Box::new(self.page_bases().map(|base| PageInfo {
            base_address: base,
            size: self.page_size,
        }))
    }

    fn erased_byte_value(&self) -> u8 {
        ERASED_BYTE_VALUE
    }
}

/// Programs AVR flash through the probe.
#[derive(Debug)]
struct AvrNvmDriver {
    geometry: AvrFlashGeometry,
}

impl AvrNvmDriver {
    fn chip_erase(&mut self, session: &mut Session) -> Result<(), FlashError> {
        // The erase wipes the planted software breakpoints along with the rest
        // of flash. Take them out first, while the old image can still be
        // restored from.
        session
            .clear_avr_software_breakpoints()
            .map_err(FlashError::Core)?;

        let mut interface = session.get_avr_interface().map_err(FlashError::Core)?;

        interface
            .erase_chip()
            .map_err(|err| FlashError::ChipEraseFailed {
                source: Box::new(err),
            })
    }

    fn write_pages(
        &mut self,
        session: &mut Session,
        progress: &mut FlashProgress<'_>,
        regions: &mut [LoadedRegion],
    ) -> Result<(), FlashError> {
        for loaded in regions.iter() {
            if !self.programs(&loaded.region) {
                return Err(FlashError::RegionNotProgrammable {
                    driver: DRIVER_NAME.to_string(),
                    range: loaded.region.range.clone(),
                });
            }
        }

        {
            let mut interface = session.get_avr_interface().map_err(FlashError::Core)?;
            halt_if_debugging(&mut interface)?;
        }

        // Each page write below erases its whole page, which takes the planted
        // software breakpoints in it with it. Remove them first, while the old
        // image is still in flash and the instructions they replaced can still
        // be restored.
        session
            .clear_avr_software_breakpoints()
            .map_err(FlashError::Core)?;
        let mut interface = session.get_avr_interface().map_err(FlashError::Core)?;

        let encoding = self.transfer_encoding();

        for loaded in regions.iter_mut() {
            for page in loaded.data.encoder(encoding, false).pages() {
                let start = Instant::now();

                // The flash builder pads every page out to the full page size,
                // so this never hands the script a partial page. That matters,
                // because a partial write leaves the rest of the page undefined.
                interface
                    .write_flash(page.address(), page.data())
                    .map_err(|err| FlashError::PageWrite {
                        page_address: page.address(),
                        source: Box::new(err),
                    })?;

                progress.page_programmed(u64::from(page.size()), start.elapsed());
            }
        }

        Ok(())
    }

    fn erase_sector_pages(
        &mut self,
        session: &mut Session,
        progress: &mut FlashProgress<'_>,
        sectors: &[FlashSector],
    ) -> Result<(), FlashError> {
        {
            let mut interface = session.get_avr_interface().map_err(FlashError::Core)?;
            halt_if_debugging(&mut interface)?;
        }

        // The page writes below take the planted software breakpoints with
        // them, so remove those first, while the old image is still in flash.
        session
            .clear_avr_software_breakpoints()
            .map_err(FlashError::Core)?;
        let mut interface = session.get_avr_interface().map_err(FlashError::Core)?;

        for sector in sectors {
            let start = Instant::now();
            let page = vec![0xff; sector.size() as usize];

            interface
                .write_flash(sector.address(), &page)
                .map_err(|err| FlashError::PageWrite {
                    page_address: sector.address(),
                    source: Box::new(err),
                })?;

            progress.sector_erased(sector.size(), start.elapsed());
        }

        Ok(())
    }
}

impl NvmDriver for AvrNvmDriver {
    fn name(&self) -> &str {
        DRIVER_NAME
    }

    fn geometry(&self) -> &dyn NvmGeometry {
        &self.geometry
    }

    /// `WriteProgmem` erases the page it writes, so the loader skips the erase
    /// phase. See the module documentation for how that was measured.
    fn program_erases_page(&self) -> bool {
        true
    }

    /// There is nothing to overlap. The host sends one page and waits for the
    /// tool to finish it.
    fn supports_double_buffering(&self) -> bool {
        false
    }

    /// Only flash. EEPROM, the fuses, and the lock bits have their own scripts
    /// that nothing has exercised yet, so they are refused rather than written
    /// with the wrong one.
    ///
    /// The loader consults this when a region joins a plan, so data for the
    /// configuration memories fails before any phase touches the target.
    fn programs(&self, region: &NvmRegion) -> bool {
        self.geometry.range.contains_range(&region.range)
    }

    fn is_chip_erase_supported(&self, _session: &Session) -> bool {
        true
    }

    fn erase_all(
        &mut self,
        session: &mut Session,
        progress: &mut FlashProgress<'_>,
    ) -> Result<(), FlashError> {
        progress.started_erasing();

        let result = self.chip_erase(session);

        match result.is_ok() {
            true => progress.finished_erasing(),
            false => progress.failed_erasing(),
        }

        result
    }

    /// Does nothing, because the following page writes do the erasing.
    ///
    /// [`NvmDriver::program_erases_page`] is `true`, so the loader never calls
    /// this. It stays a no-op rather than an error so that a caller which does
    /// reach it still ends up with correct flash contents.
    fn erase_sectors(
        &mut self,
        _session: &mut Session,
        _progress: &mut FlashProgress<'_>,
        _regions: &mut [LoadedRegion],
    ) -> Result<(), FlashError> {
        tracing::debug!("AVR page writes erase, so there is no separate erase phase");

        Ok(())
    }

    /// Erases selected sectors by writing each one full of erased bytes.
    ///
    /// A page write erases the page it writes, so a whole page of `0xff` is an
    /// erase. The erase.rs restore flow relies on this shape: bytes outside the
    /// erased range are read beforehand and written back afterwards with plain
    /// page writes, which erase and reprogram their own pages.
    fn erase_selected_sectors(
        &mut self,
        session: &mut Session,
        progress: &mut FlashProgress<'_>,
        sectors: &[FlashSector],
    ) -> Result<(), FlashError> {
        progress.started_erasing();

        let result = self.erase_sector_pages(session, progress, sectors);

        match result.is_ok() {
            true => progress.finished_erasing(),
            false => progress.failed_erasing(),
        }

        result
    }
    fn program_pages(
        &mut self,
        session: &mut Session,
        progress: &mut FlashProgress<'_>,
        regions: &mut [LoadedRegion],
        _double_buffering: bool,
    ) -> Result<(), FlashError> {
        progress.started_programming();

        let result = self.write_pages(session, progress, regions);

        match result.is_ok() {
            true => progress.finished_programming(),
            false => progress.failed_programming(),
        }

        result
    }

    fn with_reader(
        &mut self,
        session: &mut Session,
        f: &mut dyn FnMut(&mut dyn NvmReader) -> Result<(), FlashError>,
    ) -> Result<(), FlashError> {
        // With `keep_unwritten_bytes`, what is read here becomes part of the
        // new image. The planted software breakpoints have to come out first,
        // or the read picks up their BREAK instructions and programs them
        // back into the new firmware.
        session
            .clear_avr_software_breakpoints()
            .map_err(FlashError::Core)?;

        let mut interface = session.get_avr_interface().map_err(FlashError::Core)?;

        f(&mut AvrReader(&mut interface))
    }
}

/// Reads back through the probe.
///
/// The address decides the memory, so this reaches both flash and the data
/// space. See the AVR communication interface for that split.
struct AvrReader<'a, 'probe>(&'a mut AvrCommunicationInterface<'probe>);

impl NvmReader for AvrReader<'_, '_> {
    fn read(&mut self, address: u64, data: &mut [u8]) -> Result<(), FlashError> {
        self.0.read(address, data).map_err(FlashError::Core)
    }
}

/// Stops the core when a debug session is open.
///
/// A programming session holds the core in reset, so nothing is running there.
/// A debug session does not, and flash must not change under a running core.
fn halt_if_debugging(interface: &mut AvrCommunicationInterface<'_>) -> Result<(), FlashError> {
    if interface.session_state() != SessionState::Debugging {
        return Ok(());
    }

    if !interface.is_halted().map_err(core_error)? {
        interface.halt().map_err(core_error)?;
    }

    Ok(())
}

fn core_error(err: AvrError) -> FlashError {
    FlashError::Core(err.into())
}

#[cfg(test)]
mod tests {
    use probe_rs_target::MemoryAccess;

    use super::*;
    use crate::flashing::builder::{FlashBuilder, FlashSector};

    /// Flash of an AVR128DA64, which is the part the write path was measured on.
    fn dx_geometry() -> AvrFlashGeometry {
        AvrFlashGeometry {
            range: 0..0x2_0000,
            page_size: AvrFamily::Dx.flash_page_size(),
        }
    }

    /// Flash of an ATtiny406.
    fn tiny_geometry() -> AvrFlashGeometry {
        AvrFlashGeometry {
            range: 0..0x1000,
            page_size: AvrFamily::Tiny0.flash_page_size(),
        }
    }

    fn nvm_region(name: &str, range: Range<u64>, is_alias: bool) -> MemoryRegion {
        MemoryRegion::Nvm(NvmRegion {
            name: Some(name.to_string()),
            range,
            cores: vec!["main".to_string()],
            is_alias,
            access: Some(MemoryAccess::default()),
        })
    }

    #[test]
    fn a_dx_page_is_512_bytes_and_a_tiny_page_is_64() {
        assert_eq!(dx_geometry().page_info(0).unwrap().size, 512);
        assert_eq!(tiny_geometry().page_info(0).unwrap().size, 64);
    }

    /// A sector is an erase unit, and a page write erases a page, so the two
    /// have to be the same size.
    #[test]
    fn a_sector_is_one_page() {
        for geometry in [dx_geometry(), tiny_geometry()] {
            let page = geometry.page_info(0x40).unwrap();
            let sector = geometry.sector_info(0x40).unwrap();

            assert_eq!(sector.base_address, page.base_address);
            assert_eq!(sector.size, u64::from(page.size));
        }
    }

    #[test]
    fn an_address_maps_to_the_page_that_holds_it() {
        let geometry = dx_geometry();

        assert_eq!(geometry.page_info(0).unwrap().base_address, 0);
        assert_eq!(geometry.page_info(0x1ff).unwrap().base_address, 0);
        assert_eq!(geometry.page_info(0x200).unwrap().base_address, 0x200);
        // Above the 32 KiB window that is mapped into the data space.
        assert_eq!(geometry.page_info(0x1_0123).unwrap().base_address, 0x1_0000);
        // The last page of the part.
        assert_eq!(geometry.page_info(0x1_ffff).unwrap().base_address, 0x1_fe00);
    }

    #[test]
    fn an_address_outside_the_flash_has_no_page() {
        let geometry = dx_geometry();

        assert!(geometry.page_info(0x2_0000).is_none());
        assert!(geometry.sector_info(0x2_0000).is_none());
        // The data space, where the flash scripts cannot reach.
        assert!(geometry.page_info(DATA_SPACE_OFFSET).is_none());
    }

    #[test]
    fn the_pages_tile_the_whole_flash() {
        for (geometry, count) in [(dx_geometry(), 256), (tiny_geometry(), 64)] {
            let pages: Vec<PageInfo> = geometry.pages().collect();

            assert_eq!(pages.len(), count);
            assert_eq!(pages[0].base_address, geometry.range.start);
            assert_eq!(
                pages.last().unwrap().address_range().end,
                geometry.range.end
            );

            for pair in pages.windows(2) {
                assert_eq!(pair[0].address_range().end, pair[1].base_address);
            }
        }
    }

    #[test]
    fn the_sectors_tile_the_whole_flash() {
        for (geometry, count) in [(dx_geometry(), 256), (tiny_geometry(), 64)] {
            let sectors: Vec<SectorInfo> = geometry.sectors().collect();

            assert_eq!(sectors.len(), count);
            assert_eq!(
                sectors.last().unwrap().address_range().end,
                geometry.range.end
            );
        }
    }

    /// Every page the iterator yields has to be the page that `page_info`
    /// reports for the addresses inside it, or the builder pairs them up wrong.
    #[test]
    fn the_page_iterator_agrees_with_the_page_lookup() {
        let geometry = tiny_geometry();

        for page in geometry.pages() {
            for address in [page.base_address, page.address_range().end - 1] {
                let looked_up = geometry.page_info(address).unwrap();

                assert_eq!(looked_up.base_address, page.base_address);
                assert_eq!(looked_up.size, page.size);
            }
        }
    }

    #[test]
    fn erased_flash_is_all_ones() {
        assert_eq!(dx_geometry().erased_byte_value(), 0xff);
    }

    /// The flash region is the one below the data space. The mapped flash
    /// window sits above it and is an alias, so it must not be picked.
    #[test]
    fn the_flash_region_is_the_one_below_the_data_space() {
        let memory_map = [
            nvm_region("PROGMEM", 0..0x2_0000, false),
            nvm_region("EEPROM", 0x80_1400..0x80_1600, false),
            nvm_region("MAPPED_PROGMEM", 0x80_8000..0x81_0000, true),
        ];

        assert_eq!(flash_range(&memory_map), Some(0..0x2_0000));
    }

    /// The two families differ in page size, and the family is what picks between
    /// them, so walk both families with the memory maps the targets ship.
    #[test]
    fn each_family_gets_the_page_size_of_its_parts() {
        let dx = [
            nvm_region("PROGMEM", 0..0x2_0000, false),
            nvm_region("MAPPED_PROGMEM", 0x80_8000..0x81_0000, true),
        ];
        let tiny = [
            nvm_region("PROGMEM", 0..0x1000, false),
            nvm_region("MAPPED_PROGMEM", 0x80_8000..0x80_9000, true),
        ];

        let geometry = geometry_for("AVR128DA64", AvrFamily::Dx, &dx).unwrap();
        assert_eq!(geometry.range, 0..0x2_0000);
        assert_eq!(geometry.page_size, 512);

        let geometry = geometry_for("ATtiny406", AvrFamily::Tiny0, &tiny).unwrap();
        assert_eq!(geometry.range, 0..0x1000);
        assert_eq!(geometry.page_size, 64);
    }

    /// Without a flash region there is nothing to program, and that is the
    /// broken target description it is, not a missing flash algorithm.
    #[test]
    fn a_target_without_a_flash_region_names_the_broken_description() {
        let memory_map = [nvm_region("EEPROM", 0x80_1400..0x80_1600, false)];

        let error = geometry_for("AVR128DA64", AvrFamily::Dx, &memory_map).unwrap_err();

        assert!(matches!(
            error,
            FlashError::MissingAvrFlashRegion { ref name } if name == "AVR128DA64"
        ));
    }

    #[test]
    fn a_target_without_a_flash_region_has_no_range() {
        let memory_map = [nvm_region("EEPROM", 0x80_1400..0x80_1600, false)];

        assert!(flash_range(&memory_map).is_none());
    }

    /// Only flash is programmable. The configuration memories need their own
    /// scripts, which nothing has exercised.
    #[test]
    fn only_the_flash_region_is_programmable() {
        let driver = AvrNvmDriver {
            geometry: dx_geometry(),
        };

        let MemoryRegion::Nvm(progmem) = nvm_region("PROGMEM", 0..0x2_0000, false) else {
            unreachable!()
        };
        let MemoryRegion::Nvm(eeprom) = nvm_region("EEPROM", 0x80_1400..0x80_1600, false) else {
            unreachable!()
        };

        assert!(driver.programs(&progmem));
        assert!(!driver.programs(&eeprom));
    }

    #[test]
    fn the_driver_reports_the_avr_capabilities() {
        let driver = AvrNvmDriver {
            geometry: dx_geometry(),
        };

        assert!(driver.program_erases_page());
        assert!(!driver.supports_double_buffering());
        assert_eq!(driver.name(), DRIVER_NAME);
    }

    /// The builder pads every page out to the full page size and records the
    /// bytes it padded. That is what lets `WriteProgmem` see whole pages only.
    #[test]
    fn the_builder_pads_a_short_write_out_to_a_whole_page() {
        let geometry = dx_geometry();
        let region = NvmRegion {
            name: Some("PROGMEM".to_string()),
            range: 0..0x2_0000,
            cores: vec!["main".to_string()],
            is_alias: false,
            access: None,
        };

        let mut builder = FlashBuilder::new();
        builder.add_data(0x100, &[1, 2, 3]).unwrap();

        let layout = builder
            .build_sectors_and_pages(&region, &geometry, false)
            .unwrap();

        assert_eq!(layout.pages().len(), 1);
        let page = &layout.pages()[0];
        assert_eq!(page.address(), 0);
        assert_eq!(page.size(), 512);
        assert_eq!(&page.data()[0x100..0x103], &[1, 2, 3]);
        // Everything the image did not cover reads as erased flash.
        assert!(page.data()[..0x100].iter().all(|b| *b == 0xff));
        assert!(page.data()[0x103..].iter().all(|b| *b == 0xff));

        assert_eq!(layout.sectors().len(), 1);
        assert_eq!(layout.sectors()[0].size(), 512);

        // The two holes around the data, so verification can ignore them.
        let fills: Vec<(u64, u64)> = layout
            .fills()
            .iter()
            .map(|fill| (fill.address(), fill.size()))
            .collect();
        assert_eq!(fills, [(0, 0x100), (0x103, 0xfd)]);
    }
    /// The geometry emits one sector per flash page, so erasing a list of
    /// sectors writes exactly one full page each, in order.
    #[test]
    fn selected_sectors_are_whole_pages() {
        for (geometry, count) in [(dx_geometry(), 4), (tiny_geometry(), 8)] {
            let sectors: Vec<FlashSector> = geometry
                .sectors()
                .take(count)
                .map(|info| FlashSector {
                    address: info.base_address,
                    size: info.size,
                })
                .collect();

            assert_eq!(sectors.len(), count);
            for (index, sector) in sectors.iter().enumerate() {
                assert_eq!(
                    sector.address(),
                    (index as u64) * u64::from(geometry.page_size)
                );
                assert_eq!(sector.size(), u64::from(geometry.page_size));
            }
        }
    }
}

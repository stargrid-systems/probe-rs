use probe_rs_target::{MemoryRange, MemoryRegion, NvmRegion, SectorInfo};

use crate::Session;
use crate::flashing::nvm_driver::{self, NvmDriver};
use crate::flashing::progress::ProgressOperation;
use crate::flashing::{DownloadOptions, FlashError, FlashLoader, flasher::Flasher};
use crate::flashing::{FlashLayout, FlashSector};

use super::FlashProgress;

/// One [`NvmDriver`] together with the regions it covers.
struct DriverWithRegions {
    core_index: usize,
    driver: Box<dyn NvmDriver>,
    regions: Vec<NvmRegion>,
}

/// Groups the target's NVM regions by the driver that erases them.
///
/// `accept` decides whether a region takes part. It runs after the region has been logged,
/// and it logs its own reason when it skips one.
fn drivers_for_regions(
    session: &mut Session,
    read_flasher_rtt: bool,
    mut accept: impl FnMut(&NvmRegion) -> bool,
) -> Result<Vec<DriverWithRegions>, FlashError> {
    let regions = session
        .target()
        .memory_map
        .iter()
        .filter_map(MemoryRegion::as_nvm_region)
        .cloned()
        .collect::<Vec<_>>();

    let mut drivers = Vec::<DriverWithRegions>::new();

    tracing::debug!("Regions:");
    for region in regions {
        tracing::debug!(
            "    region: {:#010x?} ({} bytes)",
            region.range,
            region.range.end - region.range.start
        );

        if !accept(&region) {
            continue;
        }

        // Get the first core that can access the region
        let Some(core_name) = region.cores.first().cloned() else {
            return Err(FlashError::NoNvmCoreAccess(region));
        };

        let core_index = session.target().core_index_by_name(&core_name).unwrap();

        // A target whose probe or debug sequence programs flash directly supplies its own
        // driver. Only a target without one needs a flash algorithm, so only then is a
        // missing algorithm an error.
        let driver = match nvm_driver::driver_for_region(session, &region, core_index) {
            Some(driver) => driver,
            None => {
                let target = session.target();
                let algo =
                    FlashLoader::get_flash_algorithm_for_region(&region, target, &core_name, &[])?;

                let mut flasher = Flasher::new(target, core_index, algo)?;
                flasher.read_rtt_output(read_flasher_rtt);

                Box::new(flasher) as Box<dyn NvmDriver>
            }
        };

        tracing::debug!("     -- using driver: {}", driver.name());

        // We don't usually have more than a handful of regions, linear search should be fine.
        match drivers.iter().position(|entry| {
            entry.driver.name() == driver.name() && entry.core_index == core_index
        }) {
            Some(index) => drivers[index].regions.push(region),
            None => drivers.push(DriverWithRegions {
                core_index,
                driver,
                regions: vec![region],
            }),
        }
    }

    Ok(drivers)
}

fn to_flash_sector(info: SectorInfo) -> FlashSector {
    FlashSector {
        address: info.base_address,
        size: info.size,
    }
}

/// Mass-erase all nonvolatile memory.
///
/// The optional progress will only be used to emit RTT messages.
/// No actual indication for the state of the erase all operation will be given.
pub fn erase_all(
    session: &mut Session,
    progress: &mut FlashProgress<'_>,
    read_flasher_rtt: bool,
) -> Result<(), FlashError> {
    tracing::debug!("Erasing all...");

    let mut drivers = drivers_for_regions(session, read_flasher_rtt, |region| {
        if region.is_alias {
            tracing::debug!("Skipping alias memory region {:#010x?}", region.range);
            return false;
        }
        true
    })?;

    let mut do_chip_erase = true;

    let mut phases = vec![];

    // Walk through the drivers to create a layout of the flash.
    for el in drivers.iter() {
        // If the first driver doesn't support erase all, disable chip erase.
        // TODO: we could sort by support but it's unlikely to make a difference.
        if do_chip_erase && !el.driver.is_chip_erase_supported(session) {
            do_chip_erase = false;
        }

        let mut layout = FlashLayout::default();

        for region in el.regions.iter() {
            for info in el.driver.geometry().sectors() {
                let range = info.address_range();

                if region.range.contains_range(&range) {
                    layout.sectors.push(to_flash_sector(info));
                }
            }
        }
        phases.push(layout);
    }

    if do_chip_erase {
        progress.add_progress_bar(ProgressOperation::Erase, None);
    } else {
        for phase in phases.iter() {
            let sector_size = phase.sectors().iter().map(|s| s.size()).sum::<u64>();

            progress.add_progress_bar(ProgressOperation::Erase, Some(sector_size));
        }
    }
    progress.initialized(phases);

    for el in drivers.iter_mut() {
        tracing::debug!("Erasing with driver: {}", el.driver.name());

        if el.driver.is_chip_erase_supported(session) {
            tracing::debug!("     -- chip erase supported, doing it.");
            el.driver.erase_all(session, progress)?;
        } else {
            tracing::debug!("     -- chip erase not supported, erasing by sector.");

            // loop over all sectors erasing them individually instead.

            let sectors = el
                .driver
                .geometry()
                .sectors()
                .filter(|info| {
                    let range = info.base_address..info.base_address + info.size;
                    el.regions.iter().any(|r| r.range.contains_range(&range))
                })
                .map(to_flash_sector)
                .collect::<Vec<_>>();

            el.driver
                .erase_selected_sectors(session, progress, &sectors)?;
        }
    }

    Ok(())
}

/// Erases flash covering `address_start..address_end`.
///
/// Flash can only be erased in whole sectors. Every sector that intersects the
/// requested range is erased.
///
/// When `restore` is `true`, bytes that lie inside those erased sectors but
/// outside `address_start..address_end` are read first and programmed back
/// afterwards, so only the requested range is left erased. When `restore` is
/// `false`, those bordering bytes stay erased.
// TODO: currently no progress other than RTT output is reported by anything in this function.
pub fn erase(
    session: &mut Session,
    progress: &mut FlashProgress<'_>,
    address_start: u64,
    address_end: u64,
    restore: bool,
    read_flasher_rtt: bool,
) -> Result<(), FlashError> {
    tracing::debug!("Erasing {address_start:08x}..{address_end:08x} (restore={restore})");

    let address_range = address_start..address_end;

    let mut drivers = drivers_for_regions(session, read_flasher_rtt, |region| {
        // If we have nothing to do in this region, ignore it.
        // This avoids uselessly initializing and deinitializing its flash algorithm.
        // We do not check for alias regions here, as we'll work with them if the range explicitly
        // targets them.
        if !region.range.intersects_range(&address_range) {
            tracing::debug!("     -- doesn't overlap, ignoring!");
            return false;
        }
        true
    })?;

    for el in drivers.iter_mut() {
        tracing::debug!("Erasing with driver: {}", el.driver.name());

        let sectors = el
            .driver
            .geometry()
            .sectors()
            .filter(|info| address_range.intersects_range(&info.address_range()))
            .filter(|info| {
                let range = info.base_address..info.base_address + info.size;
                el.regions.iter().any(|r| r.range.contains_range(&range))
            })
            .map(to_flash_sector)
            .collect::<Vec<_>>();

        let restore_data = if restore {
            read_restore_data(
                el.driver.as_mut(),
                session,
                &sectors,
                address_start,
                address_end,
            )?
        } else {
            Vec::new()
        };

        el.driver
            .erase_selected_sectors(session, progress, &sectors)?;

        if !restore_data.is_empty() {
            let mut loader = session.target().flash_loader();
            loader.read_rtt_output(read_flasher_rtt);
            for (address, data) in &restore_data {
                loader.add_data(*address, data)?;
            }

            loader.commit(
                session,
                DownloadOptions {
                    skip_erase: true,
                    ..Default::default()
                },
            )?;
        }
    }

    Ok(())
}

/// Read bytes that sit in `sectors` but outside `address_start..address_end`.
fn read_restore_data(
    driver: &mut dyn NvmDriver,
    session: &mut Session,
    sectors: &[FlashSector],
    address_start: u64,
    address_end: u64,
) -> Result<Vec<(u64, Vec<u8>)>, FlashError> {
    let mut ranges = Vec::new();

    for sector in sectors {
        let sector_start = sector.address();
        let sector_end = sector.address() + sector.size();

        if sector_start < address_start {
            let head_end = address_start.min(sector_end);
            if head_end > sector_start {
                ranges.push((sector_start, (head_end - sector_start) as usize));
            }
        }

        if sector_end > address_end {
            let tail_start = address_end.max(sector_start);
            if sector_end > tail_start {
                ranges.push((tail_start, (sector_end - tail_start) as usize));
            }
        }
    }

    if ranges.is_empty() {
        return Ok(Vec::new());
    }

    let mut restore_data = Vec::with_capacity(ranges.len());

    driver.with_reader(session, &mut |reader| {
        for &(address, len) in &ranges {
            let mut buf = vec![0; len];
            reader.read(address, &mut buf)?;
            restore_data.push((address, buf));
        }
        Ok(())
    })?;

    Ok(restore_data)
}

/// Check that a memory range has been erased.
pub fn run_blank_check(
    session: &mut Session,
    progress: &mut FlashProgress<'_>,
    address_start: u64,
    address_end: u64,
    read_flasher_rtt: bool,
) -> Result<(), FlashError> {
    tracing::debug!("Blank-checking {address_start:08x}..{address_end:08x}");

    let address_range = address_start..address_end;

    let mut drivers = drivers_for_regions(session, read_flasher_rtt, |region| {
        // If we have nothing to do in this region, ignore it.
        // This avoids uselessly initializing and deinitializing its flash algorithm.
        // We do not check for alias regions here, as we'll work with them if the range explicitly
        // targets them.
        if !region.range.intersects_range(&address_range) {
            tracing::debug!("     -- doesn't overlap, ignoring!");
            return false;
        }
        true
    })?;

    for el in drivers.iter_mut() {
        tracing::debug!("Blank-checking with driver: {}", el.driver.name());

        let sectors = el
            .driver
            .geometry()
            .sectors()
            .filter(|info| address_range.contains_range(&info.address_range()))
            .filter(|info| {
                let range = info.base_address..info.base_address + info.size;
                el.regions.iter().any(|r| r.range.contains_range(&range))
            })
            .map(to_flash_sector)
            .collect::<Vec<_>>();

        el.driver.blank_check(session, progress, &sectors)?;
    }

    Ok(())
}

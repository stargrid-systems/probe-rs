use probe_rs_target::NvmRegion;

use crate::flashing::builder::FlashBuilder;
use crate::flashing::nvm_driver::{FlashData, LoadedRegion, NvmDriver, fill_pages};
use crate::flashing::{FlashError, FlashProgress};
use crate::session::Session;

/// One [`NvmDriver`] together with the regions it programs.
///
/// [`FlashLoader::commit`] groups the staged data by driver and then runs the phases
/// against each plan in turn.
///
/// [`FlashLoader::commit`]: crate::flashing::FlashLoader::commit
pub(super) struct FlashPlan {
    /// Index of the core the driver works through.
    pub(super) core_index: usize,
    pub(super) driver: Box<dyn NvmDriver>,
    pub(super) regions: Vec<LoadedRegion>,
}

impl FlashPlan {
    pub(super) fn new(core_index: usize, driver: Box<dyn NvmDriver>) -> Self {
        Self {
            core_index,
            driver,
            regions: Vec::new(),
        }
    }

    /// Plans the sectors and pages for `region` and adds it to this plan.
    ///
    /// A region the driver cannot program is refused here, before any page is
    /// planned and before any phase can touch the target.
    pub(super) fn add_region(
        &mut self,
        region: NvmRegion,
        builder: &FlashBuilder,
        restore_unwritten_bytes: bool,
    ) -> Result<(), FlashError> {
        if !self.driver.programs(&region) {
            return Err(FlashError::RegionNotProgrammable {
                driver: self.driver.name().to_string(),
                range: region.range.clone(),
            });
        }

        let layout = builder.build_sectors_and_pages(
            &region,
            self.driver.geometry(),
            restore_unwritten_bytes,
        )?;
        self.regions.push(LoadedRegion {
            region,
            data: FlashData::Raw(layout),
        });
        Ok(())
    }

    /// Erases all flash memory covered by this plan's driver.
    pub(super) fn erase_all(
        &mut self,
        session: &mut Session,
        progress: &mut FlashProgress<'_>,
    ) -> Result<(), FlashError> {
        self.driver.erase_all(session, progress)
    }

    /// Verifies the staged data against the target.
    pub(super) fn verify(
        &mut self,
        session: &mut Session,
        progress: &mut FlashProgress<'_>,
        ignore_filled: bool,
    ) -> Result<bool, FlashError> {
        self.driver
            .verify(session, progress, &mut self.regions, ignore_filled)
    }

    /// Writes the staged data to flash.
    ///
    /// If `restore_unwritten_bytes` is `true`, all bytes of a sector, that are not to be
    /// written during flashing will be read from the flash first and written again once
    /// the sector is erased.
    pub(super) fn program(
        &mut self,
        session: &mut Session,
        progress: &mut FlashProgress<'_>,
        restore_unwritten_bytes: bool,
        enable_double_buffering: bool,
        skip_erasing: bool,
        verify: bool,
    ) -> Result<(), FlashError> {
        tracing::debug!("Starting program procedure.");

        tracing::debug!("Double Buffering enabled: {:?}", enable_double_buffering);
        tracing::debug!(
            "Restoring unwritten bytes enabled: {:?}",
            restore_unwritten_bytes
        );

        if restore_unwritten_bytes {
            self.fill_unwritten(session, progress)?;
        }

        // Skip erase if necessary (i.e. chip erase was done before), or if the driver
        // erases each page as it writes it.
        if !skip_erasing && !self.driver.program_erases_page() {
            self.driver
                .erase_sectors(session, progress, &mut self.regions)?;
        }

        self.driver.program_pages(
            session,
            progress,
            &mut self.regions,
            enable_double_buffering,
        )?;

        if verify && !self.verify(session, progress, !restore_unwritten_bytes)? {
            return Err(FlashError::Verify);
        }

        Ok(())
    }

    /// Reads the target contents into the parts of the pages that hold no staged data.
    fn fill_unwritten(
        &mut self,
        session: &mut Session,
        progress: &mut FlashProgress<'_>,
    ) -> Result<(), FlashError> {
        progress.started_filling();

        let Self {
            driver, regions, ..
        } = self;

        let result =
            driver.with_reader(session, &mut |reader| fill_pages(regions, progress, reader));

        match result.is_ok() {
            true => progress.finished_filling(),
            false => progress.failed_filling(),
        }

        result
    }
}

#[cfg(all(test, feature = "builtin-targets"))]
mod tests {
    use probe_rs_target::{PageInfo, SectorInfo};
    use std::cell::RefCell;
    use std::rc::Rc;

    use super::*;
    use crate::flashing::{NvmGeometry, NvmReader};
    use crate::probe::Probe;
    use crate::probe::fake_probe::FakeProbe;
    use crate::{Permissions, Session};

    #[derive(Debug, PartialEq, Eq)]
    enum Phase {
        Read,
        EraseAll,
        EraseSectors,
        Program,
        Verify,
    }

    struct TestGeometry;

    impl NvmGeometry for TestGeometry {
        fn sector_info(&self, address: u64) -> Option<SectorInfo> {
            Some(SectorInfo {
                base_address: address & !0x3ff,
                size: 0x400,
            })
        }

        fn page_info(&self, address: u64) -> Option<PageInfo> {
            Some(PageInfo {
                base_address: address & !0xff,
                size: 0x100,
            })
        }

        fn sectors(&self) -> Box<dyn Iterator<Item = SectorInfo> + '_> {
            Box::new(std::iter::empty())
        }

        fn pages(&self) -> Box<dyn Iterator<Item = PageInfo> + '_> {
            Box::new(std::iter::empty())
        }

        fn erased_byte_value(&self) -> u8 {
            0xff
        }
    }

    struct NullReader;

    impl NvmReader for NullReader {
        fn read(&mut self, _address: u64, _data: &mut [u8]) -> Result<(), FlashError> {
            Ok(())
        }
    }

    /// A driver that records the phases the plan asks it for.
    struct RecordingDriver {
        geometry: TestGeometry,
        phases: Rc<RefCell<Vec<Phase>>>,
        erases_on_write: bool,
        programmable: bool,
    }

    impl NvmDriver for RecordingDriver {
        fn name(&self) -> &str {
            "recording"
        }

        fn geometry(&self) -> &dyn NvmGeometry {
            &self.geometry
        }

        fn program_erases_page(&self) -> bool {
            self.erases_on_write
        }

        fn programs(&self, _region: &NvmRegion) -> bool {
            self.programmable
        }

        fn is_chip_erase_supported(&self, _session: &Session) -> bool {
            true
        }

        fn erase_all(
            &mut self,
            _session: &mut Session,
            _progress: &mut FlashProgress<'_>,
        ) -> Result<(), FlashError> {
            self.phases.borrow_mut().push(Phase::EraseAll);
            Ok(())
        }

        fn erase_sectors(
            &mut self,
            _session: &mut Session,
            _progress: &mut FlashProgress<'_>,
            _regions: &mut [LoadedRegion],
        ) -> Result<(), FlashError> {
            self.phases.borrow_mut().push(Phase::EraseSectors);
            Ok(())
        }

        fn program_pages(
            &mut self,
            _session: &mut Session,
            _progress: &mut FlashProgress<'_>,
            _regions: &mut [LoadedRegion],
            _double_buffering: bool,
        ) -> Result<(), FlashError> {
            self.phases.borrow_mut().push(Phase::Program);
            Ok(())
        }

        fn with_reader(
            &mut self,
            _session: &mut Session,
            f: &mut dyn FnMut(&mut dyn NvmReader) -> Result<(), FlashError>,
        ) -> Result<(), FlashError> {
            self.phases.borrow_mut().push(Phase::Read);
            f(&mut NullReader)
        }

        fn verify(
            &mut self,
            _session: &mut Session,
            _progress: &mut FlashProgress<'_>,
            _regions: &mut [LoadedRegion],
            _ignore_filled: bool,
        ) -> Result<bool, FlashError> {
            self.phases.borrow_mut().push(Phase::Verify);
            Ok(true)
        }
    }

    fn fake_session() -> Session {
        Probe::from_specific_probe(Box::new(FakeProbe::with_mocked_core()))
            .attach("nrf51822_xxAC", Permissions::default())
            .expect("Failed to attach with 'fake' probe.")
    }

    /// Runs one program phase and returns the phases the driver was asked for.
    fn run(erases_on_write: bool, restore_unwritten: bool, skip_erasing: bool) -> Vec<Phase> {
        let phases = Rc::new(RefCell::new(Vec::new()));
        let driver = RecordingDriver {
            geometry: TestGeometry,
            phases: phases.clone(),
            erases_on_write,
            programmable: true,
        };

        let mut session = fake_session();
        let mut plan = FlashPlan::new(0, Box::new(driver));

        plan.program(
            &mut session,
            &mut FlashProgress::empty(),
            restore_unwritten,
            false,
            skip_erasing,
            true,
        )
        .unwrap();

        drop(plan);
        Rc::try_unwrap(phases).unwrap().into_inner()
    }

    #[test]
    fn program_runs_fill_erase_program_verify_in_order() {
        assert_eq!(
            run(false, true, false),
            [
                Phase::Read,
                Phase::EraseSectors,
                Phase::Program,
                Phase::Verify
            ]
        );
    }

    #[test]
    fn a_driver_that_erases_on_write_skips_the_erase_phase() {
        assert_eq!(run(true, false, false), [Phase::Program, Phase::Verify]);
    }

    #[test]
    fn skip_erasing_skips_the_erase_phase() {
        assert_eq!(run(false, false, true), [Phase::Program, Phase::Verify]);
    }

    #[test]
    fn erase_all_goes_to_the_driver() {
        let phases = Rc::new(RefCell::new(Vec::new()));
        let driver = RecordingDriver {
            geometry: TestGeometry,
            phases: phases.clone(),
            erases_on_write: false,
            programmable: true,
        };

        let mut session = fake_session();
        let mut plan = FlashPlan::new(0, Box::new(driver));

        plan.erase_all(&mut session, &mut FlashProgress::empty())
            .unwrap();

        drop(plan);
        assert_eq!(
            Rc::try_unwrap(phases).unwrap().into_inner(),
            [Phase::EraseAll]
        );
    }

    /// Data for a region the driver cannot program is refused when the region
    /// joins the plan, before the driver is asked for any phase.
    #[test]
    fn a_region_the_driver_cannot_program_is_refused_when_the_plan_is_built() {
        let phases = Rc::new(RefCell::new(Vec::new()));
        let driver = RecordingDriver {
            geometry: TestGeometry,
            phases: phases.clone(),
            erases_on_write: false,
            programmable: false,
        };

        let region = NvmRegion {
            name: Some("config".to_string()),
            range: 0x1000..0x1100,
            cores: vec!["main".to_string()],
            is_alias: false,
            access: None,
        };

        let mut builder = FlashBuilder::new();
        builder.add_data(0x1000, &[1, 2, 3]).unwrap();

        let mut plan = FlashPlan::new(0, Box::new(driver));

        let error = plan.add_region(region, &builder, false).unwrap_err();

        assert!(matches!(
            error,
            FlashError::RegionNotProgrammable { ref driver, ref range }
                if driver == "recording" && *range == (0x1000..0x1100)
        ));

        drop(plan);
        assert!(Rc::try_unwrap(phases).unwrap().into_inner().is_empty());
    }
}

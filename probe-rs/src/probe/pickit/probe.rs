//! The [`DebugProbe`] implementation for a PICkit.
//!
//! The tool speaks UPDI and nothing else. Almost every method of
//! [`DebugProbe`] describes an ARM or JTAG capability that has no meaning here,
//! so those report that the probe does not support the operation.

use crate::architecture::avr::communication_interface::{
    AvrCommunicationInterface, AvrDebugInterfaceState, AvrError,
};
use crate::probe::{
    DebugProbe, DebugProbeError, DebugProbeInfo, DebugProbeSelector, ProbeFactory, WireProtocol,
    list::{ProbeListItem, usb_probe_accessibility},
};

use super::{Pickit, PickitError, protocol::ERROR_STATUS_KEY};

/// A factory for creating [`PickitProbe`] probes.
#[derive(Debug)]
pub struct PickitFactory;

impl std::fmt::Display for PickitFactory {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str("PICkit")
    }
}

impl ProbeFactory for PickitFactory {
    fn open(&self, selector: &DebugProbeSelector) -> Result<Box<dyn DebugProbe>, DebugProbeError> {
        let pickit = Pickit::open(selector)?;

        Ok(Box::new(PickitProbe {
            pickit,
            speed_khz: 0,
        }))
    }

    fn list_probes(&self) -> Vec<ProbeListItem> {
        super::list_devices()
            .iter()
            .map(|device| ProbeListItem {
                info: DebugProbeInfo::new(
                    device.product_string().unwrap_or("PICkit").to_string(),
                    device.vendor_id(),
                    device.product_id(),
                    device.serial_number().map(str::to_string),
                    &PickitFactory,
                    None,
                    false,
                ),
                accessibility: usb_probe_accessibility(device),
            })
            .collect()
    }
}

/// A PICkit driving an AVR over UPDI.
///
/// The probe owns the session state machine and hands out an
/// [`AvrCommunicationInterface`] that borrows it. See
/// [`crate::probe::pickit`] for the rules that keep the tool from hanging.
#[derive(Debug)]
pub struct PickitProbe {
    pickit: Pickit,
    speed_khz: u32,
}

impl PickitProbe {
    /// The session state machine underneath.
    pub fn pickit(&mut self) -> &mut Pickit {
        &mut self.pickit
    }
}

impl DebugProbe for PickitProbe {
    fn get_name(&self) -> &str {
        "PICkit"
    }

    fn speed_khz(&self) -> u32 {
        self.speed_khz
    }

    /// Records the UPDI clock, which takes effect when a session opens.
    ///
    /// The tool only accepts a clock while a UPDI session exists, and the probe
    /// has none until an AVR interface enters programming mode. The value is
    /// therefore remembered and handed to that interface.
    fn set_speed(&mut self, speed_khz: u32) -> Result<u32, DebugProbeError> {
        self.speed_khz = speed_khz;

        Ok(speed_khz)
    }

    /// Checks that the tool answers, without touching the target.
    ///
    /// There is no protocol init. Scripts run from a cold open, and the first
    /// one has to be `EnterProgMode`, which the AVR interface sends.
    fn attach(&mut self) -> Result<(), DebugProbeError> {
        let status = self.pickit.status(ERROR_STATUS_KEY).map_err(probe_error)?;
        tracing::debug!("PICkit reports status {status:?}");

        Ok(())
    }

    fn detach(&mut self) -> Result<(), crate::Error> {
        self.pickit.exit().map_err(probe_error)?;

        Ok(())
    }

    /// Not supported. The PICkit Basic has no reset line to the target.
    ///
    /// Resetting an AVR over UPDI is a debug operation that needs an open
    /// session, so it belongs to the AVR interface rather than here.
    fn target_reset(&mut self) -> Result<(), DebugProbeError> {
        Err(DebugProbeError::CommandNotSupportedByProbe {
            command_name: "target_reset",
        })
    }

    fn target_reset_assert(&mut self) -> Result<(), DebugProbeError> {
        Err(DebugProbeError::CommandNotSupportedByProbe {
            command_name: "target_reset_assert",
        })
    }

    fn target_reset_deassert(&mut self) -> Result<(), DebugProbeError> {
        Err(DebugProbeError::CommandNotSupportedByProbe {
            command_name: "target_reset_deassert",
        })
    }

    /// Not supported. UPDI is neither SWD nor JTAG.
    fn select_protocol(&mut self, protocol: WireProtocol) -> Result<(), DebugProbeError> {
        Err(DebugProbeError::UnsupportedProtocol(protocol))
    }

    fn active_protocol(&self) -> Option<WireProtocol> {
        None
    }

    fn has_avr_interface(&self) -> bool {
        true
    }

    fn try_get_avr_interface<'probe>(
        &'probe mut self,
        state: &'probe mut AvrDebugInterfaceState,
    ) -> Result<AvrCommunicationInterface<'probe>, AvrError> {
        if self.speed_khz > 0 {
            state.set_speed_khz(self.speed_khz);
        }

        Ok(AvrCommunicationInterface::new(&mut self.pickit, state))
    }

    /// Always `None`. The PICkit Basic cannot measure the target voltage.
    fn get_target_voltage(&mut self) -> Result<Option<f32>, DebugProbeError> {
        Ok(None)
    }

    fn into_probe(self: Box<Self>) -> Box<dyn DebugProbe> {
        self
    }
}

fn probe_error(err: PickitError) -> DebugProbeError {
    DebugProbeError::ProbeSpecific(err.into())
}

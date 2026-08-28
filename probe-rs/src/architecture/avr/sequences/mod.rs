//! Debug sequences to operate special requirements AVR targets.

use std::fmt::Debug;
use std::sync::Arc;

/// A interface to operate debug sequences for AVR targets.
///
/// Should be implemented on a custom handle for chips that require special sequence code.
///
/// The trait has no hooks yet. AVR support is still being built up, so hooks are added
/// once the communication interface can back them.
pub trait AvrDebugSequence: Send + Sync + Debug {}

/// The default sequences that is used for AVR chips that do not specify a specific sequence.
#[derive(Debug)]
pub struct DefaultAvrSequence(pub(crate) ());

impl DefaultAvrSequence {
    /// Creates a new default AVR debug sequence.
    pub fn create() -> Arc<dyn AvrDebugSequence> {
        Arc::new(Self(()))
    }
}

impl AvrDebugSequence for DefaultAvrSequence {}

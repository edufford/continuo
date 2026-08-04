//! The Zenoh delivery sink, behind the `zenoh` feature.
//!
//! Kept to delivery alone. Everything with design content in it, meaning what
//! is observed and how it is framed, lives in the default build, so the
//! schema is exercised by tests that never link Zenoh.
//!
//! A frame is published on the message's **own** key expression with the
//! component's payload bytes unchanged, and the sim time, publisher, and
//! sequence number ride along as a Zenoh attachment. That is what makes the
//! viewer final: at milestone 7 components publish those same keys with those
//! same payloads themselves, and only the attachment's provenance changes.

use tracing::warn;
use zenoh::{Session, Wait};

use crate::sink::{VizFrame, VizSink};

/// Publishes frames onto a Zenoh session.
pub struct ZenohSink {
    session: Session,
    /// Counted rather than propagated, because a publish failure is the
    /// viewer's problem and never the run's.
    num_failures: u64,
}

impl ZenohSink {
    /// Opens a peer session with Zenoh's default configuration, which
    /// discovers other peers on the local network without a broker.
    pub fn new() -> Result<Self, Box<dyn std::error::Error + Send + Sync>> {
        let session = zenoh::open(zenoh::Config::default()).wait()?;

        // Return the sink, ready to publish.
        Ok(ZenohSink {
            session,
            num_failures: 0,
        })
    }

    /// How many publishes failed.
    pub fn num_failures(&self) -> u64 {
        self.num_failures
    }
}

impl VizSink for ZenohSink {
    fn deliver(&mut self, frame: &VizFrame) {
        let put = self
            .session
            .put(frame.key.clone(), frame.payload.clone())
            .attachment(frame.metadata.clone())
            .wait();
        if put.is_err() {
            self.num_failures += 1;
        }
    }

    fn flush(&mut self) {
        if self.num_failures > 0 {
            warn!(
                target: "continuo::viz",
                num_failures = self.num_failures,
                "some viewer frames could not be published"
            );
        }
    }
}

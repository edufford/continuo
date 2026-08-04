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
    ///
    /// Convenience over [`Self::with_config`] for the demo case. Peer
    /// discovery on the local network is the right default for watching a
    /// run on one machine and the wrong one for anything deployed.
    pub fn new() -> Result<Self, Box<dyn std::error::Error + Send + Sync>> {
        // Return a sink on a default peer session.
        ZenohSink::with_config(zenoh::Config::default())
    }

    /// Opens a session with a caller-supplied configuration, for setting the
    /// transport topology, endpoints, or anything else Zenoh exposes.
    // TODO(M7): the distributed host binary will want this surfaced through
    // scenario configuration rather than constructed in code, so a run's
    // topology is declared with the run rather than compiled into whoever
    // starts the viewer.
    pub fn with_config(
        config: zenoh::Config,
    ) -> Result<Self, Box<dyn std::error::Error + Send + Sync>> {
        let session = zenoh::open(config).wait()?;

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
    fn deliver(&mut self, frame: VizFrame) {
        // Taking the frame by value means the payload and attachment move
        // straight into `put`, so the bytes are copied once into the frame
        // and never again.
        let put = self
            .session
            .put(frame.key, frame.payload)
            .attachment(frame.metadata)
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

//! Open-loop resimulation: replaying recorded messages back into a live
//! sim as stimulus (see [`crate::record`] for the log itself and for how
//! this differs from verification).
//!
//! This lives beside the conductor rather than in `continuo-actors`, since
//! it is harness machinery built on [`EventLog`], not a sample actor, and
//! putting it in the actors crate would make that crate depend on this one.

use continuo_core::{Component, ComponentId, CoreError, KeyExpr, SimTime, StepCtx};
use serde_json::value::RawValue;

use crate::record::{EventLog, LogEvent};

/// Replays one recorded publisher's messages as an ordinary component,
/// the open-loop resimulation stimulus.
///
/// Built from an event log filtered to one publisher path (including its
/// sub-components): the recorded messages are re-published at their
/// recorded sim times, on their recorded keys, with byte-identical
/// payloads. Downstream components see them exactly as if the original
/// were running, so a live component can be swapped for its playback
/// double without consumers noticing. A playback double never reacts to
/// the live world. Its behavior is pure data, which also keeps hybrid
/// runs fully deterministic and recordable.
pub struct PlaybackComponent {
    id: ComponentId,
    /// (time, key, payload) in recorded order.
    messages: Vec<(SimTime, KeyExpr, Box<RawValue>)>,
    cursor: usize,
}

impl PlaybackComponent {
    /// Filters `log` to messages recorded from `publisher` (a component
    /// path string) or any of its sub-components. `id` is the playback
    /// double's own registration id, typically the original actor's name.
    pub fn from_log(id: ComponentId, log: &EventLog, publisher: &str) -> Self {
        let prefix = format!("{publisher}/");
        let messages = log
            .events
            .iter()
            .filter_map(|event| match event {
                LogEvent::Msg(m)
                    if m.publisher == publisher || m.publisher.starts_with(&prefix) =>
                {
                    Some((
                        m.sim_time,
                        KeyExpr::new(m.key.clone()).expect("recorded keys are valid"),
                        m.payload.clone(),
                    ))
                }
                _ => None,
            })
            .collect();

        // Return the playback double, positioned at the start of its recording.
        PlaybackComponent {
            id,
            messages,
            cursor: 0,
        }
    }
}

impl Component for PlaybackComponent {
    fn id(&self) -> ComponentId {
        self.id.clone()
    }

    fn subscriptions(&self) -> Vec<KeyExpr> {
        Vec::new()
    }

    fn step(&mut self, ctx: &mut StepCtx) -> Result<SimTime, CoreError> {
        // Publish everything recorded for this instant; skip anything the
        // schedule somehow passed over (e.g. a playback double registered after its
        // first recorded messages) rather than stalling the run on it.
        while let Some((time, key, payload)) = self.messages.get(self.cursor) {
            if *time > ctx.now() {
                break;
            }
            if *time == ctx.now() {
                // Cannot fail: a recorded payload is already-serialized JSON,
                // so publishing it is a copy, and the publisher's non-finite
                // guard cannot see into one. A `null` left by a NaN in an
                // older log therefore republishes unchanged, which is what
                // replaying a recording verbatim means. That failure still
                // lands, at whichever consumer decodes it.
                ctx.publish(key.clone(), payload)?;
            }
            self.cursor += 1;
        }

        // Return the next recorded message time, or effectively never once
        // the recording is exhausted.
        Ok(match self.messages.get(self.cursor) {
            Some((time, _, _)) => *time,
            None => SimTime::from_nanos(i64::MAX),
        })
    }
}

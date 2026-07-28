//! Join metadata (milestone 4): what the conductor needs in order to admit
//! a component, as distinct from the component itself.
//!
//! Kept separate from the `Box<dyn Component>` on purpose. A remote
//! component (milestone 7) lives in another process and the conductor never
//! holds its box — but scheduling, the visibility rule, and sequence-number
//! assignment all work off this metadata alone, so the same admission path
//! serves both once the join arrives over the transport instead of as a
//! direct call.

use continuo_core::SimTime;

/// Everything the conductor needs to admit a component.
// TODO(M4): the step budget and its timeout policy join this struct in the
// per-component timing section.
// TODO(M7): so does the coupled/decoupled flag (PLAN.md decision
// 2026-07-18) — decoupled children take next-step visibility, which frees
// their host placement.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct JoinMetadata {
    /// `""` for a world-level actor, or a composite's path to join it. The
    /// component's own id completes the path.
    pub parent: String,
    /// Sim time of the component's first step.
    ///
    /// Declared, never inferred: only the joiner knows its own phase. The
    /// conductor puts it in the schedule as the component is admitted, so
    /// by the time that instant arrives the newcomer is already among the
    /// components due there — the barrier waits for it rather than stepping
    /// the instant without it.
    ///
    /// It must be an instant the conductor has not stepped past; joining
    /// into an instant that already happened is an error, not a silent
    /// no-op.
    pub first_due: SimTime,
}

impl JoinMetadata {
    /// Joins before the run starts, first stepping at sim time zero — what
    /// every component in a statically-built world wants.
    ///
    /// This is what you get by passing just the parent path where a join is
    /// expected: `add_component("car1", component)` is shorthand for
    /// `add_component(JoinMetadata::at_start("car1"), component)`.
    pub fn at_start(parent: impl Into<String>) -> Self {
        // Return a join for the world's opening instant.
        JoinMetadata {
            parent: parent.into(),
            first_due: SimTime::ZERO,
        }
    }

    /// Joins a run already in progress, first stepping at `first_due`.
    pub fn at(parent: impl Into<String>, first_due: SimTime) -> Self {
        // Return a join scheduled for a specific instant.
        JoinMetadata {
            parent: parent.into(),
            first_due,
        }
    }
}

/// Lets the parent path be passed on its own wherever a join is expected,
/// so building a static world stays `add_component("car1", component)`
/// instead of `add_component(JoinMetadata::at_start("car1"), component)`.
///
/// It always means [`JoinMetadata::at_start`], so it is only usable before
/// the run begins. Offered to a running conductor it resolves to sim time
/// zero — long closed — and the join is rejected rather than quietly
/// landing at some instant the caller never chose.
impl From<&str> for JoinMetadata {
    fn from(parent: &str) -> Self {
        JoinMetadata::at_start(parent)
    }
}

impl From<String> for JoinMetadata {
    fn from(parent: String) -> Self {
        JoinMetadata::at_start(parent)
    }
}

impl From<&String> for JoinMetadata {
    fn from(parent: &String) -> Self {
        JoinMetadata::at_start(parent.clone())
    }
}

//! Membership metadata (milestone 4): what the conductor needs in order to
//! admit a component or remove one, as distinct from the component itself.
//!
//! Kept separate from the `Box<dyn Component>` on purpose. A remote
//! component (milestone 7) lives in another process and the conductor never
//! holds its box, but scheduling, the visibility rule, and sequence-number
//! assignment all work off this metadata alone, so the same admission path
//! serves both once the request arrives over the transport instead of as a
//! direct call.
//!
//! Both sides name the sim time they take effect at rather than inferring it
//! from when the request turned up, which is what keeps a dynamic run
//! reproducible when arrival varies. DECISIONS.md, 2026-07-28, has the rest,
//! including why the interval is half-open.

use continuo_core::SimTime;

use crate::timing::StepTiming;

/// Parent path of a component that sits directly under the world, with no
/// composite above it, so its own id is the whole path.
///
/// The empty string is not a placeholder here, it is the root path:
/// [`continuo_core::ComponentPath::parse`] maps `""` to the root, and a
/// [`continuo_core::ComponentId`] is never empty, so no real composite can
/// collide with it. Named because the bare literal reads at a call site
/// like a forgotten argument, where
/// `add_component_at_start(WORLD_LEVEL, component)` says what it means.
pub const WORLD_LEVEL: &str = "";

/// Everything the conductor needs to admit a component.
// TODO(M7): the coupled/decoupled flag (PLAN.md decision 2026-07-18) joins
// this struct too, since decoupled children take next-step visibility, which
// frees their host placement.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct JoinMetadata {
    /// [`WORLD_LEVEL`] for a world-level actor, or a composite's path to
    /// join it. The component's own id completes the path.
    pub parent_path: String,
    /// Sim time of the component's first step.
    ///
    /// Declared, never inferred: only the joiner knows its own phase. The
    /// conductor puts it in the schedule as the component is admitted, so
    /// by the time that instant arrives the newcomer is already among the
    /// components due there, so the barrier waits for it rather than stepping
    /// the instant without it.
    ///
    /// It must be an instant the conductor has not stepped past; joining
    /// into an instant that already happened is an error, not a silent
    /// no-op.
    pub first_due: SimTime,
    /// What this component's `step` may cost in wall time, and what happens
    /// when it costs more. Declares no limits by default.
    ///
    /// Registration is where a deadline belongs, because a deadline is a
    /// property of the deployment rather than of the model. See
    /// [`StepTiming`].
    pub timing: StepTiming,
}

impl JoinMetadata {
    /// Joins before the run starts, first stepping at sim time zero, which
    /// is what every component in a statically-built world wants.
    ///
    /// `Conductor::add_component_at_start` builds one of these, so reach for
    /// this directly only to add `with_timing` to a start-time join.
    pub fn at_start(parent_path: impl Into<String>) -> Self {
        // Return a join for the world's opening instant.
        JoinMetadata {
            parent_path: parent_path.into(),
            first_due: SimTime::ZERO,
            timing: StepTiming::unlimited(),
        }
    }

    /// Joins a run already in progress, first stepping at `first_due`.
    pub fn at(parent_path: impl Into<String>, first_due: SimTime) -> Self {
        // Return a join scheduled for a specific instant.
        JoinMetadata {
            parent_path: parent_path.into(),
            first_due,
            timing: StepTiming::unlimited(),
        }
    }

    /// Declares what this component's `step` may cost in wall time:
    /// `add_component(JoinMetadata::at_start("car1").with_timing(timing), c)`.
    pub fn with_timing(self, timing: StepTiming) -> Self {
        // Return the join carrying its step limits.
        JoinMetadata { timing, ..self }
    }
}

/// Everything the conductor needs to remove a component.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct LeaveMetadata {
    /// The departing component's full path.
    pub path: String,
    /// The first instant the component does *not* step, or `None` to stop
    /// it at the earliest instant still open, meaning "now".
    ///
    /// Naming an instant is half-open, mirroring
    /// [`JoinMetadata::first_due`]: a component present for `[0, 10ms)`
    /// joins at `0` and leaves at `10ms`, having stepped at `0` but not at
    /// `10ms`. Adjacent lifetimes therefore abut without off-by-one
    /// reasoning about periods, since one component's `leaves_at` is the next
    /// one's `first_due`.
    ///
    /// Prefer naming it for anything a run must reproduce. `None` stops the
    /// component wherever the caller happens to be, which is deterministic
    /// only because the caller is; a named instant gives the same run
    /// whenever the request was made, and keeps doing so once requests
    /// travel over a network.
    pub leaves_at: Option<SimTime>,
}

impl LeaveMetadata {
    /// Leaves at the earliest instant still open: the component takes
    /// no further step from here.
    pub fn now(path: impl Into<String>) -> Self {
        // Return a leave the conductor resolves to its next instant.
        LeaveMetadata {
            path: path.into(),
            leaves_at: None,
        }
    }

    /// Leaves at `leaves_at`: the component steps at every due instant
    /// before that one, and none from it onwards.
    pub fn at(path: impl Into<String>, leaves_at: SimTime) -> Self {
        // Return a leave scheduled for a specific instant.
        LeaveMetadata {
            path: path.into(),
            leaves_at: Some(leaves_at),
        }
    }
}

// A bare path used to stand in for either of these, so the conductor took
// `impl Into<JoinMetadata>` and the argument's type picked the timing. Each
// side now has a method of its own for the common case, which says in its
// name what the conversion said nowhere.

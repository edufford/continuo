use continuo_core::{ComponentPath, CoreError, SimTime};
use thiserror::Error;

#[derive(Debug, Error)]
pub enum ConductorError {
    #[error("a component is already registered at path {0}")]
    DuplicatePath(ComponentPath),

    #[error("no component is registered at path {0}")]
    UnknownPath(ComponentPath),

    #[error(
        "component {path} asked to first step at {first_due}, but the earliest \
         instant still open is {earliest_open}: a joining component must be \
         scheduled for a step that has not happened yet"
    )]
    JoinInThePast {
        path: ComponentPath,
        first_due: SimTime,
        earliest_open: SimTime,
    },

    #[error(
        "component {path} was asked to stop at {leaves_at}, but the earliest \
         instant still open is {earliest_open}: it has already stepped at \
         instants this leave claims it did not"
    )]
    LeaveInThePast {
        path: ComponentPath,
        leaves_at: SimTime,
        earliest_open: SimTime,
    },

    #[error(
        "path {new} conflicts with existing {existing}: a leaf component cannot also be a composite"
    )]
    PathConflict {
        existing: ComponentPath,
        new: ComponentPath,
    },

    #[error(
        "schedule violation by {path}: returned next_due {next_due} at sim time {now}; \
         next_due must be strictly in the future (>= 1 ns ahead) to prevent zero-time livelock"
    )]
    ScheduleViolation {
        path: ComponentPath,
        now: SimTime,
        next_due: SimTime,
    },

    #[error(transparent)]
    Core(#[from] CoreError),
}

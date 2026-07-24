use continuo_core::{ComponentPath, CoreError, SimTime};
use thiserror::Error;

#[derive(Debug, Error)]
pub enum ConductorError {
    #[error("a component is already registered at path {0}")]
    DuplicatePath(ComponentPath),

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

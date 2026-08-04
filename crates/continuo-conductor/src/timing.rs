//! Per-component step timing (milestone 4): the wall-clock limits a
//! component declares when it joins, and what the conductor does when one is
//! passed.
//!
//! Two levels in one declaration, answering different questions:
//!
//! - **budget**: soft, and permanently so. Did *this component's* `step`
//!   finish in time? A step over it is logged and counted, and that is all,
//!   so a run that misses every budget produces the identical world hash to
//!   one that misses none.
//! - **timeout**: hard, and what it does is declared with it: halt the world,
//!   or remove the component. This is the *conductor's* wait, which is why it
//!   is the level that acts.
//!
//! In-process the two coincide, because the conductor's wait *is* the call, so
//! one measured duration is what both are judged against here. They separate
//! at milestone 7, when the budget is measured by the host running the step
//! and the timeout keeps the transport in it. They are judged separately
//! either way, never collapsed into one verdict, because the pair is what
//! carries the diagnosis.
//!
//! Neither level is milestone 3's pacing overrun, which asks whether *the
//! schedule as a whole* tracked the wall clock and blames no component in
//! particular. Nor is timing a pacing mode: a wedged component hangs the
//! barrier in free-run just as readily. The default declares neither limit.
//!
//! Limits belong to registration ([`JoinMetadata`](crate::JoinMetadata))
//! rather than to the `Component` trait, since a deadline is a property of the
//! deployment and not of the model, which also keeps wall-clock types out of
//! `continuo-core`.
//!
//! DECISIONS.md, 2026-07-28, has the arguments: why the soft level is
//! permanently soft, why a worst-level verdict was rejected, and what a
//! timing verdict may not do to the tick it was measured in.

use std::time::Duration;

use serde::{Deserialize, Serialize};

/// What a component's `step` may cost in wall time, and how the conductor
/// escalates when it costs more.
///
/// Declared per component at registration, as part of
/// [`JoinMetadata`](crate::JoinMetadata). The default,
/// [`StepTiming::unlimited`], declares nothing: the component is never
/// flagged, halted, or removed for being slow.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct StepTiming {
    /// Soft limit on the component's own `step`: a step taking longer is
    /// logged and counted (see
    /// [`Conductor::budget_misses`](crate::Conductor::budget_misses)), and
    /// nothing else, ever. `None` declares no budget, meaning the component
    /// has no deadline worth flagging.
    pub budget: Option<Duration>,
    /// Hard limit on how long the conductor waits to hear the step is done:
    /// passing it triggers [`Self::on_timeout`]. `None` declares no timeout,
    /// and the conductor then waits however long the component takes.
    pub timeout: Option<Duration>,
    /// What exceeding [`Self::timeout`] does. Never consulted while that is
    /// `None`.
    pub on_timeout: OnTimeout,
}

/// What the conductor does about a component that exceeded its hard
/// [`StepTiming::timeout`].
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum OnTimeout {
    /// Stop the world: the step returns
    /// [`ConductorError::StepTimeout`](crate::ConductorError::StepTimeout)
    /// and the run ends.
    ///
    /// The default, and what anything determinism-sensitive wants.
    /// Everything published before the halt is unchanged, so the hash stream
    /// stays valid up to where it stops.
    #[default]
    Halt,
    /// Remove the component at the next tick boundary and carry on, exactly
    /// as if it had asked to leave then.
    ///
    /// This **changes the scenario**: the component stops publishing, so
    /// every tick after the removal differs from a run where it survived,
    /// and the trigger is wall-clock-dependent, so a re-run on a faster
    /// machine may remove it later, or not at all. The removal is recorded
    /// in the event log like any other leave, which keeps the run
    /// replayable, but it is no longer reproducible from (seed, scenario)
    /// alone. That is what [`OnTimeout::Halt`] is the default for.
    Remove,
}

impl StepTiming {
    /// Declares no limits at all: the component is never flagged, halted, or
    /// removed however long its steps take. The default.
    pub const fn unlimited() -> Self {
        // Return a declaration with neither level set.
        StepTiming {
            budget: None,
            timeout: None,
            on_timeout: OnTimeout::Halt,
        }
    }

    /// Declares the soft level only: steps over `budget` are logged and
    /// counted, and the run is otherwise untouched.
    pub const fn budget(budget: Duration) -> Self {
        // Return a diagnostic-only declaration.
        StepTiming {
            budget: Some(budget),
            ..StepTiming::unlimited()
        }
    }

    /// Declares the hard level only: steps over `timeout` trigger
    /// `on_timeout`.
    pub const fn timeout(timeout: Duration, on_timeout: OnTimeout) -> Self {
        // Return a policy-only declaration.
        StepTiming {
            timeout: Some(timeout),
            on_timeout,
            ..StepTiming::unlimited()
        }
    }

    /// Adds the soft level to a declaration, for flagging misses well before
    /// the hard limit is anywhere near.
    pub const fn with_budget(self, budget: Duration) -> Self {
        // Return the declaration with its budget set.
        StepTiming {
            budget: Some(budget),
            ..self
        }
    }

    /// Adds the hard level to a declaration.
    pub const fn with_timeout(self, timeout: Duration, on_timeout: OnTimeout) -> Self {
        // Return the declaration with its timeout and policy set.
        StepTiming {
            timeout: Some(timeout),
            on_timeout,
            ..self
        }
    }

    /// Whether the component's own `step`, which took `step`, passed the
    /// soft budget. A limit is a duration a step may *take*, so landing
    /// exactly on one is within it.
    pub(crate) fn over_budget(&self, step: Duration) -> bool {
        // Return whether a budget was declared and the step outlasted it.
        self.budget.is_some_and(|budget| step > budget)
    }

    /// Whether the conductor, having waited `waited` to hear the step was
    /// done, passed the hard timeout.
    ///
    /// Judged separately from [`Self::over_budget`] rather than as the upper
    /// rung of one ladder, because the two read different quantities and
    /// either can happen without the other. That matters most where they
    /// differ: once a transport sits between the conductor and the step, a
    /// timeout with the budget intact means the *network* was slow, and a
    /// timeout with the budget missed means the component was, a
    /// distinction a single worst-level verdict cannot express.
    pub(crate) fn over_timeout(&self, waited: Duration) -> bool {
        // Return whether a timeout was declared and the wait outlasted it.
        self.timeout.is_some_and(|timeout| waited > timeout)
    }

    /// The one incoherent declaration: a budget at or above the timeout can
    /// never report, because the conductor stops waiting before any step
    /// slow enough to miss it can finish. That survives the two levels
    /// becoming separate measurements, for a structural reason rather than a
    /// coincidence: a wait always contains the step it is waiting on.
    /// Returns the offending pair so the caller can name both when rejecting
    /// it.
    pub(crate) fn unreachable_budget(&self) -> Option<(Duration, Duration)> {
        // Return the pair only when the soft level sits at or above the hard
        // one.
        match (self.budget, self.timeout) {
            (Some(budget), Some(timeout)) if budget >= timeout => Some((budget, timeout)),
            _ => None,
        }
    }
}

impl Default for StepTiming {
    fn default() -> Self {
        StepTiming::unlimited()
    }
}

/// A wall-clock duration as milliseconds for a human to read, at the scale
/// step limits are declared in. For log fields and recorded observations.
///
/// Lossy on purpose, and safe to be: these numbers are only ever read. None
/// of them is compared, hashed, or fed back into a run, so nothing depends
/// on two machines rendering one identically. Exact time is integer
/// nanoseconds (`SimTime`, `SimDuration`) with a canonical text form, which
/// is a different thing under a different contract; reach for this only
/// when the destination is somebody's eyes.
pub(crate) fn diagnostic_millis(duration: Duration) -> f64 {
    duration.as_secs_f64() * 1e3
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A wall-clock duration, in milliseconds.
    fn wall_ms(millis: u64) -> Duration {
        Duration::from_millis(millis)
    }

    #[test]
    fn declaring_nothing_is_never_exceeded() {
        let timing = StepTiming::unlimited();
        assert_eq!(timing, StepTiming::default());
        for elapsed in [Duration::ZERO, Duration::from_secs(3600)] {
            assert!(!timing.over_budget(elapsed));
            assert!(!timing.over_timeout(elapsed));
        }
    }

    #[test]
    fn a_budget_alone_reports_and_never_escalates() {
        let timing = StepTiming::budget(wall_ms(10));
        assert!(!timing.over_budget(wall_ms(9)));
        assert!(timing.over_budget(wall_ms(11)));
        // However far over, a component with no timeout declared cannot halt
        // a run or be removed from one for being slow.
        assert!(timing.over_budget(Duration::from_secs(60)));
        assert!(!timing.over_timeout(Duration::from_secs(60)));
    }

    #[test]
    fn a_timeout_alone_needs_no_budget_to_fire() {
        let timing = StepTiming::timeout(wall_ms(10), OnTimeout::Remove);
        assert!(!timing.over_timeout(wall_ms(9)));
        assert!(timing.over_timeout(wall_ms(11)));
        assert!(!timing.over_budget(wall_ms(11)));
    }

    #[test]
    fn both_levels_report_when_both_are_passed() {
        // Not one worst-level verdict: a step slow enough to time out has
        // also missed the budget, and the budget is what counts that.
        let timing = StepTiming::budget(wall_ms(10)).with_timeout(wall_ms(50), OnTimeout::Halt);
        assert!(!timing.over_budget(wall_ms(5)) && !timing.over_timeout(wall_ms(5)));
        assert!(timing.over_budget(wall_ms(20)) && !timing.over_timeout(wall_ms(20)));
        assert!(timing.over_budget(wall_ms(60)) && timing.over_timeout(wall_ms(60)));
    }

    #[test]
    fn the_levels_read_their_own_quantity() {
        // What separate judging buys, and the reason it cannot be one
        // verdict on one number: distributed, these are two measurements.
        // A quick step behind a slow transport passes the timeout with its
        // budget intact, so the network was slow rather than the component,
        // and nothing about that state is expressible as "the worse level".
        let timing = StepTiming::budget(wall_ms(10)).with_timeout(wall_ms(50), OnTimeout::Halt);
        let step = wall_ms(5);
        let waited = wall_ms(60);
        assert!(!timing.over_budget(step));
        assert!(timing.over_timeout(waited));
    }

    #[test]
    fn landing_exactly_on_a_limit_is_within_it() {
        // A limit is what a step may take, so taking exactly that is
        // allowed; one nanosecond more is not.
        let timing = StepTiming::budget(wall_ms(10)).with_timeout(wall_ms(50), OnTimeout::Halt);
        assert!(!timing.over_budget(wall_ms(10)));
        assert!(timing.over_budget(wall_ms(10) + Duration::from_nanos(1)));
        assert!(!timing.over_timeout(wall_ms(50)));
        assert!(timing.over_timeout(wall_ms(50) + Duration::from_nanos(1)));
    }

    #[test]
    fn a_budget_that_could_never_report_is_reported_instead() {
        // Above the timeout: every step that misses the budget has already
        // timed out.
        let over = StepTiming::budget(wall_ms(50)).with_timeout(wall_ms(10), OnTimeout::Halt);
        assert_eq!(over.unreachable_budget(), Some((wall_ms(50), wall_ms(10))));
        // Equal is the same story: passing one passes the other.
        let equal = StepTiming::budget(wall_ms(10)).with_timeout(wall_ms(10), OnTimeout::Halt);
        assert_eq!(equal.unreachable_budget(), Some((wall_ms(10), wall_ms(10))));

        // Coherent declarations, and declarations missing a level, are fine.
        assert_eq!(
            StepTiming::budget(wall_ms(10))
                .with_timeout(wall_ms(50), OnTimeout::Halt)
                .unreachable_budget(),
            None
        );
        assert_eq!(StepTiming::budget(wall_ms(50)).unreachable_budget(), None);
        assert_eq!(
            StepTiming::timeout(wall_ms(10), OnTimeout::Halt).unreachable_budget(),
            None
        );
        assert_eq!(StepTiming::unlimited().unreachable_budget(), None);
    }

    #[test]
    fn halting_is_the_policy_a_declaration_falls_back_to() {
        // The safe half of the escalation: a run only ever loses a component
        // because someone asked for that in so many words.
        assert_eq!(OnTimeout::default(), OnTimeout::Halt);
        assert_eq!(StepTiming::budget(wall_ms(10)).on_timeout, OnTimeout::Halt);
    }

    #[test]
    fn the_two_levels_can_be_declared_in_either_order() {
        // The builders are symmetric so a caller can start from whichever
        // level it knows first. Nothing in this crate spells it hard-first,
        // which is the whole reason to pin it here: the two orders have to
        // stay the same declaration.
        let soft_first =
            StepTiming::budget(wall_ms(10)).with_timeout(wall_ms(50), OnTimeout::Remove);
        let hard_first =
            StepTiming::timeout(wall_ms(50), OnTimeout::Remove).with_budget(wall_ms(10));
        assert_eq!(soft_first, hard_first);

        // And each one-level constructor is its builder applied to no limits.
        assert_eq!(
            StepTiming::budget(wall_ms(10)),
            StepTiming::unlimited().with_budget(wall_ms(10))
        );
        assert_eq!(
            StepTiming::timeout(wall_ms(50), OnTimeout::Remove),
            StepTiming::unlimited().with_timeout(wall_ms(50), OnTimeout::Remove)
        );
    }

    #[test]
    fn declaring_a_level_twice_keeps_the_last() {
        // Plain builder semantics: `with_*` sets its level, it does not
        // merge with or defer to what was there.
        let timing = StepTiming::budget(wall_ms(10))
            .with_budget(wall_ms(20))
            .with_timeout(wall_ms(50), OnTimeout::Halt)
            .with_timeout(wall_ms(60), OnTimeout::Remove);

        assert_eq!(timing.budget, Some(wall_ms(20)));
        assert_eq!(timing.timeout, Some(wall_ms(60)));
        assert_eq!(timing.on_timeout, OnTimeout::Remove, "policy follows too");
    }
}

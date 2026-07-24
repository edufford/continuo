//! Real-time pacing (milestone 3): making a run advance at 1× wall time
//! instead of as fast as possible.
//!
//! Pacing lives entirely in the conductor and is invisible to sim logic —
//! it only ever *delays* entry to an instant, never changes what happens in
//! it, so a paced run and a free run of the same seeded world produce the
//! identical world hash (tested). Nothing here touches the fingerprint.
//!
//! The rule (PLAN.md, "Pacing"): sleep until the wall time corresponding to
//! the next instant's sim time. If the sim cannot keep up, it runs slower
//! than real time and **logs the overruns** — the wall-time anchor slips by
//! the overrun amount rather than sprinting to make up lost time. No
//! catch-up, no scaling, and steps are never skipped (that would change the
//! run).
//!
//! Testability: the anchor-and-slip arithmetic is the part worth checking,
//! and it must not depend on the real clock. [`Pacer`] is generic over a
//! [`WallClock`]; the conductor uses [`SystemClock`], and the unit tests
//! drive a manual one.

use std::time::{Duration, Instant};

use continuo_core::SimTime;
use tracing::warn;

/// How the conductor advances through simulation time — the pacing mode
/// carried by [`ConductorConfig`](crate::ConductorConfig). Invisible to sim
/// logic, and never affects the world hash — only timing.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Pacing {
    /// Advance as fast as possible: step the next instant the moment the
    /// previous one finishes.
    #[default]
    FreeRun,
    /// Advance at 1× real time: each instant waits for its wall-clock
    /// target. If the sim can't keep up it runs slower and logs the
    /// overruns — the wall anchor slips rather than catching up, and steps
    /// are never skipped (see PLAN.md "Pacing").
    ///
    /// `spin_padding` is how much of each wait's tail to busy-spin instead
    /// of sleep: [`Duration::ZERO`] sleeps the whole wait (OS-timer
    /// accuracy — a high-resolution waitable timer on modern Windows,
    /// ~0.5 ms — and no CPU spent between instants); a small positive value
    /// busy-spins that final stretch for sub-millisecond accuracy at the
    /// cost of a core. Use [`Pacing::real_time`] / [`Pacing::real_time_precise`]
    /// rather than spelling the padding out.
    RealTime { spin_padding: Duration },
}

impl Pacing {
    /// 1× real time on the OS timer alone — no busy-spin, no CPU spent
    /// between instants. Prefer this unless timing to ~0.5 ms is too coarse:
    /// the spare core is usually better spent stepping.
    pub const fn real_time() -> Self {
        Pacing::RealTime {
            spin_padding: Duration::ZERO,
        }
    }

    /// 1× real time with ~1 ms sleep-then-spin for sub-millisecond
    /// accuracy, at the cost of a core for that final stretch of each
    /// instant. Worth it when smooth 1× output matters — a live viewer or a
    /// downstream real-time consumer — and a core is free to spare.
    pub const fn real_time_precise() -> Self {
        Pacing::RealTime {
            spin_padding: Duration::from_millis(1),
        }
    }
}

/// A monotonic wall clock, abstracted so pacing can be tested without
/// sleeping. Readings are nanoseconds since the clock's own origin; only
/// differences are ever used, so the origin is arbitrary.
pub(crate) trait WallClock: Send {
    fn elapsed_nanos(&self) -> i128;
    fn sleep(&mut self, nanos: i128);
}

/// The real clock: `Instant`-based, sleeps the calling thread.
pub(crate) struct SystemClock {
    origin: Instant,
    /// How much of each wait's tail to busy-spin instead of sleep (from
    /// [`Pacing::RealTime`](crate::Pacing)). `ZERO` is pure sleeping — the
    /// whole wait is slept and the trailing spin never iterates, because
    /// [`std::thread::sleep`] is guaranteed not to return early. A positive
    /// padding trades that final stretch of sleep for a busy-spin, buying
    /// sub-millisecond accuracy at the cost of a core.
    spin_padding: Duration,
}

impl SystemClock {
    pub(crate) fn new(spin_padding: Duration) -> Self {
        // Return a clock whose origin is now.
        SystemClock {
            origin: Instant::now(),
            spin_padding,
        }
    }
}

/// The coarse-sleep portion of a wait: sleep all but the final `padding`,
/// which is busy-spun. Zero when the whole wait fits inside the padding
/// (spin the lot); the full wait when `padding` is zero (coarse mode, never
/// spins). Kept pure so the cutoff is unit-testable without a real clock.
fn coarse_sleep_nanos(total: i128, padding: i128) -> i128 {
    (total - padding).max(0)
}

impl WallClock for SystemClock {
    fn elapsed_nanos(&self) -> i128 {
        self.origin.elapsed().as_nanos() as i128
    }

    fn sleep(&mut self, nanos: i128) {
        if nanos <= 0 {
            return;
        }
        // `Duration::from_nanos` takes u64; sleeps here are small positive
        // gaps, but clamp defensively rather than wrap.
        let clamp = |n: i128| Duration::from_nanos(n.min(u64::MAX as i128) as u64);
        let target = self.elapsed_nanos() + nanos;
        let coarse = coarse_sleep_nanos(nanos, self.spin_padding.as_nanos() as i128);
        if coarse > 0 {
            std::thread::sleep(clamp(coarse));
        }
        // Busy-spin any remaining tail (none in coarse mode, since the full
        // wait was slept and `sleep` never returns early).
        while self.elapsed_nanos() < target {
            std::hint::spin_loop();
        }
    }
}

/// Accumulated overrun at which the anchor gives up and slips.
///
/// Below it, lateness is absorbed exactly as an oversleep is: the anchor
/// stays put, so the next instant's sleep swallows it. Because the anchor
/// does not move, lateness keeps accumulating against it — a sim that
/// genuinely cannot keep up crosses this eventually and is reported then,
/// aggregated rather than once per instant.
///
/// Sized above OS timer granularity (~0.5 ms on modern Windows) so ordinary
/// wake-up jitter never registers — nor do sim gaps too fine to be
/// achievable in wall time at all, like an observer sampling 1 ns past a
/// period boundary.
const OVERRUN_REANCHOR_THRESHOLD: Duration = Duration::from_millis(1);

/// Maps sim time onto wall time and blocks to keep a run at 1× real time.
///
/// The anchor `(sim, wall)` fixes one point of that map; every instant's
/// target wall time is `wall_anchor + (sim_now - sim_anchor)`. Sleeping to
/// hit the target keeps the anchor fixed, so an oversleep on one step is
/// absorbed by a shorter sleep on the next (no drift accumulation). Once
/// accumulated lateness passes [`OVERRUN_REANCHOR_THRESHOLD`], the anchor
/// re-anchors to the moment the late instant actually starts, which is
/// exactly "the anchor slips by the overrun amount" — the run falls
/// permanently behind by that much rather than sprinting to catch up.
pub(crate) struct Pacer<C: WallClock> {
    clock: C,
    /// `(sim_nanos, wall_nanos)`; `None` until the first paced instant,
    /// which establishes it and never waits.
    anchor: Option<(i128, i128)>,
    overrun_reanchors: u64,
    total_slip_nanos: i128,
}

impl<C: WallClock> Pacer<C> {
    pub(crate) fn new(clock: C) -> Self {
        // Return an un-anchored pacer; the first `pace` call anchors it.
        Pacer {
            clock,
            anchor: None,
            overrun_reanchors: 0,
            total_slip_nanos: 0,
        }
    }

    /// Called at the top of every instant with its sim time. Blocks until
    /// this instant's wall-clock target; if already past it, absorbs small
    /// lateness silently and — once the accumulated overrun passes
    /// [`OVERRUN_REANCHOR_THRESHOLD`] — records it, logs it, and slips the
    /// anchor.
    pub(crate) fn pace(&mut self, sim_now: SimTime) {
        let sim = sim_now.as_nanos() as i128;
        let wall = self.clock.elapsed_nanos();
        let Some((sim_anchor, wall_anchor)) = self.anchor else {
            // First instant: anchor here and start on time.
            self.anchor = Some((sim, wall));
            return;
        };
        let target = wall_anchor + (sim - sim_anchor);
        if wall < target {
            self.clock.sleep(target - wall);
        } else if wall > target {
            // Lateness measured against an anchor that has not moved, so it
            // is cumulative: transient jitter stays small and is absorbed
            // below, while a sim that cannot keep up grows it every instant
            // until it crosses the threshold and is reported once.
            let accumulated_overrun = wall - target;
            if accumulated_overrun >= OVERRUN_REANCHOR_THRESHOLD.as_nanos() as i128 {
                self.overrun_reanchors += 1;
                self.total_slip_nanos += accumulated_overrun;
                warn!(
                    target: "continuo::pacing",
                    sim_time = %sim_now,
                    overrun_ms = accumulated_overrun as f64 / 1e6,
                    "real-time overrun: sim is behind wall time; anchor slips (no catch-up)"
                );
                // Re-anchor to the moment this late instant actually starts.
                self.anchor = Some((sim, wall));
            }
        }
    }

    pub(crate) fn overrun_reanchor_count(&self) -> u64 {
        self.overrun_reanchors
    }

    pub(crate) fn total_slip(&self) -> Duration {
        // Return the accumulated lateness as a wall-clock duration.
        Duration::from_nanos(self.total_slip_nanos.max(0).min(u64::MAX as i128) as u64)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A clock the test moves by hand: `sleep` advances it (the pacer is
    /// waiting), and `do_work` advances it to stand in for step execution
    /// taking wall time.
    struct ManualClock {
        now: i128,
        sleeps: Vec<i128>,
    }

    impl ManualClock {
        fn new() -> Self {
            ManualClock {
                now: 0,
                sleeps: Vec::new(),
            }
        }

        fn do_work(&mut self, nanos: i128) {
            self.now += nanos;
        }
    }

    impl WallClock for ManualClock {
        fn elapsed_nanos(&self) -> i128 {
            self.now
        }

        fn sleep(&mut self, nanos: i128) {
            self.sleeps.push(nanos);
            self.now += nanos;
        }
    }

    fn t(nanos: i64) -> SimTime {
        SimTime::from_nanos(nanos)
    }

    /// Sim time in milliseconds — the scale the re-anchor threshold lives
    /// at, so tests about it are readable.
    fn t_ms(millis: i64) -> SimTime {
        SimTime::from_millis(millis)
    }

    /// Wall-clock nanoseconds from milliseconds, for `do_work` and sleep
    /// assertions.
    fn ms(millis: i64) -> i128 {
        millis as i128 * 1_000_000
    }

    #[test]
    fn first_instant_anchors_without_waiting() {
        let mut pacer = Pacer::new(ManualClock::new());
        pacer.pace(t(0));
        assert!(pacer.clock.sleeps.is_empty());
        assert_eq!(pacer.overrun_reanchor_count(), 0);
    }

    #[test]
    fn a_run_that_keeps_up_sleeps_the_full_gap_each_instant() {
        let mut pacer = Pacer::new(ManualClock::new());
        pacer.pace(t(0)); // anchor at (0, 0)
        pacer.pace(t(100)); // target 100, wall 0 -> sleep 100
        pacer.pace(t(250)); // target 250, wall 100 -> sleep 150
        assert_eq!(pacer.clock.sleeps, vec![100, 150]);
        assert_eq!(pacer.overrun_reanchor_count(), 0);
    }

    #[test]
    fn an_overrun_past_the_threshold_slips_the_anchor_and_does_not_catch_up() {
        let mut pacer = Pacer::new(ManualClock::new());
        pacer.pace(t_ms(0)); // anchor at (0, 0)
        pacer.pace(t_ms(100)); // target 100 ms, wall 0 -> sleep 100 ms
        pacer.clock.do_work(ms(250)); // this step runs long (now 350 ms)
        pacer.pace(t_ms(200)); // target 200 ms, wall 350 -> 150 ms late, re-anchor
        pacer.pace(t_ms(300)); // target 350+100 = 450 ms, wall 350 -> sleep 100 ms

        assert_eq!(pacer.clock.sleeps, vec![ms(100), ms(100)]);
        assert_eq!(pacer.overrun_reanchor_count(), 1);
        assert_eq!(pacer.total_slip(), Duration::from_millis(150));
    }

    #[test]
    fn lateness_under_the_threshold_is_absorbed_like_an_oversleep() {
        // The traffic demo's pattern: an observer samples 1 ns past a
        // boundary, a gap no amount of work can hit. Being ~0.1 ms late for
        // it is not the sim failing to keep up.
        let mut pacer = Pacer::new(ManualClock::new());
        pacer.pace(t_ms(0)); // anchor at (0, 0)
        pacer.clock.do_work(100_000); // 0.1 ms of work
        pacer.pace(t(1)); // 1 ns target, 0.1 ms late -> absorbed
        assert_eq!(pacer.overrun_reanchor_count(), 0);

        // The anchor never moved, so the next instant is still measured from
        // the true origin and lands back on the original schedule.
        pacer.pace(t_ms(10));
        assert_eq!(pacer.clock.sleeps, vec![ms(10) - 100_000]);
        assert_eq!(pacer.total_slip(), Duration::ZERO);
    }

    #[test]
    fn chronic_small_lateness_is_reported_once_it_accumulates() {
        // Every step runs 0.4 ms over its 1 ms sim gap: no single instant is
        // late enough to report, but the sim genuinely cannot keep up. The
        // fixed anchor accumulates the lateness until it crosses 1 ms.
        let mut pacer = Pacer::new(ManualClock::new());
        pacer.pace(t_ms(0));
        for instant in 1..=3 {
            pacer.clock.do_work(ms(1) + 400_000);
            pacer.pace(t_ms(instant));
        }

        assert_eq!(
            pacer.overrun_reanchor_count(),
            1,
            "0.4 + 0.4 + 0.4 ms: silent, silent, then reported on crossing"
        );
        assert_eq!(pacer.total_slip(), Duration::from_micros(1200));
    }

    #[test]
    fn spin_mode_sleeps_all_but_the_padding_then_spins_the_rest() {
        let padding = 1_000_000; // 1 ms in ns
        // A wait longer than the padding: coarse-sleep everything but the
        // last millisecond, spin that.
        assert_eq!(coarse_sleep_nanos(5_000_000, padding), 4_000_000);
        // A wait shorter than the padding: no coarse sleep, spin the lot.
        assert_eq!(coarse_sleep_nanos(500_000, padding), 0);
        // Exactly the padding: also all spin.
        assert_eq!(coarse_sleep_nanos(padding, padding), 0);
    }

    #[test]
    fn coarse_mode_is_zero_padding_and_sleeps_everything() {
        // Coarse mode is just spin mode with no tail: the whole wait is
        // slept, so the trailing spin never runs.
        assert_eq!(coarse_sleep_nanos(5_000_000, 0), 5_000_000);
        assert_eq!(coarse_sleep_nanos(1, 0), 1);
    }

    #[test]
    fn being_exactly_on_target_neither_sleeps_nor_overruns() {
        let mut pacer = Pacer::new(ManualClock::new());
        pacer.pace(t(0)); // anchor at (0, 0)
        pacer.clock.do_work(100); // step took exactly the sim gap
        pacer.pace(t(100)); // target 100, wall 100 -> nothing
        assert!(pacer.clock.sleeps.is_empty());
        assert_eq!(pacer.overrun_reanchor_count(), 0);
    }
}

//! The traffic demo paced to 1× real time: the same world as `traffic`,
//! but each instant waits for its wall-clock time instead of free-running.
//! The run takes about as many wall-seconds as it simulates, and prints how
//! far (if at all) it fell behind. The world hash is identical to the
//! free-run demo — pacing changes timing, never content.
//!
//! Run with (a short duration, since it runs in real time):
//!   cargo run -p continuo-examples --example traffic_realtime -- 3
//!   cargo run -p continuo-examples --example traffic_realtime -- 3 spin
//!
//! First optional argument is sim-seconds to run (default 5); a second
//! argument `spin` selects sub-millisecond sleep-then-spin pacing instead
//! of the default OS-timer pacing (watch the overrun count drop).

use continuo_conductor::{Conductor, Pacing};
use continuo_core::SimTime;
use continuo_examples::traffic_world;
use continuo_transport::InProcTransport;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    tracing_subscriber::fmt()
        .with_max_level(tracing::Level::INFO)
        .with_target(false)
        .init();

    let seconds: i64 = std::env::args()
        .nth(1)
        .map(|a| a.parse())
        .transpose()?
        .unwrap_or(5);
    let spin = std::env::args().nth(2).as_deref() == Some("spin");

    let pacing = if spin {
        Pacing::real_time_precise()
    } else {
        Pacing::real_time()
    };
    let mut conductor =
        Conductor::new(traffic_world::config_paced(pacing), InProcTransport::new())?;
    traffic_world::populate(&mut conductor)?;

    // Wall time is fine to read here: the example's main is outside the
    // simulation, where wall clocks are forbidden.
    let started = std::time::Instant::now();
    conductor.run_until(SimTime::from_secs(seconds))?;
    let elapsed = started.elapsed();

    println!(
        "done: world '{}' reached sim time {} in {} ticks (1x real-time, {})",
        conductor.world_name(),
        conductor.sim_time(),
        conductor.tick(),
        if spin { "spin" } else { "coarse" }
    );
    println!(
        "actual time: {:.3} s for {} sim-seconds, world hash {:016x}",
        elapsed.as_secs_f64(),
        seconds,
        conductor.world_hash()
    );
    // Spelled out in the output, not just in a comment: whoever reads this
    // line is exactly who might misread it. The zero case says *why* the
    // actual time can exceed the sim time while nothing was lost — the
    // difference is in-flight lateness that stayed under the threshold.
    match conductor.overrun_reanchor_count() {
        0 => println!(
            "pacing: the schedule kept up — lateness stayed under the reanchor \
             threshold and was absorbed, never accumulating (whole-run measure, \
             not per-component step timing)"
        ),
        reanchors => println!(
            "pacing: the schedule fell behind and was reanchored {reanchors} time(s), \
             leaving it {:.3} s permanently behind real time (whole-run measure, \
             not per-component step timing)",
            conductor.total_slip().as_secs_f64(),
        ),
    }

    // Return success for the completed paced run.
    Ok(())
}

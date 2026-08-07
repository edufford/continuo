//! What a world costs as it grows: the baseline demo's size measured against
//! a scaled one, both built the same way and free-run for thirty sim-seconds,
//! so the only thing that differs between the two runs is the population.
//!
//! Neither run has a spawner or a logger. A spawner would make the
//! population a moving target, which is the one thing this holds still, and a
//! pose logger at the larger size writes more than the run it reports on. See
//! `traffic` for the watchable demo.
//!
//! What it finds is that the cost is not linear in the population: the work
//! grows with the cast, and so does the cost of each unit of it. The notes on
//! `InProcTransport::publish` and `KeyExpr::matches` are where it goes.
//!
//! **Run it in release.** A debug build measures the optimiser rather than
//! the conductor:
//!
//! ```text
//! cargo run --release -p continuo-examples --example traffic_scale
//! cargo run --release -p continuo-examples --example traffic_scale -- 400 50
//! ```

use std::sync::Arc;
use std::sync::atomic::{AtomicU64, Ordering};

use continuo_conductor::Conductor;
use continuo_core::SimTime;
use continuo_examples::traffic_world;
use continuo_transport::{InProcTransport, MonitorTransport};

const SCALED_CARS: usize = 100;
const SCALED_LANES: usize = 50;

/// Components per car: a controller and a physics body.
const COMPONENTS_PER_CAR: usize = 2;

/// What one run of one size cost.
struct Measurement {
    cars: usize,
    lanes: usize,
    ticks: u64,
    steps: u64,
    wall_seconds: f64,
    sim_seconds: f64,
    world_hash: u64,
}

impl Measurement {
    fn components(&self) -> f64 {
        (self.cars * COMPONENTS_PER_CAR) as f64
    }

    fn steps_per_tick(&self) -> f64 {
        self.steps as f64 / self.ticks as f64
    }

    /// The rate that decides whether a bigger world is affordable, since it
    /// is what stays flat when cost is linear in the population.
    fn steps_per_wall_second(&self) -> f64 {
        self.steps as f64 / self.wall_seconds
    }

    fn real_time_factor(&self) -> f64 {
        self.sim_seconds / self.wall_seconds
    }
}

fn measure(cars: usize, lanes: usize) -> Result<Measurement, Box<dyn std::error::Error>> {
    // Every component in this cast publishes exactly once per step, so a
    // message seen at the transport is a step taken. Counting them measures
    // the work rather than deriving it from the components' periods.
    let steps = Arc::new(AtomicU64::new(0));
    let counter = Arc::clone(&steps);

    let mut conductor = Conductor::new(
        traffic_world::config(),
        MonitorTransport::new(InProcTransport::new(), move |_message| {
            counter.fetch_add(1, Ordering::Relaxed);
        }),
    )?;
    traffic_world::setup_scale_scenario(&mut conductor, cars, lanes)?;

    // Wall time is fine to read here: the example's main is outside the
    // simulation, where wall clocks are forbidden.
    let started = std::time::Instant::now();
    conductor.run_until(SimTime::from_secs(traffic_world::SIM_SECONDS))?;
    let wall_seconds = started.elapsed().as_secs_f64().max(f64::MIN_POSITIVE);

    // Return what the run cost, for comparison against another size.
    Ok(Measurement {
        cars,
        lanes,
        ticks: conductor.tick(),
        steps: steps.load(Ordering::Relaxed),
        wall_seconds,
        sim_seconds: conductor.sim_time().as_secs_f64(),
        world_hash: conductor.world_hash(),
    })
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    // The scaled size is an argument so the same example answers the question
    // one pair of sizes cannot: where the curve goes.
    let mut args = std::env::args().skip(1);
    let cars: usize = match args.next() {
        Some(value) => value.parse()?,
        None => SCALED_CARS,
    };
    let lanes: usize = match args.next() {
        Some(value) => value.parse()?,
        None => SCALED_LANES.min(cars.max(1)),
    };

    let base = measure(
        traffic_world::BASELINE_DEMO_CARS,
        traffic_world::BASELINE_DEMO_LANES,
    )?;
    let scaled = measure(cars, lanes)?;

    println!(
        "traffic_scale: {:.0} sim-seconds free-run, the baseline demo against a \
         scaled one\n",
        base.sim_seconds
    );
    println!(
        "{:<22}{:>12}{:>12}{:>10}",
        "", "baseline", "scaled", "change"
    );

    // Ratios span both directions here: counts grow by tens, rates fall to
    // fractions, and a fixed precision renders one of the two as 0.0.
    let change = |ratio: f64| match ratio {
        r if r >= 10.0 => format!("{r:.0}x"),
        r if r >= 0.1 => format!("{r:.1}x"),
        r => format!("{r:.2}x"),
    };
    let row = |label: &str, a: f64, b: f64, format: fn(f64) -> String| {
        println!(
            "{label:<22}{:>12}{:>12}{:>10}",
            format(a),
            format(b),
            change(b / a)
        );
    };
    let count = |value: f64| format!("{value:.0}");
    let rate = |value: f64| format!("{value:.1}");

    row("cars", base.cars as f64, scaled.cars as f64, count);
    row("lanes", base.lanes as f64, scaled.lanes as f64, count);
    row("components", base.components(), scaled.components(), count);
    row("ticks", base.ticks as f64, scaled.ticks as f64, count);
    row(
        "steps / tick",
        base.steps_per_tick(),
        scaled.steps_per_tick(),
        rate,
    );
    row(
        "component steps",
        base.steps as f64,
        scaled.steps as f64,
        count,
    );
    row(
        "wall time, s",
        base.wall_seconds,
        scaled.wall_seconds,
        |value| format!("{value:.3}"),
    );
    row(
        "steps / wall-second",
        base.steps_per_wall_second(),
        scaled.steps_per_wall_second(),
        count,
    );
    row(
        "real-time factor",
        base.real_time_factor(),
        scaled.real_time_factor(),
        rate,
    );

    let work = scaled.steps as f64 / base.steps as f64;
    let time = scaled.wall_seconds / base.wall_seconds;
    println!(
        "\nwork grew {work:.1}x and wall time grew {time:.1}x, so a step at {} \
         components\ncosts {:.1}x what it does at {}.",
        scaled.components(),
        time / work,
        base.components(),
    );
    println!(
        "\nworld hashes: baseline {:016x}, scaled {:016x}",
        base.world_hash, scaled.world_hash
    );

    // Return success for the completed runs.
    Ok(())
}

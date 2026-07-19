# continuo

A minimal simulation orchestration system in Rust. A conductor ticks a world
deterministically while components join and leave at runtime — enabling
many-actor scenarios such as live traffic around an autonomous vehicle. It
runs entirely in a single process today, and is designed so components can
later be split into separate processes over [Zenoh](https://zenoh.io/) without
changing component code.

See [PLAN.md](PLAN.md) for the full design, decision log, and milestone
roadmap.

## Architecture

### The conductor owns time; components own state

continuo is a discrete-event, lockstep co-simulation (the conductor plays the
role of an FMI master algorithm). There is no global tick rate: every
component reports the next sim time it should step, and the conductor advances
sim time to the earliest due time, steps exactly the components due at that
instant, and repeats.

```
            ┌──────────────────────────┐
            │        Conductor         │   owns sim time + event schedule
            │ (advance to earliest due │
            │  time, step, barrier)    │
            └────────────┬─────────────┘
             TickStart   │   TickDone { next_due }
            ┌────────────┴─────────────┐
            │        Transport         │   pub/sub on key expressions,
            │   (InProc now, Zenoh     │   e.g. continuo/demo/actor/car1/pose
            │    later — same trait)   │
            └────────────┬─────────────┘
          ┌──────────────┼──────────────┐
   ┌──────┴─────┐ ┌──────┴─────┐ ┌──────┴─────┐
   │    car1    │ │    car2    │ │   logger   │  world-level actors
   │ ┌────────┐ │ │ ┌────────┐ │ │   (1 Hz)   │  (join/leave at runtime:
   │ │  ctrl  │ │ │ │  ctrl  │ │ └────────────┘   milestone 4)
   │ ├────────┤ │ │ ├────────┤ │
   │ │physics │ │ │ │physics │ │  composites: ordered children,
   │ └────────┘ │ │ └────────┘ │  intra-step pipeline
   └────────────┘ └────────────┘
```

### Key ideas

- **Self-scheduled stepping.** `Component::step` returns its own `next_due`
  time. Fixed periods, aperiodic sensors, and event-driven components all use
  the same mechanism. A component must always schedule strictly into the
  future (≥ 1 ns) — the conductor rejects zero-time livelock.
- **Integer-nanosecond time.** `SimTime` is `i64` nanoseconds internally and
  exact decimal seconds on the wire (`1.234567891`), formatted and parsed via
  integer math only. Scheduling comparisons are integer — no float-equality
  hazards.
- **Hierarchical components.** Actors are composites of ordered
  sub-components (e.g. `controller → physics`). The visibility rule makes the
  hierarchy meaningful:
  - *Across actors:* a message published at time T is seen at the consumer's
    next step after T. Co-due actors never see each other's same-instant
    outputs — this is the lockstep isolation that makes distribution possible.
  - *Inside a composite:* children step in declared order, and messages from
    an earlier child reach later children in the same step — the
    sensor → controller → physics pipeline.
- **Deterministic by construction.** Inboxes are sorted by
  `(publisher, seq)`, never arrival order; execution order within an instant
  is declaration order; no wall clock or OS entropy in sim logic. Two runs of
  the same world produce byte-identical message streams (tested).
- **Human-readable messaging.** Every payload is canonical JSON. Time is
  decimal seconds; poses are named-field vectors and quaternions (never
  arrays); the wire format is directly inspectable and, later, hashable.
- **Pacing is a boolean.** `RealTimePacing = false` free-runs as fast as
  possible; `true` (milestone 3) sleeps to 1× wall time and logs overruns.
  Sim logic never sees which mode is active.

### Crates

| Crate | Contents |
| ----- | -------- |
| [`continuo-core`](crates/continuo-core/) | `SimTime`/`SimDuration`, ids and paths, key expressions, `Vec3`/`Quat`/Euler (canonical Z-Y-X conversions), wire messages, the `Component` trait |
| [`continuo-transport`](crates/continuo-transport/) | `Transport` trait, deterministic `InProcTransport`, `MonitorTransport` for out-of-band message recording |
| [`continuo-conductor`](crates/continuo-conductor/) | Registry (component tree as data), event schedule, the conductor loop |
| [`continuo-actors`](crates/continuo-actors/) | Sample components: waypoint path, path-follow controller, unicycle physics, pose logger |
| [`continuo-examples`](crates/continuo-examples/) | Runnable example worlds (`examples/traffic.rs`) |

Planned (see PLAN.md milestones): state hashing + record/replay (M2),
real-time pacing (M3), runtime join/leave (M4), Python visualization (M5),
FMI 3.0 CS import (M6), Zenoh transport (M7).

## Usage

Requires a recent stable Rust toolchain (edition 2024, rust ≥ 1.85).

```sh
# Build everything
cargo build --workspace

# Run all tests (unit + scheduling/visibility semantics + determinism)
cargo test --workspace

# Lint / format
cargo clippy --workspace --all-targets
cargo fmt --all

# Run the demo: three cars circulating an oval, free-run, 30 sim-seconds
cargo run -p continuo-examples --example traffic
```

The demo logs each car's pose once per sim-second and finishes in a fraction
of a wall-second:

```
INFO initial pose sim_time=0.0 key="continuo/demo/actor/car1/pose" x=40.00 y=0.00 yaw_deg=94.0
INFO initial pose sim_time=0.0 key="continuo/demo/actor/car2/pose" x=-17.02 y=22.62 yaw_deg=-162.0
INFO initial pose sim_time=0.0 key="continuo/demo/actor/car3/pose" x=-17.02 y=-22.62 yaw_deg=-18.0
...
INFO pose sim_time=1.0 key="continuo/demo/actor/car1/pose" x=38.36 y=7.79 yaw_deg=112.3
INFO pose sim_time=1.0 key="continuo/demo/actor/car2/pose" x=-24.51 y=19.83 yaw_deg=-155.9
...
done: world 'demo' reached sim time 30.0 in 3031 ticks (free-run), 9906 messages published
actual time: 0.160 s (187x real-time)
```

Two observer details worth knowing: log lines carry the *message's* sim time
(an observer is a world-level actor, so it receives time-T poses strictly
after T — next-step visibility), and the logger schedules its samples **1 ns
after** each second boundary — the smallest offset that clears same-instant
deferral, so on-boundary poses are visible and nothing can be scheduled
between a boundary and its sample.

### Observing vs. recording

There are two distinct ways to watch a world, and the demo uses both:

- **In-simulation observers** (like `PoseLogger`) are ordinary components:
  they subscribe, they step, and they see messages under the visibility rule
  like any participant. Use these when the observation is part of the world.
- **Transport monitors** (`MonitorTransport`) wrap the transport and invoke
  a sink for every published message — at publish time, independent of
  subscriptions and visibility, including messages nobody subscribes to.
  Use these for logging, debugging, and recording; the milestone 2 event log
  and record/replay build on this. A monitor is not part of the simulation
  and must never feed data back into components.

Each car is a composite of two components: a `PathFollowController` (100 ms
period) that reads the car's pose and publishes a drive command, and a
`UnicyclePhysics` (10 ms period) that integrates the latest command
(sample-and-hold) and publishes the pose. The controller is declared first,
so when both are due at the same instant its command reaches the physics in
that same step; everything crossing actor boundaries (e.g. poses to the
logger) is seen next step.

## Writing a component

```rust
use continuo_core::{Component, ComponentId, KeyExpr, SimDuration, SimTime, StepCtx};

struct Beacon;

impl Component for Beacon {
    fn id(&self) -> ComponentId {
        ComponentId::new("beacon").unwrap()
    }

    fn subscriptions(&self) -> Vec<KeyExpr> {
        vec![] // or e.g. KeyExpr::new("continuo/*/actor/*/pose").unwrap()
    }

    fn step(&mut self, ctx: &mut StepCtx) -> SimTime {
        // read ctx.inbox(), publish via ctx.publish(key, &value)
        ctx.now() + SimDuration::from_millis(500) // next due time
    }
}
```

Register it with a conductor (`""` = world level, or a composite name to make
it a child):

```rust
conductor.add_component("", Box::new(Beacon))?;
```

## License

MIT — see [LICENSE](LICENSE).

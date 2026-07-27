# continuo — Project Plan

A minimal simulation orchestration system in Rust. A conductor ticks a world at a
fixed timestep while components join and leave at runtime — enabling many-actor
scenarios such as live traffic around an autonomous vehicle. Runs entirely in a
single process initially, but is designed so components can later be split into
separate processes over Zenoh without changing component code.

## Goals

- **Deterministic**: same seed + same scenario → bit-identical runs, verifiable
  by a per-tick state hash.
- **Event-scheduled lockstep orchestration** with runtime component join/leave:
  each component reports the next sim time it should step; the conductor
  advances time to the earliest due time and barriers on the components due.
- **Hierarchical components**: actor-level components may contain ordered
  sub-components (sensors, controllers, physics models, actuators) with
  intra-tick data flow between them. Orchestration and transport work the same
  at every level.
- **Multi-rate for free**: rates are not configured centrally — any period, and
  even aperiodic behavior, falls out of self-reported next-step times.
- **Pacing modes**: free-run (as fast as possible) or 1× real-time, without
  affecting determinism.
- **Single process now, distributed later**: the tick protocol is message-shaped from
  day one; distribution is a transport swap (Zenoh), not a rearchitecture.
- **FMI 3.0 Co-Simulation FMUs as components** via an adapter.
- **Human-readable messaging**: all payloads are JSON for inspection and debugging.
- **Python visualization**: a simple 2D top-down view of actors in the scene.

## Non-goals (initially)

- FMI 2.0, or Model Exchange FMUs (would require owning a solver).
- Binary wire formats (the `Transport` boundary allows adding one later if needed).
- Physics engine, 3D rendering, sensor simulation.
- Snapshot/restore of world state. Replay-from-log is the replay mechanism.
  If snapshots become worthwhile, FMI 3.0's optional `SerializeFMUState`
  capability is the standardized hook for FMU components — but user-provided
  components make general snapshotting hard, so this stays deferred.

## Architecture

### The conductor owns time; components own state

The conductor drives a discrete-event loop — it is the equivalent of an FMI
master algorithm, and each step boundary is a communication point.

Every component reports, as part of its step-completed ack, the next sim time
at which it should step. Per iteration:

1. Conductor advances sim time to the **earliest reported next-step time** and
   publishes `TickStart { tick, sim_time }`.
2. Each **due** component (`next_due <= sim_time`) reads its inbox
   (messages published since its previous step), steps its state, publishes
   outputs. Each computes its own elapsed `dt` from its last step time.
3. Each due component replies `TickDone { tick, component_id, next_due }`.
4. Conductor barriers on the due components' acks, applies pending join/leave,
   then advances again.

Time fields are the `SimTime` type: integer nanoseconds in memory, decimal
seconds on the wire (see Messages).

This protocol works identically over the in-process transport and over Zenoh —
that is the distribution seam.

### Component trait (transport-blind)

```rust
trait Component {
    fn id(&self) -> ComponentId;
    fn subscriptions(&self) -> Vec<KeyExpr>;
    fn step(&mut self, ctx: &mut TickCtx);  // read inbox, publish via ctx
}
```

A *host* wraps components and connects them to a transport. In single-process
mode the host calls `step` directly in a loop; in distributed mode the same host
runs in its own process bridging over Zenoh. Component code never changes.

### Hierarchy: actors and sub-components

Components form a tree. The first layer under the world is actor-level
(vehicles, traffic manager, ego); an actor may be a **composite** containing an
ordered list of sub-components (e.g. `sensors → controller → physics →
actuators`). The same `Component` trait and the same transport pub/sub are used
at every level — a composite is itself just a component whose `step` runs its
children in declared order.

**Message visibility rule** (replaces a single flat rule):

- **Across the world level** (actor ↔ actor): outputs published at time T are
  visible at the consumer's next step after T. This is the lockstep barrier
  semantics that makes distribution and determinism-under-parallelism
  possible.
- **Inside a composite**: children step in declared order, and messages
  published by an earlier child are delivered to later children *within the
  same step*. This gives the sensor → controller → physics pipeline its
  intra-step data flow. Order is explicit configuration, so it is fully
  deterministic.

Sub-components publish on key expressions under their actor (e.g.
`continuo/{world}/actor/{id}/sensor/imu`), so wiring is uniform whether a
consumer is a sibling sub-component (same tick) or another actor (next tick).

Distribution note: **same-instant delivery never crosses a process (host)
boundary** — a coupled composite is always co-located. Splitting one would
put sequential network hops inside a single instant and force intra-instant
phases into the tick protocol; instead, an expensive self-contained
sub-component (camera renderer, lidar) can be declared **decoupled**: it
keeps its place in the actor's namespace and lifecycle but uses cross-actor
(next-step) visibility, so it can be placed on any host. Decoupled sensors
also *pipeline* — computing step T while consumers use T−1 — which beats
serializing the instant for throughput and matches real sensor latency.
Coupled same-instant pipelines (controller → physics) are for tight, cheap
loops and stay co-located. The coupling flag becomes part of registration
metadata (milestone 4).

### Transport

A `Transport` trait: publish/subscribe over string key expressions following
Zenoh keyexpr syntax from the start. Implementations:

- `InProcTransport` — deterministic, in-process (first).
- `continuo-transport-zenoh` — Zenoh-backed (later).

Key expression conventions (draft):

| Key expression                          | Payload                    |
| --------------------------------------- | -------------------------- |
| `continuo/{world}/tick`                 | `TickStart`                |
| `continuo/{world}/tick/done`            | `TickDone`                 |
| `continuo/{world}/actor/{id}/pose`      | actor pose                 |
| `continuo/{world}/scene`                | scene-graph state update (fixed-period publisher for renderers) |
| `continuo/{world}/conductor/join`       | registration request       |
| `continuo/{world}/conductor/leave`      | departure notice           |

## Determinism rules

Baked in from the start — hard to retrofit:

- All sim times and next-step times are **integer nanoseconds**; float
  arithmetic is allowed inside a step, but anything entering the schedule is
  rounded to the nearest ns first. Never accumulate float time.
- **Across actors, outputs published at time T are visible only at the
  consumer's next step after T** — co-due components never see each other's
  same-instant outputs (double-buffered mailboxes). Removes cross-actor
  ordering sensitivity; also what makes distributed lockstep possible. Within
  a composite, same-instant delivery follows the declared child order —
  explicit, therefore deterministic.
- Inboxes delivered sorted by `(publisher_id, sequence)`, never arrival order.
- Per-component RNG seeded from `(world_seed, component_id)`. No OS entropy or
  wall clock in sim logic; wall clock exists only in the conductor's pacing.
- Join/leave applied **only at tick boundaries** and recorded in an event log, so
  runs with dynamic actors are replayable.
- Per-tick canonical **state hash** (e.g. xxhash over serialized state) as the
  determinism check. Two runs with the same seed must produce identical hash
  streams; this becomes a CI test.
- FMU caveat: FMUs are black-box native code. If an FMU supports
  `SerializeFMUState`, its state joins the hash; otherwise hash its outputs and
  trust the vendor for internal determinism. Hashing supports both modes per
  component.
- Cross-machine float determinism holds only for same architecture + build flags
  (no fast-math). A known constraint of milestone 7, not a bug.

## Messages

All transport payloads are JSON via `serde_json`:

- `serde_json` round-trips `f64` exactly (Ryu / minimal-digits), so JSON does not
  threaten replay fidelity.
- Serialization must be canonical so the serialized bytes can be hashed directly:
  struct fields in declaration order (serde default), and **never serialize
  `HashMap`** in messages — use `BTreeMap` or `Vec` of pairs.
- **Pose convention**: SI units throughout. Right-handed, Z-up world frame
  (ENU); body frames X-forward/Y-left/Z-up (ROS REP-103 style). A pose is a
  position `{x, y, z}` in meters plus a unit quaternion `{w, x, y, z}` —
  always **named JSON fields, never arrays**, so axis/component order can't be
  misread. 2D dynamics simply publish `z = 0` and yaw-only quaternions; the
  schema never changes when models go 3D.
- **Euler angles at the human boundaries only.** User config and
  orientation-related APIs accept Euler angles with one standardized
  convention: **roll–pitch–yaw about body X, Y, Z as intrinsic Z-Y-X**
  (apply yaw, then pitch, then roll — the REP-103/aerospace convention),
  named fields `{roll, pitch, yaw}`. `continuo-core` provides the canonical
  Euler ↔ quaternion conversions (radians in API; config uses degrees via
  explicit `rpy_deg` naming). **Wire messages carry quaternions only** — Euler
  never enters hashed payloads, avoiding gimbal ambiguity (quaternion → Euler
  is only unique with pitch constrained to ±90°) and keeping one canonical
  orientation encoding.
- **Time on the wire is decimal seconds** (e.g. `"sim_time": 1.234567891`) for
  human readability, with at most 9 fractional digits (nanosecond precision).
  Internally `SimTime` stays integer nanoseconds; serialization and parsing go
  through **integer math** (whole-second and nanosecond parts formatted/parsed
  as integers, never converted through `f64`), so the representation is exact
  at any sim duration and the bytes stay canonical for hashing. Canonical
  form: trailing zeros trimmed, at least one fractional digit (`1.5`, `2.0`,
  `0.033333333`).
- Message schemas are versioned and kept flat/simple (poses, scalars) so the
  keyexpr ↔ FMU-variable binding stays a config file, not code.
- Cost: encode/decode throughput at high actor counts. Acceptable at this scale;
  a binary format could be reintroduced per-transport later without touching
  components.

## Scheduling: self-reported next-step times

There is no global rate and no divisor table. Each component returns
`next_due` in its `TickDone`; the conductor keeps a schedule (min-heap) and
advances sim time to the earliest due time each iteration.

- **All times are integer nanoseconds.** A component may compute its period or
  schedule in float, but must round to the nearest nanosecond before
  reporting. Scheduling comparisons are then exact integer comparisons — no
  float-equality hazards, no rational/multi-clock bookkeeping.
- **Strict advance guard**: `next_due` must be strictly greater than the
  current sim time (≥ 1 ns ahead). Reporting a time at or before "now" would
  allow a zero-time livelock; the conductor treats it as a component error.
- **Phase alignment caveat**: components co-step only when their integer
  nanosecond times are *exactly* equal. A rounded 1/30 s period
  (33,333,333 ns) will drift out of phase with a 10 ms component. Choose
  ns-exact periods when phase alignment matters; the drift is deterministic
  either way.
- **Latching semantics**: inboxes deliver only *new* messages; consumers keep
  the last-known value of inputs that didn't update. Standard sample-and-hold
  between unequal rates, no transport support needed.
- **Composites**: a composite's `next_due_ns` is the minimum over its
  children's; when stepped, it steps only its due children, in declared order.
- **Emergent behaviors**: fixed periods, aperiodic sensors, and event-driven
  components (reschedule sooner in response to an input) all use the same
  mechanism.
- **Fixed-interval world services still work**: a constant period is just the
  simplest case of self-scheduling, and because the conductor advances to
  the earliest due time, sim time lands *exactly* on those grid points. E.g. a
  scene-graph publisher stepping every 1/60 s aggregates latest actor poses
  (sample-and-hold) and publishes a world state update for renderers. Such
  services are ordinary components — the conductor gains no special cases.
- If a model needs finer internal resolution than its reporting period (e.g.
  stiff physics), it sub-steps internally within its `step` call.

## Pacing

Lives entirely in the conductor:

A single config field, `pacing: Pacing`:

- `Pacing::FreeRun` — advance immediately after the barrier.
- `Pacing::RealTime { spin_padding }` — 1× real-time: wait until the wall
  time corresponding to the next step's sim time. If the sim can't keep up,
  it simply runs slower than real time and **logs the overruns** — no
  catch-up (the wall-time anchor slips by the overrun amount rather than
  sprinting to make up lost time), no scaling, and **never skip steps**
  (determinism). Lateness is only counted once it accumulates past a
  re-anchor threshold, so transient jitter is absorbed by the next wait
  instead of reported — a *bounded* catch-up, capped by the threshold. The
  run never gets ahead of schedule, only back to it. `spin_padding` chooses how a wait is *spent* — OS
  timer alone, or sleep then busy-spin the tail for sub-millisecond accuracy
  — and never what the wait achieves.

Sim logic never sees which mode is active. Message timestamps are sim time
throughout, so an instant that starts late still stamps its outputs with the
sim time it was scheduled for — which is why pacing cannot move the world
hash.

## Failure handling at the barrier

The conductor must never hang indefinitely waiting for a `TickDone`. A
per-world configurable policy with a required timeout:

- `on_component_timeout = "halt"` — stop the world (default for
  determinism-sensitive runs; a dropped component silently changes the
  scenario).
- `on_component_timeout = "drop"` — log the failure, deregister the component
  at the barrier (recorded in the event log like any leave, so the run stays
  replayable), and continue.

Either way the overrun/failure is logged with the component id and step time.
Component panics in-process are caught at the host boundary and treated the
same as a timeout.

### Per-component timing: one declaration, two severities

The timeout above is the *hard* end of a single per-component measurement —
how long a component's `step` took in wall time. The soft end is a **step
budget**:

- **budget** (soft, diagnostic) — "this component's `step` should complete
  within X wall time". Exceeding it is logged and counted; the run is
  unaffected. This is what real-time scenarios need when some components
  carry deadlines an operator wants flagged on every miss, while others have
  no real-time restriction at all (`None`, the default).
- **timeout** (hard, policy) — `on_component_timeout` above: halt or drop.

They measure the same quantity, so they are declared together as part of a
component's registration metadata rather than built as two mechanisms. What
they do to a run differs sharply, though, and only the soft end is free:

- A **budget miss only observes.** Nothing about the run changes, so the
  world hash is untouched — a run that misses every budget produces the
  identical hash to one that misses none.
- **Halt** ends the run. Everything published before it is unchanged, so the
  hash stream stays valid up to where it stops.
- **Drop changes the scenario, and therefore the hash.** A deregistered
  component stops publishing, so every tick after the drop differs from a run
  where it survived — and the trigger is wall-clock-dependent, so a live
  re-run on a faster machine may drop later, or not at all. The drop is
  recorded in the event log like any other leave, which keeps the run
  **replayable**, but it is no longer reproducible from `(seed, scenario)`
  alone. That is precisely why halt is the default for
  determinism-sensitive runs.

Note this is a different measurement from the pacing overrun of milestone 3,
which asks whether the *schedule as a whole* tracked the wall clock and is
attributable to no component in particular.

## FMI 3.0 CS support

- `continuo-fmi` crate providing `FmuComponent`: an adapter that on each step
  reads its inbox → sets FMU input variables, calls `fmi3DoStep(t, dt)` with
  `dt` = elapsed sim time since its last step (FMI 3.0 CS supports variable
  communication step sizes), gets outputs → publishes them. The FMU's period
  comes from its mapping config and is reported as its next-step time.
- FMI **3.0 Co-Simulation only** — no 2.0 shims, native arrays, `float64` value
  references.
- Per-FMU mapping config binds key expressions to variable names (borrowing
  ideas from FMI's SSP standard for wiring configuration).
- Candidate crate: `fmi` (fmi-rs) for 3.0 import. Validate early — FMU loading is
  `libloading` + native code, and Windows DLL handling can be finicky.
- Demo target: an FMI 3.0 reference FMU from the Modelica Association set
  (BouncingBall, VanDerPol, Feedthrough).

## World and map

- A **generic world specification** owned by continuo — deliberately *not*
  autonomous-car specific. Initial schema (JSON, versioned like all messages):
  coordinate frames, static geometry (obstacles/bounds), and named **paths**
  (polylines / parametric loops) that actors can follow.
- Rich road-network formats (OpenDRIVE, Lanelet2, or something else — undecided)
  arrive later as **importers** that lower into this spec, so actors never
  depend on a third-party format directly.
- The world spec is published on the transport like everything else
  (`continuo/{world}/map`), so late-joining components and the visualizer can
  fetch it.
- Structure it as a minimal **scene graph** from the start — nodes with ids,
  optional parent, transform, and an open-ended properties bag — so v1 stays
  basic (frames, geometry, paths) while later growth (semantic tags, spawn
  points, richer geometry) extends nodes instead of reshaping the schema.

## Scenario configuration

- Scenario files are **JSON5** — human-friendly JSON with comments and
  trailing commas (Rust: `json5` crate; Python: `pyjson5`). Wire messages stay
  strict canonical JSON; JSON5 is for files humans edit.
- The scenario defines: world seed, the world spec (or a reference to it), and
  the **full component tree** — actors, their ordered sub-components, each
  component's type, period, and parameters. Composition is data-driven
  (SSP-style): the scenario names component *types*, and a registry in the
  host instantiates them (Rust-native types and FMUs alike — an FMU entry
  points at the `.fmu` and its variable-mapping config).
- The scenario owns initial actor spawning; runtime spawning (e.g. the traffic
  spawner) goes through the normal join protocol.
- Orientations in scenario files are written as Euler angles in degrees
  (`rpy_deg: { roll: 0, pitch: 0, yaw: 90 }`) and converted to quaternions on
  load using the standardized convention (see Messages).

## Visualization

- `continuo-viz-bridge`: a component subscribing to
  `continuo/{world}/actor/**/pose`, throttled to ~30 Hz, serving JSON over a
  WebSocket. No translation layer needed — the wire format is already JSON.
- `python/continuo_viz`: WebSocket client + 2D top-down view.
- When distribution lands, the Python side swaps its WebSocket client for
  `zenoh-python` with the same message schema — keep the schema versioned and
  stable from milestone 5 on.

## Distribution (Zenoh)

- `continuo-transport-zenoh` implements the `Transport` trait.
- A standalone host binary runs components out-of-process, in lockstep via the
  same tick protocol.
- Note: the per-tick gather must stay deterministic (sort by publisher/sequence,
  not arrival) — already guaranteed by the determinism rules above.
- One conductor per world; remote processes run **hosts** (component
  container + transport bridge + publish stamping). Data flows host↔host
  over pub/sub without routing through the conductor. Because same-instant
  delivery never crosses hosts (see Hierarchy), the remote inbox release
  rule is simply `msg.time < now` — sibling-order knowledge never leaves
  the conductor.

## Workspace layout

```
continuo/
  crates/
    continuo-core             # ids, SimTime, KeyExpr, messages, Component trait
    continuo-transport        # Transport trait + deterministic InProcTransport
    continuo-conductor        # tick loop, barrier, registry, pacing, event log, hashing
    continuo-actors           # kinematic vehicle, traffic spawner, ego stub
    continuo-viz-bridge       # poses -> JSON over WebSocket
    continuo-fmi              # FmuComponent adapter (FMI 3.0 CS)        [milestone 6]
    continuo-transport-zenoh  # Zenoh Transport impl                     [milestone 7]
    continuo-examples         # runnable example worlds (traffic demo)
  python/continuo_viz/        # WebSocket client + 2D top-down view
```

Plain threads and channels (crossbeam), no tokio initially — a deterministic sim
loop wants synchronous control flow, and Zenoh's Rust API can be used without
owning an async runtime later.

## Milestones

1. **Skeleton ticks** — workspace, core types, `InProcTransport`, conductor
   loop with composite components and next-step-time scheduling, static
   component set, free-run. Demo: cars circulating on a path, each a composite
   (`controller → physics`) with the controller at a slower period than the
   physics; poses logged.
2. **Determinism harness** — seeding, per-tick state hash (state-hash vs.
   output-hash per component), record/replay, CI test asserting identical hash
   streams.
3. **Pacing** — `pacing: Pacing` config (default `FreeRun`), 1× wall-time
   gating with anchor-slip once lateness accumulates past the re-anchor
   threshold, overrun logging.
4. **Dynamic join/leave** — registration over transport, tick-boundary
   application, live traffic spawner, replay still deterministic via event log.
   Registration metadata gains per-component timing — step budget and timeout
   policy together (see "Per-component timing").
5. **Visualization** — viz bridge + Python package; watch the traffic move.
6. **FMI** — `continuo-fmi`, mapping config, FMI 3.0 reference FMU driving an
   actor.
7. **Distribution** — Zenoh transport, standalone host binary, one component
   out-of-process in lockstep, Python viz on zenoh-python.

FMI before Zenoh: it exercises the component abstraction while everything is
still in one process — easier to debug native-code issues without networking in
the mix. They are independent and can swap if priorities shift.

## Decision log

- **2026-07-16** — Transport (not "bus") is the name of the pub/sub seam;
  keyexprs follow Zenoh syntax from day one.
- **2026-07-16** — FMI support is **3.0 Co-Simulation only**.
- **2026-07-16** — All payloads are **JSON** (human-readable beats efficient at
  this scale); canonical serialization rules make the bytes hashable.
- **2026-07-17** — Self-contained deterministic sim first; external ego stack
  joins later as a lockstep client.
- **2026-07-17** — Components are **hierarchical**: actor-level components with
  ordered sub-components; same orchestration/transport model at every level.
  Cross-actor visibility is next-tick; intra-composite is same-tick in declared
  order.
- **2026-07-17** — **Multi-rate from the start**; sample-and-hold between
  rates.
- **2026-07-17** — Scheduling is **event-driven via self-reported next-step
  times** (supersedes the earlier integer-rate-divisor design): components
  return `next_due_ns` in `TickDone`, the conductor advances sim time to the
  earliest due time and barriers only on due components. All schedule times
  are integer nanoseconds, rounded on entry.
- **2026-07-17** — World spec is generic and continuo-owned; road-network
  formats (OpenDRIVE or other, undecided) come later as importers.
- **2026-07-17** — Pacing is a single boolean `RealTimePacing`: `false` =
  free-run, `true` = 1× real-time (no scale factor). If real-time can't keep
  up, run slower and log overruns. *(Superseded 2026-07-24: still one
  setting and still no scale factor, but a `Pacing` enum rather than a bool,
  so the spin padding rides on the real-time variant — see the milestone 3
  entry below.)*
- **2026-07-17** — Component tree and wiring are **scenario-config-driven**
  (SSP-style concept, not the SSP format): scenario names component types,
  a host-side registry instantiates them.
- **2026-07-17** — Poses are **3D position + unit quaternion** with named JSON
  fields; right-handed Z-up (ENU) world frame, X-forward body frames
  (REP-103 style). 2D models publish `z = 0`.
- **2026-07-17** — Scenario files are **JSON5** (comments allowed); wire
  messages remain strict canonical JSON.
- **2026-07-17** — **Euler angles allowed in config and APIs** with one
  standardized convention (intrinsic Z-Y-X roll–pitch–yaw, REP-103 style;
  degrees in config as `rpy_deg`, radians in APIs); wire messages stay
  quaternion-only.
- **2026-07-17** — Barrier failure policy is configurable per world:
  **halt** or **timeout-and-drop** (logged, event-logged for replay). The
  conductor never waits indefinitely.
- **2026-07-17** — World spec starts minimal but is shaped as a **scene
  graph** (nodes + transforms + open properties) for later expansion.
- **2026-07-17** — **Replay-from-log over snapshot/restore**; snapshots
  deferred (FMI 3.0 `SerializeFMUState` is the hook if ever needed).
- **2026-07-18** — **Same-instant delivery never crosses a host boundary.**
  Heavy sub-components can be declared **decoupled**: they stay in the
  actor's namespace/lifecycle but use next-step visibility, freeing their
  host placement and pipelining with their actor instead of serializing the
  instant. Coupling flag joins registration metadata in milestone 4.
- **2026-07-18** — Milestone 2 implementation choices: hash and RNG are
  **owned implementations** (FNV-1a 64, SplitMix64) so fingerprints and
  random streams are bit-stable across platforms and versions forever — no
  external RNG/hash crates. Components are **output-hashed by default**;
  the opt-in `Component::state_bytes` hook adds state-hash mode. The world
  hash starts from `(seed, world name)` and chains per-tick hashes. Event
  log is JSON lines (header + interleaved msg/tick events, hashes as hex
  strings); milestone 2 replay is **re-execution + comparison** via
  `EventLog::first_divergence`. `StepCtx::rng()` gives a fresh per-step
  stream from `(component_seed, now)`; persistent streams seed a stored
  `DetRng` from `ctx.component_seed()`.
- **2026-07-21** — Replay verifies **live during the re-run** and stops at
  the first divergence (message and fingerprint callbacks; both channels
  are needed — a tampered log message leaves its neighboring fingerprints
  intact, and state-only divergence never appears in messages). Post-hoc
  `first_divergence` remains for comparing two recorded logs.
- **2026-07-21** — A recorded log has two named consumers, split by how the
  log's data flows relative to the sim: **verification** (`Verifier`,
  `--verify`) treats it as an expected-output ledger — everything re-runs
  live, nothing from the log enters the sim, divergence = broken
  determinism, halt and fail; **open-loop resimulation**
  (`PlaybackComponent`, `--resim`) treats it as input stimulus — selected
  recorded publishers are replaced by playback doubles re-publishing their
  recorded messages, changed components run live against them, nothing is
  compared, and divergence from the recording is the engineering result
  under study. Playback doubles are pure data, so hybrid runs stay fully
  deterministic and recordable — resim experiments are themselves
  verifiable. No general "divergence summary" mode: the sim cannot report
  behavioral differences usefully; observe resim runs like any other run.
- **2026-07-23** — Milestone 2 review outcomes (naming and module shape; no
  behavior change — the demo's world hash is unchanged):
  - Determinism primitives are named for what they are, algorithm
    included: `HashFnv1a64` and `RandomSplitMix64` (spelled out, not an
    acronym), with the module following the type: `core::random`.
  - **Seed derivation is its own concern**, `continuo-core::seed`: it uses
    the hash to fold names down to 64 bits and the generator's scrambler to
    combine values, and belongs to neither. `mix` → `mix_seeds`;
    `derive_component_seed` and the new `derive_step_seed` live there.
    `StepCtx::rng()` → `step_random()`.
  - The world's two identifying values are named for what they hold, and
    named the same way everywhere — **`world_name` and `world_seed`** — in
    `ConductorConfig`, the event-log header, `StepCtx`, and the derivation
    functions. A bare `world: String` reads like it holds the world; it
    holds the world's name.
  - `Recorder::new` and `Verifier::new` take **`&ConductorConfig`** instead
    of a restated world name and seed, so a log's header always names the
    run that produced it, and verification always checks the log against
    the run actually about to execute. Verification never takes the
    expected world/seed *from* the log — that would make the header check
    vacuous.
  - The event log splits by what each part does with it: `record` (log +
    `Recorder`), `verify` (`Divergence`, `EventLog::first_divergence`,
    `Verifier`), `playback` (`PlaybackComponent`), with their tests in
    `tests/event_log.rs` against one shared fixture log.
    `PlaybackComponent` stays in `continuo-conductor` — it is harness
    machinery built on `EventLog`, not a sample actor, and moving it to
    `continuo-actors` would invert the crate layering.
- **2026-07-23** — Milestone 3 (pacing) implementation choices:
  - Pacing gates each instant at the **top of `step_once`**, before any
    component runs, so every driver (`run_until` and the manual
    `next_scheduled` loops alike) gets it for free and it delays entry to an
    instant without ever touching its content. Consequence: a paced run and
    a free run of the same seeded world produce the **identical world
    hash** — the milestone's headline test.
  - The map from sim time to wall time is one **anchor `(sim, wall)`**.
    Keeping the anchor fixed while sleeping means an oversleep on one step
    is absorbed by a shorter sleep on the next (no drift). An **overrun
    re-anchors** to when the late instant actually starts — the anchor slips
    by the overrun amount, so the run stays behind rather than sprinting to
    catch up (PLAN.md "Pacing"). Steps are never skipped.
  - Overruns are logged (`tracing::warn`, target `continuo::pacing`) and
    counted; `Conductor::overrun_count()` / `total_slip()` expose them.
  - Lateness is only reported once it accumulates past
    `OVERRUN_REANCHOR_THRESHOLD` (1 ms). Below it the anchor stays put, so
    the next instant's sleep absorbs the lateness exactly as it absorbs an
    oversleep. This matters because *any* sim gap finer than the wall time
    its work costs is unachievable under pacing — the demo's 1 ns logger
    offset most starkly — and counting those as failures made
    `overrun_count` measure schedule shape rather than health. Because the
    anchor does not move, lateness keeps accumulating against it, so a sim
    that genuinely cannot keep up still crosses the threshold and is
    reported — aggregated instead of once per instant. The accessor is named
    `overrun_reanchor_count` for the event it actually counts, and documents
    that zero means "the schedule tracked the wall clock", *not* "every
    component finished within its time".
  - The threshold is therefore also a **catch-up budget**, which caps it
    from above: absorbing lateness makes the next interval run short by that
    much, briefly faster than 1× (though never ahead of schedule). It must
    stay well under the shortest component period — above a sim gap, that
    recovery becomes a run of zero-sleep instants, the sprint "no catch-up"
    exists to prevent.
  - The anchor/slip arithmetic is isolated behind a `WallClock` trait so it
    is unit-tested against a manual clock (no real sleeps); the conductor
    uses `SystemClock`.
  - Pacing is **one config field**, `pacing: Pacing` — `FreeRun` or
    `RealTime { spin_padding }` — replacing the earlier
    `real_time_pacing: bool` + separate precision enum (which left precision
    meaningless whenever the bool was false). `spin_padding` tunes only how
    each wait is *spent*, never the result: `Duration::ZERO` sleeps on the
    OS timer alone (Rust's std already uses a high-resolution waitable timer
    on modern Windows, ~0.5 ms) and spends no CPU between instants; a small
    positive value sleeps to within that padding of the target then
    busy-spins the tail for sub-millisecond accuracy at the cost of a core.
    Every mode produces the identical world hash. `Pacing::real_time()` /
    `real_time_precise()` name the two common choices so callers never
    spell the padding (or the `ZERO`-means-coarse convention) out. Coarse is
    exactly zero-padding spin — one `SystemClock::sleep` path, its
    sleep-vs-spin cutoff a pure function with its own unit test.
  - Failure handling at the barrier (`on_component_timeout`, PLAN.md) is
    **not** part of M3 — it needs the join/leave machinery of M4 to drop a
    component mid-run, so it lands there.
- **2026-07-24** — **Per-component step budgets are M4, not M3**, together
  with the timeout policy they share a measurement with (see "Per-component
  timing"). Considered for M3 since real-time scenarios want per-component
  deadline flagging, and rejected there for three reasons: a budget is the
  soft half of `on_component_timeout`, so building them apart risks two
  overlapping per-component durations instead of one declaration with an
  escalation level; the budget belongs in registration metadata, whose shape
  M4 is already reworking for remote components; and M3 stays a clean single
  concern — the conductor tracking the wall clock.
  - Two shaping decisions were settled while scoping it, so M4 need not
    re-derive them: the budget measures **the component's own `step`
    duration** (not completion relative to the instant's start, which would
    fold in queueing behind earlier components), and it is declared **at
    registration** rather than on the `Component` trait — a deadline is a
    deployment property (the same physics model has one on a HIL rig and
    none in a batch run), and this keeps wall-clock types out of
    `continuo-core` entirely.
  - Rejected alternative: **per-component pacing strictness**, letting a
    component declare that its own overruns don't matter. It cannot replace
    the re-anchor threshold — that threshold's job is absorbing transient
    jitter so it self-corrects, and strictness cannot express transient vs.
    sustained, so jitter would re-anchor and accumulate as permanent drift.
    It also mismatches the model: pacing is per-*instant*, and co-due
    components would need an arbitrary combining rule. And the lateness it
    would suppress is not the lax component's anyway — it is caused by the
    previous instant's work, and belongs to the gap.

## Deferred (decided-not-now, revisit when they bite)

- **Road-network importer**: which format (OpenDRIVE, Lanelet2, other) lowers
  into the world spec — decide when realistic road scenarios are needed.
- **Snapshot/restore**: via FMI 3.0 `SerializeFMUState` for FMUs plus a
  serialize hook for native components — only if replay-from-log proves
  insufficient.
- **Binary wire encoding**: per-transport option if JSON throughput ever
  becomes the bottleneck.
- **External (non-deterministic) ego participation**: a relaxed admission mode
  for a live AV stack under test — after milestone 7.

# continuo Project Plan

A deterministic simulation orchestration system in Rust. A conductor advances
a world while components set their own cadence and join or leave throughout,
enabling many-actor scenarios such as live traffic around an autonomous
vehicle. There is no global tick rate: every component reports the next sim
time it should step, and the conductor advances to the earliest due time. Runs
entirely in a single process initially, but is designed so components can later
be split into separate processes over Zenoh without changing component code.

## Goals

- **Deterministic**: same seed + same scenario -> bit-identical runs,
  verifiable by a per-tick state hash.
- **Event-scheduled lockstep orchestration** with runtime component join/leave:
  each component reports the next sim time it should step; the conductor
  advances time to the earliest due time and barriers on the components due.
- **Hierarchical components**: actor-level components may contain ordered
  sub-components (sensors, controllers, physics models, actuators) with
  intra-tick data flow between them. Orchestration and transport work the same
  at every level.
- **Multi-rate for free**: rates are not configured centrally. Any period, and
  even aperiodic behavior, falls out of self-reported next-step times.
- **Pacing modes**: free-run (as fast as possible) or 1x real-time, without
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
  capability is the standardized hook for FMU components, but user-provided
  components make general snapshotting hard, so this stays deferred.

## Architecture

### The conductor owns time; components own state

The conductor drives a discrete-event loop. It is the equivalent of an FMI
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

This protocol works identically over the in-process transport and over Zenoh,
which is the distribution seam.

### Component trait (transport-blind)

```rust
trait Component {
    fn id(&self) -> ComponentId;
    fn subscriptions(&self) -> Vec<KeyExpr>;
    // Read inbox, publish via ctx, report the next sim time to step at.
    // `Err` halts the world: it is how a component says it cannot do its job.
    fn step(&mut self, ctx: &mut StepCtx) -> Result<SimTime, CoreError>;
}
```

A *host* wraps components and connects them to a transport. In single-process
mode the host calls `step` directly in a loop; in distributed mode the same host
runs in its own process bridging over Zenoh. Component code never changes.

### Hierarchy: actors and sub-components

Components form a tree. The first layer under the world is actor-level
(vehicles, traffic manager, ego); an actor may be a **composite** containing an
ordered list of sub-components (e.g.
`sensors -> controller -> physics -> actuators`). The same `Component` trait
and the same transport pub/sub are used at every level, since a composite is
itself just a component whose `step` runs its children in declared order.

**Message visibility rule** (replaces a single flat rule):

- **Across the world level** (actor <-> actor): outputs published at time T are
  visible at the consumer's next step after T. This is the lockstep barrier
  semantics that makes distribution and determinism-under-parallelism
  possible.
- **Inside a composite**: children step in declared order, and messages
  published by an earlier child are delivered to later children *within the
  same step*. This gives the sensor -> controller -> physics pipeline its
  intra-step data flow. Order is explicit configuration, so it is fully
  deterministic.

Sub-components publish on key expressions under their actor (e.g.
`continuo/{world}/actor/{id}/sensor/imu`), so wiring is uniform whether a
consumer is a sibling sub-component (same tick) or another actor (next tick).

Distribution note: **same-instant delivery never crosses a process (host)
boundary**, so a coupled composite is always co-located. Splitting one would
put sequential network hops inside a single instant and force intra-instant
phases into the tick protocol; instead, an expensive self-contained
sub-component (camera renderer, lidar) can be declared **decoupled**: it
keeps its place in the actor's namespace and lifecycle but uses cross-actor
(next-step) visibility, so it can be placed on any host. Decoupled sensors
also *pipeline*, computing step T while consumers use T-1, which beats
serializing the instant for throughput and matches real sensor latency.
Coupled same-instant pipelines (controller -> physics) are for tight, cheap
loops and stay co-located. The coupling flag becomes part of registration
metadata (milestone 4).

### Transport

A `Transport` trait: publish/subscribe over string key expressions following
Zenoh keyexpr syntax from the start. Implementations:

- `InProcTransport`: deterministic, in-process (first).
- `continuo-transport-zenoh`: Zenoh-backed (later).

Key expression conventions (draft):

| Key expression | Payload | Status |
| -------------- | ------- | ------ |
| `continuo/{world}/actor/{id}/pose` | actor pose | built |
| `continuo/{world}/actor/{id}/accel_cmd` | commanded acceleration, [-1, 1] | built |
| `continuo/{world}/actor/{id}/steer_cmd` | commanded steering, [-1, 1], +1 full left | built |
| `continuo/{world}/conductor/membership/status` | join or leave that took effect | built (M5) |
| `continuo_viz/{world}/**` | observer side channel, mirroring `continuo/{world}/**` beneath it | built (M5) |
| `continuo/{world}/tick` | `TickStart` | M7 |
| `continuo/{world}/tick/done` | `TickDone` | M7 |
| `continuo/{world}/conductor/membership/join_request` | registration request | M7 |
| `continuo/{world}/conductor/membership/leave_request` | departure request | M7 |
| `continuo/{world}/scene` | scene-graph state update (fixed-period publisher for renderers) | deferred |

The status column is there because the table previously mixed what exists
with what is intended and said nothing about which was which. Scenario-owned
keys are deliberately absent: the traffic demo's spawn and despawn requests
live with the spawner, not here, for the reason recorded on 2026-07-31.

Membership is nested one level deeper than the rest so that *asking* and
*having happened* are separated by structure rather than by remembering which
verb means which. A request is something a component or host sends inward and
the conductor may reject; a status is the conductor saying it already did it,
which is what an observer subscribes to. The earlier flat
`conductor/join` and `conductor/leave` blurred that, and called one a request
and the other a notice for no reason beyond the order they were written in.

## Determinism rules

Baked in from the start, because they are hard to retrofit:

- All sim times and next-step times are **integer nanoseconds**; float
  arithmetic is allowed inside a step, but anything entering the schedule is
  rounded to the nearest ns first. Never accumulate float time.
- **Across actors, outputs published at time T are visible only at the
  consumer's next step after T**, so co-due components never see each other's
  same-instant outputs (double-buffered mailboxes). Removes cross-actor
  ordering sensitivity; also what makes distributed lockstep possible. Within
  a composite, same-instant delivery follows the declared child order, which
  is explicit and therefore deterministic.
- Inboxes delivered sorted by `(publisher_id, sequence)`, never arrival order.
- **Transcendentals go through `libm`, never through the platform's.** IEEE
  754 pins add, multiply, divide and sqrt and requires nothing of `sin` and
  its neighbours, so each C library rounds them as it likes. `libm` is a
  pure-Rust port of MUSL's, so the same bits come out everywhere and the
  world hash is portable by construction rather than by measurement, which
  is the reasoning that already made the hash and the RNG owned. A
  `disallowed-methods` entry per function keeps the inherent methods out,
  since nothing else would report reaching for one.
- Per-component RNG seeded from `(world_seed, component_id)`. No OS entropy or
  wall clock in sim logic; wall clock exists only in the conductor's pacing.
- Membership changes **only at tick boundaries**, and every request
  **names the sim time it takes effect** (a join its first step, a leave
  its first *non*-step) rather than taking effect on arrival. A dynamic run
  then reproduces however early or late the request was made, which is what
  keeps it replayable once requests travel over a transport and delivery
  timing stops being fixed. Each is recorded twice: where it took effect,
  which a re-run must reproduce, and when the request was processed, which
  is an observation because delivery is what decides it.
  - **The whole join waits** for the boundary before its `first_due`, not
    only its announcement: registry slot, subscriptions, execution order
    and schedule entry happen there too. Registering a newcomer when its
    request was processed would subscribe it then, so its first inbox
    would hold whatever was published while it waited, and its place among
    its siblings would say when it was asked for rather than when it
    arrived.
- Per-tick canonical **state hash** (e.g. xxhash over serialized state) as the
  determinism check. Two runs with the same seed must produce identical hash
  streams; this becomes a CI test.
- FMU caveat: FMUs are black-box native code, and **hashed on their outputs**.
  Hashing supports both modes per component, so an FMU could join in
  state-hash mode, but `canSerializeFMUState` is not the signal for it: the
  standard defines it as meaning the serialization functions exist, and
  promises nothing about what the bytes contain or that equal states produce
  equal bytes. Byte stability can only be established one FMU at a time, by
  measuring, so it is a per-mapping opt-in. The reference FMUs published by
  the body that wrote the standard show the pessimistic case is real:
  serialization there is a `memcpy` of the whole instance struct, pointers
  and padding included, so the bytes differ between runs of one binary on one
  machine. Restoring is unaffected, since those FMUs skip every pointer when
  state is set back. The default stays hashing outputs and
  trusting the vendor for internal determinism. DECISIONS.md, 2026-08-11, has
  the measurement.
- Cross-machine float determinism holds only for same architecture + build flags
  (no fast-math). A known constraint of milestone 7, not a bug.

## Messages

All transport payloads are JSON via `serde_json`:

- `serde_json` round-trips `f64` exactly (Ryu / minimal-digits), so JSON does not
  threaten replay fidelity.
- Serialization must be canonical so the serialized bytes can be hashed directly:
  struct fields in declaration order (serde default), and **never serialize
  `HashMap`** in messages; use `BTreeMap` or `Vec` of pairs. Hash-ordered
  collections are in fact banned everywhere, not only on the wire, and a lint
  enforces it (see the 2026-08-02 decision below).
- **Pose convention**: SI units throughout. Right-handed, Z-up world frame
  (ENU); body frames X-forward/Y-left/Z-up (ROS REP-103 style). A pose is a
  position `{x, y, z}` in meters plus a unit quaternion `{w, x, y, z}`,
  always **named JSON fields, never arrays**, so axis/component order can't be
  misread. 2D dynamics simply publish `z = 0` and yaw-only quaternions; the
  schema never changes when models go 3D.
- **Euler angles at the human boundaries only.** User config and
  orientation-related APIs accept Euler angles with one standardized
  convention: **roll-pitch-yaw about body X, Y, Z as intrinsic Z-Y-X** (apply
  yaw, then pitch, then roll, the REP-103/aerospace convention), named fields
  `{roll, pitch, yaw}`. `continuo-core` provides the canonical Euler <->
  quaternion conversions (radians in API; config uses degrees via explicit
  `rpy_deg` naming). **Wire messages carry quaternions only**, so Euler never
  enters hashed payloads, avoiding gimbal ambiguity (quaternion -> Euler is
  only unique with pitch constrained to +/-90 degrees) and keeping one
  canonical orientation encoding.
- **Time on the wire is decimal seconds** (e.g. `"sim_time": 1.234567891`) for
  human readability, with at most 9 fractional digits (nanosecond precision).
  Internally `SimTime` stays integer nanoseconds; serialization and parsing go
  through **integer math** (whole-second and nanosecond parts formatted/parsed
  as integers, never converted through `f64`), so the representation is exact
  at any sim duration and the bytes stay canonical for hashing. Canonical
  form: trailing zeros trimmed, at least one fractional digit (`1.5`, `2.0`,
  `0.033333333`).
- Message schemas are versioned and kept flat/simple (poses, scalars) so the
  keyexpr <-> FMU-variable binding stays a config file, not code.
- Cost: encode/decode throughput at high actor counts. Acceptable at this scale;
  a binary format could be reintroduced per-transport later without touching
  components.

## Scheduling: self-reported next-step times

There is no global rate and no divisor table. Each component returns
`next_due` in its `TickDone`; the conductor keeps a schedule (min-heap) and
advances sim time to the earliest due time each iteration.

- **All times are integer nanoseconds.** A component may compute its period or
  schedule in float, but must round to the nearest nanosecond before
  reporting. Scheduling comparisons are then exact integer comparisons, with no
  float-equality hazards and no rational/multi-clock bookkeeping.
- **Strict advance guard**: `next_due` must be strictly greater than the
  current sim time (>= 1 ns ahead). Reporting a time at or before "now" would
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
  services are ordinary components, so the conductor gains no special cases.
- If a model needs finer internal resolution than its reporting period (e.g.
  stiff physics), it sub-steps internally within its `step` call.

## Pacing

Lives entirely in the conductor:

A single config field, `pacing: Pacing`:

- `Pacing::FreeRun`: advance immediately after the barrier.
- `Pacing::RealTime { spin_padding }`: 1x real-time, waiting until the wall
  time corresponding to the next step's sim time. If the sim can't keep up,
  it simply runs slower than real time and **logs the overruns**, with no
  catch-up (the wall-time anchor slips by the overrun amount rather than
  sprinting to make up lost time), no scaling, and **never skip steps**
  (determinism). Lateness is only counted once it accumulates past a
  re-anchor threshold, so transient jitter is absorbed by the next wait
  instead of reported, giving a *bounded* catch-up capped by the threshold.
  The run never gets ahead of schedule, only back to it. `spin_padding`
  chooses how a wait is *spent* (OS timer alone, or sleep then busy-spin the
  tail for sub-millisecond accuracy), never what the wait achieves.

Sim logic never sees which mode is active. Message timestamps are sim time
throughout, so an instant that starts late still stamps its outputs with the
sim time it was scheduled for, which is why pacing cannot move the world
hash.

## Per-component timing

The conductor must never hang indefinitely waiting for a `TickDone`. Every
component therefore declares its own wall-clock limits as part of its
registration metadata: two levels in one declaration, answering different
questions, with only the hard one able to act.

- **budget** (soft, diagnostic): did *this component's* `step` finish in
  time? Measured around the step itself, so it is attributable to the
  component and to nothing else: not to whatever ran ahead of it in the
  instant, and not to the schedule around it. A step over it is logged and
  counted, and that is all. Nothing about the run changes, so a run that
  misses every budget produces the identical world hash to one that misses
  none. `None`, the default, declares no budget; a scenario is free to give
  one component a deadline an operator wants flagged on every miss and leave
  its neighbours with no real-time restriction at all.
- **timeout** (hard, policy): is the conductor still willing to wait? This
  is the barrier deadline, so it covers everything between dispatching a
  component and hearing back from it, transport included once that is a
  network. `OnTimeout` declares what happens when it runs out:
  - **`Halt`** stops the world, and is the default. Everything published
    before it is unchanged, so the hash stream stays valid up to where it
    stops.
  - **`Remove`** logs the failure, deregisters the component at the barrier,
    and continues, which **changes the scenario, and therefore the hash**. A
    deregistered component stops publishing, so every tick after it differs
    from a run where it survived, and the trigger is wall-clock-dependent, so
    a live re-run on a faster machine may remove it later or not at all. The
    removal is recorded in the event log like any other leave, which keeps the
    run **replayable**, but it is no longer reproducible from
    `(seed, scenario)` alone. That is precisely why halt is the default.

In-process the two limits coincide, because the conductor's wait *is* the
call: `step` is synchronous with nothing in between, so one measurement is
what both are judged against. Distributed they separate: the budget is
measured by the host running the step and rides back in its `TickDone`, while
the timeout stays the conductor's own wait. Keeping the soft level permanently
soft is what makes that split harmless: a limit that never acts never has to
mean the same thing on two machines.

Both are judged every step, never collapsed into whichever is worse. A step
slow enough to time out has missed its budget too, and that miss is worth
counting; more importantly, once a transport is in between, **a timeout with
the budget intact is how you tell a slow network from a slow component.**

Both levels are recorded in the event log as well as counted, so a run's
timing reads back from one file rather than from whichever process each step
ran in. They are the log's **observations**: every other line records what
the *run* did and a faithful re-run must reproduce it, while these record
what the *machine* did, which a re-run is free to differ on. Verification
skips them for exactly that reason, since comparing a budget miss would report
two runs that behaved identically as divergent.

The *leave* a timeout removal produces is not an observation. It changes the
scenario, so it is an ordinary `Leave` and is compared like one. It is
deliberately indistinguishable from a scripted leave, so that replaying the
run by asking for that leave at that instant still matches. What says the
removal was a timeout is the observation recorded beside it, which is also
the only trace a *halt* leaves: without it, a halted run's log simply stops
without saying why.

When a membership request was processed is an observation too, and splits
the same way. A join or leave that has taken effect is an ordinary `Join`
or `Leave`, sitting at the instant it names, and a re-run must reproduce
it. Where the conductor took the request in is a line of its own, because
that boundary is delivery's to decide once requests cross a transport, and
a run that behaved identically must not read as divergent for having taken
a request in a tick earlier. It is also the only trace left by a component
asked for and then withdrawn before its join took effect, which produces
no `Join` and no `Leave`: it was never admitted, so nothing ever saw it
arrive, and announcing its departure alone would be a leave for something
that was never there.

Whichever level a step passes, **the verdict never edits the tick it was
measured in**: the step has already run, so its outputs stand and its tick
fingerprints exactly as it would have anyway. Halting ends the run after that
tick's work; removal takes effect at the next tick boundary like any other
leave, since membership is frozen for the whole of a tick. Either way the
failure is logged with the component's path and the duration that passed the
limit. Component panics in-process are caught at the host boundary and treated
the same as a timeout.

Neither level is the pacing overrun of milestone 3, which asks whether the
*schedule as a whole* tracked the wall clock and is attributable to no
component in particular.

## FMI 3.0 CS support

- `continuo-fmi` crate providing `FmuComponent`: an adapter that on each step
  reads its inbox -> sets FMU input variables, calls `fmi3DoStep(t, dt)` with
  `dt` = elapsed sim time since its last step (FMI 3.0 CS supports variable
  communication step sizes), gets outputs -> publishes them. The FMU's period
  comes from its mapping config and is reported as its next-step time.
- FMI **3.0 Co-Simulation only**, with no 2.0 shims, native arrays, `float64`
  value references.
- Per-FMU mapping config binds key expressions to variable names (borrowing
  ideas from FMI's SSP standard for wiring configuration).
- Candidate crate: `fmi` (fmi-rs) for 3.0 import. Validate early, because FMU
  loading is `libloading` + native code, and Windows DLL handling can be finicky.
- Demo target: an FMI 3.0 reference FMU from the Modelica Association set
  (BouncingBall, VanDerPol, Feedthrough).

## World and map

- A **generic world specification** owned by continuo, deliberately *not*
  autonomous-car specific. Initial schema (JSON, versioned like all messages):
  coordinate frames, static geometry (obstacles/bounds), and named **paths**
  (polylines / parametric loops) that actors can follow.
- Rich road-network formats (OpenDRIVE, Lanelet2, or something else, undecided)
  arrive later as **importers** that lower into this spec, so actors never
  depend on a third-party format directly.
- The world spec is published on the transport like everything else
  (`continuo/{world}/map`), so late-joining components and the visualizer can
  fetch it.
- Structure it as a minimal **scene graph** from the start, with nodes carrying
  ids, optional parent, transform, and an open-ended properties bag, so v1 stays
  basic (frames, geometry, paths) while later growth (semantic tags, spawn
  points, richer geometry) extends nodes instead of reshaping the schema.

## Scenario configuration

- Scenario files are **JSON5**, human-friendly JSON with comments and
  trailing commas (Rust: `json5` crate; Python: `pyjson5`). Wire messages stay
  strict canonical JSON; JSON5 is for files humans edit.
- The scenario defines: world seed, the world spec (or a reference to it), and
  the **full component tree**: actors, their ordered sub-components, each
  component's type, period, and parameters. Composition is data-driven
  (SSP-style): the scenario names component *types*, and a registry in the
  host instantiates them (Rust-native types and FMUs alike, where an FMU entry
  points at the `.fmu` and its variable-mapping config).
- The scenario owns initial actor spawning; runtime spawning (e.g. the traffic
  spawner) goes through the normal join protocol.
- Orientations in scenario files are written as Euler angles in degrees
  (`rpy_deg: { roll: 0, pitch: 0, yaw: 90 }`) and converted to quaternions on
  load using the standardized convention (see Messages).

## Visualization

- `continuo-viz-bridge`: a component subscribing to
  `continuo/{world}/actor/**/pose`, throttled to ~30 Hz, serving JSON over a
  WebSocket. No translation layer needed, since the wire format is already JSON.
- `python/continuo_viz`: WebSocket client + 2D top-down view.
- When distribution lands, the Python side swaps its WebSocket client for
  `zenoh-python` with the same message schema, so keep the schema versioned and
  stable from milestone 5 on.

## Distribution (Zenoh)

- `continuo-transport-zenoh` implements the `Transport` trait.
- A standalone host binary runs components out-of-process, in lockstep via the
  same tick protocol.
- Note: the per-tick gather must stay deterministic (sort by publisher/sequence,
  not arrival), which the determinism rules above already guarantee.
- **Joins declared for one instant need an order that is not arrival
  order**, for the same reason and by much the same means. Sibling order is
  the execution order within an instant and the visibility rule's earlier
  sibling, so letting delivery decide it lets delivery decide the run.
  There are two parts. A request should be able to carry several components
  in a stated order, which settles the case that matters, a composite
  admitted whole by one requester, and gives admission the unit
  `remove_component` already has. What is left is two requesters declaring
  one instant, ordered by the sim time of the request and then by requester
  and its own sequence number. Never by a wall clock: a re-run stamps
  differently, two hosts have two clocks, and every wall-clock quantity
  here is deliberately an observation, which is the stream a re-run is free
  to differ on.
- One conductor per world; remote processes run **hosts** (component
  container + transport bridge + publish stamping). Data flows host<->host
  over pub/sub without routing through the conductor. Because same-instant
  delivery never crosses hosts (see Hierarchy), the remote inbox release
  rule is simply `msg.time < now`, and sibling-order knowledge never leaves
  the conductor.

### What `step_once` becomes

Worked out while building milestone 4's timing, and written down so it is
not re-derived: **the conductor does not grow a second step loop, and does
not grow per-component `Local`/`Remote` branches inside the one it has.**

Almost nothing in `step_once` depends on where a component runs. The tick
boundary, pacing, hash chaining, fingerprint emission, rescheduling, the
strict-advance guard, the timing verdict: all of it reads the same either
way. What changes is three consecutive lines in the middle (drain the
inbox, call `step`, publish the outbox), and those collapse into one idea:
*obtain this component's contribution to this instant*.

- **The seam is a value, not a branch.** Roughly
  `{ next_due, contribution, step_wall }`, where `contribution` is this
  component's fold into the tick hash. Local: call `step`, publish, hash.
  Remote: the host does all three and `TickDone` carries the outcome back.
  The conductor's own code is identical.
- **The tick hash must become a fold of per-component sub-hashes**, in
  declaration order, because the conductor never sees a remote component's
  published bytes; the host publishes them. This is the load-bearing
  prerequisite, it is a pure refactor with no distribution in sight, and it
  has an exact acceptance test: the traffic demo's world hash must not
  move. Worth landing early rather than inside the M7 pile.
- **Acks are collected unordered.** Waiting in declaration order would
  serialize the barrier at the *sum* of the hosts rather than the maximum,
  discarding the parallelism distribution is for. Determinism comes from
  folding in declaration order once the set is in, not from waiting in it,
  the same principle as sorting inboxes by `(publisher, seq)` rather than
  by arrival. This is safe precisely because same-instant delivery never
  crosses a host boundary: components the conductor can dispatch
  concurrently are exactly the components with no same-instant
  relationship, and a coupled composite's internal ordering is its host's
  problem.
- **A remote component must always carry a timeout.** `StepTiming::timeout`
  is optional today, which is safe only because an in-process `step` is a
  synchronous call that always returns. A remote component without one is
  an unbounded barrier wait, the hang this document opens by forbidding.
  Either require it at admission for remote components or default it from
  the world.
- **Skip the budget check when no measurement arrived.** The step duration
  becomes `Option`: the host measures it and reports it in `TickDone`, so a
  host that never answers reports nothing. Never substitute the conductor's
  wait for it. That would attribute transport delay to the component, and
  would make every timeout also a budget miss, destroying the one signal
  that tells them apart.
- **A timeout then has three diagnoses, and whatever is recorded should say
  which**: the measurement arrived and was within budget (the network or
  the host was slow), it arrived and was over budget (the component was
  slow), or nothing arrived at all (the host is silent, and nothing is
  known about the component). Only the third is new; the first two are the
  in-process pair.

The rule that covers both modes at the barrier: **a tick is the fold of the
contributions that arrived by its deadline, and a verdict never retracts one
that did.** In-process a synchronous call always arrives, which is why
milestone 4 could state that as "the verdict never edits the tick it was
measured in".

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

Plain threads and channels (crossbeam), no tokio initially: a deterministic sim
loop wants synchronous control flow, and Zenoh's Rust API can be used without
owning an async runtime later.

## Milestones

1. **Skeleton ticks**: workspace, core types, `InProcTransport`, conductor
   loop with composite components and next-step-time scheduling, static
   component set, free-run. Demo: cars circulating on a path, each a composite
   (`controller -> physics`) with the controller at a slower period than the
   physics; poses logged.
2. **Determinism harness**: seeding, per-tick state hash (state-hash vs.
   output-hash per component), record/replay, CI test asserting identical hash
   streams.
3. **Pacing**: `pacing: Pacing` config (default `FreeRun`), 1x wall-time
   gating with anchor-slip once lateness accumulates past the re-anchor
   threshold, overrun logging.
4. **Dynamic join/leave**: registration metadata shaped for the transport,
   tick-boundary application, live traffic spawner, replay still deterministic
   via event log. Registration metadata gains per-component timing: step
   budget and timeout policy together (see "Per-component timing"). Requests
   still arrive as direct calls: a join hands over a `Box<dyn Component>`,
   which no transport can carry, so sending one only means something once a
   remote host can run what it admits (milestone 7).
5. **Visualization**: viz bridge + Python package; watch the traffic move.
6. **FMI**: `continuo-fmi`, mapping config, FMI 3.0 reference FMU driving an
   actor.
7. **Distribution**: Zenoh transport, standalone host binary, one component
   out-of-process in lockstep, Python viz on zenoh-python.

FMI before Zenoh: it exercises the component abstraction while everything is
still in one process, making native-code issues easier to debug without
networking in the mix. They are independent and can swap if priorities shift.

## Decision log

Moved to [DECISIONS.md](DECISIONS.md), which records why the design is
what it is, in the order the questions were settled. It lives apart because
it had grown to two fifths of this document, and a plan should lead with
what the system *is* rather than the history of how it got there.

## Deferred (decided-not-now, revisit when they bite)

### Determinism and correctness

Both are about the numbers a run produces rather than about what it is
computing, and each changes the world hash when fixed, which invalidates
recorded logs. They want landing together, and alongside the binary mode
under "Wire format", rather than churning the fingerprint once apiece.

- **Folding yaw into [0, TAU) breaks mirror symmetry.** `rem_euclid` is exact,
  but the fold it performs is not even-handed: it leaves a positive angle
  alone and rewrites a negative one as `TAU` minus it, which rounds. Two cars
  mirrored about the road therefore drift apart. Driving `UnicyclePhysics`
  directly, one 10 ms step at 0.3 rad/s gives yaws of 0.003 and
  -0.0030000000000003583, and after 500 steps the pair of positions has parted
  by about 1e-12 m.

  `wrap_to_pi` in `control_laws` is this problem already solved, and its doc
  comment argues exactly this case: it wraps the magnitude and puts the sign
  back, so it is odd to the bit. Folding to [-pi, pi] through it is a two-line
  change, and the only reason it is not made in passing is that it moves the
  world hash, which belongs to a change that is about it.

- **A pose does not survive its own JSON round trip.** `serde_json::from_str`
  into an `f64` returns a value one ulp off what `to_string` produced, for
  about one number in eight over 200,000 of the kind a run makes. The same
  numbers read through `serde_json::Value`, which `arbitrary_precision` keeps
  exact, come back unchanged, as they do through Rust's own `str::parse`, and
  stock serde_json outside this workspace behaves identically, so the feature
  is not the cause.

  Determinism is unharmed. The error is a pure function of the bytes and the
  parser is pure Rust, so every run on every platform makes the same mistake,
  which is why nothing has ever failed over it. Fidelity is another matter:
  every component decoding a pose works from a number its publisher does not
  hold, and `continuo-fmi` feeds decoded values across a boundary where the
  golden tests compare them against the native laws. The fixes are routing
  decodes through `Value`, which allocates on a hot path, or writing a
  deserializer for `f64`. Either moves the hash, so it belongs here with the
  rest.

### Wire format

- **Getting large payloads out of a viewer's way.** A camera frame or lidar
  sweep is canonical JSON like everything else, so it travels base64-encoded
  inside a JSON string: about a third larger than the raw bytes, UTF-8
  validated and re-wrapped on the way through the bridge, and copied per
  frame. The event log has the same problem for the same reason. A compact
  binary mode is where large payloads stop being text, and **decoupled**
  sub-components exist precisely so a camera can be placed on its own host, so
  both are part of the answer.

  What is still open is how a viewer avoids carrying sensor traffic it will
  never draw. Three shapes, none chosen:
  1. **A size threshold at the bridge.** Simplest, and arbitrary: no single
     number is right for both a pose and a point cloud.
  2. **The viewer declares which signals it wants.** Precise, but it is a
     filter the native Zenoh path does not have, so the two diverge. It only
     filters *relay* rather than production, so it costs fidelity rather than
     determinism.
  3. **Components skip work nothing subscribes to.** The most efficient and
     the most dangerous: published bytes feed the tick hash, so a component
     that produces less when unobserved makes the **world hash depend on who
     is watching**. That is the exact property the bridge is a transport
     monitor to protect. Recoverable only by excluding conditional output from
     the fingerprint, at which point what is hashed depends on runtime
     subscription state, which is worse. If it is ever wanted, the scenario
     should declare which outputs are optional, so the decision is static and
     reproducible rather than dependent on who happened to connect.

- **A compact binary mode alongside JSON, chosen like debug versus release.**
  JSON stays the readable mode for development, inspection, and the event log;
  binary becomes the mode for throughput. The hash is taken over the wire
  bytes, so naively the two modes would disagree about a run's identity, which
  is the opposite of what the debug/release comparison promises. Resolution:
  make the canonical binary encoding the hash input in **both** modes. Binary
  mode hashes bytes it already produced at no extra cost; JSON mode serializes
  a second time purely for the hash and eats that cost, because throughput is
  not what the readable mode is for. `StepCtx::publish` takes the *value*
  rather than pre-serialized bytes, so both encodings come from one value with
  no re-parsing and no float round-trip question. `Component::state_bytes` is
  the loose end: it is documented as canonical JSON and would have to become
  canonical binary in both modes.

  Format: CBOR, staying self-describing so logs remain inspectable and the
  Python viewer needs no generated schema. Naming CBOR does **not** pin the
  bytes, which was measured: encoding `Vec3 { x: 40.0, y: 0.0, z: 0.0 }` gives
  16 bytes under `ciborium` (which narrows floats to f16 when lossless, per RFC
  8949 section 4.2.2) and 34 under `minicbor-serde` (which always emits f64),
  sharing no bytes. So the crate and version become part of the fingerprint's
  definition. **Prefer owning the encoder** and using a crate only to decode,
  since determinism constrains only the bytes we produce, and a pinned crate
  version is the coupling the project deliberately removed for the hash and the
  RNG.

  Non-finite floats used to be the decisive argument here: CBOR encodes NaN
  payload bits faithfully where JSON collapses them, so two values that are
  both `NaN` give different bytes, and NaN payload propagation is a classic
  x86-versus-ARM difference. `StepCtx::publish` now rejects them before any
  encoder sees one, so that hazard is closed whichever way this goes. Golden
  byte tests are required either way.

  **Whoever does this must revisit that guard's fast path.** It walks the value
  only when the serialized payload contains `null`, which is sound only because
  `serde_json` writes every non-finite float as exactly those four bytes. CBOR
  writes `NaN` as its float bits and null as a single `0xf6`, so the scan would
  find nothing, the walk would be skipped, and the guard would stop working
  while every test that exercises it through JSON still passed. That is a
  silent failure in the permissive direction, so `the_fast_path_premise_holds`
  fails the moment the premise does. Either drop the fast path, costing about
  4% of the scaled world's step rate, or have the owned encoder reject
  non-finite floats itself and delete the guard's walk entirely, which is the
  better end state since it removes a second traversal rather than repairing
  it.

### Features

- **A consolidated scene view, and a switch to turn raw relay off.** The
  scene half already exists as a design: `continuo/{world}/scene` is in the key
  table above, and "Fixed-interval world services" describes a scene-graph
  publisher aggregating latest poses at 1/60 s as an **ordinary component**.
  Build it when the viewer exists and there is something to measure it
  against. The viz bridge then relays that like any other message, so the
  consolidated view and raw traffic coexist rather than trading off.

  The other half is a switch on the bridge, so a run using the scene component
  can stop paying for per-message relay. Worth building only once there is a
  scene component to switch to; adding the option first is building the choice
  before the thing it selects between.

  The distinction that decides how each is treated: **the scene publisher is a
  component, so enabling it changes the world hash**, while the bridge's switch
  cannot, being outside the sim. They look like two settings and are not the
  same kind of knob.

- **The plant should publish its whole kinematic state**, rather than a pose
  that has grown a speed. `CarState` is what the integrator carries, and the
  first thing it leaves unobservable is the actual acceleration, which differs
  from the commanded value wherever the clamp at zero bites; yaw rate would
  follow it. The cost is a rename, which is why it is a deliberate act rather
  than another field: the key says `pose` and the viewer's `pose_from_payload`
  reads it as one, so a third stretch of that message would be the dishonest
  version of this. `CarState` carries the TODO pointing here.

- **A road built from curves rather than corners.** `Waypoints` is a polyline,
  so its heading steps at every vertex and its curvature is unbounded there.
  Three things follow. An arc length recovered by projection steps at a vertex,
  which causes discontinuities in a radar's range calculated from it;
  `RadarSensor` quantifies this. And a lane walked out at a fixed offset along
  that arc length traces no continuous curve at all: each of its points is the
  road's displaced along the heading's normal, so where the heading steps the
  displacement swings and the point jumps with it, doubling back inside a bend
  and gapping outside it. **Both want only a heading that does not step**,
  which adding curvature instead of raw polylines gives.

  The third wants more, and a heading that does not step is one derivative
  short of it. Curvature is how fast the heading turns, so a road of straights
  and circular arcs has a continuous heading and a curvature that still jumps
  where the two meet. A car holding a lane is steered at a point on it, so its
  commanded yaw rate follows that curvature and jumps with it. **That needs the
  curvature itself to be continuous**, which is what a clothoid is for: ASAM
  OpenDRIVE joins the two with one so the curvature ramps rather than leaping.

  Drawing the polyline more finely shrinks all three without removing any,
  since every vertex is still a corner. Short of the geometry, the lane half
  can be papered over by clipping where the offsets cross inside a bend and
  rounding the vertex outside it, which relocates the mismatch rather than
  removing it: a lane round a corner is not the same length as the road it
  follows, so the road's arc length cannot measure both. The planned fix
  belongs with "World and map", where a road adopts a proper standardized
  geometry definition, rather than as a change to `Waypoints` on its own.

- **A component asking to retire itself**: `StepCtx` has no way back to the
  conductor, so nothing can say "I am done" (see `Component`'s TODO).
  Milestone 4 expected its spawner to need this and it did not. Worth
  building for a scripted actor that finishes its own work, and then only
  at *component* scope, never on behalf of an actor.

- **Reclaiming vacated registry slots**: `Registry::entries` never shrinks,
  so a long run with heavy turnover accumulates one dead slot per departed
  component, and the due loop skips past them for the rest of the run. Fine
  at demo scale, where thirty sim-seconds of traffic leaves eight holes, and the
  cost is bounded by *total* joins rather than by live components, so it
  only bites where a scenario churns many actors over a long run. Not
  fixable by compacting: an index **is** the execution order within an
  instant, so shifting one silently reorders components that had nothing to
  do with the departure. It needs a free list plus a generation counter on
  each slot, so a reused index cannot be mistaken for its predecessor.

- **A spatial index, so a sensor asks what is near it.** `RadarSensor`
  projects every pose in the world onto the road and filters, which is
  work proportional to the population for every sensor at every scan. A
  quadtree, or a per-lane sort by arc length, would answer against what
  is local instead, and the collision monitor's pair scan wants the same
  structure. It would also bound a scan properly: `MAX_DETECTIONS` is a
  defensive cap that sorts by range to choose what to drop, where
  locality would have decided already. Deferred because no world here
  reaches the cap, and because it belongs with the consolidated scene
  view above, the other consumer that wants the world rather than every
  message in it.

- **A subscriber cannot ask for only the latest message per key.** The
  visibility rule queues every message until the subscriber next runs, so a
  low-rate observer of a high-rate stream receives the whole interval's
  accumulation and pays to decode all of it. A 1 Hz view of a 100 Hz signal is
  an ordinary thing to want, and the work is proportional to the rate it is
  trying not to watch.

  Measured on the demo's pose logger, which sees a sim-second of poses at once:
  `drain` costs ~155 us for that batch and the logger's step ~231 us, against
  ~14 us if it decoded only the last message per key. Sixteen times less work
  for the same result, since it keeps only the latest anyway.

  A `Transport` question rather than a component one, and it interacts with
  `drain` taking a per-subscriber release condition. Note it changes what a
  component sees, so in general it changes the world hash: it is hash-neutral
  only for a subscriber that already keeps just the latest, which the API
  cannot check.

- **Host-local parallelism, built so components stay on the thread that made
  them.** Stepping components concurrently inside one process is the
  distribution protocol above run over channels instead of Zenoh: each worker
  thread owns a set of components and constructs them itself, a step request
  and its reply are plain data, and the conductor's barrier plus a
  declaration-order fold keeps the hash byte-identical. So it reuses the seam
  "What `step_once` becomes" already describes rather than adding a second
  way to step, and local and remote parallelism stay one mechanism.

  The construct-where-it-runs shape is the requirement, not an incidental
  choice of how to build it. A thread pool handed components as work items
  would need `Component: Send`, a bound this project dropped and does not
  intend to restore (DECISIONS.md, 2026-08-10), and it would exclude any
  component wrapping foreign state tied to its thread, an imported FMU
  instance being the first real case. What must be `Send` is messages and
  constructors.

  A composite is the unit of assignment, because same-instant delivery is a
  within-composite relationship: the components a conductor may dispatch
  concurrently are exactly those with no same-instant edge between them, and
  a composite's internal ordering is its owner's problem. Distribution states
  that same rule one scale up, where it reads as "same-instant delivery never
  crosses hosts".

  Worth building when component work dominates the conductor's own per tick,
  which `traffic_scale` is the instrument for measuring. Nothing at demo
  scale needs it.

- **Road-network importer**: which format (OpenDRIVE, Lanelet2, other) lowers
  into the world spec; decide when realistic road scenarios are needed.

- **Snapshot/restore**: via FMI 3.0 `SerializeFMUState` for FMUs plus a
  serialize hook for native components, only if replay-from-log proves
  insufficient.

- **External (non-deterministic) ego participation**: a relaxed admission mode
  for a live AV stack under test, after milestone 7.

- **Requiring every leave to declare its instant, decided at milestone 7.**
  A leave may still name no instant, which stops its component at the
  earliest one still open. That is reproducible today only because the
  caller is, and `remove_component_now` exists so a call site says which
  bargain it took rather than looking like a complete request.

  Milestone 7 is the trigger rather than a guess: a leave arriving over a
  transport has no meaningful "now", so it would take effect wherever
  delivery put it, which is the fault DECISIONS.md, 2026-08-23, removed
  from the join half. Requiring an instant also makes
  `LeaveMetadata::leaves_at` and `RecordedLeaveRequest.leaves_at` plain
  `SimTime` values, matching their join counterparts.

  The demo is where the work is. `SpawnTrafficRequest` carries `first_due`
  and `DespawnTrafficRequest` carries only a name, so
  `run_live_traffic_scenario`'s claim that every request declares the
  instant it takes effect is true of arrivals and not of retirements.
  A despawn given the slack a spawn has would name an instant a spawner
  period ahead, where today it takes effect at the next boundary after the
  request, so every car retires a period later and `DEMO_WORLD_HASH`
  moves.

- **The viewer's `--check` counts components and calls them joins.** Its
  summary prints `joins / leaves` directly above `actors seen`, and the two
  count different things: membership is published per component, so a car
  joining raises two events, one for its controller and one for its physics.
  A demo run reports 32 joins and 16 leaves against 15 actors, which reads as
  a discrepancy and is not one: 15 cars at two components each, plus the
  logger and the spawner at one, and eight cars retiring. Saying "component
  joins / leaves" would settle it. The CI smoke asserts those exact labels, so
  the two move together.

- **A native component that follows, so both control laws have a caller on
  each side of the FMI boundary.** The laws live in `continuo-actors` so that
  the native side and the FMU call one implementation rather than two kept in
  step by hand, and only the steering law actually does:
  `PathFollowController` calls `pure_pursuit_yaw_rate`, while `idm_accel` and
  `nearest_detection` are reached only from the FMU. The argument for where
  the laws live is sound and currently half demonstrated.
  - Which shape it takes is the open question. A second component beside
    `PathFollowController` keeps that one purely lateral, which M6 made it
    on purpose: two small commands travel separately so a learned
    longitudinal FMU can run beside native steering, and one component
    publishing both would argue against that. Teaching the existing one to
    follow is less code and gives that back.
  - What it buys beyond symmetry is a native baseline. Today the only thing
    that runs IDM in a world is the FMU, so a scenario cannot be run both
    ways to see what the boundary costs in throughput, and the packaged-FMU
    comparison is the only place the two are checked against each other at
    all.

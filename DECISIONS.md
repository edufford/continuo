# continuo Decision log

Why the design is what it is, in the order the questions were settled. Split
out of [PLAN.md](PLAN.md) once it grew past two fifths of that document and
started crowding the design it was meant to explain.

Entries are dated and kept even once superseded, with the superseding entry
noted inline, because the reason something changed is usually worth more than
the change. PLAN.md describes the system as it stands; this file describes how
it got there, including the roads not taken.


- **2026-07-16**: Transport (not "bus") is the name of the pub/sub seam;
  keyexprs follow Zenoh syntax from day one.
- **2026-07-16**: FMI support is **3.0 Co-Simulation only**.
- **2026-07-16**: All payloads are **JSON** (human-readable beats efficient at
  this scale); canonical serialization rules make the bytes hashable.
- **2026-07-17**: Self-contained deterministic sim first; external ego stack
  joins later as a lockstep client.
- **2026-07-17**: Components are **hierarchical**: actor-level components with
  ordered sub-components; same orchestration/transport model at every level.
  Cross-actor visibility is next-tick; intra-composite is same-tick in declared
  order.
- **2026-07-17**: **Multi-rate from the start**; sample-and-hold between
  rates.
- **2026-07-17**: Scheduling is **event-driven via self-reported next-step
  times** (supersedes the earlier integer-rate-divisor design): components
  return `next_due_ns` in `TickDone`, the conductor advances sim time to the
  earliest due time and barriers only on due components. All schedule times
  are integer nanoseconds, rounded on entry.
- **2026-07-17**: World spec is generic and continuo-owned; road-network
  formats (OpenDRIVE or other, undecided) come later as importers.
- **2026-07-17**: Pacing is a single boolean `RealTimePacing`: `false` =
  free-run, `true` = 1x real-time (no scale factor). If real-time can't keep
  up, run slower and log overruns. *(Superseded 2026-07-24: still one
  setting and still no scale factor, but a `Pacing` enum rather than a bool,
  so the spin padding rides on the real-time variant; see the milestone 3
  entry below.)*
- **2026-07-17**: Component tree and wiring are **scenario-config-driven**
  (SSP-style concept, not the SSP format): scenario names component types,
  a host-side registry instantiates them.
- **2026-07-17**: Poses are **3D position + unit quaternion** with named JSON
  fields; right-handed Z-up (ENU) world frame, X-forward body frames
  (REP-103 style). 2D models publish `z = 0`.
- **2026-07-17**: Scenario files are **JSON5** (comments allowed); wire
  messages remain strict canonical JSON.
- **2026-07-17**: **Euler angles allowed in config and APIs** with one
  standardized convention (intrinsic Z-Y-X roll-pitch-yaw, REP-103 style;
  degrees in config as `rpy_deg`, radians in APIs); wire messages stay
  quaternion-only.
- **2026-07-17**: Barrier failure policy is configurable per world:
  **halt** or **timeout-and-drop** (logged, event-logged for replay). The
  conductor never waits indefinitely. *(Superseded 2026-07-28: still the same
  two policies and still no indefinite wait, but declared per component
  rather than per world, and "drop" is now "remove"; see the milestone 4
  timing entry below.)*
- **2026-07-17**: World spec starts minimal but is shaped as a **scene
  graph** (nodes + transforms + open properties) for later expansion.
- **2026-07-17**: **Replay-from-log over snapshot/restore**; snapshots
  deferred (FMI 3.0 `SerializeFMUState` is the hook if ever needed).
- **2026-07-18**: **Same-instant delivery never crosses a host boundary.**
  Heavy sub-components can be declared **decoupled**: they stay in the
  actor's namespace/lifecycle but use next-step visibility, freeing their
  host placement and pipelining with their actor instead of serializing the
  instant. Coupling flag joins registration metadata in milestone 4.
- **2026-07-18**: Milestone 2 implementation choices: hash and RNG are
  **owned implementations** (FNV-1a 64, SplitMix64) so fingerprints and
  random streams are bit-stable across platforms and versions forever, with no
  external RNG/hash crates. Components are **output-hashed by default**;
  the opt-in `Component::state_bytes` hook adds state-hash mode. The world
  hash starts from `(seed, world name)` and chains per-tick hashes. Event
  log is JSON lines (header + interleaved msg/tick events, hashes as hex
  strings); milestone 2 replay is **re-execution + comparison** via
  `EventLog::first_divergence`. `StepCtx::rng()` gives a fresh per-step
  stream from `(component_seed, now)`; persistent streams seed a stored
  `DetRng` from `ctx.component_seed()`.
- **2026-07-21**: Replay verifies **live during the re-run** and stops at
  the first divergence (message and fingerprint callbacks; both channels
  are needed, since a tampered log message leaves its neighboring fingerprints
  intact, and state-only divergence never appears in messages). Post-hoc
  `first_divergence` remains for comparing two recorded logs.
- **2026-07-21**: A recorded log has two named consumers, split by how the
  log's data flows relative to the sim: **verification** (`Verifier`,
  `--verify`) treats it as an expected-output ledger, so everything re-runs
  live, nothing from the log enters the sim, divergence = broken
  determinism, halt and fail; **open-loop resimulation**
  (`PlaybackComponent`, `--resim`) treats it as input stimulus, so selected
  recorded publishers are replaced by playback doubles re-publishing their
  recorded messages, changed components run live against them, nothing is
  compared, and divergence from the recording is the engineering result
  under study. Playback doubles are pure data, so hybrid runs stay fully
  deterministic and recordable, so resim experiments are themselves
  verifiable. No general "divergence summary" mode: the sim cannot report
  behavioral differences usefully; observe resim runs like any other run.
- **2026-07-23**: Milestone 2 review outcomes (naming and module shape; no
  behavior change, and the demo's world hash is unchanged):
  - Determinism primitives are named for what they are, algorithm
    included: `HashFnv1a64` and `RandomSplitMix64` (spelled out, not an
    acronym), with the module following the type: `core::random`.
  - **Seed derivation is its own concern**, `continuo-core::seed`: it uses
    the hash to fold names down to 64 bits and the generator's scrambler to
    combine values, and belongs to neither. `mix` → `mix_seeds`;
    `derive_component_seed` and the new `derive_step_seed` live there.
    `StepCtx::rng()` → `step_random()`.
  - The world's two identifying values are named for what they hold, and
    named the same way everywhere (**`world_name` and `world_seed`**) in
    `ConductorConfig`, the event-log header, `StepCtx`, and the derivation
    functions. A bare `world: String` reads like it holds the world; it
    holds the world's name.
  - `Recorder::new` and `Verifier::new` take **`&ConductorConfig`** instead
    of a restated world name and seed, so a log's header always names the
    run that produced it, and verification always checks the log against
    the run actually about to execute. Verification never takes the
    expected world/seed *from* the log, which would make the header check
    vacuous.
  - The event log splits by what each part does with it: `record` (log +
    `Recorder`), `verify` (`Divergence`, `EventLog::first_divergence`,
    `Verifier`), `playback` (`PlaybackComponent`), with their tests in
    `tests/event_log.rs` against one shared fixture log.
    `PlaybackComponent` stays in `continuo-conductor`, since it is harness
    machinery built on `EventLog`, not a sample actor, and moving it to
    `continuo-actors` would invert the crate layering.
- **2026-07-23**: Milestone 3 (pacing) implementation choices:
  - Pacing gates each instant at the **top of `step_once`**, before any
    component runs, so every driver (`run_until` and the manual
    `next_scheduled` loops alike) gets it for free and it delays entry to an
    instant without ever touching its content. Consequence: a paced run and
    a free run of the same seeded world produce the **identical world
    hash**, the milestone's headline test.
  - The map from sim time to wall time is one **anchor `(sim, wall)`**.
    Keeping the anchor fixed while sleeping means an oversleep on one step
    is absorbed by a shorter sleep on the next (no drift). An **overrun
    re-anchors** to when the late instant actually starts, so the anchor slips
    by the overrun amount and the run stays behind rather than sprinting to
    catch up (PLAN.md "Pacing"). Steps are never skipped.
  - Overruns are logged (`tracing::warn`, target `continuo::pacing`) and
    counted; `Conductor::overrun_count()` / `total_slip()` expose them.
  - Lateness is only reported once it accumulates past
    `OVERRUN_REANCHOR_THRESHOLD` (1 ms). Below it the anchor stays put, so
    the next instant's sleep absorbs the lateness exactly as it absorbs an
    oversleep. This matters because *any* sim gap finer than the wall time
    its work costs is unachievable under pacing, the demo's 1 ns logger
    offset most starkly, and counting those as failures made
    `overrun_count` measure schedule shape rather than health. Because the
    anchor does not move, lateness keeps accumulating against it, so a sim
    that genuinely cannot keep up still crosses the threshold and is
    reported, aggregated instead of once per instant. The accessor is named
    `overrun_reanchor_count` for the event it actually counts, and documents
    that zero means "the schedule tracked the wall clock", *not* "every
    component finished within its time".
  - The threshold is therefore also a **catch-up budget**, which caps it
    from above: absorbing lateness makes the next interval run short by that
    much, briefly faster than 1x (though never ahead of schedule). It must
    stay well under the shortest component period, because above a sim gap that
    recovery becomes a run of zero-sleep instants, the sprint "no catch-up"
    exists to prevent.
  - The anchor/slip arithmetic is isolated behind a `WallClock` trait so it
    is unit-tested against a manual clock (no real sleeps); the conductor
    uses `SystemClock`.
  - Pacing is **one config field**, `pacing: Pacing` (`FreeRun` or
    `RealTime { spin_padding }`), replacing the earlier
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
    exactly zero-padding spin: one `SystemClock::sleep` path, its
    sleep-vs-spin cutoff a pure function with its own unit test.
  - Failure handling at the barrier (`on_component_timeout`, see
    "Per-component timing") is **not** part of M3, since it needs the
    join/leave machinery of M4 to drop a component mid-run, so it lands there.
- **2026-07-24**: **Per-component step budgets are M4, not M3**, together
  with the timeout policy they share a measurement with (see "Per-component
  timing"). Considered for M3 since real-time scenarios want per-component
  deadline flagging, and rejected there for three reasons: a budget is the
  soft half of `on_component_timeout`, so building them apart risks two
  overlapping per-component durations instead of one declaration with an
  escalation level; the budget belongs in registration metadata, whose shape
  M4 is already reworking for remote components; and M3 stays a clean single
  concern, the conductor tracking the wall clock.
  - Two shaping decisions were settled while scoping it, so M4 need not
    re-derive them: the budget measures **the component's own `step`
    duration** (not completion relative to the instant's start, which would
    fold in queueing behind earlier components), and it is declared **at
    registration** rather than on the `Component` trait, because a deadline is
    a deployment property (the same physics model has one on a HIL rig and
    none in a batch run), and this keeps wall-clock types out of
    `continuo-core` entirely.
  - Rejected alternative: **per-component pacing strictness**, letting a
    component declare that its own overruns don't matter. It cannot replace
    the re-anchor threshold, whose job is absorbing transient
    jitter so it self-corrects, and strictness cannot express transient vs.
    sustained, so jitter would re-anchor and accumulate as permanent drift.
    It also mismatches the model: pacing is per-*instant*, and co-due
    components would need an arbitrary combining rule. And the lateness it
    would suppress is not the lax component's anyway. It is caused by the
    previous instant's work, and belongs to the gap.
- **2026-07-28**: Milestone 4 membership design (joining and leaving a
  running world):
  - **Requests name the instant they take effect, and it is half-open.** A
    join declares `first_due` (its first step), a leave declares `leaves_at`
    (its first *non*-step), so a component present for `[0, 10ms)` joins at 0
    and leaves at 10 ms, and one component's `leaves_at` is the next one's
    `first_due` with no off-by-one reasoning about periods. Declared rather
    than inferred because only the requester knows the phase it wants, and
    because it is what makes a dynamic run reproducible when a request's
    *arrival* varies, which it will as soon as requests cross a transport.
  - **A departure vacates a registry slot rather than removing it.** An index
    *is* the execution order within an instant, so compacting the vector
    would silently reorder components that had nothing to do with the
    departure, and with them the visibility rule's "earlier sibling"
    relations. Reoccupying a freed path is a *new arrival*, taking a fresh slot
    at the end of the parent's child list, so arrival order drives both
    the execution order (index) and the visibility rule (tree position), and
    the two cannot disagree about who is earlier. A disagreement would not
    fail loudly; it would just stop a same-instant hand-off arriving.
  - **The log records the declared instant, never the applied one.** The
    applied instant is redundant, since the event's position between tick
    fingerprints already says which instants it fell between, and it is
    exactly the part that varies with delivery, so comparing it would report
    divergences for runs that behave identically.
  - **A request naming an already-stepped instant is an error**, not a
    silent no-op, and validation precedes mutation so a rejected request
    leaves no half-registered entry or stray subscription. Quietly resolving
    a late request to the next open instant would put it a nanosecond after
    the last one: an arbitrary phase, and the too-fine-gap pathology pacing
    already has to absorb.
  - **Scheduled leaves apply at the tick boundary before the instant is
    entered**, found by *peeking* the schedule rather than popping it.
    Popping first hands the due loop a set that still lists the departing
    component, and leaves an instant holding only that component unprunable,
    so it becomes a tick with nobody in it, numbered and fingerprinted and
    chained into the world hash. That pending-leave queue is the
    tick-boundary queue the timeout policy's removal will reuse.
  - Vocabulary: the conductor **adds and removes**; a component **joins and
    leaves**. `add_component`/`remove_component` against
    `JoinMetadata`/`LeaveMetadata` and `LogEvent::Join`/`Leave`, with no third
    verb for the same event.
  - Deferred within the milestone, both to section 5 because its traffic
    spawner is what needs them: removing a composite should take its whole
    subtree (**one leave per leaf**, since every join names a leaf), and a
    component should be able to ask to leave. A car that has driven out of
    the scene should retire itself rather than have the spawner watch every
    pose to notice. Only the way back from `StepCtx` is missing; what the
    request does when it arrives is the `pending_leaves` queue built in
    section 3, which applies a leave at the next tick boundary, exactly
    where a mid-tick request has to take effect. Both were settled on
    2026-07-31: the subtree removal built, the voluntary departure rejected.
  - Deferred to **M7**: requests arriving over the transport rather than as
    direct calls. Not a scheduling matter after all, since a join carries a
    `Box<dyn Component>`, which no transport can carry, so the request only
    means something once a remote host owns and steps the component it
    admits. What M4 can and does deliver is the half that survives the
    crossing: metadata split from the component, and declared instants
    (`first_due`, `leaves_at`) chosen precisely so a run reproduces when a
    request's *arrival* varies.
- **2026-07-28**: Milestone 4 per-component timing, as built (see
  "Per-component timing"):
  - **The timeout policy is declared per component, not per world**,
    superseding the 2026-07-17 entry above. A world-level setting cannot
    express the case that motivates the feature at all: one component
    carrying a deadline while its neighbours have no real-time restriction
    whatsoever.
  - **The two levels measure different things**, correcting the 2026-07-24
    premise that they share one. The budget is the component's own `step`,
    measured where the step runs; the timeout is the conductor's *wait*, the
    barrier deadline, which once components are distributed necessarily
    includes the transport. They coincide only in-process, where that wait is
    a synchronous call, so one measured duration is what both are judged
    against today; at M7 each reads its own.
    - **Judged separately, never as one worst-level verdict.** Either can be
      passed without the other, and the pair is what carries the diagnosis: a
      timeout with the budget intact says the transport was slow, one with the
      budget missed says the component was, a state a single verdict on a
      single number cannot even represent. Worst-wins also quietly dropped the
      budget miss that accompanies every timeout.
    - That is what settles the soft level as **permanently** soft: a limit
      that never acts never has to mean the same thing on two machines, so a
      host can measure its own step and report it for diagnosis with none of
      the cross-machine comparability a policy trigger would demand.
      Escalating on a host-measured step, the rejected alternative, would
      have needed a second hard limit at the conductor anyway, since a host
      that dies reports nothing at all.
  - **`drop` is called `remove`** (`OnTimeout::Remove`), for the vocabulary
    rule above: a third verb for the same event is exactly what that rule
    exists to prevent. It surfaces in the log as an ordinary `Leave`.
  - **A timing verdict never edits the tick it was measured in**, so removal
    is queued as a leave at the earliest open instant and goes out through the
    same `pending_leaves` path a declared leave takes. Discarding a timed-out
    step's outputs instead, imitating a distributed barrier giving up on a
    missing `TickDone`, would break the invariant that membership is frozen
    for a whole tick: the component was a member of that tick, so its work
    belongs to it.
  - **A schedule violation still always halts**, whatever the timeout policy
    says, a violation being a component returning a `next_due` at or before
    the instant it just stepped, breaking the strict-advance guard that keeps
    sim time moving. Determinism is what decides it: a timeout is
    wall-clock-dependent, which is the whole reason `Remove` exists at all,
    while a violation is a pure function of the component's logic and the sim
    state and so reproduces at the identical instant on every machine and
    every re-run. Removing the component would trade a loud, perfectly
    reproducible bug for a silent scenario change, and a changed hash. Nor is
    there anything to carry on from: the only handle the conductor has on when
    to wake a component is the `next_due` it just returned.
  - **A budget at or above its timeout is rejected at registration**, since
    the conductor gives up before any step slow enough to miss it can finish.
    That holds however the two are measured, since a wait always contains the
    step it is waiting on, so it survives them separating. Misdeclarations are
    rejected rather than silently ignored, as joins and leaves in the past
    already are.
  - Timing applies in free-run too: a budget measures what a step costs
    whichever way the run advances, and the barrier needs a deadline
    regardless of pacing.
  - Counted per component (`Conductor::budget_misses(path)`) rather than as a
    run-wide total, because attribution is the whole point: it answers what
    `overrun_reanchor_count` structurally cannot, namely whether *this*
    component finished within its time.
  - **Timing is recorded in the event log too, which splits the log into
    expectations and observations.** Counting alone dies with the run, and
    once components are distributed a host's local log only knows its own
    steps, so this is reported to the conductor and written centrally. But it
    cannot be *verified*: a budget miss changes nothing, so a faster machine
    records none, and comparing them would call two identical runs divergent.
    Both readers (live checking and log-vs-log) therefore filter
    observations out.
    - Every observation nests under one `LogEvent::Observed` variant rather
      than getting a top-level variant each, so the category is structural.
      A new kind added to `RecordedObservation` is classified correctly the
      moment it exists, where a new top-level variant would silently become
      an expectation and start reporting false divergences. Pacing overruns
      are the obvious next member: a run-level wall-clock measurement that
      today only exists as a counter dying with the process.
    - **The reason a component timed out is an observation; the leave it
      causes is not.** The leave changes the scenario, so it is compared like
      any other, and is deliberately indistinguishable from a scripted one,
      so replaying the run by asking for that leave still matches. Putting a
      reason *on* `RecordedLeave` would either break that replay or need a
      struct whose fields are selectively compared. The observation beside it
      is also the only trace a **halt** leaves, which otherwise ends a log
      with no indication why.

- **2026-07-31**: Milestone 4's live traffic demo, and what it settled about
  who may change membership:
  - **The scenario is a straight highway**, not the milestone 1 oval: an ego
    holding the centre lane at 30 m/s while slower traffic spawns ahead in
    the lanes either side and is retired once overtaken. Traffic never
    shares the ego's lane, because nothing here models a collision, so cars in
    front would be driven through. `Waypoints` grew an **open** mode for it:
    a road that clamps at its ends rather than wrapping, so a lookahead past
    the end keeps pointing down the road instead of teleporting a follower
    back to the start.
  - **Lanes are Frenet offsets, not paths.** One road is shared by every car
    ever spawned, and a car holds a lateral offset `d` while following the
    arc length `s` it projects onto, so `PathFollowController` takes a road
    plus an offset, and a spawn request naming `(start_s, lane_offset)` is
    already `(s, d)`. Giving each lane its own polyline instead looked
    simpler and was worse in three ways: it allocated geometry per car, it
    made the spawner compare arc lengths measured on *different* curves
    (equal only because the curves were parallel straight lines), and it
    could not survive the road bending, since parallel curves have
    different lengths. A lane change also becomes a varying `d` rather than
    new geometry.
  - **A component decides the population; something outside builds it.** The
    spawner watches poses and publishes `SpawnTrafficRequest` and
    `DespawnTrafficRequest`; the run loop turns those into
    `add_component`/`remove_component`. The split is forced, since a component
    cannot hand over a `Box<dyn Component>`, the same reason
    join-over-transport is M7, but it is also what keeps the traffic
    pattern *inside* the determinism guarantee: the choices come from sim
    state and a seeded stream, so a recorded run verifies. A loop that
    picked spawn times itself would put the pattern outside what the log
    can check.
  - **Removing a composite takes its whole subtree**, one leave per leaf in
    declaration order, the deferral recorded on 2026-07-28, built here
    because retiring a car is what needed it. A car is a composite, so
    "remove `traffic7`" has to reach both halves, and the log shows two
    leaves rather than one: joins name leaves, so leaves must too, or a
    recorded run could not be replayed by reissuing what the log contains.
  - Timing of the application is deliberately not load-bearing. Requests
    declare `first_due`, so *when* the loop applies one does not shape the
    run, only that it lands before that instant.
  - **Verification drives the same loop as an ordinary run**, which takes an
    optional verifier and stops at the first divergence, rather than each
    example hand-rolling its own step loop. A second loop is not a small
    duplication here: forget to apply the spawner's requests in it and it
    verifies a *different* world from the one recorded, and reports the
    difference as a divergence in the sim.
  - **Rejected: a component asking to leave on behalf of its actor.** Built
    first, then reverted. An actor has no runtime existence, since the tree is
    registry data and a composite never steps, so a component speaking for
    one claims authority over siblings it is told nothing about, on behalf
    of something that is not there. Nothing joins as an actor either (joins
    are per-leaf), so there was no arrival for it to mirror. Population
    turned out to be somebody's job rather than each car's, and the spawner
    is that somebody.
  - The request type is **scenario-specific on purpose** and lives in
    `continuo-actors` beside the spawner, not in `continuo-core`: a lane
    offset in meters is not framework vocabulary. Its general form is the
    scenario config's type-name-plus-parameters request, resolved by a
    host-side registry, the same registry the run loop is standing in for, and the
    part a host takes over at M7.
- **2026-08-02**: **`HashMap` and `HashSet` are banned workspace-wide**, not
  merely in wire messages, enforced by `disallowed-types` in `clippy.toml`.
  CI already runs clippy with `-D warnings`, so the ban is a build failure
  rather than a convention.
  - The hazard is *iteration order*, not serialization. Serializing a
    hash-ordered collection breaks canonical bytes outright, but iterating
    one anywhere its order can reach sim state, the schedule, or a
    fingerprint breaks determinism just as thoroughly and far less visibly.
    A map that is only ever looked up is safe today and one `for` loop away
    from being a bug tomorrow, and that bug surfaces as a divergence far
    from the type that caused it rather than as a compile error.
  - Banning the type outright rather than reviewing each use is the same
    reasoning that made the hash and RNG owned implementations: remove the
    possibility instead of relying on vigilance.
  - Free to adopt, which is why it is a ban rather than a guideline: the
    workspace contained **zero** uses of either type when this landed, so
    nothing needed grandfathering and no `#[allow]` escapes were required.
    `BTreeMap`/`BTreeSet` iterate in key order and are what every map here
    already used.
  - The escape hatch is `#[allow(clippy::disallowed_types)]` with a comment
    saying why the order cannot matter. Having to write that comment is the
    point.
- **2026-08-03**: The viz bridge **observes the transport; it is not a
  component** (supersedes PLAN.md's milestone 5 sketch of a component that
  throttles poses and serves them over a WebSocket, along with the three
  entries below).
  - Every component's path and `next_due` feed the tick hash, so a viz
    *component* would make a watched run fingerprint differently from the same
    run unwatched, and every scenario would need a viz-enabled variant. A
    transport monitor is hash-neutral and attaches to any existing example, so
    `traffic_verify` still verifies while you watch.
  - The test that gives the others meaning is
    `an_observer_built_as_a_component_would_change_the_hash`: a component that
    reads nothing and publishes nothing still moves the fingerprint by being
    present. Without it, `hash_neutrality.rs` would also pass against a bridge
    that quietly did nothing, which is why each case additionally asserts the
    sink saw traffic.
- **2026-08-03**: **Zenoh now, for viewer output only.** The bridge
  republishes what it observes onto a Zenoh session; it does **not** implement
  `Transport` for Zenoh, which is milestone 7's job and carries the whole tick
  protocol in lockstep.
  - Inventing a TCP or WebSocket protocol would mean writing something M7
    throws away. Going through Zenoh now makes the Python viewer *final*: when
    components publish these keys natively, the viewer cannot tell the
    difference.
- **2026-08-03**: **No throttling in the bridge.** It passes messages through
  and the viewer renders at its own frame rate, decoupled from message rate.
  - Zenoh will not throttle for you, so a bridge that did would be behaviour
    the M7 path cannot reproduce, which would make the swap dishonest.
- **2026-08-03**: **Presence is added on a first pose and removed on an
  explicit leave**, rather than by a staleness timer.
  - The asymmetry is the point. A live viewer attaches at any moment and Zenoh
    replays no history, so waiting for a join would mean never learning about
    cars that were already driving. Departure cannot work that way, because
    nothing is published when a car stops existing, which is why the conductor
    publishes an explicit leave.
  - A staleness timer cannot tell a departed actor from a stalled simulation,
    and that is the distinction a viewer most needs to keep.
- **2026-08-04**: **Every frame carries metadata and states its own message
  type.** Nothing is identified by absence, and nothing by matching a key
  against a pattern.
  - An earlier cut used `Option<Metadata>`, so a membership notification was
    known by *having no* metadata. A later one matched the key against
    `conductor/membership/status`, which only moves the problem: every
    consumer re-implements the same string matching, and it breaks the day a
    key moves.
  - `MessageType` extends to the tick protocol and the join and leave requests
    at M7, and a viewer that predates them ignores what it does not know
    rather than reading it as something it is not.
- **2026-08-04**: **The viewer side channel is rooted outside the simulation's
  keys**: `continuo/{world}/...` is mirrored as `continuo_viz/{world}/...`.
  - Components publish under `continuo/` and the mirror sits outside it, so a
    relayed key cannot equal a published one **by construction** rather than
    by a fallback branch, and a message cannot be echoed back onto the key it
    arrived on once there is a real Zenoh transport.
- **2026-08-04**: **`KEY_ROOT` belongs to `continuo-core`, and rooting is
  built into a constructor** (`KeyExpr::new_rooted`) rather than left to
  string concatenation at call sites. Shipped separately as its own PR, since
  it changed already-merged code that milestone 5 happened to discover.
  - Auto-rooting plain `KeyExpr::new` was tried and rejected after measuring
    it: **140 of 141 tests passed with the demo's world hash unchanged**,
    which makes it a silent footgun rather than a safe default. The one
    failure showed `w/a` quietly becoming `continuo/w/a`.
  - A separate constructor makes rooting a choice the caller states, and the
    viz bridge is the only thing that swaps a root, so it does that part on
    its own.
- **2026-08-07**: **The viewer is a main path, not an extra.** `pygame` and
  `eclipse-zenoh` are plain dependencies of the Python package rather than
  extras, and the `viz` and `zenoh` cargo features are on by default.
  - One install runs every mode, and which mode you get is a run flag rather
    than an install choice.
  - It gives up the empty dependency list the package used to state. The code
    still needs nothing but the standard library to read a log, so the tests
    stay headless; it is only the install that is no longer minimal.
  - On the Rust side the cost is real: Zenoh is 300-odd transitive crates
    against the workspace's 21, and every build and lint now pays it. What it
    buys is that the sink compiles, which is the only thing standing between
    it and rot, since exercising it needs a live session and two processes.
    `--no-default-features` remains for anyone who wants the rest cheaply.
- **2026-08-07**: **The in-process transport answers each published key once**
  and reuses the answer, rather than matching every subscription per message.
  - A subscription is a pattern rather than a literal key, so there is nothing
    to look a published key up by: finding who wants it means testing it
    against every subscription in the world. Doing that per message made a
    world's cost grow with the **square** of its component count.
  - Measured rather than assumed, by `traffic_scale`: 100 cars went from 13.4 s
    to 0.6 s for thirty sim-seconds, 800 cars from not finishing in two minutes
    to 5.4 s, and the cost of a step stopped growing with the population.
  - Membership changes update the answers rather than emptying them, because
    components join and leave *while* a run publishes, and emptying would hand
    the cost straight back.
  - The constraint that shapes it: **delivery order reaches the world hash**,
    so recipients are held in the order the old scan produced them. The demo's
    hash and a full `traffic_verify` replay are what check that.
- **2026-08-08**: **CI compares each agent's world hash against a written-down
  value**, rather than comparing agents to each other.
  - Every matrix job ran the demo and *printed* its hash, and nothing read the
    values back, so two agents could disagree and the run would still be green.
  - Comparing agents to each other passes whenever they all move together.
    Comparing each to `DEMO_WORLD_HASH` also catches an unintended change to
    the scenario, the seed, or the hashing, needs no YAML, and fails inside the
    run that produced it.
  - The matrix grew to four agents to make the answer diagnostic:
    `ubuntu-24.04-arm` varies the architecture while holding the libm family
    constant, so architecture alone is known not to move the hash, and
    `macos-latest` varying both on top of that means Apple's libm agrees with
    glibc for every value the demo reaches. Routing the transcendentals through
    the `libm` crate is therefore deferred on evidence rather than on hope.
- **2026-08-08**: **Non-finite floats are rejected where they are published**,
  rather than reaching the wire as `null`.
  - `serde_json` writes `NaN` and `±inf` as `null` without complaint, so a
    component whose arithmetic diverged published a payload that decodes
    nowhere, and the first sign of it was a decode failure at a different
    component, at a later instant.
  - Neither obvious hook can catch it, which was measured rather than assumed.
    A custom `Formatter` never sees the float, because `serialize_f64` routes a
    non-finite value straight to `write_null`, the same call an `Option::None`
    produces; `to_value` collapses both to `Value::Null` for the same reason.
    So the guard walks the value itself with a `Serializer` that writes nothing.
  - The walk runs only when the serialized payload contains `null`, which is
    worth about 4% of the scaled world's step rate. That premise is specific to
    JSON: CBOR writes `NaN` as its float bits and null as a single `0xf6`, so a
    binary mode would break the fast path silently and in the permissive
    direction. A test pins the premise rather than a comment.
  - Halting is safe because the value is a pure function of the component's
    logic and the sim state, so it reproduces at the identical instant
    everywhere: the argument the conductor already makes for a schedule
    violation.
- **2026-08-09**: **`Component::step` returns `Result<SimTime, CoreError>`**, so
  a component can say it cannot do its job. Supersedes this plan's own proposal
  of a `StepCtx`-mediated decode reporting centrally.
  - What it closes: every `decode::<T>()` call site discarded the error, so
    physics integrated the last command, the controller steered from a stale
    pose, and a spawn request could vanish. All deterministic, so the hash held
    steady and verification passed against a recording carrying the same fault.
    The determinism apparatus catches divergence, and this never diverges.
  - PLAN.md proposed working around the signature rather than changing it,
    on the grounds that moving a public trait was the expensive part. It was
    not: of 16 `impl Component`, 5 are production and the rest are test
    components whose whole body is a next-due time, and no composite calls its
    children's `step`.
  - Removing the obstacle beat working around it three ways. It needs no new
    API rather than three pieces of one. `?` stops the step where the failure
    is, where recording it centrally lets the step finish on stale data and
    publish from it. And five `publish` sites dropped `expect`, so the
    non-finite guard halts through the error path rather than by unwinding past
    a conductor that has a standing `TODO(M7)` saying it cannot catch a panic.
  - Swallowing stays available by matching on the `Result`, which is a
    component saying so where a reader can see it, rather than an `if let
    Ok(..)` that says nothing.
  - Sites outside a step have nowhere to return to. The demo's
    `MonitorTransport` callback records the first failure and the next thing
    downstream with an error channel reports it, which stops the run at the
    following tick boundary.
- **2026-08-09**: **Every variable-length field in the tick hash carries its own
  length**, and the `b"|state|"` marker is gone. The demo world hash moved once,
  to `d747a81be039c5f1`.
  - The hash absorbed a run of fields with nothing between them, so where two
    variable-length fields touched, moving a byte from the end of one to the
    start of the next gave the identical byte stream. Two different worlds
    hashed alike.
  - A component contributes `path | next_due | [key | seq | payload]* |
    state?`. The touching pairs are payload to the next message's key, payload
    to the state after it, and one component's last field to the next
    component's path. Only the middle one had a separator.
  - PLAN.md justified the change by calling the marker unsound, and that
    overstated it. In principle it is right, since a separator must be a
    sequence the content cannot contain and payloads are arbitrary bytes. In
    practice it was unreachable: a payload is canonical JSON, `|state|` can only
    appear inside a string literal, and any split there leaves a prefix that is
    not valid JSON. The marker guarded its boundary; the holes were the two with
    nothing at all.
  - A length rather than a separator, because no separator can be safe for
    arbitrary bytes, and the encoding no longer depends on payloads staying
    JSON.
  - Not called framing, though that is the usual word for it. This codebase
    already spends "frame" on `VizFrame`, `dropped_frames`, and the viewer's
    rendered frames, so the helper is `write_with_length_prefix`.
- **2026-08-09**: **Every timestamp says `sim_time`**, including the event log's
  `msg` line, which was the last one spelling it `time`.
  - The tick line, the observed lines, and the viz bridge's wire metadata all
    said `sim_time` already, so the log's `msg` line was the odd one out and the
    Python viewer mapped one name onto the other on the way in.
  - The log format changes and the world hash does not. Field names are not
    hashed; only paths, keys, payloads, and state are.
  - `Message::sim_time` in `continuo-core` moved with it. It is never
    serialized, so it cost nothing, and leaving it would have kept the same
    confusion one layer above the log.
- **2026-08-09**: **The re-anchor threshold is 3 ms**, sized against the OS
  timer rather than against how much work an instant does.
  - The demo reported a real-time overrun once a sim-second under coarse
    pacing, always at the pose logger's instant. PLAN.md attributed it to that
    logger's accumulated inbox and the `(publisher, seq)` sort in `drain`.
  - That is not what it was, which measuring settled. Per-instant work at the
    second boundary and the logger's instant is 82 µs and 91 µs; `drain` of a
    sim-second of poses costs ~155 µs and the logger's whole step ~231 µs. None
    of it approaches the 1 to 2 ms being reported.
  - The cause is the schedule meeting the timer. The logger samples 1 ns past
    each second boundary, and a coarse sleep aimed at the boundary overshoots
    past that instant, so it is late before it runs. Every other gap in the
    demo is 10 ms or more and absorbs the same overshoot silently, which is why
    it appeared once a second rather than at every period boundary.
  - So the threshold was too small for the timer it sits on, at 1 ms against a
    measured 1 to 2 ms. At 3 ms the demo is quiet across repeated runs, and the
    value stays a fraction of the millisecond-scale periods a paced run
    schedules. 2 ms also silenced it but only just covers the observed
    overshoot, and 5 ms would reach half of a 10 ms period.
  - `Pacing::real_time_precise` already silenced it too, by spending a core to
    sleep-then-spin. That remains the answer when 1x output has to be smooth;
    the threshold is what keeps the cheap mode from reporting its own expected
    imprecision.
  - The latest-per-key idea PLAN.md proposed is still worth having, and its
    entry now argues for it on its own terms: sixteen times less work for a
    low-rate observer. It is not a fix for this, and justifying it by this
    would have been justifying it by a pacing artefact.
- **2026-08-10**: **`Component` no longer requires `Send`.** The bound came
  in with the milestone 1 skeleton and was never argued for. No design
  document mentions it, nothing in the workspace moves a component or a
  conductor between threads, and removing it changes nothing anywhere: the
  workspace compiles untouched.
  - It surfaced because milestone 6 wants to wrap an imported FMU, whose
    instance is a raw pointer into a loaded library and so is `!Send`. The
    obvious move was an `unsafe impl Send` on a wrapper, which would have
    been this workspace's first `unsafe`, and it deserved more than a
    reflex.
  - What the bound buys is one capability: constructing a component on one
    thread and moving it to another. Nothing wants that. Running a
    conductor on a background thread does not need it, since the conductor
    and its components are built inside the closure that runs them, which
    is what every in-tree use already does.
  - The parallel futures do not need it either, and this is the part that
    settled it. Stepping components concurrently within a process is the
    milestone 7 host protocol run over channels: each component is
    constructed on the thread that owns it, a step request and its reply
    are plain data, and a barrier plus a declaration-order fold keeps the
    hash byte-identical. Supervising a component that hangs is a timed wait
    on that reply. Moving one between hosts is membership: remove it and
    admit a replacement built from the same data, which is already how a
    `Box<dyn Component>` avoids having to cross a transport. What must be
    `Send` in all of that is messages and constructors. Never components.
  - So the bound guarded nothing that exists or is planned, while an FMU
    instance truthfully is not `Send`. Dropping it keeps the workspace free
    of `unsafe` and lets the type say what is true.
  - The trigger to revisit, if it ever comes, is a genuine
    build-here-run-there hand-off. Restoring the bound would be a breaking
    change for exactly one implementor, the FMU adapter, which would then
    carry an `unsafe impl Send` claiming thread-agnosticism only. Concurrent
    calls into one FMU instance are already impossible, since stepping takes
    `&mut self`.

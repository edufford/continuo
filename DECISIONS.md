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
    `next_due_instant` loops alike) gets it for free and it delays entry to an
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
    PLAN.md's Deferred list carries that shape as an item of its own, so the
    constraint sits where the work would be planned rather than only here.
  - So the bound guarded nothing that exists or is planned, while an FMU
    instance truthfully is not `Send`. Dropping it keeps the workspace free
    of `unsafe` and lets the type say what is true.
  - The trigger to revisit, if it ever comes, is a genuine
    build-here-run-there hand-off. Restoring the bound would be a breaking
    change for exactly one implementor, the FMU adapter, which would then
    carry an `unsafe impl Send` claiming thread-agnosticism only. Concurrent
    calls into one FMU instance are already impossible, since stepping takes
    `&mut self`.

- **2026-08-11**: **FMU import goes through the `fmi` crate, and it is the
  workspace's first native-code dependency.** PLAN.md named `fmi-rs` as the
  candidate. That is a real and separate project, from CATIA-Systems, who also
  maintain FMPy, and it is more substantial than its star count suggests, with
  its own `fmi2` and `fmi3` modules and schema handling. Two things decided
  against it. It is unreleased, at version 0.1.0 with nothing on crates.io, so
  depending on it means pinning a git revision of a moving target. And its
  build script compiles C on every build, so the native-toolchain cost it
  appears to save by skipping bindgen is not actually saved.
  - `fmi` 0.8.0 is released, documented and widely downloaded, and its
    `fmi-export` and `cargo-fmi` siblings are what authoring our own FMU will
    need. Taken with `default-features = false, features = ["fmi3"]`, which
    leaves out FMI 2.0 and the layered LS-BUS standard. `zip` arrives with it,
    so `.fmu` archives are read directly rather than needing a separate step.
  - Its `fmi-sys` runs bindgen, so libclang is now a build prerequisite. All
    four CI runner images ship LLVM. A Windows dev box may already have it
    through Visual Studio, whose bundled toolchain sets `LIBCLANG_PATH`;
    otherwise `winget install LLVM`.
  - Cost of the dependency, measured: the workspace's normal-edge tree goes
    from 24 crates to 97, most of it `zip` and the XML schema stack.
  - Worth revisiting if `fmi-rs` releases. The provenance is strong, its
    hand-written bindings are appealing beside bindgen, and `continuo-fmi`
    would be the only crate that changed.

- **2026-08-11**: **An FMU is data, not a Rust type.** There is one
  `FmuComponent`, and adding an FMU to a world writes a `.fmu` path and an
  `FmuMapping` rather than a type. A trait with a subtype per FMU would mean
  recompiling the host to add a model, which is the thing a standard that
  ships models as binaries exists to avoid, and it would sink the
  demonstration this milestone owes: that a third-party `.fmu` runs with no
  code written.
  - This is also where PLAN.md's scenario configuration already points, since
    a registry instantiating component types from data cannot know a model's
    layout. Today's Rust literals become JSON5 later with no new Rust either
    way.
  - The polymorphism lives one level up, at `Component`. If some future FMU
    ever needs behavior a mapping cannot express, the answer is an ordinary
    component wrapping an `FmuComponent`: composition rather than subclassing.

- **2026-08-11**: **A mapping addresses the decoded payload, not its bytes.**
  Bindings carry JSON Pointers resolved against the value `Message::decode`
  returns, and outputs are published through `ctx.publish`, so this crate
  never calls `serde_json` to encode or decode anything.
  - That is what keeps the deferred binary wire format out of it. CBOR's data
    model is the same shape for these payloads, so `/detections/3/range` means
    the identical thing, and integers above 2^53 stay exact in either mode.
  - One hazard is core's rather than this crate's but has its most likely
    trigger here: CBOR can carry NaN and Inf natively, so the non-finite
    publish guard's fast path needs revisiting when binary mode lands, and a
    diverging FMU is the likeliest thing to emit one. The existing tripwire
    is `the_fast_path_premise_holds`.

- **2026-08-11**: **An array input binds through one pattern, whose `*` the
  FMU's own dimensions expand.** `/detections/*/range` feeds a variable of any
  size, one `*` per dimension and row-major, so a mapping never writes down a
  count the model already declares. The plan had a helper generating a pointer
  per element from a prefix, a count and a field name, which meant stating a
  size the FMU states too, and the dimension check existed because the two
  could drift.
  - Omitting the source entirely derives it from the variable's name, plus one
    wildcard per dimension, so an FMU authored beside its host writes no
    addresses at all whatever the rank.
  - Pointers written out stay, for a payload no single pattern reaches:
    elements scattered rather than lying in one array, or an order the message
    does not carry. That is now the only form stating a count of its own, and
    so the only one the dimension check still guards.
  - The two forms are told apart by shape rather than by a tag, which is a
    decision about the scenario file rather than about Rust. A pointer is a
    string and a list of them is a list, so neither can be read as the other,
    and no name of a Rust variant has to leak into a config format. An object
    was the obvious third form, `{array, field}`, and it is the one that was
    dropped: a string is constrained by being a string, but "an object"
    constrains nothing, so every key inside it would have to be policed by
    hand. Putting the field into the pointer removes the open container
    instead of fencing it, and reaches nested fields an object form could not
    name.
  - The cost is a payload key spelled exactly `*`, which no escape gives back,
    since RFC 6901 has none and inventing one would be extending the RFC. Only
    a whole token counts, so `/a*b` still addresses `a*b`, and payload keys
    come from serde field names, where `*` is not a legal identifier.

- **2026-08-11**: **Every FMI 3.0 variable type binds except Clock, dispatched
  from what the FMU declares.** The adapter reads each variable's type out of
  `modelDescription.xml`; a mapping never names one, since that would be a
  second source of truth able to disagree with the binary it points at.
  - Dispatching by type is a correctness requirement rather than generality
    for its own sake. Routing an Int64 or UInt64 through `f64` loses digits
    above 2^53 in silence, and `serde_json` carries big integers exactly here
    because the workspace enables `arbitrary_precision`, so the round trip is
    lossless only if the integer path stays integral end to end. A test pins
    the trap at 2^53 + 1.
  - Outputs publish as their natural type, integers as integers, because `3`
    and `3.0` are different bytes and so different hashes.
  - Where the two type systems disagree, the stricter reading wins: a
    fractional number is not an integer, `1` is not `true`, and a Float32
    accepts precision loss but not range overflow. Anything that does not fit
    halts naming the variable, its declared type and the value.
  - String and Binary are in, though nothing in the demo uses them, because an
    importer that covers only what one FMU happens to need is not general, and
    both are cheap now against a later retrofit of the dispatch and its tests
    together. Binary travels base64, which is why that encoding moved into
    core beside the hash and the random stream.
  - Clock stays out. It is a scheduling concept rather than data, and it
    belongs with the event mode this adapter switches off.

- **2026-08-11**: **An FMU that fails to step halts the world**, as
  `CoreError::ComponentFailure` naming the instance and the call. Core gained
  that variant because a failure originating in a foreign binary has no error
  type of ours to carry it, and core cannot depend on the crate that wraps one.
  - `do_step` setting `terminate_simulation`, `early_return` or
    `event_handling_needed` all halt. The last two cannot legitimately arrive,
    since the instance is created with early return disallowed and event mode
    off, so either one means the FMU is not where the next step assumes.
  - Construction-time failures are a different type, `FmuConstructionError`,
    named for when it happens rather than what it wraps. Everything there is a
    wiring mistake that fails before a run starts, and since `step` may return
    nothing but `CoreError`, that type can never grow a step-time variant.

- **2026-08-11**: **What running the reference FMUs settled, which the plan had
  left open.** Recorded because each cost an experiment and would cost another.
  - **A value set during initialization survives.** Feedthrough handed 3.5
    between `EnterInitializationMode` and `ExitInitializationMode` publishes
    3.5 at the first step, with no `do_step` in between. So the adapter sets
    the mapping's values there, before applying the inbox, and no second
    placement is needed.
  - **An FMU handles its own events when event mode is off.** BouncingBall
    crosses a bounce, a state event at a time nothing predicted, without ever
    setting `event_handling_needed`.
  - **`fmi` logs through `log`, and tracing-subscriber's default features
    bridge it** with no `LogTracer::init()`. That is what makes a model's own
    diagnostics visible, and it turned an opaque "Error" into the sentence
    that identified the resource-path bug below.
  - **Nothing promises that serialized FMU state is a fingerprint**, so an
    imported FMU is covered in output-hash mode rather than joining the tick
    hash directly. This reverses what the plan assumed, and the flag it keyed
    on is the wrong question.
    - FMI 3.0 documents `canSerializeFMUState` as meaning those three
      functions are supported, and `fmi3SerializeFMUState` as copying the
      referenced data into a byte vector. Neither says what the bytes
      contain, and nothing in the standard says equal states serialize to
      equal bytes. So byte stability cannot be assumed from any FMU: it can
      only be established one FMU at a time, by measuring, which is why it
      belongs in a mapping rather than keyed off a capability flag. The plan
      expected an override forcing output-hash for the odd unusable FMU, and
      it wants the opposite polarity.
    - Our own FMU will not raise the question at all, which is worth knowing
      before milestone 6's PR B assumes otherwise. `fmi-export` 0.3.0 leaves
      every state function as `todo!()` and never emits
      `canSerializeFMUState`, so the capability is absent, a conforming
      importer never calls those functions, and the panic behind them is
      unreachable.
    - The reference FMUs show the pessimistic case is real, and they carry
      some weight, being published by the same body that wrote the standard
      and meant as what an importer tests against. Serialization there is a
      `memcpy` of the whole `ModelInstance` struct, which holds an instance
      name pointer,
      a `componentEnvironment`, and five callback pointers including the
      logger, which points back into the importer's own binary. Those are
      addresses, differing run to run on one machine, and the padding between
      fields is never written.
    - Restoring is unaffected, which is what makes the two capabilities
      different rather than one of them broken. The standard's own example
      for serializing is storing to a file and restarting from it later, so
      surviving a process is the intent, and a conforming FMU deals with its
      own pointers. How is its business, and the reference FMUs copy field by
      field when state is set back, skipping every pointer. So the surplus
      bytes are ignored by the only consumer that reads them, and bytes
      nobody reads are free to be anything.
    - The plan's escape hatch had the polarity backwards too. It expected a
      mapping override forcing output-hash for the odd vendor FMU whose bytes
      are not deterministic. The reference FMUs are the standard's own
      examples and the ones other implementations copy, so the unusable case
      is the ordinary one, and this wants an opt-in.
    - A second reason sits behind the first and would matter only if it were
      solved: `fmi` 0.8.0 wraps no serialization call and disables
      `get_fmu_state` with `#[cfg(false)]`, and `Instance` keeps its library
      handle and instance pointer private, so the raw bindings cannot be
      reached either.
  - **`Fmi3Import::canonical_resource_path_string` omits the trailing
    separator FMI 3.0 requires**, so an FMU that appends its own file name
    builds `resourcesy.txt` and cannot open it. The Resource fixture exists to
    catch exactly this and did. A five-line fix is verified against a local
    patch, and the test stays ignored until a released version carries it.
  - **StateSpace's `x0` is inert**, so its initial state cannot be set through
    co-simulation at all. Its `setStartValues` copies `x = x0` and assigns
    `x = 0` on the next line, and runs only at instantiate and reset, before
    an importer can write a parameter. Setting `x` directly is refused outside
    Continuous Time Mode and Event Mode.

- **2026-08-15**: **The demo's FMU is the whole controller, and the laws it
  runs stay in `continuo-actors`.** The FMU crate holds the FMI interface
  declaration and nothing else, delegating every answer to `idm_accel`,
  `nearest_detection` and `pure_pursuit_yaw_rate`. A planner publishing an
  acceleration for a component to track was the alternative, and it lost on
  reading the signal flow out: its longitudinal half consumed a number and
  republished it unchanged, where IDM is the follow controller rather than a
  reference for one.
  - Only the steering law has a native caller today.
    `PathFollowController` calls `pure_pursuit_yaw_rate`, so those two
    agree by being one implementation rather than by being kept in step,
    which is the whole argument for this arrangement. Nothing native calls
    `idm_accel` or `nearest_detection`, because M6 gives traffic its
    longitudinal behavior through the FMU and leaves that controller purely
    lateral. So the argument is currently half demonstrated, and PLAN.md
    defers the component that would finish it.
  - The packaging is its own crate because FMI allows one model per shared
    library, the model identifier follows the cdylib's name, and
    `crate-type` cannot be feature-gated.
  - What that costs is a `.fmu` carrying a compiled snapshot of the laws,
    since the cdylib links them statically and nothing calls back into the
    host. Editing a law without packaging again leaves the copy behind, and
    the comparison in `crates/continuo-fmu-controller-idm/tests/` is the
    only thing that would notice. Its failures name
    `cargo xtask package-fmus` for that reason.

- **2026-08-15**: **The following law is the published equation with one
  departure, and its parameters are the published five.** A wanted gap does
  not go below zero. A lead pulling away contributes a negative closing
  term, and the equation as written lets that take the wanted gap negative,
  where squaring turns it back into braking: at 20 m/s with 20 m of gap and
  a lead pulling away to 30, it commands -1.28 m/s^2 where the answer wanted
  is +1.20. A guard on the output cannot fix that, since -1.28 is an
  ordinary number to command and only its sign is wrong.
  - The command is held inside `[-b, a]`, the two rates the equation already
    names, so `IdmParams` invents no number of its own. The plan had a
    `b_max` of 4.0 for emergency braking and a `GAP_FLOOR` guarding the
    division; both are gone, the first because the parameter set should be
    the published one and the second because flooring the wanted gap is what
    the departure above already does.
  - The consequence is that a car brakes no harder than comfortably, which
    is right for a law that is here to be representative rather than to
    drive well. It does mean the ego-lane work needs more spawn distance
    than the plan budgeted, which assumed braking at 4.0.
  - Two of the values are Treiber's own, the 1.5 s headway and the 2 m
    standstill gap his simulator lists for a car. The acceleration and
    comfortable braking there are picked to bring stop-and-go waves out of a
    crowded road, which is a different thing to demonstrate, so those two
    are this project's, and the plan overstated all of them as calibrated.
  - Cross-checked once against highway-env's `IDMVehicle`, which implements
    the unclamped form: `desired_gap` has no floor and `acceleration`
    applies no limit of its own. Fed our parameters, it agrees with this
    implementation to the last bit or two at every sampled point where the
    wanted gap stays positive, and parts from it exactly where the two
    adaptations say it should. At 20 m/s with 20 m of gap it wants -25.735 m
    of room from a lead pulling away to 30 and commands -1.280 m/s^2 where
    this commands +1.204, and -54.603 m and -9.977 m/s^2 from one reaching
    35. What was compared is the equation rather than the tuning, so it was
    fed ours throughout. Its own parameters differ and were not used:
    `DISTANCE_WANTED` is 10 m against our 2, since it folds a car length
    into the standstill gap, `COMFORT_ACC_MAX` is 3.0 against our 1.5, and
    `COMFORT_ACC_MIN` is -5.0 where we brake at 2.0.

- **2026-08-15**: **The road crosses into the FMU as fixed arrays with a
  count, not as a structurally sized one.** The plan preferred a structural
  parameter sizing the arrays, so a road would cross as exactly as many
  points as it has. `fmi-export` 0.3.0 cannot: a `[T; N]` field emits a
  fixed dimension and `#[variable(...)]` has no key naming a sizing
  variable, while a `Vec<T>` field hardcodes value reference 0, which is the
  derive's own `time`, and implements neither the get nor the set trait. So
  the road is `road_x` and `road_y` at `MAX_WAYPOINTS` with
  `road_point_count` beside them, and that count is an ordinary parameter
  rather than a structural one, which would claim a role it does not have
  and send an importer into configuration mode for nothing. None of this
  reaches `continuo-fmi`, which supports both forms because StateSpace
  requires it.

- **2026-08-15**: **The FMU refuses at run time what its model description
  cannot state.** `fmi-export` 0.3.0 has no `min` or `max` key, so the
  description carries no bounds and a host's own checker has nothing to
  check against. A doc comment protects whoever builds the parameters in
  Rust and nobody who loads the packaged `.fmu` in another tool, sets the
  target speed to zero and gets NaN in every command after. So the model
  checks the four parameters its laws divide by, a point count no road could
  have, and a road of no length, and reports what it was sent through FMI's
  own status and log rather than panicking, since an unwind through the C
  interface would take the host process with it.

- **2026-08-15**: **`cargo xtask package-fmus` packages every
  `continuo-fmu-*` crate, and `cargo-fmi` stays a binary rather than a
  dependency.** Discovery is by crate-name prefix through `cargo metadata`,
  so a second FMU crate is packaged by the task and by CI with no edit
  anywhere. An xtask rather than a `build.rs`, because packaging is a real
  entry point somebody types rather than a side effect of building, and
  rather than a cargo alias, because aliases cannot chain commands. It
  passes `--release`, since how a packaged FMU is optimized is settled when
  it is packaged and a host loads the binary it finds.
  - Each agent packages its own platform's binary, so CI's four artifacts
    are four FMUs nobody can hand to anyone else.
    `python/scripts/merge_fmus.py` combines any set that differs only in the
    binaries it carries, checking first that the model descriptions agree:
    two packagings of one commit differ in `generationDateAndTime` alone,
    and the instantiation token is a v5 UUID of the model name, so it is the
    same everywhere. It lives beside the viewer rather than in the xtask
    because merging needs several platforms' output and so has no use on one
    machine, and because the job that runs it then needs no Rust toolchain.
  - That job arrives with a manual trigger alone, taking the CI run to
    read artifacts from as an input. A workflow cannot be dispatched until
    it sits on the default branch, so a `workflow_run` trigger arriving
    with it would make the first run of it an automatic one that nothing
    had tried. That trigger follows once a dispatch has shown the
    cross-run download works.

- **2026-08-15**: **The packaged-FMU comparison sits behind a feature and
  runs inside the ordinary integration step.** Everything in it reads a file
  `cargo xtask package-fmus` writes, so `packaged-fmu` gates it and a plain
  `cargo test --workspace` needs nothing packaged. The gate is a `#![cfg]`
  on the file rather than `required-features` in the manifest, because CI
  runs its integration tests as `--test '*'`, and a glob names every target
  it matches, so cargo refuses the whole step over a named target whose
  features are off.
  - CI packages before it tests rather than after, which is what lets the
    comparison run as one of the integration targets. Running it afterwards
    means a second cargo invocation naming that one target, and **asking for
    a single target resolves fewer packages**: 142 units against 399 here,
    the whole Zenoh tree among the missing. Their dependencies then unify
    features differently, so the crate links a different `thiserror` and
    every crate between the two rebuilds, which cargo reports as `info of
    dependency thiserror changed`. That cost 12 to 31 seconds a run on every
    agent until the steps were reordered.
  - Worth knowing before optimizing anything else here: the same step
    measured 79, 70, 71, 37 and 92 seconds across five runs of materially
    identical work, so a single run says almost nothing.

- **2026-08-15**: **`proc-macro-error2` is patched to its fix rather than
  suppressed.** `fmi-export-derive` pulls it in, and it re-exports a private
  `extern crate proc_macro`, which rustc will make an error and warns about
  meanwhile on every build touching the FMU crate. The crate is archived, so
  2.0.1 is the last release there will be and there is nothing to upgrade
  to. `[patch.crates-io]` points at the two-line fix rustc's own help
  suggests, from the pull request that was open when the repository was
  archived, pinned by revision so what builds is the commit that was read.
  `[future-incompat-report] frequency = "never"` was tried and dropped: it
  is workspace wide, so it would hide every other dependency's warnings to
  quiet one, and the hard error would still be coming.
  - **The patch cannot reach `cargo install`, so CI still prints that
    warning when it installs `cargo-fmi`.** A patch is taken from the root
    manifest being built, and for that command the root is cargo-fmi's, so
    the build resolves its tree from the registry and takes
    `proc-macro-error2` unpatched. Putting the patch in
    `.cargo/config.toml`, which cargo does read from the working directory
    upward, was measured rather than assumed and does not work either,
    with or without `--locked`: the install compiles 2.0.1 from crates.io
    and does not object. The workspace's own builds take the patched
    revision, which their output shows by printing the source URL beside
    the crate name.
  - **When rustc makes it an error, that install stops working and CI
    stops with it.** Our own builds are fine, the patch being the fix
    rustc's help suggests, so what breaks is the tool rather than the
    workspace. It will not break everywhere at once: an agent holding a
    warm `cargo-fmi` cache skips the install and passes, so this arrives
    as some agents failing without a change, which is the shape of the
    bug `--force` was added for and is worth recognizing quickly.
    - Nothing upstream to move to when it happens. 0.3.0 is the latest
      cargo-fmi and `proc-macro-error2` is archived, so the routes are
      forking or vendoring cargo-fmi with the patch in its own manifest,
      or `fmi-export-derive` dropping the dependency.
    - Which is why the warning is left alone. Suppressing the report on
      that step is a one-line change and would work, and it would also
      remove the only notice that this is coming.

- **2026-08-16**: **Each situation in the packaged-FMU comparison is a run
  of its own, with a reset between them.** The sweeps drive situations
  picked to reach corners of the input space, and no car could drive from
  one to the next, so stepping them in sequence asked the model to account
  for a trajectory that never happened. That was sound only because these
  laws carry nothing across a step, which is a property of the
  implementation rather than anything the test established. `fmi3Reset`
  takes the assumption out, and it is what the standard offers for exactly
  this: a reset instance costs about a hundredth of a fresh one, since
  construction extracts the archive and loads the library where a reset
  does neither.
  - **A reset does not restore an FMU's size.** It returns an instance to
    Instantiated, which is before it was ever configured, and Configuration
    Mode closes before Initialization Mode opens, so the next step is
    already too late. The adapter therefore writes the mapping's structural
    parameters again as part of the reset. Without that a reset StateSpace
    would come back at the size its description declares while the bindings
    went on addressing the size the mapping asked for.
  - Each situation is stepped twice, because the first step after a reset
    is Initialization Mode and `fmi3DoStep` needs an interval behind it
    that a start time does not have. Resetting and stepping once instead
    would have taken `fmi3DoStep` out of the sweeps altogether while
    costing no measurable time, which is what a safety gain and a coverage
    loss look like from the outside.

- **2026-08-19**: **The plant integrates a commanded acceleration and owns
  speed, and the two axes travel as separate messages.** A controller
  publishing a speed left nothing between the command and the pose to
  refuse it: a car asked for 30 m/s was doing 30 m/s that instant.
  Acceleration is what a driver has, and integrating it puts a state in
  between. The clamp at zero is where commanded and actual visibly part,
  since a stopped car told to brake is doing nothing.
  - **Speed belongs to the plant, so the plant publishes it.** Nobody else
    can see it: a controller knows only what it asked for, and the clamp
    means that is not what happened. So `.../pose` carries `speed` beside
    `position` and `orientation`, flat, and every existing reader goes on
    reading a pose and ignores the field it did not ask for. The viewer
    included, which is why nothing in `python/` changed.
  - **Two commands rather than one**, on `.../accel_cmd` and
    `.../steer_cmd`, held independently. Following and steering are
    different laws and one component need not hold both, so a learned
    longitudinal model beside native steering wants a second publisher and
    no new shape. Both say *commanded* in the payload as well as the key,
    because commanded is not actual.
  - **Both are normalized to [-1, 1] and carry no unit.** A pedal and a
    steering wheel travel between stops, and how much car is behind them is
    the car's business, so `DriveLimits` lives on the plant and a command
    says only what fraction of one it wants. *(Renamed `PlantLimits`
    2026-08-28, since the type has to say whose limits it holds.)* A
    controller naming an acceleration would be asserting something about a
    vehicle it does not own, and two cars given one command would have to
    behave alike.
    - The cost is that the two sides must agree on the limits with no way
      to check. A number with no unit has nothing to disagree about, so a
      controller working from a different `yaw_rate_max` steers to a rate
      it did not intend and nothing anywhere fails. The scenario hands both
      halves of a car one `DriveLimits`, and that is the whole of why they
      match.
    - Braking gets a limit of its own, because a car brakes harder than
      it accelerates. One number for both would get one of them wrong.
  - **A plant holds its last command rather than clearing it.** Nothing in
    the demo commands an acceleration, so every car keeps the speed its
    plant was built with and needs no longitudinal publisher at all. A
    controller saying nothing about acceleration is not saying zero, which
    is why a held zero is the right start.
  - **The plant's state hash is the integrator state alone.** The held
    commands are copies of what reached the plant, and every published
    command is in the fingerprint already, so hashing them counted the same
    bytes twice.
    - A divergence in what *arrived* rather than in what was sent would
      then wait for the pose to show it. That is true of any component
      holding a decoded input, though, so guarding it here would mean
      guarding it in every component. This hook is for state a component
      makes and does not publish, which is what an integrator's is.
  - **What moved in the hash, and what did not.** `d747a81be039c5f1` to
    `eccd08f9a316bbbc`, for three things: the payload shapes and keys, the
    normalizing, and the state hash dropping those commands. No car moved
    for any of them. Every pose a car publishes driving the ellipse in
    `determinism.rs` folds into one pinned fingerprint, which holds
    throughout. The ellipse rather than the demo's straight road, because
    there the steering law works the whole way round and a difference in
    the integration would show; on a straight road every yaw rate is
    exactly zero and two quite different plants would agree. README's
    sample poses are unchanged for the same reason, so the diff shows one
    number moving in a block of numbers that did not.
    - **The world hash cannot make this claim**, which is why the
      fingerprint exists beside it. It is taken over payload bytes, and
      this change rewrote those on purpose, so it had to move whether or
      not a car went anywhere. Folding the decoded numbers is what
      separates the two questions.
    - Normalizing survives it as well, which was not a given: dividing by
      a limit and multiplying by it again is not an exact round trip in
      general, and here it happens to be at every step of the run.
  - **Actual acceleration stays unobservable**, deferred rather than
    dismissed. It differs from the commanded value wherever the clamp bites,
    and yaw rate would follow it. When something wants them the honest move
    is a message of their own rather than stretching `pose` a third time,
    since the key says `pose` and the viewer's `pose_from_payload` reads it
    as one. Nothing needs it yet.

- **2026-08-19**: **A plant's initial state is one deserializable struct
  rather than positional arguments.** `CarState { position, orientation,
  speed }`, and since the constructor was changing anyway it cost nothing.
  It names the integrator state in one place, a later model adds a field
  rather than a fifth argument, and when scenarios come from files it is
  what those files carry, with no signature to change. The FMU side already
  works this way, since initial values are name-keyed data checked against
  each variable's declared type, so both paths are converging on the shape
  scenario configuration needs: a registry instantiating component types
  from data cannot know any model's state layout.
  - **Named for its contents rather than for the model.** A `UnicycleState`
    would be renamed by the first plant that is not a unicycle, and renamed
    for nothing, since where a car is and how fast it is going is what any
    of them integrate.
  - **One struct for the constructor and for the wire**, because they
    carry the same thing: a position, an orientation and a speed. That is
    also what fixes its layout: the fields
    are flat and `position` and `orientation` sit where `Pose` puts them, so
    what the plant publishes is a pose with a speed after it rather than a
    new shape. A nested `pose` field would read better in Rust and be
    readable by nothing that reads poses today.
  - **No acceleration in it.** That is a held command rather than integrator
    state, replaced by the first message to arrive, and zero is right for a
    car nobody commands.
  - **The `Component` trait stays out of it**, deliberately. An
    initial-state hook there would have to know a shape that is per-model,
    and plumbing a generic parameter before scenario configuration says what
    it must carry would be guessing at it.

- **2026-08-20**: **Transcendentals go through `libm` rather than the
  platform's, so the world hash is portable by construction.** PLAN.md
  deferred this on the grounds that four CI agents agreed on
  `DEMO_WORLD_HASH`, so the exposure was real but not biting. Both halves
  of that turned out to be wrong in the same way, and the same measurement
  settles it.
  - **The demo hash was never testing this.** Its road is straight, so
    every yaw rate in it is exactly zero, and `sin`, `cos`, `atan2` and
    `sincos` are only ever evaluated where every implementation agrees
    anyway. Switching the whole workspace to `libm` moves that hash *not at
    all*, which is the proof: a check that cannot move when the arithmetic
    under it is replaced was not watching the arithmetic.
  - **A world that steers does not agree.** The ellipse in
    `continuo-actors`' determinism test fingerprinted three ways across the
    four agents: the two glibc agents agreed with each other across
    architectures, and the MSVC CRT and Apple's libm each differed. That is
    a libm signature rather than an architecture one, which is what
    `ubuntu-24.04-arm` is in the matrix for.
  - **So the fix ships with the check that would fail without it.**
    `a_curved_world_traces_the_same_path_on_every_platform` pins that
    ellipse trajectory, and all four agents produce the one value. It is
    the first check here that exercises a transcendental at an argument
    implementations may round differently, and a straight-road hash cannot
    replace it.
  - **`disallowed-methods` keeps the inherent methods out**, one entry per
    function, in the same file and for the same reason `HashMap` is banned:
    no compile error would report `x.sin()`, and the world hash would
    report it only from whichever agent disagreed, long after and far from
    the line. `sqrt`, `powi`, `rem_euclid` and the rounding family stay,
    being exact.
  - **It cost no hash move at all**, which was not the expectation going
    in: the deferred item budgeted one. Recorded logs stay valid, and the
    two pinned values on main are untouched.

- **2026-08-20**: **`cargo xtask verify` and `cargo xtask verify-fmus` say
  how many tests ran, and a step that ran none fails.** A table of elapsed
  times says a command ran, never that it found anything to do, so a filter
  resolving no targets reads exactly like a full pass. `verify-fmus` raised
  it, its output ending in "running 0 tests" from doc-test targets with no
  examples, and nothing in the summary telling that apart from the
  packaged-FMU tests having been skipped. Zero being a failure rather than
  a small number is what `verify-fmus` already says about validating no
  FMUs.
  - **Counted by reading the output**, since `cargo test` offers the number
    no other way on stable: `--format json`, `--format junit` and
    `--report-time` are all nightly-only, `--list` says what would run
    rather than what did, and `--message-format json` reports build
    artifacts. `cargo-nextest` does write JUnit XML, and runs no doc-tests.
  - **One rule reads both tools**, the last run of digits before the word
    `passed`, which cargo writes once per test binary and pytest once per
    run. Reading digits rather than the last word is what makes color
    irrelevant, and that is the whole reason there is one rule rather than a
    parser per tool: an earlier attempt asked each tool to keep coloring
    through the pipe and needed to see through escape sequences to do it.
  - **A format that changes reads as zero**, and zero already fails naming
    the step, so the worst case is a loud failure rather than a wrong number
    reported as fact.
  - **CI's split had to learn `--bins`.** `--lib` and `--test '*'` named
    every target that existed until `xtask` grew unit tests, and a binary's
    are in neither, so they would have run nowhere. The rule those two
    flags have to follow is now written beside them: together they name
    every target that runs.

- **2026-08-23**: **A membership change takes effect where it says it
  does, in the world as well as in the log, and when its request was
  processed is an observation.** Renaming the ambiguous "applied" turned
  this up: the word covered both the conductor taking a request in and the
  change taking effect, and the two halves disagreed about which they
  meant. A leave was announced at the boundary where it takes effect, so
  it sat where the scenario put it. A join was announced at the request,
  so it sat wherever the caller happened to be, in a stream verification
  compares line by line. The request moves to a `RecordedObservation`,
  which verification skips, and leaves get one too. "Processed" over
  "received", which says only that a request arrived.
  - **The same fault ran deeper, and that half reached the run.**
    Registering a component when its request was processed subscribed it
    then, so everything published before `first_due` reached its first
    inbox: a talker publishing every 10 ms hands a listener declaring
    25 ms two messages when asked for at 0 ms and none when asked for at
    20 ms. Its declaration index and tree position went the same way, and
    those are the execution order and the visibility rule's earlier
    sibling. So the whole join waits now, and `admit` is the one place one
    takes effect.
  - **The registry's checks wait too, which is more correct rather than
    less.** A path is free or taken only at the instant the newcomer would
    occupy it, so checking at the request would refuse a path that a leave
    frees in between, and allow one that another join takes. What it costs
    is where a bad path is reported, and only for a join declared ahead: a
    world built before it runs declares sim time zero, which is the
    earliest instant still open, so it is admitted inside `add_component`
    and hands the caller its own error as before. A join declared for a
    later instant reports from `step_once` at the boundary instead, which
    is the only place an answer exists.
  - **A leave retires what is registered at the path, and a waiting join
    only when nothing is.** Say `car1/physics` is running, its replacement
    is declared for 30 ms, and its own leave for 30 ms as well. The
    boundary before 30 ms settles leaves first, so the incumbent retires
    and the newcomer is admitted into the path it freed: a `Leave` and a
    `Join`, in that order, at one instant. Answering that leave by
    withdrawing the newcomer instead would keep the incumbent running and
    lose its replacement. Where nothing is registered, the leave does
    withdraw the waiting join, and that join is recorded as neither a
    `Join` nor a `Leave`, since nothing ever saw it arrive.
  - **`DEMO_WORLD_HASH` does not move**, and not because the demo avoids
    the case: its spawner declares a period ahead, so every traffic car
    waits for its instant. Nothing accumulates in the gap because a car
    subscribes only to keys under its own actor name, and the siblings
    publishing them do not exist until it is admitted.

- **2026-08-28**: **An FMU controller commands a normalized fraction of what
  its plant can do, and the FMU is told the plant's limits.** Commands
  became normalized when the plant took over acceleration, and the exported
  FMU controller went on publishing an acceleration in m/s^2 and a yaw rate
  in rad/s. Nothing failed, because no car runs on the FMU yet and the
  comparison against the laws ran both sides through the same unconverted
  numbers: the two agreed with each other while both disagreed with the
  plant.
  - **Converting a rate into a normalized command is a control law**, so
    `accel_fraction` and `steer_fraction` sit beside the laws whose answers
    they convert and both controllers call them, rather than the division
    being written afresh at each publisher.
  - **A control law's limits are separate from the plant's.** IDM
    accelerates at 1.5 m/s^2 and brakes at 2.0 where the plant does 3.0 and
    5.0, since a law's pair says what a driver finds comfortable rather than
    what the car allows. So the FMU gains `plant_accel_max`,
    `plant_decel_max` and `plant_yaw_rate_max` and divides by those, and
    they are fixed rather than tunable: a car does not become a different
    car between steps, and driving a different one is a different instance.
    `DriveLimits` is renamed `PlantLimits` to say whose limits they are.
  - **Steering keeps a clamp of its own beside them.** `max_yaw_rate`
    defaults to the plant's rate limit, so a car steers as hard as it can,
    but it could be tuned differently. The native controller takes its
    car's `PlantLimits` for the same reason. The clamp still has to be
    positive, now because `clamp` panics when the low bound is above the
    high one and a panic through the C interface would take the host process
    with it.
  - **`yaw_rate_cmd` becomes `steer_cmd`**, which settles the contract as
    well as the unit. An FMU's variable name is the payload path its output
    publishes at, so under the old name a plant would have been handed
    `{"yaw_rate_cmd": ...}` and refused to decode it.
  - **The check is a round trip through the plant**, since a controller
    dividing by the limits and a plant multiplying by them is the one place
    the two meet: the plant's tests hand it what a law asked for and read
    the rate back off it. What they cannot check is that a scenario handed
    both halves of a car one `PlantLimits`, which is the cost the 2026-08-19
    entry already records.
  - `DEMO_WORLD_HASH` does not move. Every default is the number it was, and
    nothing in the demo is an FMU yet.

- **2026-08-29**: **`Waypoints::frenet` returns an arc length along the
  road and a lateral offset across it, both measured to the nearest road,
  and both carrying on past the end of an open one.** A point whose
  projection lands past a segment's end means one of two things:

  1. Off either end of an open path, the road ran out so the arc length
     carries on past the end or below zero before the start, and the offset
     goes on measuring across the extension line the road was holding.
  2. Everywhere else, the road turned and on a closed path that is wherever
     the projection gets clamped to a segment's vertex: the point sits in
     the wedge outside the bend that neither segment's perpendicular
     reaches, and the vertex between them is the nearest road, so the offset
     becomes the direct distance to it instead of a perpendicular.

  Case 1 is where the arc length calculation method needed improvement:

  - Stopping it at an open road's end puts every car that has driven past
    that end on one value, so none is ahead of another and a radar sees none
    of them.
  - It still jumps when inside a corner point, where the segment
    perpendiculars overlap and the nearest point crosses between the
    segments at the bisecting line before reaching the end of the first
    segment. That is inherent to following the nearest segment on a raw
    polyline with no curvature, so a test pins the expected magnitude of
    this type of arc length discontinuity.
  - Using the straight line between two cars instead of the arc length
    projection was considered but rejected: a chord understates the road a
    follower has to cover and swings as the pair rounds a bend so it does
    not improve the following distance accuracy. highway-env and SUMO both
    measure arc length along lanes and avoid this in other ways, highway-env
    by fitting a spline before projecting onto a polyline lane and SUMO by
    never projecting at all, carrying a vehicle's lane position as state.
    For now, `RadarSensor` documents the impact of arc length deviation from
    inside corner points on its range values.
  - `project_arc_length` is the same call's arc length half, for a caller
    with no use for the offset, so the two cannot disagree about which
    segment won.

  Case 2 is where the offset calculation method needed improvement. The
  initial implementation measured the perpendicular to that segment's
  extension line, but was replaced by the direct distance to the vertex for
  the reason below:

  ```
                    |
                    | the road, turning north at B
                    |
                    |           R2
        ------------B- - - - - -+- - - - -    the extension line
                     . \ . . .  ^  . . . . . .
                     . .  \  .  |  . . . . . .  the wedge: past the end
                     . . . . \  |  . . . . . .  of both segments, so the
                     . . . . .  R1 . . . . . .  vertex is the nearest
                     . . . . . . . . . . . . .  road there is

          \   to the vertex, direct distance to the road    (current)
          |   to the extension line                         (initial)
  ```

  For example, take a car from R1 up to R2. Measured to the extension line,
  the offset distance reduces to zero exactly where the car crosses that
  line, then steps straight up to the offset it should have had all along. A
  lane band watching that sees a car drift onto the centerline and jump back
  out again. Measured as direct distance to the vertex B, the offset value
  stays continuous around the corner and matches the distance to the road
  the whole way.

  Which side of the road that distance falls on is a second question, and in
  the wedge the point cannot answer it:

  - An extension line says nothing about which side of the road a point is
    on, so only a segment the point projects inside has a perpendicular
    worth reading. That is why an exact tie goes to a segment holding the
    projection between its own ends and only ties after that go to the
    earliest.
  - In the wedge outside of a corner neither segment holds it, so the side
    belongs to the corner rather than to the point: the outside of a left
    turn is the right. One cross product of the two segment directions gives
    it, with the point playing no part.

  Either way the offset distance itself is `cross / len`. It is written as a
  cross product but computes a dot product: the same one that gives the arc
  length, taken against the normal rather than along the segment, the normal
  being the segment turned a quarter circle. `sqrt(dist^2 - proj^2)` gives
  the same number but loses more digits as the square of the along-to-across
  ratio, where this loses them in proportion to it.

  `Waypoints::point_at_offset` runs the other way, from an arc length and
  offset back to a point, and is left alone. A lane round a corner is not
  the same length as the road it follows, so measuring both by the road's
  arc length breaks down at a vertex and the two are not inverses there.
  Fixing that belongs to the road's geometry rather than either call, and
  PLAN.md's deferred list has it.

  `DEMO_WORLD_HASH` and the ellipse's `CAR1_TRAJECTORY` do not move.

- **2026-08-29**: **A detection is a measurement rather than a tracked
  object, so the radar reports what it found and chooses nothing.** A
  scan carries a range and a closing rate per car ahead, in no order the
  type promises, and a slot means nothing from one scan to the next.
  Identity would have to come from a tracker, which does not exist here,
  and no consumer wants one anyway: `nearest_detection` takes the
  minimum, and a learned follower will encode the set so that its order
  cannot matter. Determinism comes from reading the inbox in its own
  order rather than from sorting on a float, so there is no tiebreak to
  get wrong.
  - **Except at the cap, where the farthest go.** Which detections a
    full scan drops still has to be decided, and dropping whichever were
    found last would sometimes lose the car being followed. So there is
    one sort, on range and by `total_cmp`, and it decides membership
    alone: what survives keeps the order it was found in, so nothing
    downstream can start reading a scan as though it were sorted. No
    road here fills a scan, which makes this a bound rather than a
    working limit.
  - **A scan is a `Vec<Detection>` on the type core already owns**,
    where the plan had a `RadarDetection` of its own. The exported FMU
    controller has read `Detection` since it was written, so a second
    type would have been two names for the same two numbers.

- **2026-08-29**: **The radar keeps nothing between steps, and the inbox
  window is the whole of its freshness rule.** Each scan is built from
  the poses delivered for that step and nothing else. So `state_bytes`
  is `None`, no age horizon has to be given a value nothing argues for,
  and a car that leaves needs no cleaning up after, because nothing was
  kept. What that costs is a bound rather than a guarantee: a departed
  car ghosts for at most one scan, its last pose still being in the
  window read after it left. It also requires anything watched to
  publish at least once per radar period, or it blinks.
  - **Range and range rate are read off ground truth**, the arc length
    between two projected positions less one `CAR_LENGTH`, and the other
    car's published speed less this one's. Reading the rate rather than
    differencing two scans is what makes one sample per car enough, so a
    car joining mid-run is in the very next scan. A real sensor model
    replaces the arithmetic and keeps the interface: relative
    measurements only, since a radar knows nothing about the car it is
    bolted to.
  - **That one subtraction stands in for two things**, and a car length
    is neither of them. A radar sits somewhere on its own car rather
    than at its origin, and it measures to where the line between them
    meets the other car's body rather than to that car's origin. Both
    wait on the simulation publishing extents, which is why `CAR_LENGTH`
    is a constant here and an invented rectangle in the viewer. A range
    below zero is reported as it stands, since a follower told the road
    ahead was clear would drive further into what it has already hit.
  - **`DEMO_WORLD_HASH` does not move**, nothing in the demo carrying a
    radar yet. The determinism test's cars do carry one, so that two
    runs have scans to compare and so a loop's wrap around its own seam
    is exercised under the conductor rather than only in a unit test.
    `CAR1_TRAJECTORY` not moving is the proof they changed no car:
    nothing reads a scan, so a radar publishing beside a car cannot
    steer it.


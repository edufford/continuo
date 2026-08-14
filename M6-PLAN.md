# M6: FMI 3.0 CS import, a car-controller FMU with IDM inside, collision monitoring

**Working scaffolding, deleted when the milestone lands.** This is not a
third permanent document beside [PLAN.md](PLAN.md) and
[DECISIONS.md](DECISIONS.md). PLAN.md holds the design as it stands and
DECISIONS.md holds why, dated; everything here is raw material for one or
the other. It is checked in so that eight PRs of choreography, and the
reasoning behind the first one's shape, do not live only in one session,
and so the upstream experiments have somewhere to write their findings.

Treat it as a snapshot, not a living document: no churn per PR. Where
reality diverges from it, the divergence goes in that PR's DECISIONS
entry, which is where it belongs permanently, and the final PR
reconciles the lot into PLAN.md and deletes this file. The one part worth
updating in place is anything the upstream checks settle, since a
resolved behavior is exactly what the next PR needs to read.

## Context

M6 is the milestone where an FMU becomes a component. PLAN.md's "FMI 3.0 CS
support" section committed to a `continuo-fmi` crate with an `FmuComponent`
adapter and named a Modelica reference FMU as the demo target. The demo
target changes: the FMU is a **complete car controller**, a drop-in
replacement for `PathFollowController`, whose internal control laws are IDM
(the Intelligent Driver Model) for following and pure pursuit for steering.
A new radar-like sensor feeds it. Today faster traffic silently drives
through slower traffic in the shared side lanes, and the ego's lane is kept
empty because "nothing here models a collision". M6 ends with traffic in the
ego's lane, the ego following on the same controller, and a collision
monitor that warns for traffic-traffic overlap and halts the scenario for
ego overlap.

The interfaces are shaped for what comes after: the radar publishes multiple
detections ahead, and the controller FMU is a stateless observation-to-action
map, because the IDM law will later be replaced by a learned model
(behavioral cloning of IDM, then RL) that consumes several cars ahead to
smooth out pack velocities. That lands as a sibling shell crate,
`continuo-fmu-controller-ai`, stepping the ML model internally through ONNX
Runtime: the FMU boundary hides the inference stack, and because the radar's
whole scan already enters the FMU as fixed-size arrays, that sibling declares
the same inputs and changes nothing outside itself. One
flag recorded now for then: ONNX Runtime does not promise cross-platform
bit-identity the way rustc-compiled math does, so the four-agent world-hash
check will force a choice at that milestone (per-platform pins, a relaxed
cross-platform claim for AI worlds, or owned inference).

## Signal flow

| Stage (rate) | Consumes | Computes | Publishes |
|---|---|---|---|
| RadarSensor (100 ms) | every car's `.../pose` | own (s, lane, v); ahead-in-lane filter; range = delta s minus CAR_LENGTH; range rate = detected car's published speed minus own | `.../radar` `{detections: [{range, range_rate}, ...]}`, order unspecified |
| Controller FMU (100 ms) | own `.../pose` (x, y, quat, speed) + own `.../radar` | picks the nearest detection as its lead; a = IDM(speed, gap, approach rate); yaw rate = pure pursuit(pose, road) | `.../accel_cmd` `{accel_cmd}`, `.../steer_cmd` `{yaw_rate_cmd}` |
| UnicyclePhysics (10 ms) | `.../accel_cmd`, `.../steer_cmd` (held independently) | v = (v + a*dt).max(0); midpoint-heading unicycle | `.../pose` `{position, orientation, speed}` |

Composites, declaration order = same-instant delivery order:
`[radar, controller, physics]` for an FMU car (the FMU registers under the
id `controller`, so its path reads like the native car's);
`[controller, physics]` for a constant-speed native car, where nobody
publishes an acceleration command and the speed is physics's initial
state (silence-as-hold is the system's grammar). Every stage computes a
real transformation; nothing relays.

What a composite declares first is the scenario's choice, and it decides
whether a command lands same-instant or at the next step. The rule
itself belongs to the conductor and its semantics tests already pin it,
so nothing in M6 re-tests it per component.

## Decisions settled in review (2026-08-10)

1. **Import goes through the `fmi` crate** (0.8.0, from the rust-fmi
   project). PLAN.md's candidate, `fmi-rs`, is a real and separate
   project (CATIA-Systems, who also maintain FMPy, BSD-2-Clause), so the
   two were compared rather than conflated. It is more substantial than
   its 2 stars suggest, carrying `fmi2` and `fmi3` modules, its own
   model-description and schema handling, and optional Sundials for
   Model Exchange. Two things decide it against us anyway. It is
   **unreleased**: version 0.1.0, 48 commits, nothing published to
   crates.io, no usable documentation, and no readable statement of
   which interface types it covers, so this project would be pinning a
   git revision of a moving target. And it needs a **C compiler
   unconditionally**, since its build.rs compiles a logger proxy and a
   Bison/Flex variable-name validator on every build, so the
   native-toolchain cost it appears to avoid by skipping bindgen is not
   actually avoided. rust-fmi is released, documented, and heavily
   downloaded through 0.8.0, and its `fmi-export` and `cargo-fmi`
   siblings are what PR B needs to author and package the controller
   FMU. Worth revisiting when fmi-rs releases: the provenance is strong,
   the hand-written bindings are appealing beside bindgen, and the
   adapter is the only crate that would change. The third option,
   `fmu-runner`, is FMI 2.0 only and excluded by the 3.0-only decision.
2. **The FMU is the whole controller, not a planner beside one.** Writing
   out the signal flow showed a mediating ECU's longitudinal half consuming
   an acceleration and republishing it unchanged; IDM is not a reference for
   a controller to track, it is the follow-controller. Folding in pure
   pursuit makes the FMU a one-for-one replacement for
   `PathFollowController`, which is a stronger demonstration of "FMUs as
   components" than adding a new role beside the existing ones. Actuation
   limits, when they come, belong to the plant.
3. **The control laws live in `continuo-actors`; the FMU packaging is its
   own crate, `continuo-fmu-controller-idm`.** Actors gains
   `idm_accel(params, speed, gap, approach_rate)` and a
   `pure_pursuit_yaw_rate(road, pose, params)` extracted from
   `PathFollowController::step`, so the native component and the FMU call
   the same functions and bit-identity is true by construction. The shell
   crate holds only the FMI interface declaration (`#[derive(FmuModel)]`
   struct), `impl UserModel` delegating to the actors functions,
   `export_fmu!`, and `packaged_fmu_path()`. A separate crate because FMI
   allows one model per dylib, the modelIdentifier follows the cdylib name
   (`continuo_fmu_controller_idm.fmu`), crate-type cannot be feature-gated,
   and `cargo fmi bundle` builds with default features. `continuo-fmi`
   stays purely generic.
4. **The road enters the FMU as waypoint arrays**, not as scalars for a
   two-point line: `road_x` and `road_y` alongside a boolean
   `road_closed`, reconstructed inside the FMU with the same `Waypoints`
   code the native controller uses. Any polyline works from the first
   commit, so the FMU is not quietly straight-road-only, and the deferred
   road-network importer finds the interface already waiting. These are
   parameters, set once during initialization, so a long road costs
   nothing per step.
   - **Prefer a structural parameter for the road's length**, if
     `fmi-export` can declare one: the arrays are then exactly as long
     as the road, and the parameter that sizes them is the point count,
     so no separate count variable exists to disagree with them. That is
     simpler than the fixed form rather than fancier, and it lifts a
     ceiling a genuinely unbounded road will eventually hit. The
     fallback, if the derive cannot, is `road_x[MAX_WAYPOINTS]` with an
     integer `road_point_count` beside it; PR B checks this alongside
     the fixed-dimension question. **It cannot**, and the PR B entry
     below records what was tried.
   - The detection arrays stay fixed at `MAX_DETECTIONS` either way.
     There is no count variable to eliminate, since unused slots take
     the free-road defaults, so a structural parameter would only add a
     knob; 64 is a design bound rather than a property of the world; and
     a learned successor's input vector is fixed by its architecture, so
     it would declare a fixed dimension regardless. The one thing it
     would buy, decoupling the FMU binary from the constant and so
     retiring the cross-crate compile-time assert, is worth revisiting
     only if the road work makes structural parameters free to reach
     for.
   - None of this affects the importer, which supports both forms
     because StateSpace requires it. The capability is exercised whether
     or not our own FMU uses it.
   - A `Waypoints` is three things, and all three must cross: the points,
     the derived arc-length table, and `is_closed`. The table is
     recomputed from the points by the same code, so it need not travel,
     but `is_closed` changes what projection means and would otherwise
     turn any loop into an open polyline in silence. The demo's road is
     open, so nothing would fail today, which is exactly why it is worth
     carrying now rather than after `ellipse()` meets an FMU controller.
   - Those two fields are the first honest users of the adapter's
     non-float support: a count is not a float, a flag is not a number,
     and padding the tail by repeating the last point instead would hand
     `Waypoints::project` a zero-length segment. Boolean joins the
     supported set for this, which the type dispatch already anticipated.
   - Each FMU builds its own copy, which is the FMI boundary rather than
     a choice: a black box cannot hold a reference into the host's
     memory. At 64 points that is a kilobyte or two per instance against
     one shared `Arc` for every native consumer, and it is one of the
     honest costs of putting a controller behind the standard.
5. **`Component` no longer requires `Send`.** The bound was an
   undeliberated M1 default: "Send" appears in no design document, and
   nothing in-tree moves a component or a conductor across threads. The
   plausible parallel futures do not need it either: local parallelism is
   the M7 host protocol run over channels (components constructed on the
   thread that runs them, step request and reply as plain data, barrier
   plus declaration-order fold keeps the hash byte-identical), supervised
   timeouts are a timed wait on the reply, and migration is membership
   (despawn here, respawn from data there). What must be Send in that world
   is messages and constructors, never components. Dropping the bound gives
   the FMU instance its truthful type (`Instance<CS>` is `!Send` upstream)
   and keeps the workspace 100% safe Rust: no `unsafe` anywhere through M6.
   Revisit trigger, recorded in DECISIONS: if a build-here-run-there
   hand-off is ever wanted, the bound returns as a breaking change and the
   FMU wrapper carries an `unsafe impl Send` whose claim is
   thread-agnosticism only (calls are serialized by `&mut` regardless).
6. **Longitudinal and lateral stay separate command messages** even though
   one controller currently publishes both: two small messages keep the
   peer-composition door open (a learned longitudinal FMU beside native
   steering is a plausible future), and physics holding each channel
   independently is the general shape. The mapping routes each output
   variable to its own key.
7. **The sensor reads ground truth, as a starting point.** Physics
   publishes its speed, and the radar derives range and range rate from
   true poses. M6 is proving FMU integration, not sensing, so the sensor
   model is the simplest thing that produces the right quantities. A
   realistic model replaces the derivation later (noise, field of view,
   occlusion, estimation from returns) without touching the interface,
   which is the part shaped to last: relative measurements only, since a
   radar does not know its own car's speed. Tests pin reported values,
   not how they are derived.
8. **The controller FMU is authored in Rust** (`fmi-export` derive,
   cdylib): one compiler on all four CI agents keeps the pinned world
   hash portable. No reference IDM implementation exists to import
   (Treiber's own simulators are GPL JS/Java, SUMO's is embedded
   C++/EPL, the only Rust crate is 0.0.1); the published equation is the
   reference, pinned by hand-derived spot values cross-checked once
   against MIT-licensed highway-env.
9. **Zip `.fmu` support now** (it comes with `fmi`); the importer is proven
   against four vendored Modelica reference FMUs before the controller FMU
   exists.
10. **The collision monitor lands after traffic-on-FMU, before the
    ego-lane PR**, so the net is in place before the ego first shares a
    lane.

## Verified upstream facts

- `fmi` 0.8.0 (MIT OR Apache-2.0). Use `default-features = false,
  features = ["fmi3"]` (default also pulls fmi2 and ls-bus). Direct deps:
  fmi-schema, fmi-sys, itertools, libloading, log, paste, tempfile,
  thiserror, zip.
- **fmi-sys runs bindgen at build time** (bindgen 0.72 + cc): libclang is a
  build prerequisite. All four GitHub runner images ship LLVM; a fresh dev
  box needs it (Windows: `winget install LLVM`). First native-code
  dependency of the workspace. Note that the alternative would not have
  spared this: `fmi-rs` compiles C on every build too, just with `cc`
  rather than bindgen.
- Import API: `fmi::import::from_path(".fmu") -> Fmi3Import` (extracts the
  zip to a tempdir), `instantiate_cs(...) -> Instance<CS>` (owned). Common
  trait: `enter_initialization_mode(tolerance, start_time, stop_time)`,
  `exit_initialization_mode()`, `terminate()`, `reset()`; GetSet by `u32`
  value references; CoSimulation trait: `do_step(...)`. `Instance<Tag>` is
  `!Send` and `!Sync` upstream, which decision 5 makes a non-issue.
- `fmi-export` (0.3.0 on crates.io, from the same workspace):
  `#[derive(FmuModel)]` + `#[variable(...)]` + `export_fmu!(Model)`
  generates the full FMI 3.0 C API **including Co-Simulation**;
  `#[model(co_simulation = true)]` must be set explicitly. `UserModel`
  methods all have defaults; a stateless map implements only
  `calculate_values` (outputs recompute lazily when read after `do_step`
  marks them dirty).
  - **It does pull fmi-sys and so bindgen**, through a plain `fmi`
    dependency, so authoring an FMU costs libclang exactly as importing
    one does. Only the `fmi3` feature is enabled, so the workspace's
    feature trimming survives.
  - **The exporting crate needs `fmi` as a direct dependency of its
    own**, because the generated code writes `::fmi::...` paths. Without
    it the derive fails with "cannot find `fmi` in the crate root" and a
    cascade of missing-trait errors behind it.
  - **A field carrying no `#[variable]` is ignored**, which is what lets
    the FMU hold ordinary Rust state beside its interface.
  - **A dotted `name` override passes through verbatim**, so
    `name = "position.x"` declares the structured name the pose inputs
    want and no JSON Pointer has to be written for them.
  - **Int64 and UInt64 cannot be declared.** The builder covers f32, f64,
    the 8, 16 and 32 bit integers, `bool`, `String`, `Binary` and
    `Clock`. It is an export-side gap only, since the importer binds all
    ten numeric types, and nothing here wants a 64 bit integer.
  - **Only the first line of a doc comment becomes the FMI
    description**, so a first line has to be a whole sentence and a
    field wanting more says the rest on the lines below.
  - **An output needs `initial = Calculated` spelled out.** Left off,
    the derive lists no initial unknowns and the output keeps whatever
    start value it declares, which FMI forbids of a calculated variable.
    fmpy refuses to load the result, which is how this was found.
  - **`variableNamingConvention` cannot be set** and comes out `flat`.
    Dotted names are still legal and fmpy accepts them, so what is lost
    is the model description saying out loud that `position.x` is
    structured. Nothing here depends on it, since the mapping derives
    its pointers by this project's own rule.
  - **`canSerializeFMUstate` is false**, so the adapter runs this FMU in
    output-hash mode. Nothing in `fmi-export` implements state
    serialization.
- Packaging: `cargo install cargo-fmi`, then `cargo fmi --package <pkg>
  bundle` (wrapped by `cargo xtask package-fmus`) builds the cdylib
  itself, extracts
  variable metadata from the built dylib, generates modelDescription.xml,
  and writes `target/fmu/continuo_fmu_controller_idm.fmu`. cargo-fmi as a
  library was examined and rejected (its public API is a clap CLI
  entrypoint that starts its own logger); it stays a CI-installed binary,
  never in the lockfile.
- Modelica Reference-FMUs v0.0.40 (BSD-2-Clause): FMI 3.0 `.fmu`s contain
  binaries for x86_64-windows, x86_64-linux, aarch64-linux, x86_64-darwin,
  aarch64-darwin, so all four CI agents are covered. `fmi-test-data`
  downloads
  over HTTP at runtime, so fixtures are vendored in-repo instead.

## Cross-cutting decisions

- **Step-failure error**: `CoreError` gains a reason-string variant (core
  cannot depend on continuo-fmi, and the failure originates outside the
  workspace where no error type of ours exists):

  ```rust
  /// A component whose own machinery failed: the FMU case, where a foreign
  /// model refused to set a value or take a step.
  #[error("component failure: {reason}")]
  ComponentFailure { reason: String },
  ```

  The adapter writes the reason naming the instance and the call;
  `ConductorError::StepFailed` already adds path and sim time.
- **Physics owns speed.** `UnicyclePhysics` integrates
  `v = (v + a*dt).max(0.0)` before the unicycle update. It subscribes
  `.../accel_cmd` and `.../steer_cmd`, holds each independently starting
  at 0, and adds `v` and both held commands to its `state_bytes`.
  - **Commanded is not actual**, because the zero clamp parts them: a
    stopped car commanded to brake is doing nothing. The wire says
    `accel_cmd` wherever a command travels.
  - **Initial state arrives as one deserializable struct**, not as
    positional arguments: `UnicyclePhysics::new(actor_name, period,
    UnicycleState { pose, speed })`, deriving `Serialize` and
    `Deserialize`. Since the constructor is changing anyway, this is the
    moment it costs nothing. It names the model's integrator state in
    one place, the same set `state_bytes` hashes; a later model adds a
    field rather than a fifth argument; and when scenario files land,
    that struct is what they deserialize, with no signature to change.
    The FMU side already works this way, since `initial_values` is
    name-keyed data checked against each variable's declared type, so
    the two paths are converging on the shape PLAN.md's scenario
    configuration needs, where a registry instantiates component types
    from data and cannot know any model's state layout.
  - No initial acceleration in that struct, because it is a held
    command rather than integrator state: the first message to arrive
    replaces it, and 0 is right for a `ConstantSpeed` car, where nobody
    publishes one.
  - Out of scope for M6, deliberately: an initial-state hook on the
    `Component` trait, since the shape is per-model and the trait has no
    business knowing it, and any generic parameter plumbing before
    scenario configuration defines what it must carry.
- **Physics publishes `{position, orientation, speed}` flat.** The rule
  is publish what you alone know: nothing commands speed, so this is the
  only way to see it, while `accel_cmd` is already on its own key.
  Existing `Pose` decoders ignore the extra field, the Python viewer
  included.
  - **Actual acceleration stays unobservable in M6**, deferred rather
    than dismissed. The plant should eventually publish its whole
    kinematic state, and that message wants a new name rather than a
    third field stretching `pose`. PR H records it. Nothing needs it yet,
    since `range_rate` comes from published speed, so a braking car reads
    truthfully whatever it was told to do.
- **`Cmd` retires** in favor of `AccelCmd { accel_cmd }` and
  `SteerCmd { yaw_rate_cmd }`: the payload field names match the FMU's
  output variable names, which is what the structured-naming rule
  requires, and they say commanded out loud.
- **`PathFollowController` becomes purely lateral** and stays: the
  constant-speed controller for the scale world and tests, and the lateral
  reference implementation whose extracted `pure_pursuit_yaw_rate` the FMU
  shares. Its `speed` constructor argument leaves (to physics); it
  publishes `SteerCmd` on `.../steer_cmd`; `state_bytes: None` as today.
  The name stays too. Path following is the lateral problem in the
  control literature, so it is accurate, and renaming it would be churn
  for a distinction the type's one output already makes.
  - **Both controllers answer `"controller"` from `id()`**, on purpose,
    so an FMU car's paths read like a native car's. That holds while a
    car has one or the other, and collides the moment they compose: a
    learned longitudinal FMU running beside native steering, which is
    the future the split commands exist for. Ids would become `steering`
    and `longitudinal` then. Not now, because changing one moves the
    world hash and buys nothing yet, but this is the thing that forces
    it.
- **The controller FMU's interface**, all Float64 except where noted:
  - Scalar inputs from the pose message, named `position.x`,
    `position.y`, the quaternion components and `speed`, with yaw
    recovered inside using core's own math.
  - Two array inputs from the radar scan, `range` and `range_rate`,
    fixed at `MAX_DETECTIONS`.
  - Two array parameters for the road, `road_x` and `road_y`, sized per
    decision 4, with a boolean `road_closed` and, in the fallback form
    only, an integer `road_point_count`.
  - Scalar parameters: lane offset, lookahead, gain, max_yaw_rate, and
    the IDM set (v0, t_headway, s0, a_max, b_comfort, b_max).
  - Outputs `accel_cmd` and `yaw_rate_cmd`, each routed to its own key.
  - **The pose inputs carry no JSON Pointers**, because FMI's structured
    naming and JSON Pointer line up: `position.x` derives `/position/x`
    on its own. modelDescription declares
    `variableNamingConvention="structured"` to say so to other tools.
    Whether `fmi-export` accepts a dotted name, by attribute override or
    by flattening a nested field, is a PR B check rather than a risk: if
    it does not, the variable is `position_x` and the mapping supplies
    an explicit pointer, which costs a line.
  - **The radar arrays do carry JSON Pointers**, and always would:
    nothing derives `/detections/0/range` from a variable named `range`.
    That is the case explicit pointers exist for, and it is the same
    reason a third-party FMU needs them, since it names things in its
    own vocabulary rather than after the schemas it happens to consume.
    The derivation is a convenience for an FMU authored beside its host,
    never an assumption in the adapter. The road arrays need no pointers
    at all, arriving as initial values rather than from a message.
- **The whole scan enters the FMU, and choosing from it is the
  controller's job.** The sensor reports what it detects in no
  particular order; relevance ordering is a consumer concern, so IDM
  scans the array
  for the smallest range and follows that car. Two things fall out. The
  free-road case needs no special handling, since elements past the end
  of a short scan hold the defaults and the minimum of an empty scan is
  simply 1e9. And the array input is earned by IDM itself rather than
  only by its successor: a controller that cannot see the whole scan
  cannot find its own lead. This is what PLAN.md's FMI section meant by
  native arrays, and it makes the interface outlive the law inside it:
  `continuo-fmu-controller-ai` declares the identical inputs and is free
  to canonicalize the set however its architecture wants, whether by
  sorting or by a permutation-invariant encoder, without the radar, the
  mapping, or the wiring changing.
- **Own speed comes from the pose, not the radar** (a real radar does not
  know how fast its own car is going; wheel-speed sensors and the vehicle
  bus do). The published pose's `speed` field stands in for that sensor,
  and the FMU fuses it with the radar's relative measurements exactly as a
  real controller would. IDM wants the approach rate anyway, so no absolute
  lead speed is needed anywhere: `approach_rate = -range_rate`, negated
  inside the FMU because a mapping binds JSON Pointers and does no
  arithmetic
  (a mapping that can negate is a mapping that can do anything).
- **Radar is stateless per step, with no freshness filter.** The inbox
  window is the filter: the scan reports the newest pose per actor
  published since the last scan, so a despawned car ghosts for at most one
  scan and in the demo's geometry never observably (detections are
  ahead-only and the rearmost car in a lane retires first; after the
  ego-lane PR, cars ahead of the ego never retire at all). Documented
  constraint: observed publishers must publish at least once per radar
  period, or cars blink. `state_bytes: None` (it publishes everything it
  derives each step). First step publishes nothing (empty inbox).
- **`CAR_LENGTH` lives in continuo-actors** (`src/lib.rs`, beside
  `pose_key`): `pub const CAR_LENGTH: f64 = 4.5;`, doc comment naming
  `python/continuo_viz/render.py:39` as the constant it must move with,
  and a matching comment added on the Python side.
- **Examples error enum**: `ScenarioError` in traffic_world.rs:
  `Conductor(#[from] ConductorError)` (transparent) + `EgoCollision {
  other, sim_time, gap }`. `run_live_traffic_scenario` returns it; example
  mains use `Box<dyn Error>` so `?` keeps working; `unreadable_request.rs`
  adjusts its helper's return type.
- **Collision scan runs between ticks, not per message**, and the pair
  scan is one function taking per-lane groupings so a future spatial
  version (per-lane sort by arc length, adjacent pairs) replaces a single
  function. The collision monitor's car tracking is cross-tick state,
  unlike the radar's per-step window, so a departed car has to be taken
  out of it or its
  frozen pose sits on the road and every vehicle passing that spot
  overlaps it. **Membership events do the removing**, not an age
  horizon: the monitor takes a membership callback beside its transport
  one and drops an actor when its pose publisher leaves. That is exact
  rather than eventually, it spares a tuning constant with no
  principled value, and it is the presence rule the viz bridge already
  settled and DECISIONS already records. Leaves arrive as component
  paths, so the monitor keys on the first segment exactly as the Python
  viewer binds to its pose source.
- **One `Fmi3Import` per FmuComponent** (own tempdir). Struct field order
  puts the instance before the import, with a comment: drop order is
  declaration order, and Windows cannot delete a still-loaded DLL's
  tempdir.
  - **Log the extraction directory at debug**, with the instance name
    and the source `.fmu`, and again when the component is dropped. A
    demo run extracts a dozen or more times, the directories are chosen
    by the `fmi` crate, and `tempfile` discards its cleanup errors on
    drop, so a Windows delete that fails because the DLL is still loaded
    leaves directories behind and says nothing. Knowing the paths is
    what makes that diagnosable at all. `continuo-fmi` takes `tracing`
    for this, as the rest of the workspace does. Whether the import
    exposes its extracted path is a PR A check; if it does not, log the
    source path and instance name, which still narrows the search.
- **An FMU is data, not a Rust type.** `FmuComponent` is one concrete
  struct implementing the existing `Component` trait, never a base trait
  with a subtype per FMU: the controller is a `.fmu` path plus an
  `FmuMapping` handed to that struct, and since a mapping is built where
  the world is known, its keys are concrete rather than wildcarded.
  Anything else would mean
  recompiling the host to add an FMU, which is what FMI exists to avoid
  and would sink the demonstration M6 owes, that a third-party `.fmu`
  runs with no code written. It is also where PLAN.md's scenario
  configuration already points: a registry instantiates "Rust-native
  types and FMUs alike, where an FMU entry points at the `.fmu` and its
  variable-mapping config", so today's Rust literals become JSON5 later
  with no new Rust either way. The polymorphism lives one level up, at
  `Component`. The one Rust type per FMU sits on the *other* side of the
  boundary, in the exporting crate, implementing upstream's `UserModel`.
  If some future FMU ever needs behavior a mapping cannot express, the
  answer is an ordinary `Component` wrapping an `FmuComponent`:
  composition, not subclassing.
- **Paths address the decoded payload, not its bytes**, written in JSON
  Pointer syntax. The adapter resolves them against the value
  `Message::decode` hands back and publishes through `ctx.publish`; it
  never calls `serde_json` itself. That is what keeps the deferred binary
  wire format from reaching this crate: CBOR's data model is the same
  shape for these payloads, so `/detections/3/range` means the identical
  thing, and integers above 2^53 stay exact in either mode. The adapter
  decodes to a dynamic value because it cannot know a payload's shape,
  and that line is already correct for the day `decode` dispatches on
  format. `when_missing` is an in-memory default, never serialized, so it
  is unaffected (a genuinely format-neutral dynamic type would be a
  core-owned rename, not a redesign). Publishing integers as integers
  matters in both modes, since CBOR separates integer and float
  encodings too. One hazard is core's rather than this crate's but has
  its most likely trigger here: CBOR can carry NaN and Inf natively, so
  the non-finite publish guard's fast path needs revisiting when binary
  mode lands, and a diverging FMU is the likeliest thing to emit one.
  PR #18's `the_fast_path_premise_holds` is the existing tripwire.
- **An array variable is fed by one JSON Pointer per element**, which is
  the scalar mechanism unchanged rather than a second one: a binding
  carries a list of them, length 1 for a scalar and N for an array, and
  each element's own `when_missing` covers a scan shorter than N. A
  helper generates `["/detections/0/range", "/detections/1/range", ...]`
  from a prefix, a count, and a field name. This is also why the scan
  stays an array of detection records rather than parallel arrays of
  numbers: the message keeps a detection's fields together, and the
  transposition into FMI's homogeneous arrays happens once in the adapter,
  where it is tested, instead of distorting the sensor's payload for one
  consumer's convenience.
  - Multi-dimensional variables are one flat pointer list of the
    product of the dimensions, in row-major order per the standard. A
    silently transposed matrix still runs, so the order is pinned by a
    test rather than left to a comment.
- **Array dimensions are resolved, not read literally.** A dimension is
  either a constant in modelDescription or a reference to a
  `structuralParameter`'s value, as in StateSpace, where `A` is n by n
  with `n` a structural parameter. The adapter resolves each to the
  effective value, meaning the mapping's initial value where one is
  given and the declared start otherwise, and only then compares against
  the binding's pointer count.
  - **Structural parameters are settable at construction**, through
    `fmi3EnterConfigurationMode`, the values, then
    `fmi3ExitConfigurationMode`, before initialization begins. That mode
    is the only place they may be set. Without this an FMU like
    StateSpace could only ever be the size its defaults declare, which
    is a poor showing for something billed as a general importer.
  - **Changing them mid-run is not planned**, and probably never should
    be. FMI permits it for `tunable` structural parameters by
    re-entering configuration mode and re-setting every array the change
    resized, but this project already has an answer for a component that
    needs to become something else: membership. Remove it and admit a
    replacement configured differently, which is deterministic, recorded
    in the event log and replayable, where an in-place resize would be
    none of those and would change a component's shape under its
    subscribers mid-run. The one thing despawn and respawn cannot
    preserve is the FMU's internal state, so if that ever becomes the
    requirement, the question reopens alongside FMU state serialization
    rather than on its own. A mapping asking for it fails with a message
    saying which mechanism to use instead.
- **Every FMI 3.0 variable type binds except Clock, and the mapping never
  names one.** The ten numeric types, plus Boolean, String and Binary;
  the adapter reads each variable's declared type from modelDescription
  at construction and dispatches to the matching get/set family (a macro
  over the families, mirroring how the `fmi` crate generates them). A
  mapping that repeated the type would be a second source of truth that
  could disagree with the FMU it points at. This supersedes PLAN.md's
  "float64 value references" line.
  - **Dispatching by type is a correctness requirement, not generality
    for its own sake**: routing an Int64 or UInt64 through f64 silently
    loses precision above 2^53. serde_json (with the workspace's
    `arbitrary_precision`) carries those exactly, so the round trip is
    lossless only if the integer path stays integral end to end. Integer
    variables are also exactly portable, unlike the float ones the world
    hash already worries about.
  - Outputs publish as their natural type, integers as integers: `3` and
    `3.0` are different bytes and therefore different hashes.
  - Conversion failures name the variable, its declared type and the
    offending value, and halt as `ComponentFailure`: a value that is
    not a number, or one outside the target's range (an Int8 fed 999
    fails rather than wrapping). Float32 accepts precision loss but not
    range overflow.
  - Boolean is in, since the road's `is_closed` wants it and encoding a
    flag as an integer to keep the set numeric would be the dishonesty
    this design keeps avoiding. `fmi3Boolean` is a C `bool` and
    serde_json has `Value::Bool`, so it is one more arm.
  - **String and Binary are in too**, though nothing in the demo uses
    them: a general importer that covers only what one FMU happens to
    need is not general, and both are cheap now against a later
    retrofit of the dispatch, the defaults, and the tests together.
    - Both carry the same lifetime rule, and it is the sharp edge: what
      `fmi3GetString` and `fmi3GetBinary` hand back is valid only until
      the next call on that instance, so the adapter copies into owned
      values immediately rather than holding a borrow. Getting this
      wrong reads as intermittent corruption, not as a failure.
    - String is `Value::String`, with two conversion failures worth
      naming: a JSON string containing an interior NUL cannot cross a
      C boundary, and an FMU may return bytes that are not UTF-8.
      Both halt as `ComponentFailure` naming the variable, like a
      range overflow.
    - Binary has no JSON representation, so it travels base64 in a
      string. The encoding is pinned as canonical, standard alphabet
      with padding and no line breaks, because payload bytes reach the
      world hash. Owned rather than taken as a dependency, on the same
      argument that owns the hash and the RNG: about sixty lines, and
      determinism constrains the bytes we produce.
    - Binary is also where the format-neutral in-memory model strains,
      honestly noted rather than hidden: CBOR carries byte strings
      natively, so a binary wire mode would want a bytes variant that
      `serde_json::Value` does not have. That is the concrete thing
      that would earn core a `PayloadValue` of its own, and the
      deferred large-payload item is where the size pressure lands.
  - Clock stays out. It is a scheduling concept rather than data, and
    it belongs with the event mode this adapter switches off.
- **Structured naming maps FMU variables to JSON.** Each output variable
  routes to a key and its name is the payload path (dots nest, `[i]`
  indexes; `accel_cmd` publishes `{"accel_cmd": x}`). Inputs keep
  explicit JSON Pointers (the FMU's variable vocabulary and the message
  shapes are different languages), defaulting to the name-derived path.
- **Whether initial values survive the mode changes is an open question
  for PR A to settle by experiment**, not an assumption to design
  around. The starting point is to set them after
  `EnterInitializationMode`; FMI 3.0 also permits setting parameters in
  the Instantiated state, before entering, so there are two legal
  placements to try. Nothing in the standard promises a value set in one
  of them is still there after `ExitInitializationMode`, and generated
  FMUs are the likeliest place for a reset to hide, including our own
  from `fmi-export`.
  - The tripwires are the instrument rather than a confirmation:
    BouncingBall with a non-default restitution must bounce accordingly,
    and a Feedthrough input set during initialization must appear at the
    first output. Run them against both placements before choosing.
  - Contingencies if a reset turns up. Inputs can be re-applied after
    `ExitInitializationMode`, since inputs are settable in StepMode, and
    the adapter already applies its inbox on the first step. Parameters
    are the real exposure: a `fixed` one cannot be set once
    initialization is over, so if those do not survive, the answer is
    the other placement, or `tunable` variability on our own FMU, and
    for a third-party FMU that resets its parameters, a documented
    limitation. Record what is actually observed, since this is the sort
    of behavior that is expensive to rediscover.
- **Only the newest message on each bound key is applied.** The inbox is
  scanned in reverse and the first match for a key wins, so older
  messages on that key are skipped rather than applied and overwritten,
  and are never decoded at all. Each distinct key parses once per step,
  however many bindings read pointers out of it. This is FMI's own input
  semantics, where an FMU sees the value at the step boundary rather
  than a sequence of intermediate ones, and it matches the
  sample-and-hold physics already does. Should two publishers ever share
  a key, the inbox's `(publisher, seq)` order decides, which is
  deterministic but arbitrary; one publisher per key is the norm and the
  only unambiguous case.
- **FMU state joins the hash when the FMU offers it**: if modelDescription
  declares `canSerializeFMUState`, `state_bytes` returns the serialized
  state (the hook PLAN.md's determinism rules anticipated); else `None`
  (output-hash mode). A mapping override forces output-hash for a vendor
  FMU whose serialization bytes are not deterministic. Implementation-time
  check: confirm the `fmi` crate exposes state serialization on import;
  if not, fall back to `None` with a note.

Each PR merges to main on its own, with no long-lived M6 branch: every
one leaves main coherent, and the four-agent matrix is the instrument
for this milestone's platform risk, so it should run against each
increment rather than against one large merge at the end.

## PR A: `continuo-fmi` (import, FmuComponent, vendored fixtures)

First commit: **remove the `Send` supertrait from `Component`** (decision
5) with its DECISIONS entry. Compile-only change; nothing in-tree
constrains on it.

New crate `crates/continuo-fmi` (deps: continuo-core, fmi, serde_json,
thiserror, tracing). Root Cargo.toml adds the crate and `fmi` to
`[workspace.dependencies]`. `serde_json` is here for its `Value` as the
dynamic in-memory model and for JSON Pointer resolution, not for
encoding: encoding and decoding go through core, so the deferred binary
mode does not reach this crate.

- `src/mapping.rs`: `FmuMapping { period, inputs, outputs,
  initial_values }`.
  - `InputBinding { variable, key, pointers, when_missing }`. One JSON
    Pointer per element, so length 1 is a scalar and N feeds an array of
    dimension N; the resolved dimension is checked against the pointer
    count at construction, which is what stops a rebuilt FMU and a stale
    mapping from drifting apart silently. Pointers default to the
    name-derived path. A pointer finding nothing in a fresh message uses
    `when_missing`; no message at all means hold.
  - `when_missing` is a `serde_json::Value`, so one field serves every
    type, and it is checked against the variable's declared type at
    construction: a default the FMU could never accept fails at build
    rather than at the first gap in the data.
  - `OutputBinding { variable, key }`. The payload path derives from the
    variable name, and variables sharing a key merge into one object.
  - `initial_values: Vec<(String, serde_json::Value)>`, set during
    initialization, which is what makes per-car parameterization
    possible without editing the FMU. A `Value` for the same reason
    `when_missing` is one, with two live cases in the demo: the boolean
    `road_closed`, and the road's waypoint arrays, whose length is
    checked against the declared dimension like everything else.
- `src/error.rs`: `FmuError` for construction-time failures (`Import`,
  `Instantiate`, `UnknownVariable { variable, available }`,
  `NotCoSimulation`). Step-time failures map to
  `CoreError::ComponentFailure`.
- `src/component.rs`: `FmuComponent`.
  - `new(id, fmu_path, mapping)` imports, then resolves every mapped
    variable name to its value reference, failing on an unknown name
    with the list of what exists. It checks the mapping's period against
    `fixedInternalStepSize` where the FMU declares one and fails unless
    it is an integer multiple: StateSpace declares 1 second, so a 100 ms
    period would not land where the caller thinks it does.
    `canHandleVariableCommunicationStepSize` is not required, since a
    fixed period makes every interval identical; it becomes load-bearing
    only when event mode varies them.
  - Then instantiate CS with `event_mode_used = false`, so
    ExitInitializationMode lands directly in StepMode and an FMU handles
    its own events inside a step. If the mapping sets structural
    parameters, they go in between: `EnterConfigurationMode`, set,
    `ExitConfigurationMode`, and only then are array dimensions resolved
    and pointer counts checked.
  - **First step** (dt is None): enter-init, set initial values, apply
    the inbox, exit init, get and publish the initial outputs, no
    do_step. Where "set initial values" lands is what the experiment
    above decides.
  - **Later steps**: apply the newest message per bound key, then
    `do_step(last_time, now - last_time, ...)`. An `Err` from that call,
    or a `terminate_simulation` flag, returns
    `CoreError::ComponentFailure` there and then, so a failed step reads
    no outputs and publishes nothing; the conductor discards a failed
    step's outbox anyway, so the two agree. Otherwise get the outputs,
    publish per output key, and return `now + period`.
  - An array is one set or get call on its type's family with
    `nValues = N` against a single value reference, which is what the C
    API's separate reference and value counts exist for. `state_bytes`
    follows the capability-gated rule. The non-finite publish guard
    means an FMU emitting NaN halts the world with the value named,
    which the type's docs say.
- Fixtures:
  `tests/fixtures/{BouncingBall,Feedthrough,StateSpace,Resource}.fmu`
  from Reference-FMUs v0.0.40, plus `fixtures/README.md` with origin,
  version,
  why vendored, and the BSD-2-Clause text verbatim. `.gitattributes`
  gains `*.fmu binary`. Feedthrough exists precisely to carry one variable
  of each type, so it is the type-dispatch fixture (verify the roster at
  implementation). StateSpace is the array fixture and earns its place
  three times over: its matrices are sized by structural parameters, so
  it exercises configuration mode and resolved dimensions rather than
  the easy fixed-dimension case; it declares `canSerializeFMUState`,
  giving the state-hash path a second fixture beside BouncingBall; and
  it declares `hasEventMode`, so it is the obvious fixture when event
  mode is eventually taken on. Resource reads `resources/y.txt` during
  initialization, which is the only thing that checks the
  `resourcePath` handed to `fmi3InstantiateCoSimulation`: FMI 3.0
  changed that argument from 2.0's file URI to a plain path, our own FMU
  ships no resources, and real vendor FMUs routinely do, so without it
  the first FMU needing its own files is the one that finds the bug. It
  also cross-checks the extraction plumbing the tempdir logging exists
  to diagnose, and it fails legibly, with `y` simply wrong. It matters
  that these are somebody else's artifacts: our own FMU comes from
  `fmi-export`, the same upstream workspace as the importer, so testing
  only against it would be close to circular.
  - Dahlquist and VanDerPol are deliberately left out, being smooth
    ODEs with no events, arrays or resources, so the adapter treats
    them exactly as it treats BouncingBall. Stair is left out for now
    and named in the event-mode entry instead: its one-second time
    events are the natural fixture for `nextEventTime`, but with event
    mode off it only shows output changing at second boundaries.
- README crates table row; refresh the viz-bridge Cargo.toml comment's
  workspace crate count while touching the tree.
- **`M6-PLAN.md` at the root**, this document, checked in as working
  scaffolding and deleted in PR H. Its first line says so, because a
  root-level plan is otherwise easy to mistake for a keeper beside
  PLAN.md. It is here so eight PRs of choreography and the reasoning
  behind PR A's shape do not live only in one session, and so the
  upstream experiments have somewhere to write their findings down.
  - Treat it as a snapshot, not a living document: no churn per PR.
    Where reality diverges, the divergence goes in that PR's DECISIONS
    entry, which is where it belongs permanently, and PR H reconciles
    the lot into PLAN.md. The one part worth updating in place is the
    open-questions list, since a resolved upstream behavior is exactly
    what the next PR needs to read.
  - It does not become a third permanent document. PLAN.md holds the
    design as it stands and DECISIONS.md holds why, dated; everything
    here is raw material for one or the other.

Tests (conductor-driven, style of actors/determinism.rs):
`a_reference_fmu_loads_instantiates_and_steps` (BouncingBall, h falls),
`an_fmu_component_survives_the_whole_conductor_lifecycle` (mid-run join
and leave; also the Windows tempdir-vs-DLL drop-order pin),
`inputs_reach_the_fmu_and_outputs_carry_them_back` (Feedthrough),
`outputs_route_to_their_declared_keys`,
`every_supported_type_round_trips_through_a_mapping` (Feedthrough, which
exists to carry one variable of each type, the value in equal to the
value out),
`an_integer_larger_than_a_float_can_hold_survives_the_round_trip` (the
2^53 trap: fails outright if anything routes integers through f64),
`an_integer_output_publishes_as_an_integer` (payload bytes, since `3` and
`3.0` hash differently),
`a_string_output_survives_a_second_get_on_the_same_instance` (the FMI
lifetime rule: the test fails if the adapter holds a borrow rather than
copying, which otherwise shows up as intermittent corruption),
`a_string_containing_an_interior_nul_halts_rather_than_truncating`,
`a_non_utf8_string_from_an_fmu_halts_naming_the_variable`,
`a_binary_value_round_trips_through_canonical_base64` (and a second test
pinning the encoded bytes themselves, since they reach the world hash),
`a_value_outside_a_variables_range_halts_naming_the_variable_and_type`,
`a_default_the_variable_could_never_accept_fails_at_construction`,
`resolution_and_conversion_are_pure_functions_over_a_decoded_value` (the
unit-level suite for JSON Pointer resolution, padding and range checks,
run
with no FMU loaded, which is also what keeps the wire format out of this
crate),
`an_array_binding_whose_pointer_count_misses_the_declared_dimension_fails_at_construction`,
`a_short_scan_fills_the_tail_of_an_array_with_its_defaults`,
`an_array_sized_by_a_structural_parameter_binds_at_its_declared_start`
(StateSpace at its default 3 by 3),
`a_structural_parameter_set_at_construction_resizes_its_arrays`
(StateSpace at 2 by 2, proving configuration mode ran and dimensions
were resolved from the mapping rather than the XML),
`a_matrix_binds_row_major` (a transposed matrix still runs, so this is
pinned rather than commented),
`a_period_that_is_not_a_multiple_of_the_fixed_internal_step_size_fails_at_construction`,
`a_mapping_that_asks_to_change_a_structural_parameter_mid_run_is_refused`
(naming membership as the mechanism to use instead),
`an_fmu_reads_its_own_resource_files` (Resource, whose `y` is wrong if
the resource path we hand across is),
`a_missing_pointer_uses_the_declared_default_and_no_message_holds`,
`an_unknown_variable_name_fails_at_construction_naming_the_alternatives`,
`initial_values_set_parameters_before_the_first_step`,
`an_input_set_during_initialization_survives_to_the_first_output` (these
two are the experiment on where initial values may be set and whether
they survive the mode changes; run against both legal placements, then
record what was observed and which one the adapter uses),
`two_identical_fmu_runs_fingerprint_identically`,
`fmu_state_joins_the_hash_when_the_fmu_can_serialize_it` (BouncingBall
has real internal state),
`an_fmu_step_failure_halts_the_world_naming_the_instance_and_call`.

DECISIONS titles: Component no longer requires Send (the parallel future
is the host protocol run locally; migration is membership; the bound
guarded nothing, and an FMU instance truthfully is not Send; revisit
trigger recorded). FMU import goes through the `fmi` crate and is the
workspace's first native-code dependency (bindgen and libclang recorded;
feature-trimmed to fmi3; zip arrives as a side effect; what PLAN.md's
`fmi-rs` candidate was weighed against and why it lost).
An FMU that fails to step halts the
world as `ComponentFailure`. An FMU is data, not a Rust type, so there is
one `FmuComponent` and no per-FMU trait (recompiling to add an FMU would
defeat the standard; the scenario registry PLAN.md already describes is
where this ends up). A mapping addresses the decoded payload rather than
its bytes, which is how the deferred binary wire format stays out of this
crate. Every FMI 3.0 numeric type binds, dispatched from the declaration
(the 2^53 argument, not generality for its own sake), String and Binary
included so the importer is general rather than shaped to one FMU, and
the radar scan reaches the controller as native arrays, which is the
half of
PLAN.md's "native arrays, float64 value references" that survives.

## PR B: control laws in actors, packaged as `continuo-fmu-controller-idm`

- **`continuo-actors` gains a `control_laws` module**, holding every law
  the FMU links against and nothing else: `idm_accel`,
  `nearest_detection`, `pure_pursuit_yaw_rate` and their parameter
  structs. The crate has always been sample components; M6 makes it also
  the control library an out-of-process artifact compiles into a `.dll`,
  and one module makes that shared surface a single import instead of
  something to infer from two files. Components use it exactly as the
  FMU does.
  - Named for what it holds, because a module called `control` beside
    `controller.rs` would read as the place controllers live, when it is
    the pure functions controllers call. The boundary stays legible if a
    second controller component ever arrives.
  - Nothing else moves. At 972 lines across 6 modules today and roughly
    1,400 across 8 after M6, the crate is not big; the mixed kinds are
    the only real change, and subdirectory grouping would be cosmetic
    since `pub use` hides layout from consumers anyway. A structural
    refactor riding along would also make PR C harder to review against
    its provably-inert claim.
  - PR H records the trigger for the move that would matter: splitting
    the laws out into their own crate, so an FMU shell stops depending
    on the sample-components crate to reach three functions. The trigger
    is the second FMU crate, since
    `continuo-fmu-controller-ai` reaching into actors as well confirms
    the edge is structural rather than incidental. By then it is a file
    move and a Cargo.toml.
- In `control_laws`, `IdmParams { v0, t_headway,
  s0, a_max, b_comfort, b_max }` and `pub fn idm_accel(p, speed, gap,
  approach_rate) -> f64`: `s_star = s0 + max(0, v*T + v*dv/
  (2*sqrt(a_max*b_comfort)))` with `dv = approach_rate`, then
  `a = a_max*(1 - (v/v0)^4 - (s_star/gap.max(GAP_FLOOR))^2)` clamped to
  `[-b_max, a_max]`. IDM needs the approach rate, never the lead's
  absolute speed, which is why a Doppler-style relative measurement is the
  natural input rather than a concession. Only add, multiply, divide,
  sqrt, powi: bit-portable, no new trig joins the hash's exposure. The
  `max(0, ...)` clamp is the book/MovSim practical variant, keeping a
  fast-receding lead from driving the desired gap below `s0`. Free road
  falls out of the defaults: gap 1e9 makes the interaction term vanish and
  approach rate 0 leaves the headway term positive, so no clamping is even
  involved.
  - Defaults are the literature's: `T = 1.5 s`, `s0 = 2 m`,
    `a_max = 1.5`, `b_comfort = 2.0`, with `v0` supplied per car. These
    are calibrated values rather than knobs, and the doc comment says
    so, because a demo that misbehaves with them has a wiring bug and
    refitting them would hide it.
  - `b_max` and `GAP_FLOOR` are this project's additions, not IDM's, and
    are documented as such. Standard IDM bounds braking nowhere, so
    `b_max = 4.0` is a deliberate choice of firm rather than emergency;
    `GAP_FLOOR` guards a division and its value is irrelevant in any
    healthy run, since reaching it means the cars already overlapped.
- `control_laws` also carries `pub fn
  nearest_detection(ranges: &[f64], rates: &[f64]) -> (f64, f64)`: the
  lead selection a car-following controller owns, returning the pair for
  the smallest range and the free-road defaults for an empty scan. Tests:
  `the_nearest_detection_wins_regardless_of_its_slot`,
  `an_empty_scan_selects_the_free_road_defaults`,
  `a_tie_picks_one_of_the_two_and_does_so_the_same_way_every_time`.
- `pure_pursuit_yaw_rate(road, pose, params)` is extracted from
  `PathFollowController::step` into `control_laws` as well, so the laws sit
  together rather than one beside its component; the component calls it
  (pure refactor, hash-neutral, asserted by the untouched demo hash).
- `continuo-actors/src/path.rs`: `Waypoints` gains `pub fn points(&self)
  -> &[(f64, f64)]` and `pub fn is_closed(&self) -> bool`; all three
  fields are private today, and the FMU cannot be handed the object. Test
  `a_road_rebuilt_from_its_points_is_the_same_road`: reconstruct through
  `build_open`/`build_closed` and assert projections agree bit for bit at
  sampled positions. That is what makes the FMU's copy and the shared
  `Arc` the same geometry, and what would catch any future `Waypoints`
  state that is not derivable from its points.
- `continuo-actors/src/lib.rs` gains `MAX_DETECTIONS = 64`, defined here
  because the FMU's declaration is its first user and the sensor picks it
  up in PR D. It is a defensive bound rather than a working limit: a
  120 m lane cannot physically hold more than about two dozen cars at car
  length plus a following gap, so truncation never triggers in any world
  this project runs, and the cost is padding inside the FMU rather than
  anything on the wire. `MAX_WAYPOINTS = 64` joins it only in the
  fallback form, where the road is a fixed array; it covers the demo's
  two points with room for a hand-built polyline, and it is where the
  road-network importer's "whole map or a local window" question will
  first show.
- New crate `crates/continuo-fmu-controller-idm`, `publish = false`,
  `[lib] crate-type = ["cdylib", "rlib"]`. Deps: continuo-actors,
  continuo-core, fmi-export.
  - The `#[derive(FmuModel)]` struct, `#[model(co_simulation = true,
    model_exchange = false, user_model = false)]`, declaring the
    interface roster from Cross-cutting. The radar inputs are
    `[f64; MAX_DETECTIONS]`, with `const _: () = assert!(..)` against
    the actors constant so a cap raised without rebuilding the FMU
    cannot go unnoticed.
  - **The derive cannot size an array by a structural parameter**, so
    the road takes the fallback form: `[f64; MAX_WAYPOINTS]` with
    `road_point_count` beside it and the same compile-time assert. The
    count is an ordinary `Parameter`, since a `StructuralParameter` that
    sizes nothing would claim a role it does not have and would send the
    importer into configuration mode for no reason.
    - `causality = StructuralParameter` is accepted and declared, but
      nothing connects one to a dimension. A `[T; N]` field declares
      `Dimension::Fixed(N)`, and `#[variable(...)]` has no key naming a
      sizing variable.
    - `Vec<T>` is the only path to a variable dimension, and it is
      unusable twice over: it hardcodes value reference 0, which is the
      derive's own `time` variable, and it implements neither of the get
      and set traits, so the variable could not be read or written even
      if the dimension named the right thing.
  - A non-`#[variable]` `Option<Waypoints>` field, invisible to FMI and
    ordinary Rust state that drops at `fmi3FreeInstance`. `impl
    UserModel` builds the road **once, on first use**:
    `calculate_values` takes `&mut self`, so
    `self.road.get_or_insert_with(..)` costs one branch per step and is
    correct whenever any lifecycle hook happens to fire. Road
    parameters are settable only during initialization, so first use is
    always after they are final, and rebuilding per control step would
    reallocate and recompute the arc-length table for every car at
    100 ms. `configurate` is an optimization to adopt if it proves to
    run after parameters are set, not a prerequisite; if the derive
    rejects non-variable fields, the fallback is that per-step rebuild,
    wasteful rather than wrong.
  - `calculate_values` then selects its lead through
    `nearest_detection`, which returns the free-road default when
    nothing is detected so there is no empty case to write, and
    delegates to `pure_pursuit_yaw_rate` and `idm_accel`.
  - `export_fmu!`, and `pub fn packaged_fmu_path() -> Result<PathBuf,
    ...>` walking `current_exe()` ancestors for
    `fmu/continuo_fmu_controller_idm.fmu`, its error Display carrying
    the fix (`cargo install cargo-fmi`, then `cargo xtask
    package-fmus`). Tests fail with that message rather than skip.
- **The `.fmu` embeds a snapshot of `continuo-actors`**, since the cdylib
  links it statically: the control laws and the whole `Waypoints`
  implementation are compiled into the shared library the zip ships, and
  nothing calls back into the host. Editing `path.rs` or `control_laws`
  without
  packaging it again therefore leaves the native code and the FMU's copy
  disagreeing. CI cannot hit this, because packaging runs before the tests
  on every job; locally the golden bit-identity tests are the detector
  and fail loudly, which is the right outcome. What they cannot do is
  say *why*, since a stale package presents as the IDM math disagreeing
  with itself. So their failure messages name it: the laws may have
  changed since the FMU was built, and here is the command to rebuild
  it. That is the whole guard, deliberately. Anything cleverer, a
  timestamp comparison or a `build.rs` that shells out to cargo, buys
  accuracy this does not need.
- **`xtask`**, a workspace member binary plus `.cargo/config.toml` with
  `[alias] xtask = "run --package xtask --"`, so `cargo xtask
  package-fmus` packages every FMU in the repo. It discovers them by the
  `continuo-fmu-*` crate-name prefix, so `continuo-fmu-controller-ai`
  is picked up later with no change anywhere, and CI's packaging step
  never has to learn about a second FMU. It shells out to `cargo fmi`
  and fails with the install command when that is missing, in the same
  style as `packaged_fmu_path`. Cargo has no user-defined targets and
  its aliases cannot chain commands, so this is the idiomatic form; it
  is also a real entry point rather than a hidden side effect of
  `cargo build`, which is why it beats a `build.rs`. First `xtask` and
  first `.cargo/config.toml` in the workspace, which is new structure
  in a repo that has deliberately little, and worth it because CI is
  the immediate beneficiary.
- README: dev-setup note pointing at `cargo xtask package-fmus` (install
  `cargo-fmi` once, package before `cargo test`, and again after editing
  the shared laws), plus bindgen needing libclang (stock on CI images;
  `winget install LLVM` locally). Crates table row.

Tests: `the_packaged_fmu_reproduces_the_native_idm_bit_for_bit` and
`the_packaged_fmu_steers_bit_identically_to_the_native_pursuit` (dev-dep
continuo-fmi; sweep input grids and parameter sets through the imported
FMU against the actors functions; `f64::to_bits` equality; also proves
the lazy calculate_values path fires; the fallback if it does not is to
override
`do_step` to call the same functions).
`a_multi_point_road_steers_identically_to_the_native_pursuit` runs the
same comparison over a curved polyline the demo never uses, which is what
keeps the road arrays from being a two-point special case in disguise;
`a_closed_loop_road_wraps_inside_the_fmu_as_it_does_natively` does the
same for `road_closed`, the field whose absence would have been silent;
`a_point_count_shorter_than_the_array_ignores_the_padding` pins that the
tail is never read. IDM unit tests in actors:
`free_road_accelerates_toward_v0_and_no_further`,
`a_close_slow_lead_commands_braking_within_b_max`,
`at_v0_with_ample_gap_accel_is_near_zero`,
`the_gap_floor_keeps_a_touching_lead_finite`,
`output_is_always_finite_and_inside_the_clamp`, plus hand-derived spot
values from the published formula (cross-checked once against
highway-env, noting it uses the unclamped variant).

CI (before the test steps, after "Lint: docs"): cache
`~/.cargo/bin/cargo-fmi*` keyed on version + os + arch; `cargo install
cargo-fmi --version <current> --locked` on miss; then `cargo xtask
package-fmus`, which names no package and so needs no edit when the
second FMU arrives.

DECISIONS titles: The demo FMU is a complete car controller and a drop-in
replacement for the native one (why a planner-plus-relay lost; the laws
live in actors, the FMU packaging is a shell crate, and the `.fmu`
therefore carries a compiled snapshot of those laws, which is what makes
packaging part of the edit-test loop, the golden tests its detector,
and their failure text the only guard worth having). `cargo xtask
package-fmus` packages every `continuo-fmu-*` crate (why an xtask rather
than a `build.rs` or an alias, and why CI calls it instead of naming a
package). The road crosses as waypoint arrays, a count and a closed
flag, and is rebuilt inside the FMU (three pieces of state, only two of
them obvious; each instance owns its copy because a black box cannot
share the host's memory). IDM math is restricted to add,
multiply, divide, sqrt, powi, and the max(0, .) clamp doubles as the
free-road convention (no reference IDM exists to import; the published
equation is the reference). cargo-fmi is a CI-installed binary, not a
dependency.

## PR C: physics takes acceleration (hash move 1 of 3)

The plant reshape, behavior provably unchanged: `AccelCmd`/`SteerCmd`
replace `Cmd`; `UnicyclePhysics` takes a `UnicycleState { pose, speed }`
in place of its `initial_pose` argument, integrates v, routes two
subscriptions, publishes `{position, orientation, speed}`;
`PathFollowController` loses `speed` and its longitudinal role,
publishing `SteerCmd` on `.../steer_cmd`. Key helpers `accel_cmd_key`,
`steer_cmd_key` in actors. Constant worlds trace bit-identical
trajectories (the command holds at 0, v never moves), so recorded pose
series match pre-change values exactly; only payload shapes, keys, and
the hash move. `DEMO_WORLD_HASH` and README's hash line move together
(the sample pose values themselves stay, which is the PR's own proof of
inertness). Call sites: `traffic_world::add_car`,
`setup_scale_scenario`, `actors/tests/determinism.rs`.

Tests: `a_constant_speed_car_traces_the_same_path_as_before_the_reshape`
(pin a few sampled positions to the old values),
`accel_and_steer_are_held_independently`,
`held_acceleration_integrates_into_speed`,
`speed_never_integrates_below_zero`,
`an_initial_state_round_trips_through_json` (the struct is the shape
scenario files will deserialize, so its serde form is part of the
contract from the start).

DECISIONS titles: Physics integrates commanded acceleration and owns
speed; longitudinal and lateral commands travel separately (the
highway-env convention; what moved in the hash and why the trajectories
did not). A plant's initial state is one deserializable struct rather
than positional arguments (what it costs now, what it saves when
scenario files arrive, and why the `Component` trait stays out of it).

## PR D: RadarSensor (not yet wired into the demo)

- `continuo-actors/src/path.rs`: `pub fn frenet(&self, x, y) -> (f64,
  f64)`, arc length plus signed lateral (left positive); `project`
  delegates to it. Test:
  `frenet_recovers_both_arc_length_and_signed_lateral`.
  (`points()` and `is_closed()` arrived in PR B, which needed them.)
- `continuo-actors/src/lib.rs`: `CAR_LENGTH`, `radar_key`, module wiring.
  `CAR_LENGTH`'s doc comment names `python/continuo_viz/render.py:39` as
  the constant it must move with, and the Python side gains the matching
  comment pointing back. While in `render.py`, reword its "a lorry and a
  hatchback are the same rectangle" TODO to American English, the
  codebase's only British usage.
- The sensor's default cap is PR B's `MAX_DETECTIONS`, the same constant
  the FMU's arrays were built from, so the two cannot drift. The scan
  carries only the cars actually detected, so the cap costs nothing on
  the wire; the padding to 64 happens in the FMU, at two set calls of 64
  doubles per control step, which is noise beside one `fmi3DoStep`.
- `continuo-actors/src/radar.rs`: `RadarDetection { range, range_rate }`,
  `RadarScan { detections }` in **no specified order**, capped at
  `max_detections`. `RadarSensor { actor_name, road, period, max_range,
  max_detections, lane_tolerance }`, subscribes `*/actor/**/pose`, id
  "radar". Detections: other actors within `lane_tolerance` laterally,
  ahead along s, within `max_range`. `range` is bumper to bumper (delta s
  minus CAR_LENGTH), which is what a front-mounted radar sees;
  `range_rate` is its time derivative, negative when closing, the
  quantity a Doppler radar measures directly.
  - **That one subtraction stands in for two things a real sensor
    model needs**, and the comment should say so. A sensor has a
    mounting pose relative to its parent actor, so range is measured
    from where the sensor sits rather than from the car's origin; and a
    detected object has extent, so the measurement runs to where the
    line between them meets the target's body rather than to the
    target's origin. Subtracting one `CAR_LENGTH` collapses both,
    correct only while every car is the same length, the sensor sits at
    the front bumper, and the two are collinear along the road.
  - Doing it properly needs the simulation to publish extents, which it
    does not: `CAR_LENGTH` is a Rust constant here and an invented
    rectangle in the viewer, where a semi truck and a compact car draw
    the same. PLAN.md's "World and map" work is where that arrives, and it
    is what a mounting pose and a traced-line intersection both wait
    on. Both are taken from ground
  truth for now: `range` from the two projected positions and
  `range_rate` from the detected car's published speed minus own,
  read straight out of the pose payloads. That is the simplest thing that
  works while the FMU integration is what M6 is proving, and it keeps the
  sensor rate-independent (one sample per car suffices, so a newly joined
  car appears on the first scan carrying its pose). A realistic sensor
  model replaces the lot later, and may well estimate rate rather than
  read it. The doc comment says so and names what such a model would add:
  a mounting pose on its parent actor, target extents to measure to,
  noise, field of view, occlusion, several returns per vehicle needing
  clustering before anything is an object, and tracking before those
  objects have identity. **One detection per car is itself the
  idealization**, not merely the values inside it. What must survive that
  replacement is the interface, not the arithmetic: relative measurements
  only, since the sensor does not know its own car's speed.
- **The scan is unordered and carries no identity**, which is what a
  detection is: a per-scan measurement, not a tracked object. Slot
  position means nothing across scans, and the type says so, because
  identity would arrive as an ID field from a tracker that does not exist
  here. No consumer wants it anyway: IDM takes the minimum range, and a
  learned model will sort or encode the set permutation-invariantly.
  Deterministic is all the world hash needs, and grouping the inbox by
  publisher gives that for free, with no float tiebreak to specify.

Tests: `a_scan_reports_every_car_ahead_in_lane_exactly_once` (membership,
asserted as a set, since the order is not part of the contract),
`a_car_in_another_lane_is_not_detected`, `cars_behind_are_not_detected`,
`range_rate_is_negative_when_closing_and_positive_when_opening`,
`a_lead_holding_a_steady_gap_reports_zero_range_rate`,
`range_and_range_rate_match_the_known_geometry_of_a_staged_pair` (values,
not sourcing: the tests pin what the sensor reports, leaving how it
derives that free to change),
`a_departed_car_vanishes_from_the_scan_after_its_last_pose`,
`the_first_step_publishes_nothing_rather_than_guessing`,
`two_identical_radar_runs_fingerprint_identically`.

Demo hash: untouched (the component exists but is not registered
anywhere).

DECISIONS titles: The radar keeps no state across steps and needs no
freshness filter; each scan is built from the inbox window alone (and
what the one-scan ghost bound means). A detection is a measurement, not a
tracked object: the scan is unordered, carries no identity, and picking a
lead is the controller's job (determinism comes from the grouping rather
than from a sort, so no float tiebreak exists to get wrong; the consumer
knows what relevance means and a learned one may not want an ordering at
all).

## PR E: traffic drives on the controller FMU (hash move 2 of 3)

- `continuo-examples`: deps gain continuo-fmi and
  continuo-fmu-controller-idm. `traffic_world.rs`: radar and controller
  FMU at 100 ms (declaration order radar > controller > physics chains
  same-instant at every control instant); `RADAR_MAX_RANGE 120.0` and the
  sensor capped at `continuo_actors::MAX_DETECTIONS` (64), the same
  constant the FMU's arrays are built from; `controller_fmu_path()`
  OnceLock over `packaged_fmu_path()`.
- `controller_mapping(world, actor, v0, &UnicycleState)` builds one
  car's `FmuMapping`, the sheet saying which messages feed which FMU
  variables and where its outputs go. Four parts, and they are different
  kinds of thing:
  - **Pose inputs**: `InputBinding`s on the car's `.../pose` key for
    `position.x`, `position.y`, the quaternion components and `speed`.
    Variable name and key only, no JSON Pointers, since the structured
    names derive their own.
  - **Radar inputs**: two `InputBinding`s on the car's `.../radar` key,
    `range` and `range_rate`, each carrying `MAX_DETECTIONS` explicit
    JSON Pointers from the pointer-list helper (`/detections/0/range`
    upward). Spelled out because no derivation reaches them.
  - **Parameters, which are not bindings at all**: `initial_values`,
    name and value pairs set once during initialization and fed by no
    message. The road's points and closed flag read off `shared_road()`
    through the new accessors (plus its point count, in the fallback
    form), so the FMU and the
    native controller steer identical geometry by construction rather
    than by two literals that agree today; then `lane_offset`,
    `lookahead`, `gain`, `max_yaw_rate` and the IDM set; then the spawn
    pose and speed as start values for the input variables.
  - **Outputs**: `accel_cmd` to `accel_cmd_key(world, actor)` and
    `yaw_rate_cmd` to `steer_cmd_key(world, actor)`, the payload field
    name deriving from the variable name.
  - Taking the same `UnicycleState` the plant is built from is the
    point: one spawn state feeds both, so a car cannot be handed to its
    controller and its physics differently. It is needed because at
    instant 0 the controller steps before its physics sibling has
    published anything, so without it the car would steer from the
    origin and accelerate from a standstill it is not in.
  - **The keys are concrete on both sides**, which is why the world name
    is an argument. Native components cannot do this: they wildcard the
    world in `subscriptions()` and reach for `ctx.world_name()` when
    publishing, because a constructor does not know it, and
    `controller.rs` carries a `TODO(PLAN "Scenario configuration")`
    saying to pass it in properly. A scenario does know its world, so
    the FMU path subscribes and publishes exactly, which is a small
    early instance of what that TODO is asking for.
- `add_car` gains `Guidance { ConstantSpeed, IdmFollowing }`:
  `IdmFollowing` registers `[radar, FmuComponent("controller", ..),
  physics]`. Spawned traffic switches to `IdmFollowing` with v0 = spawn
  speed (queues compress to the slowest leader: the new visible physics);
  the ego stays `ConstantSpeed` until PR G; the scale scenario stays
  `ConstantSpeed` through M6 (see PR H).
- highway.rs: `DEMO_WORLD_HASH` moves; README sample output moves in the
  same commit. `a_car_leaves_as_a_whole_actor` expects `[radar,
  controller, physics]`. New: `no_two_same_lane_cars_ever_overlap` (fold
  the recorded log's poses per actor, sample on a grid, assert every
  same-lane pair keeps |delta s| >= CAR_LENGTH; PR G reuses it unchanged)
  and `following_actually_engages_somewhere` (some traffic car's
  published speed range exceeds 1 m/s, guarding against a mapping that
  quietly feeds free-road forever). Turnover assertions expected to
  survive (the ego still overtakes at 30 m/s); re-verify the seed-42
  numbers.
- Contract test: the FMU's output variable names `accel_cmd` and
  `yaw_rate_cmd` match physics's `AccelCmd`/`SteerCmd` decodes, which is
  the one place a rename on either side would pass every other test and
  still deliver nothing.
- Record the demo's steps-per-wall-second before and after for PR H's
  deferred-list note. Refresh the "60 crates" comment on the scaled-world
  smoke if the no-default build's crate count moved.

DECISIONS titles: Traffic drives on the controller FMU and the world hash
moved for it (what joined the fingerprint). The FMU registers as
`controller`, so an FMU car's paths read like a native car's.

## PR F: CollisionMonitor (hash-neutral, wired into the loop)

- `continuo-examples/src/collision.rs`: `CollisionMonitor::new(ego_name,
  road, car_length, lane_tolerance)`; `callback()`/`wrap_transport()`
  mirroring `TrafficRequestHandler`, plus a `membership_callback()`
  shaped like `Recorder`'s and registered through
  `conductor.add_membership_callback`. Poses go into a mutex-held
  `BTreeMap<actor, (SimTime, Pose)>`, and a leave removes that actor,
  keying on the path's first segment. A comment records the asymmetry
  with the radar, whose per-step window needs no such cleanup.
  `check() ->
  Result<(), Collision>`: the pair scan, one replaceable function taking
  per-lane groupings; overlap = same lane band and |delta s| <
  car_length. Traffic-traffic: `tracing::warn!` once per pair-episode
  (active-pair set, separation hysteresis of car_length + 1 m). Ego pair:
  `Err(Collision)`. Episode counter exposed for tests.
- `traffic_world.rs`: `ScenarioError`; `run_live_traffic_scenario` takes
  the monitor (non-optional, the "loop cannot forget" argument) and calls
  `check()` beside `handler.apply`; a `collision_monitor()` factory keeps
  the constants private, and the setup path registers the membership
  callback so a caller cannot wire half a monitor. `tracing` and
  `thiserror` become regular deps of examples. Call-site sweep: five
  example mains, highway.rs helpers, unreadable_request.rs.

Tests: `a_traffic_overlap_warns_once_per_episode_and_does_not_halt` (two
`ConstantSpeed` cars converging in one lane: the old mode earns its keep
as the fixture for exactly the physics the FMU removed),
`an_ego_overlap_halts_the_scenario_naming_both_cars`,
`a_retired_cars_last_pose_cannot_collide` (despawn a car mid-approach
and drive another through where it stood),
`watching_for_collisions_does_not_change_the_world_hash` (equal final
hash with and without the monitor, and the monitor ingested poses, so it
proves something).

Demo hash: unchanged, and asserted so.

DECISIONS title: Collision detection is a transport monitor, warns for
traffic and halts for the ego (outside the sim because poses are already
on the wire and a component would move the hash and the schedule; halting
because the demo's contract is that the follow-controller prevents ego
overlap, so reaching it is a scenario bug worth stopping for).

## PR G: the ego joins the traffic (hash move 3 of 3)

- `TRAFFIC_LANES` gains `EGO_LANE` (0.0); `add_ego` switches to
  `IdmFollowing` (v0 = EGO_SPEED). The four "nothing here models a
  collision" comments: the two demo-side ones rewritten, the
  scale-scenario one stays true and stays, the spawner's is rewritten to
  describe the parameter rather than the world. README narrative
  rewritten; sample output and `DEMO_WORLD_HASH` move together.
- highway.rs assertions retuned to the new story: overlap-free everywhere
  (now covering the ego's lane), the ego's published speed drops
  materially below EGO_SPEED (following engaged), retirement still
  occurs, leaves less than joins, determinism/verify/hash tests
  unchanged. A doc comment on the constants block points at the
  DECISIONS entry below, since that is where someone stands when they
  need the method.

Empirical tuning loop, done by hand: edit a constant, run
`cargo test -p continuo-examples --test highway -- --skip
the_demo_world_hashes --no-fail-fast`, read which assertions moved and
by how much, repeat. The hash pin is skipped because every constant
change moves it, so it fails on every iteration until the end;
`--no-fail-fast` because seeing overlap pass while turnover fails is the
information. A demo run is under half a second, so an iteration is a few
seconds, and the assertion messages already carry their numbers, so
nothing needs scripting.

**The IDM parameters are not knobs.** `T = 1.5 s`, `s0 = 2 m`,
`a_max = 1.5`, `b_comfort = 2.0` are calibrated values from the
literature and `v0` is data, each car getting its spawn speed. If the
demo misbehaves with those, that is a wiring bug rather than a reason to
fit numbers, and fitting them would let a mapping error hide behind
plausible-looking output. Only two values are this project's own rather
than the literature's, and both are design choices with a stated reason
instead of tuning targets: `b_max = 4.0` bounds commanded braking, which
standard IDM does not do at all, at firm rather than emergency (roughly
8); and `GAP_FLOOR` guards a division, its value irrelevant in any
healthy run because reaching it means the cars already overlapped.

Everything that gets tuned is therefore a scenario constant. Expected
failure modes and their knobs, in order: traffic spawns closer than it
wants to be, since at 20 m/s the desired gap is `s0 + v*T` of about
32 m against a `SPAWN_GAP` floor of 20 m, so cars brake from the first
second and the demo opens with a spurious slow-down wave (raise the
floor above the desired headway, do not lower `T`); approach too hot
(30 m/s onto a 16 m/s leader 40 m ahead needs about 2.9 m/s^2 against
b_max 4.0; raise SPAWN_AHEAD to 60-80), turnover dies once the ego locks
to its lane's pace (lane-banded spawn speeds: one slow side lane 14-17,
the others 19-22, so slow-lane cars reliably retire past the ego; widen
TRAFFIC_SPEED; lower RETIRE_BEHIND), thirty seconds of nothing (banding
fixes spawn-luck by construction). Terminate when all assertions are
green, the collision monitor logs zero episodes, the viewer GIF still
shows visible following (one human replay), and the demo still finishes
in a fraction of a wall-second. Then run the wall unfiltered: its one
remaining failure prints the new hash in the hex the constant is written
in and names both files to update, so freezing the hash and README needs
no separate tool.

DECISIONS titles: The ego's lane carries traffic and the ego follows it.
How a deterministic world is retuned, with M6 as the worked example
(determinism makes tuning-to-the-seed reproducible rather than
superstitious; the assertion wall defines done rather than eyeballing
the viewer; the knobs are scenario constants and never the IDM
parameters, which are literature values whose misbehavior would mean a
bug; which predicted failure modes actually appeared and which knob
fixed each; the final numbers).

## PR H: documentation close-out

- **Delete `M6-PLAN.md`.** It was scaffolding, and this is the PR that
  reconciles it: what it decided is in DECISIONS entries PR by PR, what
  it designed is in the PLAN.md rewrite below, and what its experiments
  found is in whichever of the two owns that answer. Read it once more
  before deleting, so nothing it settled goes unrecorded. The tuning
  method is the piece most easily lost, so check it reached the PR G
  DECISIONS entry in full, worked example included, before this file
  goes.
- PLAN.md "FMI 3.0 CS support" rewritten as-built: `fmi` 0.8 adopted
  (bindgen/libclang cost recorded), the demo FMU is the project-built car
  controller with reference FMUs as importer fixtures, mapping is a Rust
  struct until the scenario-configuration work brings files, zip arrived
  via `fmi`. The section's "native arrays, `float64` value references"
  line is now half honoured and half superseded: native arrays are how the
  radar scan reaches the controller, and the adapter binds every FMI 3.0
  numeric type rather than float64 alone. Workspace layout gains
  `continuo-fmu-controller-idm`. Milestone list item 6 refreshed; README
  M6 checkbox ticked.
- PLAN.md Deferred gains evidence with the PR E measurement: "latest
  message per key" (a radar reads the newest pose per car in range per
  scan; a scaled world with radars is O(cars^2) message deliveries per
  second) and "consolidated scene view" (a radar is precisely a consumer
  of latest-poses, and the collision monitor's pair scan wants the same
  spatial index: a many-actor bottleneck candidate). A sentence on why
  traffic_scale kept the two-component car in M6 (it measures conductor
  cost per step; per-car radar changes the message complexity class and
  an FMU per car adds instantiation, each its own experiment); a comment
  on `COMPONENTS_PER_CAR = 2` saying the pre-FMU car is on purpose.
- PLAN.md Deferred gains a new item: **the plant should publish its full
  ground-truth kinematic state**, not a pose that has quietly grown a
  speed field. Actual acceleration is the first thing M6 leaves
  unobservable (it differs from the commanded value wherever the zero
  clamp bites), yaw rate would follow, and the honest move at that point
  is renaming the message rather than stretching `pose` a third time,
  since the key is `.../pose` and the Python viewer's
  `pose_from_payload` reads it as one. Names the rename as the cost, so
  it is a deliberate act rather than a surprise.
- PLAN.md Deferred gains a new item: **FMI 3.0 Event Mode**, which M6
  switches off (`event_mode_used = false`, so
  `ExitInitializationMode` lands in Step Mode and an FMU handles its own
  events inside a step). What turning it on would take, so the size is
  known rather than guessed:
  - Read `hasEventMode` from the `<CoSimulation>` element rather than
    taking a config flag, the same principle as reading a variable's
    declared type. Startup grows: with event mode on, initialization
    exits into Event Mode, so the adapter iterates
    `fmi3UpdateDiscreteStates` until it settles, then calls
    `fmi3EnterStepMode` before the first step.
  - After a step reporting `eventHandlingNeeded`: `EnterEventMode`,
    iterate `UpdateDiscreteStates` until `discreteStatesNeedUpdate`
    clears, `EnterStepMode`, resume. Cap the iterations and fail as
    `ComponentFailure`, since an unbounded loop inside one step is a
    hang the conductor can only catch by timeout. `terminateSimulation`
    from that call is handled exactly as the `do_step` path handles it.
  - **`nextEventTime` becomes `next_due`**, as `min(now + period,
    next_event_time)`. This is the part that fits: self-reported
    next-step times are what the scheduler is built on, so an FMU's
    internal events become ordinary scheduling and land on step
    boundaries instead of inside them.
  - Early return needs a decision, and the answer is to keep the
    `Component` contract: `do_step` may stop at a `lastSuccessfulTime`
    short of the requested end, so the adapter loops until it reaches
    `ctx.now()` and early return stays invisible to the conductor. With
    `nextEventTime` scheduling it should be rare, since predictable
    events already land on boundaries.
  - Clocked FMUs are separable and larger: they add the Clock type to
    the dispatch plus the interval and shift APIs. An FMU with plain
    state events needs none of it, so defer until one wants it.
  - Two fixtures, covering both kinds of event: BouncingBall's bounce
    is a state event whose time is not predictable, so it exercises the
    handling loop and early return, while Stair generates a time event
    every second, which is what `nextEventTime` scheduling is for.
    Stair would be vendored at that point; it is not now, because with
    event mode off it only shows a counter changing at second
    boundaries.
- PLAN.md Deferred gains a small one: **split the control laws out of
  `continuo-actors` into their own crate.** M6 leaves that crate holding
  two kinds of thing, sample components and the library an FMU compiles
  into a `.dll`, so an FMU shell links the spawner and the logger to
  reach three functions. The trigger is the second FMU crate, since
  `continuo-fmu-controller-ai` reaching in as well confirms the edge is
  structural rather than incidental; by then the `control_laws` module
  makes it a file move and a Cargo.toml.
  - The naming question comes with it, and turns on whether `Waypoints`
    goes too. A laws-only crate is `continuo-control-laws` or
    `continuo-control`, with the road geometry moving instead to
    `continuo-core`, where `Vec3` and `Quat` already live and where
    PLAN.md's "World and map" work is heading anyway. A crate carrying
    both wants a domain name covering geometry as well as laws, since
    `project` and `point_at_offset` are not control. What to avoid
    either way is a generic name: a crate called `algorithms` or
    `utils` excludes nothing and so accumulates everything.
  - Internal subdirectories are not the answer, since `pub use` hides
    layout from consumers and the dependency edge would be unchanged.
- PLAN.md Deferred, existing items gain what M6 learned: the
  large-payload item now has a concrete first case, since FMU Binary
  variables travel base64 in JSON and would travel natively under CBOR,
  which is also the thing that would earn core a `PayloadValue` of its
  own; and the binary-mode item can note that `continuo-fmi` is already
  format-agnostic, resolving against decoded values rather than bytes.
- DECISIONS title: M6 landed with three deltas from the plan's FMI
  section.

## Risks

- **Array and type coverage upstream**, verified in PR A and PR B before
  the demo depends on either. The import side must set and get arrays
  (`nValues` larger than the value-reference count), enter configuration
  mode for structural parameters, and handle every family the adapter
  claims, including String and Binary with their size-array and lifetime
  rules. The derive must declare dimensions.
  - String and Binary are the likeliest gaps and the cheapest to absorb,
    since nothing in the demo binds them: a missing family becomes a
    documented hole that fails at construction, not a blocked milestone.
  - If arrays cannot be declared or set, the FMU falls back to N scalar
    variables (`range_0`, `range_1`, and so on) and **the mapping does
    not care**, because a binding is a list of JSON Pointers either way.
    Only the FMU crate changes, and the plan loses the word "native".
- **fmi-export maturity** (0.3.0, young): the PR B golden tests on four
  platforms are the acid test, run before anything depends on it.
  Fallback if CS export is broken: hand-write the ~10 fmi3* exports in
  the shell cdylib; the import side, mapping, and demo wiring are
  unaffected by how the FMU was authored.
- **libclang missing on a dev machine**: CI images ship LLVM; README note
  covers local setup; PR A fails at build with bindgen's clear error.
- **Windows tempdir vs loaded DLL**: field order pins drop order; the
  lifecycle test fails on regression.
- **Cross-platform FMU arithmetic**: IDM adds no trig; pure pursuit's
  atan2/sin/cos move into the FMU but are the same rustc-compiled code
  the native controller runs, so the exposure class is unchanged and the
  four-agent DEMO_WORLD_HASH check remains the tripwire.
- **`fmi` logs via `log`, not `tracing`**: tracing-subscriber's default
  tracing-log feature should bridge in `fmt().init()`; verify once in
  PR A, else one `LogTracer::init()` in example mains.
- **PR G turnover collapse**: the tuning loop, with lane-banded speeds as
  the strong knob; assertions written against behaviors, not counts,
  wherever possible.

## Verification (every PR)

`cargo fmt --all --check`; `cargo clippy --workspace --all-targets -- -D
warnings`; `RUSTDOCFLAGS=-D warnings cargo doc --workspace --no-deps`;
`cargo xtask package-fmus` (from PR B on);
`cargo test --workspace`; demo smoke `cargo run -p continuo-examples
--example traffic`. Hash discipline: PRs A, B, D, F, H leave
`DEMO_WORLD_HASH` untouched (asserted by the existing test); PRs C, E, G
move it deliberately, README in the same commit, a DECISIONS entry
recording old to new. traffic_scale is unchanged by construction through
M6, so its printed hashes and throughput should hold; the demo world's
throughput is measured and recorded in the PR E description.

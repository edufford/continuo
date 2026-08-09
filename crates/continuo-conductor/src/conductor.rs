use std::collections::BTreeMap;
use std::time::{Duration, Instant};

use continuo_core::{
    Component, ComponentPath, HashFnv1a64, Message, SimDuration, SimTime, StepCtx, TickDone,
    TickStart, hash_bytes, mix_seeds,
};
use continuo_transport::Transport;
use tracing::{trace, warn};

use crate::config::ConductorConfig;
use crate::error::ConductorError;
use crate::membership::{JoinMetadata, LeaveMetadata};
use crate::pacing::{Pacer, Pacing, SystemClock};
use crate::record::{
    MembershipChange, RecordedBudgetMiss, RecordedJoin, RecordedLeave, RecordedObservation,
    RecordedTimeout, TickFingerprint,
};
use crate::registry::Registry;
use crate::schedule::Schedule;
use crate::timing::{OnTimeout, StepTiming, diagnostic_millis};

type TickCallback = Box<dyn FnMut(&TickFingerprint) + Send>;
type MembershipCallback = Box<dyn FnMut(&MembershipChange) + Send>;
type ObservationCallback = Box<dyn FnMut(&RecordedObservation) + Send>;

/// Owns simulation time and drives the discrete-event loop over a
/// [`Transport`].
pub struct Conductor<T: Transport> {
    config: ConductorConfig,
    transport: T,
    registry: Registry,
    schedule: Schedule,
    /// `Some` in 1× real-time mode, gating each instant to wall time;
    /// `None` in free-run. Pacing never affects `world_hash`.
    pacer: Option<Pacer<SystemClock>>,
    /// Leaves declared for a future instant, applied at the tick
    /// boundary before that instant is stepped. Sorted by `leaves_at`, so
    /// draining the front is enough to find the ones that have come due.
    pending_leaves: BTreeMap<SimTime, Vec<ComponentPath>>,
    sim_time: SimTime,
    tick: u64,
    /// Running determinism fingerprint: seeded from the world config, then
    /// chained with each tick's hash. Identical runs produce identical
    /// values at every tick.
    world_hash: u64,
    /// Observers, in the order they were added. Lists rather than single
    /// slots because more than one observer legitimately wants the same
    /// hookup: recording a run while watching it, for instance. A slot that
    /// silently kept only the last registration turned that into a log
    /// quietly missing a channel.
    tick_callbacks: Vec<TickCallback>,
    membership_callbacks: Vec<MembershipCallback>,
    observation_callbacks: Vec<ObservationCallback>,
}

impl<T: Transport> Conductor<T> {
    pub fn new(config: ConductorConfig, transport: T) -> Result<Self, ConductorError> {
        // 1× real-time pacing (PLAN.md "Pacing") gates each instant to wall
        // time; free-run leaves the pacer off and advances immediately. The
        // spin padding only picks how the wait is spent (OS sleep vs.
        // sleep-then-spin), never what happens in the instant.
        let pacer = match config.pacing {
            Pacing::FreeRun => None,
            Pacing::RealTime { spin_padding } => Some(Pacer::new(SystemClock::new(spin_padding))),
        };
        // Fold the seed and world name into the initial hash so runs with
        // different seeds have different fingerprints even before (or
        // without) any component using randomness.
        let world_hash = mix_seeds(config.world_seed, hash_bytes(config.world_name.as_bytes()));

        // Return a conductor at sim time zero with an empty schedule.
        Ok(Conductor {
            config,
            transport,
            registry: Registry::default(),
            schedule: Schedule::default(),
            pacer,
            pending_leaves: BTreeMap::new(),
            sim_time: SimTime::ZERO,
            tick: 0,
            world_hash,
            tick_callbacks: Vec::new(),
            membership_callbacks: Vec::new(),
            observation_callbacks: Vec::new(),
        })
    }

    // TODO(M7): these three adders plus the `MonitorTransport` wrap are four
    // hookups a caller must remember, and forgetting one writes a log that
    // quietly omits a channel rather than failing. Wanted: an `Observer` trait
    // with no-op defaults and one `add_observer`, so a new observation point
    // cannot be silently skipped.
    //
    // Waits for M7 because messages are observed at the *transport*, wrapped
    // before the conductor exists, and where that seam belongs is the same
    // question as the hash fold (PLAN.md, "What `step_once` becomes").

    /// Adds a callback invoked with every tick's [`TickFingerprint`].
    /// This is the hook for recording (see
    /// [`crate::Recorder::tick_callback`]) or live divergence checking.
    ///
    /// Observers accumulate, and every one added is invoked in the order it
    /// was added. Adding never displaces an earlier observer, so recording a
    /// run and watching it are not mutually exclusive.
    pub fn add_tick_callback(&mut self, callback: impl FnMut(&TickFingerprint) + Send + 'static) {
        self.tick_callbacks.push(Box::new(callback));
    }

    /// Adds a callback invoked whenever a component joins or leaves.
    /// The third observation point, alongside published messages and tick
    /// fingerprints, and the one that makes a dynamic run recordable
    /// (see [`crate::Recorder::membership_callback`]).
    ///
    /// Accumulates, in the order added, like [`Self::add_tick_callback`].
    pub fn add_membership_callback(
        &mut self,
        callback: impl FnMut(&MembershipChange) + Send + 'static,
    ) {
        self.membership_callbacks.push(Box::new(callback));
    }

    /// Adds a callback invoked for everything the *machine* did rather
    /// than the run: steps over their budget, and the timeouts that say why
    /// a component left or a run stopped (see
    /// [`crate::Recorder::observation_callback`]).
    ///
    /// The fourth observation point, and the one whose reports a re-run is
    /// free to differ on. See [`RecordedObservation`].
    ///
    /// Accumulates, in the order added, like [`Self::add_tick_callback`].
    pub fn add_observation_callback(
        &mut self,
        callback: impl FnMut(&RecordedObservation) + Send + 'static,
    ) {
        self.observation_callbacks.push(Box::new(callback));
    }

    /// Reports an applied membership change to every observer.
    fn emit_membership(&mut self, change: MembershipChange) {
        for callback in self.membership_callbacks.iter_mut() {
            callback(&change);
        }
    }

    /// Reports an observation to every observer. An observation is
    /// something worth noting about the run rather than a part of it.
    fn emit_observation(&mut self, observation: RecordedObservation) {
        for callback in self.observation_callbacks.iter_mut() {
            callback(&observation);
        }
    }

    /// The current running determinism fingerprint (see PLAN.md,
    /// Determinism rules). Two runs of the same seeded scenario must agree
    /// on this value at every tick.
    pub fn world_hash(&self) -> u64 {
        self.world_hash
    }

    pub fn world_name(&self) -> &str {
        &self.config.world_name
    }

    pub fn sim_time(&self) -> SimTime {
        self.sim_time
    }

    /// The earliest scheduled due time, if anything remains scheduled. This
    /// lets callers drive `step_once` themselves (e.g. live replay checking
    /// that stops at the first divergence).
    pub fn next_scheduled(&self) -> Option<SimTime> {
        self.schedule.earliest()
    }

    /// Total steps executed so far (each instant with due components is one
    /// tick).
    pub fn tick(&self) -> u64 {
        self.tick
    }

    /// How many times the run fell far enough behind real time that the
    /// wall-clock anchor gave up and slipped, in 1× real-time mode; always
    /// 0 in free-run.
    ///
    /// This measures **the schedule as a whole against the wall clock**,
    /// not components. Zero does *not* mean every component finished
    /// quickly, or that anything met a deadline: lateness under the
    /// re-anchor threshold is deliberately absorbed, and a component that
    /// runs long stays invisible here as long as the run recovers before
    /// the threshold. [`Self::budget_misses`] answers the question this
    /// metric cannot: did *this* component finish within its time.
    pub fn overrun_reanchor_count(&self) -> u64 {
        self.pacer.as_ref().map_or(0, Pacer::overrun_reanchor_count)
    }

    /// How many of this component's steps have run over the wall-clock
    /// budget it declared when it joined; `None` if nothing is registered at
    /// `path`, and always 0 for a component that declared no budget.
    ///
    /// Attributable by construction, which is the point of it: it counts
    /// one component's own `step` calls, not lateness the schedule
    /// accumulated around it (see [`Self::overrun_reanchor_count`]). Purely
    /// diagnostic: missing a budget changes nothing about the run, so this
    /// can differ between two runs with the identical world hash.
    ///
    /// Counted against the live component, so it resets if a path is
    /// vacated and reoccupied. The newcomer is a different component that
    /// happens to share a name.
    pub fn budget_misses(&self, path: &ComponentPath) -> Option<u64> {
        // Return the live component's miss count, if one is registered here.
        let index = self.registry.index_of(path)?;
        self.registry.entry(index).map(|entry| entry.budget_misses)
    }

    /// Total wall-clock time the run has permanently fallen behind real
    /// time, summed over the slips counted by
    /// [`Self::overrun_reanchor_count`] (0 in free-run, or when 1× pacing
    /// kept up). Lateness that was absorbed rather than slipped is not
    /// included, because by definition it was recovered.
    pub fn total_slip(&self) -> std::time::Duration {
        self.pacer
            .as_ref()
            .map_or(std::time::Duration::ZERO, Pacer::total_slip)
    }

    /// The earliest instant a component can still be admitted for. Nothing
    /// has stepped before the first tick, so the world's opening instant is
    /// open; afterwards the instant at `sim_time` has been executed and
    /// scheduling into it would step it a second time.
    fn earliest_open_instant(&self) -> SimTime {
        // Return the first instant that has not already happened.
        if self.tick == 0 {
            self.sim_time
        } else {
            self.sim_time + SimDuration::from_nanos(1)
        }
    }

    /// Admits a component. Pass a [`JoinMetadata`] to say when a newcomer to
    /// a running world first steps, or, before the run starts, just the
    /// parent path ([`WORLD_LEVEL`](crate::WORLD_LEVEL) for a world-level
    /// actor, `"car1"` to join that composite), which is shorthand for first
    /// stepping at sim time zero.
    ///
    /// Sibling order is arrival order, which fixes both the execution order
    /// within an instant and the "earlier sibling" of the visibility rule.
    /// A component admitted mid-run is therefore the newest sibling of
    /// whatever it joins.
    ///
    /// The first step is scheduled here, as the component is admitted,
    /// rather than discovered when the instant arrives, so the barrier at
    /// `first_due` already counts the newcomer among the components it
    /// waits for.
    ///
    /// Joining is also where a component says what its steps may cost in
    /// wall time, if anything: see [`JoinMetadata::with_timing`] and
    /// [`StepTiming`](crate::StepTiming).
    // TODO(M7): joining stays a direct call until a remote host can own and
    // step the component it admits, for the reason `membership` documents: a
    // join hands over a `Box<dyn Component>`, which no transport can carry.
    pub fn add_component(
        &mut self,
        join: impl Into<JoinMetadata>,
        component: Box<dyn Component>,
    ) -> Result<ComponentPath, ConductorError> {
        let join = join.into();
        let parent_path = ComponentPath::parse(&join.parent_path)?;

        // Validate before touching anything, so a rejected join leaves no
        // trace: no half-registered entry, no stray subscriptions.
        let earliest_open = self.earliest_open_instant();
        if join.first_due < earliest_open {
            return Err(ConductorError::JoinInThePast {
                path: parent_path.join(component.id()),
                first_due: join.first_due,
                earliest_open,
            });
        }
        if let Some((budget, timeout)) = join.timing.unreachable_budget() {
            return Err(ConductorError::UnreachableStepBudget {
                path: parent_path.join(component.id()),
                budget,
                timeout,
            });
        }

        let subscriptions = component.subscriptions();
        let (index, path) =
            self.registry
                .add(&parent_path, component, self.config.world_seed, join.timing)?;
        for key in subscriptions {
            self.transport.subscribe(path.clone(), key);
        }
        self.schedule.insert(join.first_due, index);
        self.emit_membership(MembershipChange::Joined(RecordedJoin {
            path: path.to_string(),
            first_due: join.first_due,
        }));

        // Return the registered component's full path.
        Ok(path)
    }

    /// Applies every leave due at or before `instant`, in declared
    /// order. Called at the tick boundary, before anything steps, so a
    /// component that has left takes no part in the instant it left at.
    fn apply_due_leaves(&mut self, instant: SimTime) {
        while let Some(entry) = self.pending_leaves.first_entry() {
            if *entry.key() > instant {
                break;
            }
            let leaves_at = *entry.key();
            for path in entry.remove() {
                // A path may have been removed directly in the meantime;
                // a leave that finds nothing to remove is already satisfied.
                if self.registry.index_of(&path).is_some() {
                    self.apply_leave(&path, leaves_at);
                }
            }
        }
    }

    /// Deregisters a component and tells observers. The one place a
    /// leave is applied, whichever route asked for it.
    ///
    /// `leaves_at` is the instant recorded as the component's last: the
    /// declared one for a scheduled leave, or the earliest still-open
    /// instant for an immediate removal, which is when it stops either way.
    fn apply_leave(&mut self, path: &ComponentPath, leaves_at: SimTime) {
        let Some(index) = self.registry.remove(path) else {
            return;
        };
        self.schedule.remove_index(index);
        self.transport.unsubscribe(path);
        self.emit_membership(MembershipChange::Left(RecordedLeave {
            path: path.to_string(),
            leaves_at,
        }));
    }

    /// Removes a component. Pass a [`LeaveMetadata`] to name the instant it
    /// stops at, or just its path to stop it immediately, at this tick
    /// boundary, since membership never changes mid-tick.
    ///
    /// Prefer naming the instant for anything a run must reproduce: the
    /// bare-path form stops the component wherever the caller happens to
    /// be, which is deterministic only because the caller is. A named
    /// instant gives the same run whenever the request was made.
    ///
    /// The component stops being scheduled, stops receiving messages, and
    /// is dropped. Everything it published stays published, because
    /// departing is not a rollback.
    ///
    /// Survivors are untouched. Their declaration indexes do not shift, so
    /// execution order within an instant and every "earlier sibling"
    /// relationship the visibility rule depends on are exactly as they were.
    ///
    /// The path becomes free again; a later component may take it and
    /// arrives as a new sibling, ordered by its arrival like any other.
    ///
    /// **A composite's path takes its whole subtree.** `"car1"` removes
    /// every leaf under it (`car1/controller`, `car1/physics`, and anything
    /// nested below), because an actor leaving a world leaves whole, and
    /// removing only some of its parts would leave a controller publishing
    /// at a physics model that is gone.
    ///
    /// The log records **one leave per leaf**, never one for the composite.
    /// Every join names a leaf, because a leaf is what joins, so departures
    /// stay symmetrical with arrivals and nothing reading the log needs to
    /// know the shape of the tree. They go out in declaration order, which
    /// is the order those components step in.
    // TODO(M7): departure over the transport
    // (`continuo/{world}/conductor/leave`) waits on distribution too, though
    // for a weaker reason than the join above: `LeaveMetadata` is a path and
    // an instant, so it could cross a transport today. There is simply
    // nobody to send it. Everything in this process holds `&mut Conductor`
    // and calls this.
    pub fn remove_component(
        &mut self,
        leave: impl Into<LeaveMetadata>,
    ) -> Result<(), ConductorError> {
        let leave = leave.into();
        let path = ComponentPath::parse(&leave.path)?;
        // One leaf, or every leaf of a composite. Checking for a registered
        // leaf first means a leaf always wins: a path cannot be both, since
        // `Registry::add` refuses to make a leaf into a composite or the
        // reverse.
        let departing = if self.registry.index_of(&path).is_some() {
            vec![path]
        } else {
            let subtree = self.registry.components_under(&path);
            if subtree.is_empty() {
                return Err(ConductorError::UnknownPath(path));
            }
            subtree
        };
        let earliest_open = self.earliest_open_instant();

        let Some(leaves_at) = leave.leaves_at else {
            // Unnamed: stop them at the earliest instant still open, which
            // is now, so there is nothing to defer.
            for path in &departing {
                self.apply_leave(path, earliest_open);
            }
            return Ok(());
        };
        if leaves_at < earliest_open {
            // The instant it would stop at has already been stepped, so the
            // component has already done work this leave claims it did
            // not. Refuse rather than silently apply it late.
            let path = departing.into_iter().next().expect("non-empty above");
            return Err(ConductorError::LeaveInThePast {
                path,
                leaves_at,
                earliest_open,
            });
        }
        // Queued in declaration order, and `apply_due_leaves` drains the
        // vector in order, so the log's leaves come out in the order the
        // components would have stepped.
        self.pending_leaves
            .entry(leaves_at)
            .or_default()
            .extend(departing);

        // Return success; the leaves apply at the boundary before that
        // instant is stepped.
        Ok(())
    }

    /// Advances to the earliest due instant and steps every component due at
    /// it, in declaration order. Returns `false` when nothing is scheduled.
    pub fn step_once(&mut self) -> Result<bool, ConductorError> {
        // Remove anyone whose declared leave has come due before the
        // instant is entered, so a component that leaves at T takes no part
        // in T. This is the tick boundary: membership settles here, and is
        // frozen for the rest of the tick.
        //
        // Peek rather than pop, because popping takes the due set *and*
        // the instant with it, leaving the leave nothing to affect: the due
        // set would already list the departing component, and an instant
        // holding only that component could no longer be pruned, so it
        // would still become a tick, numbered and fingerprinted and chained
        // into the world hash for an instant where nobody stepped.
        if let Some(next) = self.schedule.earliest() {
            self.apply_due_leaves(next);
        }
        let Some((now, due)) = self.schedule.pop_earliest() else {
            return Ok(false);
        };
        debug_assert!(now >= self.sim_time, "schedule went backwards");
        self.sim_time = now;
        self.tick += 1;

        // In 1× real-time mode, wait for this instant's wall-clock target
        // before doing any of its work. This only delays entry to the
        // instant; it never changes what happens in it, so the world hash
        // is identical to a free run.
        if let Some(pacer) = self.pacer.as_mut() {
            pacer.pace(now);
        }

        // Per-tick determinism fingerprint: covers, in declaration order,
        // every stepped component's path, next-due time, published bytes,
        // and (when provided via `state_bytes`) internal state.
        let mut tick_hasher = HashFnv1a64::new();
        tick_hasher.write_u64(self.tick);
        tick_hasher.write_i64(now.as_nanos());

        // TODO(M7): distributed, this is published on the transport and hosts
        // step themselves against it, with the conductor barriering on
        // TickDone acks. PLAN.md, "What `step_once` becomes", works out what
        // that does to this function. The prerequisite is making the tick hash
        // a fold of per-component sub-hashes, since the conductor never sees a
        // remote component's published bytes.
        let tick_start = TickStart {
            tick: self.tick,
            sim_time: now,
        };
        trace!(target: "continuo::conductor", tick = tick_start.tick, sim_time = %now, "tick start");

        for index in due {
            // Membership only changes at tick boundaries, so every due slot
            // is live. Assert that, then skip rather than panic if it ever
            // stops holding: not stepping is the right outcome for a
            // component that has left.
            let Some(entry) = self.registry.entry(index) else {
                debug_assert!(false, "slot {index} vacated mid-tick");
                continue;
            };
            let path = entry.path.clone();
            let dt = entry.last_step.map(|prev| now - prev);
            let component_seed = entry.component_seed;
            let timing = entry.timing;

            // The visibility rule (PLAN.md): everything published before this
            // instant is released; same-instant messages only from an
            // earlier-ordered sibling branch within the same composite.
            let tree = &self.registry.tree;
            let release_condition = |m: &Message| {
                m.time < now || (m.time == now && tree.releases_same_instant(&m.publisher, &path))
            };
            let inbox = self.transport.drain(&path, &release_condition);

            let mut ctx = StepCtx::new(now, dt, &self.config.world_name, component_seed, inbox);
            let entry = self
                .registry
                .entry_mut(index)
                .expect("checked live above, and membership is frozen mid-tick");
            // Time the component's own step. In-process this one duration
            // answers both declared levels, because the conductor's wait for
            // a synchronous call is the call. They part company only when a
            // transport gets between them (see `timing`). The verdict waits
            // until the step's effects below have been applied; see the end
            // of this loop body.
            let started = Instant::now();
            // A component saying it cannot do its job halts the world, and
            // does so here, before the outbox below is applied, so a failed
            // step publishes nothing. Same reasoning as the schedule
            // violation further down: the failure is a pure function of the
            // component's logic and the sim state, so it reproduces exactly
            // and halting cannot introduce divergence.
            let next_due =
                entry
                    .component
                    .step(&mut ctx)
                    .map_err(|source| ConductorError::StepFailed {
                        path: entry.path.clone(),
                        now,
                        source,
                    })?;
            let step_wall = started.elapsed();

            // A schedule violation always halts, whatever the timeout policy
            // says. Unlike a timeout it does not depend on the wall clock.
            // It is a pure function of the component's logic and the sim
            // state, so it reproduces exactly, and removing the component
            // would trade a loud, reproducible bug for a silent scenario
            // change. There is no next due time to carry on from either.
            // TODO(M7): a component panicking is the third failure at the
            // barrier, and PLAN.md treats it like a timeout. Catching one
            // needs the host boundary an out-of-process component has, so it
            // lands with distribution.
            if next_due <= now {
                return Err(ConductorError::ScheduleViolation {
                    path: entry.path.clone(),
                    now,
                    next_due,
                });
            }

            // The conductor (not the component) turns outbox entries into
            // published Messages so that the authoritative metadata
            // (publisher identity, per-publisher seq, and timestamp) is
            // stamped centrally. Components stay transport-blind and cannot
            // misattribute or reorder their own traffic, which the
            // deterministic (publisher, seq) delivery order depends on. In
            // distributed mode the component's *host* plays this role.
            tick_hasher.write(path.to_string().as_bytes());
            tick_hasher.write_i64(next_due.as_nanos());
            for (key, payload) in ctx.take_outbox() {
                let seq = entry.next_seq;
                entry.next_seq += 1;
                tick_hasher.write(key.as_str().as_bytes());
                tick_hasher.write_u64(seq);
                tick_hasher.write(&payload);
                self.transport.publish(Message {
                    key,
                    publisher: path.clone(),
                    seq,
                    time: now,
                    payload,
                });
            }

            // Components exposing internal state join the hash in
            // state-hash mode; the rest are covered by their output bytes
            // above (output-hash mode). The b"|state|" marker below
            // separates state bytes from the payload bytes they follow:
            // without it, two runs over the same concatenation but a
            // different split (published "ab" with state "c" vs. published
            // "a" with state "bc") hash alike, and a divergence that only
            // moves the boundary would go unseen.
            // TODO(PLAN "Deferred"): replace this marker with a byte-length
            // prefix on every variable-length field. A separator is unsound
            // (a payload may contain the marker) and guards only this one
            // boundary: a payload also runs straight into the next message's
            // key, and the last payload of a component into the next
            // component's path, neither of which is separated at all.
            if let Some(state) = entry.component.state_bytes() {
                tick_hasher.write(b"|state|");
                tick_hasher.write(&state);
            }
            entry.last_step = Some(now);
            self.schedule.insert(next_due, index);

            // TickDone is the step-completed ack of the tick protocol: it
            // both closes the barrier for this component and carries its
            // next_due, which is how the schedule learns when to wake it
            // again. In-proc both facts arrive via step()'s return value, so
            // TickDone exists here as a trace event; in distributed mode
            // (M7) it is the actual message a remote host sends back and the
            // barrier blocks on.
            let tick_done = TickDone {
                tick: self.tick,
                component_id: path
                    .segments()
                    .last()
                    .expect("leaf path is non-empty")
                    .clone(),
                next_due,
            };
            trace!(
                target: "continuo::conductor",
                tick = tick_done.tick,
                component = %path,
                next_due = %tick_done.next_due,
                "tick done"
            );

            // Per-component timing (PLAN.md, "Per-component timing"). Both
            // levels are judged, not just the worse one: they read
            // different quantities, the step itself and how long the
            // conductor waited to hear about it, which happen to be the
            // one measurement in-process, and are two the moment a
            // transport gets between them. Hence one call each, with
            // `step_wall` standing in for both. The soft one goes first so
            // a run that is about to halt still records what it observed.
            self.judge_step_budget(&path, now, step_wall, &timing);
            self.judge_step_timeout(&path, now, step_wall, &timing)?;
        }

        // Chain this tick into the running world hash and emit the
        // fingerprint.
        let tick_hash = tick_hasher.finish();
        let mut chain = HashFnv1a64::resume(self.world_hash);
        chain.write_u64(tick_hash);
        self.world_hash = chain.finish();
        if !self.tick_callbacks.is_empty() {
            let fingerprint = TickFingerprint {
                tick: self.tick,
                sim_time: now,
                tick_hash,
                world_hash: self.world_hash,
            };
            for callback in self.tick_callbacks.iter_mut() {
                callback(&fingerprint);
            }
        }

        // Return true: a tick was executed (more may be scheduled).
        Ok(true)
    }

    /// Judges how long a component's own `step` took against the **budget**
    /// it declared (PLAN.md, "Per-component timing").
    ///
    /// Soft, permanently: it counts the miss, says so, and returns. Nothing
    /// about the run changes, which is what makes the level safe to measure
    /// on whichever machine the step ran on, and so what keeps it harmless
    /// once components are distributed and `step` no longer runs here.
    /// Returns nothing because there is no verdict to act on.
    fn judge_step_budget(
        &mut self,
        path: &ComponentPath,
        now: SimTime,
        step: Duration,
        timing: &StepTiming,
    ) {
        if !timing.over_budget(step) {
            return;
        }
        let budget = timing.budget.expect("over a budget, so one was set");
        // Found by path rather than by an index threaded in from the caller.
        // The lookup only happens on a miss, which is the diagnostic path.
        let entry = self
            .registry
            .entry_mut_by_path(path)
            .expect("just stepped, and membership is frozen mid-tick");
        entry.budget_misses += 1;
        warn!(
            target: "continuo::timing",
            component = %path,
            sim_time = %now,
            step_ms = diagnostic_millis(step),
            budget_ms = diagnostic_millis(budget),
            misses = entry.budget_misses,
            "step over its budget (diagnostic only; the run is unaffected)"
        );
        self.emit_observation(RecordedObservation::BudgetMissed(RecordedBudgetMiss {
            path: path.to_string(),
            sim_time: now,
            step_ms: diagnostic_millis(step),
            budget_ms: diagnostic_millis(budget),
        }));
    }

    /// Judges how long the conductor **waited** to hear that a component
    /// had stepped against the **timeout** it declared (PLAN.md,
    /// "Per-component timing").
    ///
    /// Hard: the declared policy either ends the run or takes the component
    /// out of the next tick. Neither touches this one, since the step has
    /// already happened and everything it did stands, so the tick
    /// fingerprints exactly as it would have anyway.
    fn judge_step_timeout(
        &mut self,
        path: &ComponentPath,
        now: SimTime,
        waited: Duration,
        timing: &StepTiming,
    ) -> Result<(), ConductorError> {
        if timing.over_timeout(waited) {
            let timeout = timing.timeout.expect("over a timeout, so one was set");
            warn!(
                target: "continuo::timing",
                component = %path,
                sim_time = %now,
                waited_ms = diagnostic_millis(waited),
                timeout_ms = diagnostic_millis(timeout),
                policy = ?timing.on_timeout,
                "timed out waiting for its step"
            );
            // Record *why*, before the policy is applied. The leave a
            // removal produces is deliberately indistinguishable from a
            // scripted one, so that a replay which asks for the same leave
            // matches; this observation is what carries the reason, and for
            // a halt it is the only trace the log gets.
            self.emit_observation(RecordedObservation::TimedOut(RecordedTimeout {
                path: path.to_string(),
                sim_time: now,
                waited_ms: diagnostic_millis(waited),
                timeout_ms: diagnostic_millis(timeout),
                policy: timing.on_timeout,
            }));
            match timing.on_timeout {
                OnTimeout::Halt => {
                    return Err(ConductorError::StepTimeout {
                        path: path.clone(),
                        now,
                        elapsed: waited,
                        timeout,
                    });
                }
                OnTimeout::Remove => {
                    // Queue it exactly as a leave declared for the next open
                    // instant, so it goes out through the one path that
                    // removes a component: applied at the coming tick
                    // boundary, and recorded like any other leave. It keeps
                    // the tick it timed out in, because membership is frozen
                    // for the whole of a tick.
                    let leaves_at = self.earliest_open_instant();
                    self.pending_leaves
                        .entry(leaves_at)
                        .or_default()
                        .push(path.clone());
                }
            }
        }

        // Return Ok when the wait was within the timeout, or when the policy
        // was to remove rather than halt: either way the run continues.
        Ok(())
    }

    /// Runs until the earliest scheduled instant would exceed `end`
    /// (inclusive: an instant exactly at `end` is executed).
    pub fn run_until(&mut self, end: SimTime) -> Result<(), ConductorError> {
        while let Some(earliest) = self.schedule.earliest() {
            if earliest > end {
                break;
            }
            self.step_once()?;
        }

        // Return once nothing remains scheduled at or before `end`.
        Ok(())
    }
}

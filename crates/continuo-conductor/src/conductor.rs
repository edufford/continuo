use continuo_core::{
    Component, ComponentPath, HashFnv1a64, Message, SimDuration, SimTime, StepCtx, TickDone,
    TickStart, hash_bytes, mix_seeds,
};
use continuo_transport::Transport;
use tracing::trace;

use crate::config::ConductorConfig;
use crate::error::ConductorError;
use crate::join::JoinMetadata;
use crate::pacing::{Pacer, Pacing, SystemClock};
use crate::record::TickFingerprint;
use crate::registry::Registry;
use crate::schedule::Schedule;

type TickCallback = Box<dyn FnMut(&TickFingerprint) + Send>;

/// Owns simulation time and drives the discrete-event loop over a
/// [`Transport`].
pub struct Conductor<T: Transport> {
    config: ConductorConfig,
    transport: T,
    registry: Registry,
    schedule: Schedule,
    sim_time: SimTime,
    tick: u64,
    /// Running determinism fingerprint: seeded from the world config, then
    /// chained with each tick's hash. Identical runs produce identical
    /// values at every tick.
    world_hash: u64,
    tick_callback: Option<TickCallback>,
    /// `Some` in 1× real-time mode, gating each instant to wall time;
    /// `None` in free-run. Pacing never affects `world_hash`.
    pacer: Option<Pacer<SystemClock>>,
}

impl<T: Transport> Conductor<T> {
    pub fn new(config: ConductorConfig, transport: T) -> Result<Self, ConductorError> {
        // 1× real-time pacing (PLAN.md "Pacing") gates each instant to wall
        // time; free-run leaves the pacer off and advances immediately. The
        // spin padding only picks how the wait is spent (OS sleep vs.
        // sleep-then-spin) — never what happens in the instant.
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
            sim_time: SimTime::ZERO,
            tick: 0,
            world_hash,
            tick_callback: None,
            pacer,
        })
    }

    /// Installs a callback invoked with every tick's [`TickFingerprint`] — the hook
    /// for recording (see [`crate::Recorder::tick_callback`]) or live
    /// divergence checking.
    pub fn set_tick_callback(&mut self, callback: impl FnMut(&TickFingerprint) + Send + 'static) {
        self.tick_callback = Some(Box::new(callback));
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

    /// The earliest scheduled due time, if anything remains scheduled —
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
    /// This measures **the schedule as a whole against the wall clock** —
    /// not components. Zero does *not* mean every component finished
    /// quickly, or that anything met a deadline: lateness under the
    /// re-anchor threshold is deliberately absorbed, and a component that
    /// runs long stays invisible here as long as the run recovers before
    /// the threshold.
    // TODO(M4): per-component step budgets answer the question this metric
    // cannot — "did *this* component finish within its time" — declared with
    // the timeout policy they share a measurement with (PLAN.md,
    // "Per-component timing").
    pub fn overrun_reanchor_count(&self) -> u64 {
        self.pacer.as_ref().map_or(0, Pacer::overrun_reanchor_count)
    }

    /// Total wall-clock time the run has permanently fallen behind real
    /// time, summed over the slips counted by
    /// [`Self::overrun_reanchor_count`] (0 in free-run, or when 1× pacing
    /// kept up). Lateness that was absorbed rather than slipped is not
    /// included — by definition it was recovered.
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
    /// a running world first steps, or — before the run starts — just the
    /// parent path (`""` for a world-level actor, `"car1"` to join that
    /// composite), which is shorthand for first stepping at sim time zero.
    ///
    /// Sibling order is arrival order, which fixes both the execution order
    /// within an instant and the "earlier sibling" of the visibility rule.
    /// A component admitted mid-run is therefore the newest sibling of
    /// whatever it joins.
    ///
    /// The first step is scheduled here, as the component is admitted,
    /// rather than discovered when the instant arrives — so the barrier at
    /// `first_due` already counts the newcomer among the components it
    /// waits for.
    // TODO(M4): joins currently arrive as this direct call. Requesting one
    // over the transport (continuo/{world}/conductor/join), and recording
    // admissions in the event log so dynamic runs replay, land in the later
    // sections of this milestone.
    //
    // TODO(M7): for a remote component the conductor never holds the Box —
    // it lives in its host process — so the registry entry becomes
    // Local(Box<dyn Component>) vs. Remote(metadata). Scheduling, the
    // visibility rule, and seq assignment already work off metadata alone,
    // which is why `JoinMetadata` is a separate type from the component.
    pub fn add_component(
        &mut self,
        join: impl Into<JoinMetadata>,
        component: Box<dyn Component>,
    ) -> Result<ComponentPath, ConductorError> {
        let join = join.into();
        let parent = ComponentPath::parse(&join.parent)?;

        // Validate before touching anything, so a rejected join leaves no
        // trace: no half-registered entry, no stray subscriptions.
        let earliest_open = self.earliest_open_instant();
        if join.first_due < earliest_open {
            return Err(ConductorError::JoinInThePast {
                path: parent.join(component.id()),
                first_due: join.first_due,
                earliest_open,
            });
        }

        let subscriptions = component.subscriptions();
        let (index, path) = self
            .registry
            .add(&parent, component, self.config.world_seed)?;
        for key in subscriptions {
            self.transport.subscribe(path.clone(), key);
        }
        self.schedule.insert(join.first_due, index);

        // Return the registered component's full path.
        Ok(path)
    }

    /// Removes a component from the world: it stops being scheduled, stops
    /// receiving messages, and is dropped. Everything it published stays
    /// published — departing is not a rollback.
    ///
    /// Survivors are untouched. Their declaration indexes do not shift, so
    /// execution order within an instant and every "earlier sibling"
    /// relationship the visibility rule depends on are exactly as they were.
    ///
    /// The path becomes free again; a later component may take it and
    /// arrives as a new sibling, ordered by its arrival like any other.
    // TODO(M4): this is the direct in-process call. Departure over the
    // transport (`continuo/{world}/conductor/leave`), queuing at step
    // boundaries, and recording the departure in the event log for replay
    // land in the later sections of this milestone.
    pub fn remove_component(&mut self, path: &str) -> Result<(), ConductorError> {
        let path = ComponentPath::parse(path)?;
        let index = self
            .registry
            .remove(&path)
            .ok_or_else(|| ConductorError::UnknownPath(path.clone()))?;
        self.schedule.remove_index(index);
        self.transport.unsubscribe(&path);

        // Return success; the slot is vacated and nothing references it.
        Ok(())
    }

    /// Advances to the earliest due instant and steps every component due at
    /// it, in declaration order. Returns `false` when nothing is scheduled.
    pub fn step_once(&mut self) -> Result<bool, ConductorError> {
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

        // TODO(M7): in distributed mode the conductor publishes TickStart on
        // the transport; every component (host) subscribes, and steps itself
        // when TickStart.sim_time reaches its own next_due. The conductor
        // barriers on TickDone acks from exactly the components it knows are
        // due (their next_due values arrived in prior TickDones). In-proc,
        // activation is a direct call and the messages exist as trace events.
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
            let next_due = entry.component.step(&mut ctx);

            // TODO(PLAN "Failure handling"): apply the per-world policy
            // (halt vs. timeout-and-drop, with drops event-logged for
            // replay) instead of always halting; extend beyond schedule
            // violations to component panics and step timeouts.
            if next_due <= now {
                return Err(ConductorError::ScheduleViolation {
                    path: entry.path.clone(),
                    now,
                    next_due,
                });
            }

            // The conductor (not the component) turns outbox entries into
            // published Messages so that the authoritative metadata —
            // publisher identity, per-publisher seq, and timestamp — is
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
        }

        // Chain this tick into the running world hash and emit the
        // fingerprint.
        let tick_hash = tick_hasher.finish();
        let mut chain = HashFnv1a64::resume(self.world_hash);
        chain.write_u64(tick_hash);
        self.world_hash = chain.finish();
        if let Some(callback) = self.tick_callback.as_mut() {
            callback(&TickFingerprint {
                tick: self.tick,
                sim_time: now,
                tick_hash,
                world_hash: self.world_hash,
            });
        }

        // Return true: a tick was executed (more may be scheduled).
        Ok(true)
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

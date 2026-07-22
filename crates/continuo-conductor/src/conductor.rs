use continuo_core::{
    Component, ComponentPath, Fnv1a64, Message, SimTime, StepCtx, TickDone, TickStart, hash_bytes,
    rng::mix,
};
use continuo_transport::Transport;
use tracing::trace;

use crate::config::ConductorConfig;
use crate::error::ConductorError;
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
}

impl<T: Transport> Conductor<T> {
    pub fn new(config: ConductorConfig, transport: T) -> Result<Self, ConductorError> {
        // TODO(M3): implement 1x real-time pacing (sleep until the wall time
        // corresponding to the next step's sim time; on overrun, log and let
        // the wall anchor slip — see PLAN.md "Pacing"). Until then only
        // free-run is supported.
        if config.real_time_pacing {
            return Err(ConductorError::RealTimePacingUnsupported);
        }
        // Fold the seed and world name into the initial hash so runs with
        // different seeds have different fingerprints even before (or
        // without) any component using randomness.
        let world_hash = mix(config.seed, hash_bytes(config.world.as_bytes()));

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

    pub fn world(&self) -> &str {
        &self.config.world
    }

    pub fn sim_time(&self) -> SimTime {
        self.sim_time
    }

    /// Total steps executed so far (each instant with due components is one
    /// tick).
    pub fn tick(&self) -> u64 {
        self.tick
    }

    /// Registers a component under `parent` (`""` for a world-level actor;
    /// `"car1"` makes it a child of the `car1` composite). Sibling order is
    /// registration order — it defines both execution order within an
    /// instant and the "earlier sibling" of the visibility rule.
    ///
    /// Milestone 1 supports registration before running only; the component
    /// is first due at sim time zero. Runtime join/leave is milestone 4.
    pub fn add_component(
        &mut self,
        parent: &str,
        component: Box<dyn Component>,
    ) -> Result<ComponentPath, ConductorError> {
        // TODO(M4): runtime join/leave — accept join requests over the
        // transport (continuo/{world}/conductor/join), apply them at step
        // boundaries, schedule the first step at the join time instead of
        // ZERO, and record admissions in the event log for replay.
        //
        // For remote components the conductor never holds the Box — the
        // component lives in its host process. Registration then needs only
        // the component's *metadata*, carried in the join request:
        //   { parent path, id (=> declared sibling order from arrival order),
        //     subscriptions, first_due, coupled/decoupled flag (PLAN.md
        //     decision 2026-07-18: decoupled children use next-step
        //     visibility, freeing their host placement) }.
        // The registry entry becomes Local(Box<dyn Component>) vs.
        // Remote(metadata); scheduling, the visibility rule, and seq
        // assignment work identically on both since they only use metadata.
        let parent = ComponentPath::parse(parent)?;
        let subscriptions = component.subscriptions();
        let (index, path) = self.registry.add(&parent, component, self.config.seed)?;
        for key in subscriptions {
            self.transport.subscribe(path.clone(), key);
        }
        self.schedule.insert(SimTime::ZERO, index);

        // Return the registered component's full path.
        Ok(path)
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

        // Per-tick determinism fingerprint: covers, in declaration order,
        // every stepped component's path, next-due time, published bytes,
        // and (when provided via `state_bytes`) internal state.
        let mut tick_hasher = Fnv1a64::new();
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
            let path = self.registry.entries[index].path.clone();
            let dt = self.registry.entries[index]
                .last_step
                .map(|prev| now - prev);
            let component_seed = self.registry.entries[index].component_seed;

            // The visibility rule (PLAN.md): everything published before this
            // instant is released; same-instant messages only from an
            // earlier-ordered sibling branch within the same composite.
            let tree = &self.registry.tree;
            let release_condition = |m: &Message| {
                m.time < now || (m.time == now && tree.releases_same_instant(&m.publisher, &path))
            };
            let inbox = self.transport.drain(&path, &release_condition);

            let mut ctx = StepCtx::new(now, dt, &self.config.world, component_seed, inbox);
            let entry = &mut self.registry.entries[index];
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

            let entry = &mut self.registry.entries[index];
            // Components exposing internal state join the hash in
            // state-hash mode; the rest are covered by their output bytes
            // above (output-hash mode).
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
        let mut chain = Fnv1a64::resume(self.world_hash);
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

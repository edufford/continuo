use continuo_core::{Component, ComponentPath, Message, SimTime, StepCtx, TickDone, TickStart};
use continuo_transport::Transport;
use tracing::trace;

use crate::config::ConductorConfig;
use crate::error::ConductorError;
use crate::registry::Registry;
use crate::schedule::Schedule;

/// Owns simulation time and drives the discrete-event loop over a
/// [`Transport`].
pub struct Conductor<T: Transport> {
    config: ConductorConfig,
    transport: T,
    registry: Registry,
    schedule: Schedule,
    sim_time: SimTime,
    tick: u64,
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
        Ok(Conductor {
            config,
            transport,
            registry: Registry::default(),
            schedule: Schedule::default(),
            sim_time: SimTime::ZERO,
            tick: 0,
        })
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
        //     subscriptions, first_due }.
        // The registry entry becomes Local(Box<dyn Component>) vs.
        // Remote(metadata); scheduling, the visibility rule, and seq
        // assignment work identically on both since they only use metadata.
        let parent = ComponentPath::parse(parent)?;
        let subscriptions = component.subscriptions();
        let (index, path) = self.registry.add(&parent, component)?;
        for key in subscriptions {
            self.transport.subscribe(path.clone(), key);
        }
        self.schedule.insert(SimTime::ZERO, index);
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

            // The visibility rule (PLAN.md): everything published before this
            // instant is released; same-instant messages only from an
            // earlier-ordered sibling branch within the same composite.
            let tree = &self.registry.tree;
            let release = |m: &Message| {
                m.time < now || (m.time == now && tree.releases_same_instant(&m.publisher, &path))
            };
            let inbox = self.transport.drain(&path, &release);

            let mut ctx = StepCtx::new(now, dt, &self.config.world, inbox);
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
            //
            // TODO(M2): feed the canonical payload bytes into the per-tick
            // state hash and the record/replay event log (a MonitorTransport
            // sink over these publishes).
            for (key, payload) in ctx.take_outbox() {
                let seq = entry.next_seq;
                entry.next_seq += 1;
                self.transport.publish(Message {
                    key,
                    publisher: path.clone(),
                    seq,
                    time: now,
                    payload,
                });
            }

            let entry = &mut self.registry.entries[index];
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
        Ok(())
    }
}

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

            if next_due <= now {
                return Err(ConductorError::ScheduleViolation {
                    path: entry.path.clone(),
                    now,
                    next_due,
                });
            }

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

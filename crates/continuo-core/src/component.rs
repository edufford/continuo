use serde::Serialize;

use crate::error::CoreError;
use crate::ids::ComponentId;
use crate::keyexpr::KeyExpr;
use crate::messages::Message;
use crate::time::{SimDuration, SimTime};

/// A simulation component: steps when due, reads its inbox, publishes
/// outputs, and reports the next sim time it should step.
///
/// Components are transport-blind and hierarchy-blind: the conductor decides
/// when `step` runs and what the inbox contains (see PLAN.md's visibility
/// rule); components only see `StepCtx`.
// TODO(M4): components currently live for the whole run; a departure
// mechanism (voluntary leave, or conductor-initiated drop under the failure
// policy) arrives with runtime join/leave.
pub trait Component: Send {
    /// This component's name within its parent (one path segment).
    fn id(&self) -> ComponentId;

    /// Key expressions this component wants delivered to its inbox.
    fn subscriptions(&self) -> Vec<KeyExpr>;

    /// Advance internal state to `ctx.now()`. Returns the next sim time this
    /// component should step — must be strictly greater than `ctx.now()`
    /// (the conductor enforces this to prevent zero-time livelock).
    fn step(&mut self, ctx: &mut StepCtx) -> SimTime;
}

/// Everything a component may observe or do during one step.
pub struct StepCtx<'a> {
    now: SimTime,
    dt: Option<SimDuration>,
    world: &'a str,
    inbox: Vec<Message>,
    outbox: Vec<(KeyExpr, Vec<u8>)>,
}

impl<'a> StepCtx<'a> {
    /// Constructed by the conductor (or by tests driving a component
    /// directly).
    pub fn new(now: SimTime, dt: Option<SimDuration>, world: &'a str, inbox: Vec<Message>) -> Self {
        StepCtx {
            now,
            dt,
            world,
            inbox,
            outbox: Vec::new(),
        }
    }

    /// Current sim time.
    pub fn now(&self) -> SimTime {
        self.now
    }

    /// Elapsed sim time since this component's previous step; `None` on its
    /// first step.
    pub fn dt(&self) -> Option<SimDuration> {
        self.dt
    }

    /// The world name, for building key expressions.
    pub fn world(&self) -> &str {
        self.world
    }

    /// Messages released to this component for this step, sorted by
    /// `(publisher, seq)`.
    pub fn inbox(&self) -> &[Message] {
        &self.inbox
    }

    /// Publishes a value as canonical JSON on `key`, stamped with this step's
    /// time. Visibility to other components follows the delivery rule
    /// (PLAN.md): later-ordered siblings within the same composite see it
    /// this instant; everyone else from their next step.
    pub fn publish<T: Serialize>(&mut self, key: KeyExpr, value: &T) -> Result<(), CoreError> {
        let payload = serde_json::to_vec(value).map_err(|source| CoreError::PayloadSerialize {
            key: key.as_str().to_string(),
            source,
        })?;
        self.outbox.push((key, payload));
        Ok(())
    }

    /// Drains the accumulated publishes; called by the conductor after
    /// `step` returns.
    pub fn take_outbox(&mut self) -> Vec<(KeyExpr, Vec<u8>)> {
        std::mem::take(&mut self.outbox)
    }
}

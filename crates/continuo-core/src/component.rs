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
// TODO(M4): a component cannot yet ask to leave. The conductor can remove
// one (`Conductor::remove_component`), but voluntary departure — and the
// failure policy's drop, which uses the same path — arrive with the rest of
// runtime join/leave.
pub trait Component: Send {
    /// This component's name within its parent (one path segment).
    fn id(&self) -> ComponentId;

    /// Key expressions this component wants delivered to its inbox.
    fn subscriptions(&self) -> Vec<KeyExpr>;

    /// Advance internal state to `ctx.now()`. Returns the next sim time this
    /// component should step — must be strictly greater than `ctx.now()`
    /// (the conductor enforces this to prevent zero-time livelock).
    fn step(&mut self, ctx: &mut StepCtx) -> SimTime;

    /// Canonical serialized internal state for the per-tick determinism
    /// check, or `None` (the default) if the component's state is opaque.
    ///
    /// Components returning `Some` join the per-tick hash in *state-hash*
    /// mode: hidden internal state is fingerprinted directly, so divergence
    /// is caught when it happens — even state that would only influence
    /// outputs many steps later (integrators, counters, stored RNG
    /// streams), or state in components that publish less often than they
    /// step. Components returning `None` are covered in *output-hash* mode
    /// — everything they publish is hashed anyway (the only option for
    /// opaque black boxes like FMUs without `SerializeFMUState`; see
    /// PLAN.md, Determinism rules). For a component that publishes its
    /// whole state every step, the two modes catch divergence at
    /// essentially the same tick.
    ///
    /// Implementations must follow the canonical JSON rules (serde_json,
    /// declaration-order fields, no `HashMap`) — typically
    /// `serde_json::to_vec` of a state struct. Called by the conductor after
    /// `step`, at most once per step.
    fn state_bytes(&self) -> Option<Vec<u8>> {
        None
    }
}

/// Everything a component may observe or do during one step.
pub struct StepCtx<'a> {
    now: SimTime,
    dt: Option<SimDuration>,
    world_name: &'a str,
    component_seed: u64,
    inbox: Vec<Message>,
    outbox: Vec<(KeyExpr, Vec<u8>)>,
}

impl<'a> StepCtx<'a> {
    /// Constructed by the conductor (or by tests driving a component
    /// directly). `component_seed` derives from
    /// `(world_seed, component_path)` — see [`crate::derive_component_seed`].
    pub fn new(
        now: SimTime,
        dt: Option<SimDuration>,
        world_name: &'a str,
        component_seed: u64,
        inbox: Vec<Message>,
    ) -> Self {
        StepCtx {
            now,
            dt,
            world_name,
            component_seed,
            inbox,
            outbox: Vec::new(),
        }
    }

    /// This component's deterministic seed, stable for the whole run:
    /// derived from `(world_seed, component_path)`. Seed a stored
    /// [`RandomSplitMix64`](crate::RandomSplitMix64) from it for a stream
    /// that persists across steps.
    pub fn component_seed(&self) -> u64 {
        self.component_seed
    }

    /// A fresh deterministic random stream for this `(component, step)`
    /// pair — the zero-state-to-carry way to add noise. The stream depends
    /// only on the component seed and the current sim time, so replays
    /// reproduce it exactly; draws are independent between steps. For a
    /// stream that is continuous *across* steps, store
    /// `RandomSplitMix64::new(ctx.component_seed())` in the component at
    /// first step instead.
    pub fn step_random(&self) -> crate::RandomSplitMix64 {
        // Return a stream seeded by who is stepping and when.
        crate::RandomSplitMix64::new(crate::seed::derive_step_seed(
            self.component_seed,
            self.now.as_nanos(),
        ))
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
    pub fn world_name(&self) -> &str {
        self.world_name
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

        // Return success; the conductor stamps and routes the queued message after the step.
        Ok(())
    }

    /// Drains the accumulated publishes; called by the conductor after
    /// `step` returns.
    pub fn take_outbox(&mut self) -> Vec<(KeyExpr, Vec<u8>)> {
        std::mem::take(&mut self.outbox)
    }
}

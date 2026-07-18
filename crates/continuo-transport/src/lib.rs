//! The transport seam: pub/sub over Zenoh-style key expressions.
//!
//! Milestone 1 ships the deterministic in-process implementation. The trait
//! is intentionally minimal and in-proc-shaped for now (`drain` with a
//! release predicate is an in-proc convenience); it will be revisited when
//! the Zenoh transport lands (milestone 7).

mod inproc;
mod monitor;

pub use inproc::InProcTransport;
pub use monitor::MonitorTransport;

use continuo_core::{ComponentPath, KeyExpr, Message};

/// Pub/sub transport for continuo messages.
///
/// Implementations must be deterministic: routing depends only on
/// subscriptions and message metadata, never on arrival order or wall time.
pub trait Transport {
    /// Registers a subscription: messages whose key matches `key` are queued
    /// for `subscriber` until drained.
    fn subscribe(&mut self, subscriber: ComponentPath, key: KeyExpr);

    /// Routes a message into the queue of every matching subscriber.
    fn publish(&mut self, message: Message);

    /// Removes and returns the queued messages for `subscriber` that satisfy
    /// `release`, sorted by `(publisher, seq)`. Messages not released stay
    /// queued in order.
    ///
    /// The release predicate is supplied by the conductor, which is the only
    /// party that knows the component tree and can apply the visibility rule
    /// (PLAN.md): published-before-now always releases; published-at-now
    /// releases only from an earlier-ordered sibling branch within the same
    /// composite.
    fn drain(
        &mut self,
        subscriber: &ComponentPath,
        release: &dyn Fn(&Message) -> bool,
    ) -> Vec<Message>;
}

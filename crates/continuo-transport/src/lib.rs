//! The transport seam: pub/sub over Zenoh-style key expressions.
//!
//! Milestone 1 ships the deterministic in-process implementation. The trait
//! is intentionally minimal and in-proc-shaped for now (`drain` with a
//! release condition is an in-proc convenience); it will be revisited when
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

    /// Drops every subscription for `subscriber`, along with anything still
    /// queued for it — called when a component leaves (milestone 4).
    ///
    /// Discarding the queue is the point, not a side effect: a departed
    /// component will never step again to drain it, so keeping the messages
    /// would leak, and delivering them to a later component that reuses the
    /// path would hand it traffic from before it existed.
    fn unsubscribe(&mut self, subscriber: &ComponentPath);

    /// Routes a message into the queue of every matching subscriber.
    fn publish(&mut self, message: Message);

    /// Removes and returns the queued messages for `subscriber` that satisfy
    /// `release_condition`, sorted by `(publisher, seq)`. Messages not
    /// released stay queued in order.
    ///
    /// The release condition is supplied by the conductor, which is the only
    /// party that knows the component tree and can apply the visibility rule
    /// (PLAN.md): published-before-now always releases; published-at-now
    /// releases only from an earlier-ordered sibling branch within the same
    /// composite.
    // TODO(M7): drain-with-condition is in-proc-shaped. The Zenoh transport
    // will receive messages asynchronously and gather/order them per step on
    // the consuming host; revisit this trait when it lands.
    fn drain(
        &mut self,
        subscriber: &ComponentPath,
        release_condition: &dyn Fn(&Message) -> bool,
    ) -> Vec<Message>;
}

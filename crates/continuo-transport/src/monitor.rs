use continuo_core::{ComponentPath, KeyExpr, Message};

use crate::Transport;

type MessageCallback = Box<dyn FnMut(&Message) + Send>;

/// Wraps a transport and invokes its callbacks for every published message
/// before it is routed.
///
/// This is the out-of-band observation mechanism: the callbacks see **all**
/// traffic at publish time, regardless of subscriptions (including messages
/// nobody subscribes to), with no visibility delay and no presence in the
/// schedule. Use it for logging, debugging, and recording (the milestone 2
/// event log builds on this).
///
/// The callbacks are not part of the simulation: they must never feed data back
/// into components. In-simulation observers (which see messages under the
/// visibility rule, like any participant) are ordinary components instead.
pub struct MonitorTransport<T: Transport> {
    inner: T,
    callbacks: Vec<MessageCallback>,
}

impl<T: Transport> MonitorTransport<T> {
    /// Wraps `inner`, with no callbacks until some are added.
    pub fn new(inner: T) -> Self {
        MonitorTransport {
            inner,
            callbacks: Vec::new(),
        }
    }

    pub fn add_message_callback(&mut self, callback: impl FnMut(&Message) + Send + 'static) {
        self.callbacks.push(Box::new(callback));
    }

    pub fn into_inner(self) -> T {
        self.inner
    }
}

impl<T: Transport> Transport for MonitorTransport<T> {
    fn subscribe(&mut self, subscriber: ComponentPath, key: KeyExpr) {
        self.inner.subscribe(subscriber, key);
    }

    fn unsubscribe(&mut self, subscriber: &ComponentPath) {
        self.inner.unsubscribe(subscriber);
    }

    fn publish(&mut self, message: Message) {
        for callback in &mut self.callbacks {
            callback(&message);
        }
        self.inner.publish(message);
    }

    fn drain(
        &mut self,
        subscriber: &ComponentPath,
        release_condition: &dyn Fn(&Message) -> bool,
    ) -> Vec<Message> {
        self.inner.drain(subscriber, release_condition)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::InProcTransport;
    use continuo_core::SimTime;
    use std::sync::{Arc, Mutex};

    fn msg(key: &str, seq: u64) -> Message {
        Message {
            key: KeyExpr::new(key).unwrap(),
            publisher: ComponentPath::parse("pub").unwrap(),
            seq,
            sim_time: SimTime::ZERO,
            payload: b"{}".to_vec(),
        }
    }

    #[test]
    fn callback_sees_every_publish_even_without_subscribers() {
        let seen = Arc::new(Mutex::new(Vec::new()));
        let mut monitor = MonitorTransport::new(InProcTransport::new());
        monitor.add_message_callback({
            let seen = seen.clone();
            move |m: &Message| seen.lock().unwrap().push(m.key.to_string())
        });

        // No subscribers at all: the inner transport drops it, the monitor
        // sees it.
        monitor.publish(msg("w/unsubscribed", 0));

        monitor.subscribe(
            ComponentPath::parse("sub").unwrap(),
            KeyExpr::new("w/data").unwrap(),
        );
        monitor.publish(msg("w/data", 1));

        assert_eq!(
            *seen.lock().unwrap(),
            vec!["w/unsubscribed".to_string(), "w/data".to_string()]
        );

        // Delegation: the subscribed message is still routed and drainable.
        let released = monitor.drain(&ComponentPath::parse("sub").unwrap(), &|_| true);
        assert_eq!(released.len(), 1);
        assert_eq!(released[0].seq, 1);
    }

    #[test]
    fn added_callbacks_each_see_every_publish_in_the_order_added() {
        let seen = Arc::new(Mutex::new(Vec::new()));
        let tagging = |tag: &'static str| {
            let seen = seen.clone();
            move |m: &Message| seen.lock().unwrap().push((tag, m.seq))
        };
        let mut monitor = MonitorTransport::new(InProcTransport::new());
        monitor.add_message_callback(tagging("first"));
        monitor.add_message_callback(tagging("second"));

        monitor.publish(msg("w/data", 7));

        assert_eq!(*seen.lock().unwrap(), vec![("first", 7), ("second", 7)]);
    }
}

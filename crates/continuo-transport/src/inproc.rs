use std::collections::BTreeMap;

use continuo_core::{ComponentPath, KeyExpr, Message};

use crate::Transport;

/// Deterministic in-process transport: `BTreeMap`-backed queues, no threads,
/// no wall time. Message copies are queued per subscriber at publish time.
#[derive(Debug, Default)]
pub struct InProcTransport {
    /// Subscriber → subscribed key expressions.
    subscriptions: BTreeMap<ComponentPath, Vec<KeyExpr>>,
    /// Subscriber → queued messages, in publish order.
    queues: BTreeMap<ComponentPath, Vec<Message>>,
}

impl InProcTransport {
    pub fn new() -> Self {
        Self::default()
    }
}

impl Transport for InProcTransport {
    fn subscribe(&mut self, subscriber: ComponentPath, key: KeyExpr) {
        self.subscriptions.entry(subscriber).or_default().push(key);
    }

    fn publish(&mut self, message: Message) {
        for (subscriber, keys) in &self.subscriptions {
            if keys.iter().any(|k| k.matches(&message.key)) {
                self.queues
                    .entry(subscriber.clone())
                    .or_default()
                    .push(message.clone());
            }
        }
    }

    fn drain(
        &mut self,
        subscriber: &ComponentPath,
        release: &dyn Fn(&Message) -> bool,
    ) -> Vec<Message> {
        let Some(queue) = self.queues.get_mut(subscriber) else {
            return Vec::new();
        };
        let mut released = Vec::new();
        let mut kept = Vec::new();
        for message in queue.drain(..) {
            if release(&message) {
                released.push(message);
            } else {
                kept.push(message);
            }
        }
        *queue = kept;
        released.sort_by(|a, b| (&a.publisher, a.seq).cmp(&(&b.publisher, b.seq)));
        released
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use continuo_core::SimTime;

    fn kx(s: &str) -> KeyExpr {
        KeyExpr::new(s).unwrap()
    }

    fn path(s: &str) -> ComponentPath {
        ComponentPath::parse(s).unwrap()
    }

    fn msg(key: &str, publisher: &str, seq: u64, t_ns: i64) -> Message {
        Message {
            key: kx(key),
            publisher: path(publisher),
            seq,
            time: SimTime::from_nanos(t_ns),
            payload: b"{}".to_vec(),
        }
    }

    #[test]
    fn routes_by_keyexpr_match() {
        let mut t = InProcTransport::new();
        t.subscribe(path("logger"), kx("w/actor/**/pose"));
        t.subscribe(path("other"), kx("w/actor/car9/pose"));

        t.publish(msg("w/actor/car1/pose", "car1/physics", 0, 0));

        assert_eq!(t.drain(&path("logger"), &|_| true).len(), 1);
        assert!(t.drain(&path("other"), &|_| true).is_empty());
    }

    #[test]
    fn drain_releases_selectively_and_keeps_the_rest() {
        let mut t = InProcTransport::new();
        t.subscribe(path("sub"), kx("w/**"));

        t.publish(msg("w/a", "p1", 0, 100));
        t.publish(msg("w/a", "p1", 1, 200));

        let released = t.drain(&path("sub"), &|m| m.time < SimTime::from_nanos(200));
        assert_eq!(released.len(), 1);
        assert_eq!(released[0].seq, 0);

        let rest = t.drain(&path("sub"), &|_| true);
        assert_eq!(rest.len(), 1);
        assert_eq!(rest[0].seq, 1);
    }

    #[test]
    fn drain_sorts_by_publisher_then_seq() {
        let mut t = InProcTransport::new();
        t.subscribe(path("sub"), kx("w/**"));

        // Published in "arrival" order that differs from (publisher, seq).
        t.publish(msg("w/a", "zeta", 0, 0));
        t.publish(msg("w/a", "alpha", 1, 0));
        t.publish(msg("w/a", "alpha", 0, 0));

        let released = t.drain(&path("sub"), &|_| true);
        let order: Vec<(String, u64)> = released
            .iter()
            .map(|m| (m.publisher.to_string(), m.seq))
            .collect();
        assert_eq!(
            order,
            vec![
                ("alpha".to_string(), 0),
                ("alpha".to_string(), 1),
                ("zeta".to_string(), 0)
            ]
        );
    }
}

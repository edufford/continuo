use std::collections::BTreeMap;

use continuo_core::{ComponentPath, KeyExpr, Message};

use crate::Transport;

/// Deterministic in-process transport: `BTreeMap`-backed queues, no threads,
/// no wall time.
///
/// A message is copied into a queue per matching subscriber rather than shared
/// out of one queue, and that follows from [`Transport::drain`] taking a
/// *per-subscriber* release condition. The visibility rule can release a
/// same-instant message to one subscriber while holding it back for another,
/// depending on where each sits in the composite tree, so there is no single
/// order a shared queue would be consumed in. The copy is the queue design, not
/// a decision about bytes.
///
/// It costs little while payloads are poses. Sharing them instead would mean
/// `Message::payload` becoming an `Arc<[u8]>`, which leaves the queues alone
/// and is worth revisiting with PLAN.md's deferred large-payload work, where a
/// camera frame would otherwise be copied once per subscriber.
///
/// What limits how large a world can be is routing rather than copying, since
/// subscriptions are key expressions and every published key has to be matched
/// against them. That is what `recipients` is for. `traffic_scale` measures
/// the result.
#[derive(Debug, Default)]
pub struct InProcTransport {
    /// Subscriber → subscribed key expressions.
    subscriptions: BTreeMap<ComponentPath, Vec<KeyExpr>>,
    /// Subscriber → queued messages, in publish order.
    queues: BTreeMap<ComponentPath, Vec<Message>>,
    /// Published key → the subscribers it reaches.
    ///
    /// A subscription is a pattern, such as `continuo/*/actor/ego/pose`,
    /// rather than a literal key. So there is nothing to look a published key
    /// up by: finding who wants it means testing it against every subscription
    /// in the world. That is far too much to repeat per message, because a
    /// world publishes the same handful of keys thousands of times.
    ///
    /// Who a key reaches changes only when someone subscribes or
    /// unsubscribes, so it is worked out on that key's first message and
    /// reused by the rest.
    ///
    /// Held in ascending subscriber order, which is what iterating
    /// [`Self::subscriptions`] gives. Delivery order reaches the world hash,
    /// so it is part of the contract rather than a detail.
    recipients: BTreeMap<KeyExpr, Vec<ComponentPath>>,
}

impl InProcTransport {
    pub fn new() -> Self {
        Self::default()
    }
}

impl Transport for InProcTransport {
    fn subscribe(&mut self, subscriber: ComponentPath, key: KeyExpr) {
        // A new subscription can only add its subscriber, and only to keys it
        // matches, so `recipients` is kept current rather than emptied.
        // Components join while a run publishes, and emptying it would mean
        // matching every subscription again on the next message.
        for (published, recipients) in self.recipients.iter_mut() {
            if key.matches(published) && !recipients.contains(&subscriber) {
                let at = recipients.partition_point(|existing| existing < &subscriber);
                recipients.insert(at, subscriber.clone());
            }
        }

        self.subscriptions.entry(subscriber).or_default().push(key);
    }

    fn unsubscribe(&mut self, subscriber: &ComponentPath) {
        self.subscriptions.remove(subscriber);
        self.queues.remove(subscriber);

        // Dropping out of every key it reached leaves the order of the rest
        // as it was.
        for recipients in self.recipients.values_mut() {
            recipients.retain(|existing| existing != subscriber);
        }
    }

    fn publish(&mut self, message: Message) {
        // Destructured so `recipients` and `queues` can be borrowed at once.
        let Self {
            subscriptions,
            queues,
            recipients,
        } = self;

        // `entry` takes the key by value since it may store it, so this clones
        // on every publish and not only when it inserts. `or_insert_with` runs
        // its closure only for a key not seen before, which is what leaves
        // every later message on that key with nothing to do but read.
        let matched = recipients.entry(message.key.clone()).or_insert_with(|| {
            subscriptions
                .iter()
                // Keep a subscriber if any of its patterns matches this key.
                .filter(|(_, keys)| keys.iter().any(|key| key.matches(&message.key)))
                // Having decided, keep the name and drop the patterns.
                .map(|(subscriber, _)| subscriber.clone())
                // Ascending subscriber order, since that is how a `BTreeMap`
                // iterates, and what `subscribe` inserts into later.
                .collect()
        });

        for subscriber in matched.iter() {
            queues
                .entry(subscriber.clone())
                .or_default()
                .push(message.clone());
        }
    }

    fn drain(
        &mut self,
        subscriber: &ComponentPath,
        release_condition: &dyn Fn(&Message) -> bool,
    ) -> Vec<Message> {
        let Some(queue) = self.queues.get_mut(subscriber) else {
            return Vec::new();
        };
        let mut released = Vec::new();
        let mut kept = Vec::new();
        for message in queue.drain(..) {
            if release_condition(&message) {
                released.push(message);
            } else {
                kept.push(message);
            }
        }
        *queue = kept;
        released.sort_by(|a, b| (&a.publisher, a.seq).cmp(&(&b.publisher, b.seq)));

        // Return the released messages in deterministic (publisher, seq) order.
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
            sim_time: SimTime::from_nanos(t_ns),
            payload: b"{}".to_vec(),
        }
    }

    /// Who a key reaches is worked out once and kept, so every membership
    /// change has to mend it. A world spawns and retires while it publishes,
    /// which is exactly when a stale answer would be used.
    #[test]
    fn a_subscriber_that_joins_after_a_key_was_published_still_receives_it() {
        let mut t = InProcTransport::new();
        t.subscribe(path("early"), kx("w/actor/car1/pose"));
        t.publish(msg("w/actor/car1/pose", "car1/physics", 0, 0));

        t.subscribe(path("late"), kx("w/actor/**/pose"));
        t.publish(msg("w/actor/car1/pose", "car1/physics", 1, 1));

        assert_eq!(t.drain(&path("early"), &|_| true).len(), 2);
        assert_eq!(
            t.drain(&path("late"), &|_| true).len(),
            1,
            "the newcomer gets what was published after it joined, and no more"
        );
    }

    #[test]
    fn a_subscriber_that_leaves_stops_receiving_without_disturbing_the_rest() {
        let mut t = InProcTransport::new();
        t.subscribe(path("staying"), kx("w/actor/**/pose"));
        t.subscribe(path("leaving"), kx("w/actor/**/pose"));
        t.publish(msg("w/actor/car1/pose", "car1/physics", 0, 0));

        t.unsubscribe(&path("leaving"));
        t.publish(msg("w/actor/car1/pose", "car1/physics", 1, 1));

        assert_eq!(t.drain(&path("staying"), &|_| true).len(), 2);
        assert!(t.drain(&path("leaving"), &|_| true).is_empty());
    }

    /// Recipients are held in the order subscriptions are iterated, and a
    /// subscriber added later has to land in its place rather than at the end.
    /// Delivery order reaches the world hash.
    #[test]
    fn a_late_subscriber_is_delivered_to_in_path_order() {
        let mut t = InProcTransport::new();
        t.subscribe(path("c"), kx("w/actor/**/pose"));
        t.publish(msg("w/actor/car1/pose", "car1/physics", 0, 0));
        t.subscribe(path("a"), kx("w/actor/**/pose"));

        let mut fresh = InProcTransport::new();
        fresh.subscribe(path("c"), kx("w/actor/**/pose"));
        fresh.subscribe(path("a"), kx("w/actor/**/pose"));
        fresh.publish(msg("w/actor/car1/pose", "car1/physics", 0, 0));

        assert_eq!(
            t.recipients[&kx("w/actor/car1/pose")],
            fresh.recipients[&kx("w/actor/car1/pose")],
            "a mended answer must match one built from scratch"
        );
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

        let released = t.drain(&path("sub"), &|m| m.sim_time < SimTime::from_nanos(200));
        assert_eq!(released.len(), 1);
        assert_eq!(released[0].seq, 0);

        let rest = t.drain(&path("sub"), &|_| true);
        assert_eq!(rest.len(), 1);
        assert_eq!(rest[0].seq, 1);
    }

    #[test]
    fn unsubscribe_stops_routing_and_discards_whatever_was_queued() {
        let mut t = InProcTransport::new();
        t.subscribe(path("gone"), kx("w/**"));
        t.publish(msg("w/a", "p", 0, 0));

        t.unsubscribe(&path("gone"));

        // The undrained message goes too: a departed component will never
        // step again to collect it.
        assert!(t.drain(&path("gone"), &|_| true).is_empty());
        // And nothing further is routed to it.
        t.publish(msg("w/a", "p", 1, 0));
        assert!(t.drain(&path("gone"), &|_| true).is_empty());
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

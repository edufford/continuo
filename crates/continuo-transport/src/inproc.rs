use std::collections::BTreeMap;

use continuo_core::{ComponentPath, Delivery, KeyExpr, Message};

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
    /// Subscriber -> subscribed key expressions, each with how it delivers.
    subscriptions: BTreeMap<ComponentPath, Vec<(KeyExpr, Delivery)>>,
    /// Subscriber -> messages delivered one by one, in publish order.
    queues: BTreeMap<ComponentPath, Vec<Message>>,
    /// Subscriber -> published key -> the messages on that key that could
    /// still be the newest one the subscriber may see, in publish order.
    ///
    /// A message published before the drain's instant always releases. So
    /// once a key holds a message from an earlier instant than the one being
    /// published, nothing before that message can be the newest released
    /// again, and `publish` drops it. A key then holds one message from an
    /// earlier instant and those from the current one, however often it is
    /// published between drains.
    latest: BTreeMap<ComponentPath, BTreeMap<KeyExpr, Vec<Message>>>,
    /// Published key -> the subscribers it reaches, and how.
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
    recipients: BTreeMap<KeyExpr, Vec<(ComponentPath, Delivery)>>,
}

impl InProcTransport {
    pub fn new() -> Self {
        Self::default()
    }

    /// How `published` reaches a subscriber holding `subscriptions`, if at
    /// all. A pattern delivering every message wins over one delivering the
    /// latest, so a key subscribed both ways loses nothing.
    fn delivery_of(subscriptions: &[(KeyExpr, Delivery)], published: &KeyExpr) -> Option<Delivery> {
        let mut found = None;
        for (key, delivery) in subscriptions {
            if !key.matches(published) {
                continue;
            }
            if *delivery == Delivery::Every {
                return Some(Delivery::Every);
            }
            found = Some(Delivery::Latest);
        }

        // Return `Latest` if a pattern matched and none asked for every message.
        found
    }
}

impl Transport for InProcTransport {
    fn subscribe(&mut self, subscriber: ComponentPath, key: KeyExpr, delivery: Delivery) {
        let subscriptions = self.subscriptions.entry(subscriber.clone()).or_default();
        subscriptions.push((key, delivery));

        // A new subscription can only add its subscriber to a key, or change
        // how it receives one, so `recipients` is kept current rather than
        // emptied. Components join while a run publishes, and emptying it
        // would mean matching every subscription again on the next message.
        for (published, recipients) in self.recipients.iter_mut() {
            let Some(delivery) = Self::delivery_of(subscriptions, published) else {
                continue;
            };
            let at = recipients.partition_point(|(existing, _)| existing < &subscriber);
            match recipients.get_mut(at) {
                Some((existing, held)) if existing == &subscriber => *held = delivery,
                _ => recipients.insert(at, (subscriber.clone(), delivery)),
            }
        }
    }

    fn unsubscribe(&mut self, subscriber: &ComponentPath) {
        self.subscriptions.remove(subscriber);
        self.queues.remove(subscriber);
        self.latest.remove(subscriber);

        // Dropping out of every key it reached leaves the order of the rest
        // as it was.
        for recipients in self.recipients.values_mut() {
            recipients.retain(|(existing, _)| existing != subscriber);
        }
    }

    fn publish(&mut self, message: Message) {
        // Destructured so `recipients` and the queues can be borrowed at once.
        let Self {
            subscriptions,
            queues,
            latest,
            recipients,
        } = self;

        // `entry` takes the key by value since it may store it, so this clones
        // on every publish and not only when it inserts. `or_insert_with` runs
        // its closure only for a key not seen before, which is what leaves
        // every later message on that key with nothing to do but read.
        let matched = recipients.entry(message.key.clone()).or_insert_with(|| {
            subscriptions
                .iter()
                // Keep a subscriber if any of its patterns matches this key,
                // along with how it receives it.
                .filter_map(|(subscriber, keys)| {
                    Self::delivery_of(keys, &message.key)
                        .map(|delivery| (subscriber.clone(), delivery))
                })
                // Ascending subscriber order, since that is how a `BTreeMap`
                // iterates, and what `subscribe` inserts into later.
                .collect()
        });

        for (subscriber, delivery) in matched.iter() {
            match delivery {
                Delivery::Every => queues
                    .entry(subscriber.clone())
                    .or_default()
                    .push(message.clone()),
                Delivery::Latest => {
                    let held = latest
                        .entry(subscriber.clone())
                        .or_default()
                        .entry(message.key.clone())
                        .or_default();
                    // The newest message from an earlier instant releases at
                    // every drain from here on, so nothing before it can be
                    // delivered as the newest again.
                    let earlier_instant = held
                        .iter()
                        .rposition(|queued| queued.sim_time < message.sim_time);
                    if let Some(settled) = earlier_instant {
                        held.drain(..settled);
                    }
                    held.push(message.clone());
                }
            }
        }
    }

    fn drain(
        &mut self,
        subscriber: &ComponentPath,
        release_condition: &dyn Fn(&Message) -> bool,
    ) -> Vec<Message> {
        let mut released = Vec::new();

        if let Some(queue) = self.queues.get_mut(subscriber) {
            let mut kept = Vec::new();
            for message in queue.drain(..) {
                if release_condition(&message) {
                    released.push(message);
                } else {
                    kept.push(message);
                }
            }
            *queue = kept;
        }

        if let Some(keys) = self.latest.get_mut(subscriber) {
            // On each key the newest message that releases is delivered, and
            // everything published before it is superseded. Messages after it
            // were held back, and stay for a later drain.
            for held in keys.values_mut() {
                if let Some(newest) = held.iter().rposition(release_condition) {
                    let newest = held.drain(..=newest).next_back();
                    released.extend(newest);
                }
            }
            // A key whose publisher has left is not published again, so its
            // empty entry would only pile up as actors come and go.
            keys.retain(|_, held| !held.is_empty());
        }

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
        t.subscribe(path("early"), kx("w/actor/car1/pose"), Delivery::Every);
        t.publish(msg("w/actor/car1/pose", "car1/physics", 0, 0));

        t.subscribe(path("late"), kx("w/actor/**/pose"), Delivery::Every);
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
        t.subscribe(path("staying"), kx("w/actor/**/pose"), Delivery::Every);
        t.subscribe(path("leaving"), kx("w/actor/**/pose"), Delivery::Every);
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
        t.subscribe(path("c"), kx("w/actor/**/pose"), Delivery::Every);
        t.publish(msg("w/actor/car1/pose", "car1/physics", 0, 0));
        t.subscribe(path("a"), kx("w/actor/**/pose"), Delivery::Every);

        let mut fresh = InProcTransport::new();
        fresh.subscribe(path("c"), kx("w/actor/**/pose"), Delivery::Every);
        fresh.subscribe(path("a"), kx("w/actor/**/pose"), Delivery::Every);
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
        t.subscribe(path("logger"), kx("w/actor/**/pose"), Delivery::Every);
        t.subscribe(path("other"), kx("w/actor/car9/pose"), Delivery::Every);

        t.publish(msg("w/actor/car1/pose", "car1/physics", 0, 0));

        assert_eq!(t.drain(&path("logger"), &|_| true).len(), 1);
        assert!(t.drain(&path("other"), &|_| true).is_empty());
    }

    #[test]
    fn drain_releases_selectively_and_keeps_the_rest() {
        let mut t = InProcTransport::new();
        t.subscribe(path("sub"), kx("w/**"), Delivery::Every);

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
        t.subscribe(path("gone"), kx("w/**"), Delivery::Every);
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
        t.subscribe(path("sub"), kx("w/**"), Delivery::Every);

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

    /// The `(seq, sim_time)` of each released message, for comparing what a
    /// drain delivered.
    fn seqs_and_times(released: &[Message]) -> Vec<(u64, i64)> {
        released
            .iter()
            .map(|m| (m.seq, m.sim_time.as_nanos()))
            .collect()
    }

    #[test]
    fn a_latest_subscriber_receives_only_the_newest_message_on_each_key() {
        let mut t = InProcTransport::new();
        t.subscribe(path("sub"), kx("w/actor/*/pose"), Delivery::Latest);
        for seq in 0..3 {
            t.publish(msg("w/actor/car1/pose", "car1/physics", seq, seq as i64));
            t.publish(msg("w/actor/car2/pose", "car2/physics", seq, seq as i64));
        }

        let released = t.drain(&path("sub"), &|_| true);
        let delivered: Vec<(String, u64)> = released
            .iter()
            .map(|m| (m.publisher.to_string(), m.seq))
            .collect();
        assert_eq!(
            delivered,
            vec![
                ("car1/physics".to_string(), 2),
                ("car2/physics".to_string(), 2)
            ]
        );
        assert!(
            t.latest[&path("sub")].is_empty(),
            "a drained key leaves no entry behind"
        );
    }

    /// The visibility rule can hold back a message from the drain's own
    /// instant. The newest message is then the newest one released, not the
    /// newest one published.
    #[test]
    fn a_held_back_message_leaves_the_newest_released_one_to_be_delivered() {
        let mut t = InProcTransport::new();
        t.subscribe(path("sub"), kx("w/a"), Delivery::Latest);
        t.publish(msg("w/a", "p", 0, 1));
        t.publish(msg("w/a", "p", 1, 2));

        let before_instant_two = |m: &Message| m.sim_time.as_nanos() < 2;
        let first = t.drain(&path("sub"), &before_instant_two);
        assert_eq!(seqs_and_times(&first), vec![(0, 1)]);

        let second = t.drain(&path("sub"), &|_| true);
        assert_eq!(seqs_and_times(&second), vec![(1, 2)]);
    }

    /// A message from an earlier instant releases at every later drain, so
    /// whatever was published before it is dropped as it arrives rather than
    /// queued until the subscriber steps.
    #[test]
    fn a_latest_key_holds_one_message_from_an_earlier_instant() {
        let mut t = InProcTransport::new();
        t.subscribe(path("sub"), kx("w/a"), Delivery::Latest);
        for seq in 0..10 {
            t.publish(msg("w/a", "p", seq, seq as i64));
        }

        let held = &t.latest[&path("sub")][&kx("w/a")];
        assert_eq!(seqs_and_times(held), vec![(8, 8), (9, 9)]);
    }

    #[test]
    fn a_key_subscribed_both_ways_delivers_every_message() {
        let mut t = InProcTransport::new();
        t.subscribe(path("sub"), kx("w/**"), Delivery::Latest);
        t.subscribe(path("sub"), kx("w/a"), Delivery::Every);
        t.publish(msg("w/a", "p", 0, 0));
        t.publish(msg("w/a", "p", 1, 1));
        t.publish(msg("w/b", "q", 0, 0));
        t.publish(msg("w/b", "q", 1, 1));

        let released = t.drain(&path("sub"), &|_| true);
        let delivered: Vec<(String, u64)> = released
            .iter()
            .map(|m| (m.key.to_string(), m.seq))
            .collect();
        assert_eq!(
            delivered,
            vec![
                ("w/a".to_string(), 0),
                ("w/a".to_string(), 1),
                ("w/b".to_string(), 1)
            ]
        );
    }
}

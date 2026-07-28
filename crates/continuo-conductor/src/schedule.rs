use std::collections::{BTreeMap, BTreeSet};

use continuo_core::SimTime;

/// The event schedule: due time → declaration indexes of due components.
///
/// `BTreeMap` + `BTreeSet` give deterministic iteration: earliest instant
/// first, declaration order within an instant.
#[derive(Debug, Default)]
pub(crate) struct Schedule {
    queue: BTreeMap<SimTime, BTreeSet<usize>>,
}

impl Schedule {
    pub(crate) fn insert(&mut self, due: SimTime, index: usize) {
        self.queue.entry(due).or_default().insert(index);
    }

    pub(crate) fn earliest(&self) -> Option<SimTime> {
        self.queue.keys().next().copied()
    }

    pub(crate) fn pop_earliest(&mut self) -> Option<(SimTime, BTreeSet<usize>)> {
        self.queue.pop_first()
    }

    /// Unschedules `index` everywhere it is due — called when a component
    /// leaves, so a departed slot can never be stepped.
    pub(crate) fn remove_index(&mut self, index: usize) {
        self.queue.retain(|_, due| {
            due.remove(&index);

            // Return whether this instant still has anyone due; emptied
            // instants are dropped so they never become no-op ticks.
            !due.is_empty()
        });
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A sim-time instant, in nanoseconds.
    fn t_sim_ns(nanos: i64) -> SimTime {
        SimTime::from_nanos(nanos)
    }

    #[test]
    fn removing_an_index_leaves_co_due_components_untouched() {
        let mut schedule = Schedule::default();
        schedule.insert(t_sim_ns(10), 0);
        schedule.insert(t_sim_ns(10), 1);
        schedule.insert(t_sim_ns(20), 0);

        schedule.remove_index(0);

        assert_eq!(schedule.earliest(), Some(t_sim_ns(10)));
        let (time, due) = schedule.pop_earliest().expect("instant 10 survives");
        assert_eq!(time, t_sim_ns(10));
        assert_eq!(due, BTreeSet::from([1]), "only the departed index is gone");
        // t=20 held nothing but the departed index, so it is gone entirely
        // rather than left as an instant with no one due.
        assert_eq!(schedule.earliest(), None);
    }
}

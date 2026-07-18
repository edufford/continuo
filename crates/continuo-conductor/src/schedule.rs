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
}

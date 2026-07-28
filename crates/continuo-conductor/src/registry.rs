use std::collections::BTreeMap;

use continuo_core::{Component, ComponentId, ComponentPath, SimTime};

use crate::error::ConductorError;

/// The component tree as data: node path → declared order of its children.
/// Declared order = registration order, and it is what "earlier sibling"
/// means in the visibility rule.
#[derive(Debug, Default)]
pub(crate) struct Tree {
    /// Every non-leaf node (including the world root, the empty path) maps
    /// to its children's ids in declared order. This ordering is the sole
    /// source of truth for the visibility rule's "earlier sibling branch"
    /// comparison in [`Tree::releases_same_instant`]; leaves never appear
    /// as keys.
    children: BTreeMap<ComponentPath, Vec<ComponentId>>,
}

impl Tree {
    fn note_child(&mut self, parent: &ComponentPath, child: &ComponentId) {
        let children = self.children.entry(parent.clone()).or_default();
        if !children.contains(child) {
            children.push(child.clone());
        }
    }

    fn is_internal_node(&self, path: &ComponentPath) -> bool {
        self.children.contains_key(path)
    }

    /// Forgets a departed leaf, so its id no longer counts as a declared
    /// sibling. Survivors keep their relative order — the only thing the
    /// visibility rule reads — and a later component may reuse the id,
    /// arriving at the end like any new sibling.
    fn forget_child(&mut self, parent: &ComponentPath, child: &ComponentId) {
        let Some(children) = self.children.get_mut(parent) else {
            return;
        };
        children.retain(|c| c != child);
        if children.is_empty() {
            // A composite with no children left is no longer an internal
            // node, so the path is free for a leaf to take. Its own mention
            // in *its* parent's list stays, keeping that branch's position
            // if another child arrives under it later.
            self.children.remove(parent);
        }
    }

    /// The visibility rule's same-instant clause: may `subscriber`, stepping
    /// now, receive a message `publisher` published at this same instant?
    ///
    /// True iff their nearest common ancestor is below the world root (i.e.
    /// they live inside the same composite) and the publisher's branch is
    /// declared earlier than the subscriber's. Cross-actor messages (common
    /// ancestor = world root) always wait for the subscriber's next step.
    pub(crate) fn releases_same_instant(
        &self,
        publisher: &ComponentPath,
        subscriber: &ComponentPath,
    ) -> bool {
        if publisher == subscriber {
            return false;
        }
        let shared = publisher.common_prefix_len(subscriber);
        if shared == 0 {
            return false; // world-level: lockstep isolation
        }
        let (Some(pub_branch), Some(sub_branch)) = (
            publisher.segments().get(shared),
            subscriber.segments().get(shared),
        ) else {
            return false; // one is an ancestor of the other: not siblings
        };
        let ancestor = publisher.prefix(shared);
        let Some(order) = self.children.get(&ancestor) else {
            return false;
        };
        let pub_order = order.iter().position(|c| c == pub_branch);
        let sub_order = order.iter().position(|c| c == sub_branch);

        // Return whether the publisher's branch is declared earlier than
        // the subscriber's.
        matches!((pub_order, sub_order), (Some(p), Some(s)) if p < s)
    }
}

/// One scheduled leaf component and its bookkeeping.
pub(crate) struct Entry {
    pub(crate) path: ComponentPath,
    pub(crate) component: Box<dyn Component>,
    pub(crate) last_step: Option<SimTime>,
    pub(crate) next_seq: u64,
    /// Derived from `(world_seed, path)`; handed to every `StepCtx`.
    pub(crate) component_seed: u64,
}

/// All registered leaves (indexed by declaration order — the deterministic
/// execution order within an instant) plus the tree.
///
/// A slot is `None` once its component has left. Vacating rather than
/// removing is what keeps every surviving index stable: indexes are the
/// execution order within an instant, so shifting them would silently
/// reorder components that had nothing to do with the departure.
#[derive(Default)]
pub(crate) struct Registry {
    pub(crate) entries: Vec<Option<Entry>>,
    by_path: BTreeMap<ComponentPath, usize>,
    pub(crate) tree: Tree,
}

impl Registry {
    /// The live component at `index`, or `None` if that slot has been
    /// vacated by a departure.
    pub(crate) fn entry(&self, index: usize) -> Option<&Entry> {
        self.entries.get(index).and_then(Option::as_ref)
    }

    /// Mutable access to the live component at `index`; `None` for a vacated
    /// slot.
    pub(crate) fn entry_mut(&mut self, index: usize) -> Option<&mut Entry> {
        self.entries.get_mut(index).and_then(Option::as_mut)
    }

    /// Removes a leaf: vacates its slot, frees its path for reuse, and drops
    /// it from its parent's declared children. Returns the vacated index so
    /// the caller can unschedule it, or `None` if no such component is
    /// registered.
    ///
    /// Rejoining under the same path is allowed and arrives like any new
    /// sibling — a fresh slot at the end, and the end of the parent's child
    /// list. Arrival order therefore stays the single source of truth for
    /// both execution order and the visibility rule, so the two can never
    /// disagree about which sibling is "earlier".
    pub(crate) fn remove(&mut self, path: &ComponentPath) -> Option<usize> {
        let index = self.by_path.remove(path)?;
        self.entries[index] = None;
        // Registered paths are always leaves, so both of these hold; the
        // pattern just avoids asserting it.
        if let (Some(parent), Some(id)) = (path.parent(), path.segments().last()) {
            self.tree.forget_child(&parent, id);
        }

        // Return the vacated index; its schedule entries are now stale.
        Some(index)
    }

    /// Registers a leaf under `parent`, creating intermediate tree nodes as
    /// needed. Returns the declaration index and full path.
    pub(crate) fn add(
        &mut self,
        parent: &ComponentPath,
        component: Box<dyn Component>,
        world_seed: u64,
    ) -> Result<(usize, ComponentPath), ConductorError> {
        let path = parent.join(component.id());

        if self.by_path.contains_key(&path) {
            return Err(ConductorError::DuplicatePath(path));
        }
        if self.tree.is_internal_node(&path) {
            return Err(ConductorError::PathConflict {
                existing: path.clone(),
                new: path,
            });
        }
        // No existing leaf may be an ancestor of the new path. Exclusive
        // range on purpose: only *strict* prefixes (depths 1..len-1 plus the
        // parent itself) are ancestors — depth == len is the full path,
        // already handled by the duplicate check above.
        for depth in 1..path.segments().len() {
            let ancestor = path.prefix(depth);
            if self.by_path.contains_key(&ancestor) {
                return Err(ConductorError::PathConflict {
                    existing: ancestor,
                    new: path,
                });
            }
        }

        // Inclusive range here, in contrast: every segment of the new path
        // (including the leaf itself, at depth == len) must be recorded as a
        // child of the node above it.
        for depth in 1..=path.segments().len() {
            let node_parent = path.prefix(depth - 1);
            self.tree
                .note_child(&node_parent, &path.segments()[depth - 1]);
        }

        let index = self.entries.len();
        self.by_path.insert(path.clone(), index);
        let component_seed = continuo_core::derive_component_seed(world_seed, &path);
        self.entries.push(Some(Entry {
            path: path.clone(),
            component,
            last_step: None,
            next_seq: 0,
            component_seed,
        }));

        // Return the new entry's declaration index and full path.
        Ok((index, path))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use continuo_core::{KeyExpr, StepCtx};

    struct Dummy(&'static str);
    impl Component for Dummy {
        fn id(&self) -> ComponentId {
            ComponentId::new(self.0).unwrap()
        }
        fn subscriptions(&self) -> Vec<KeyExpr> {
            Vec::new()
        }
        fn step(&mut self, ctx: &mut StepCtx) -> SimTime {
            ctx.now() + continuo_core::SimDuration::from_millis(1)
        }
    }

    fn path(s: &str) -> ComponentPath {
        ComponentPath::parse(s).unwrap()
    }

    /// These tests exercise tree structure only; the seed just has to be
    /// some fixed value, since no component here draws random numbers.
    const TEST_WORLD_SEED: u64 = 0;

    #[test]
    fn same_instant_rule() {
        let mut reg = Registry::default();
        let car1 = path("car1");
        let car2 = path("car2");
        reg.add(&car1, Box::new(Dummy("controller")), TEST_WORLD_SEED)
            .unwrap();
        reg.add(&car1, Box::new(Dummy("physics")), TEST_WORLD_SEED)
            .unwrap();
        reg.add(&car2, Box::new(Dummy("controller")), TEST_WORLD_SEED)
            .unwrap();

        let t = &reg.tree;
        // Earlier sibling → later sibling: released.
        assert!(t.releases_same_instant(&path("car1/controller"), &path("car1/physics")));
        // Later sibling → earlier sibling: not released.
        assert!(!t.releases_same_instant(&path("car1/physics"), &path("car1/controller")));
        // Cross-actor: never released same-instant.
        assert!(!t.releases_same_instant(&path("car1/physics"), &path("car2/controller")));
        // Self: never.
        assert!(!t.releases_same_instant(&path("car1/physics"), &path("car1/physics")));
    }

    #[test]
    fn removing_a_leaf_vacates_its_slot_without_shifting_others() {
        let mut reg = Registry::default();
        let car1 = path("car1");
        let (controller, controller_path) = reg
            .add(&car1, Box::new(Dummy("controller")), TEST_WORLD_SEED)
            .unwrap();
        let (physics, _) = reg
            .add(&car1, Box::new(Dummy("physics")), TEST_WORLD_SEED)
            .unwrap();

        assert_eq!(reg.remove(&controller_path), Some(controller));

        // The slot is empty, and the survivor keeps the index that fixes its
        // execution order within an instant.
        assert!(reg.entry(controller).is_none());
        assert_eq!(
            reg.entry(physics).expect("physics survives").path,
            path("car1/physics")
        );
        // A departed component is no longer a declared sibling, so nothing
        // waits on it for same-instant delivery.
        assert!(
            !reg.tree
                .releases_same_instant(&controller_path, &path("car1/physics"))
        );
    }

    #[test]
    fn a_departed_path_can_be_reused_and_arrives_as_the_newest_sibling() {
        let mut reg = Registry::default();
        let car1 = path("car1");
        let (_, controller_path) = reg
            .add(&car1, Box::new(Dummy("controller")), TEST_WORLD_SEED)
            .unwrap();
        reg.add(&car1, Box::new(Dummy("physics")), TEST_WORLD_SEED)
            .unwrap();
        // Declared first, so its same-instant output reaches the physics.
        assert!(
            reg.tree
                .releases_same_instant(&controller_path, &path("car1/physics"))
        );

        reg.remove(&controller_path);
        let (rejoined, _) = reg
            .add(&car1, Box::new(Dummy("controller")), TEST_WORLD_SEED)
            .unwrap();

        // Rejoining is a new arrival, not a restoration: a fresh slot at the
        // end, and the end of the parent's child list. The physics is now the
        // earlier sibling — and note the two orders agree, which is the point.
        // The visibility rule reads the tree while execution reads the index,
        // so if arrival did not drive both they could disagree about who is
        // "earlier" and a same-instant hand-off would silently stop working.
        assert_eq!(rejoined, 2);
        assert!(
            !reg.tree
                .releases_same_instant(&controller_path, &path("car1/physics"))
        );
        assert!(
            reg.tree
                .releases_same_instant(&path("car1/physics"), &controller_path)
        );
    }

    #[test]
    fn removing_something_unregistered_reports_it() {
        let mut reg = Registry::default();
        assert_eq!(reg.remove(&path("nobody")), None);
    }

    #[test]
    fn path_conflicts_rejected() {
        let mut reg = Registry::default();
        let root = ComponentPath::root();
        reg.add(&root, Box::new(Dummy("a")), TEST_WORLD_SEED)
            .unwrap();
        // Duplicate leaf.
        assert!(matches!(
            reg.add(&root, Box::new(Dummy("a")), TEST_WORLD_SEED),
            Err(ConductorError::DuplicatePath(_))
        ));
        // Leaf "a" cannot become a composite.
        assert!(matches!(
            reg.add(&path("a"), Box::new(Dummy("b")), TEST_WORLD_SEED),
            Err(ConductorError::PathConflict { .. })
        ));
        // Composite "c" (via c/x) cannot become a leaf.
        reg.add(&path("c"), Box::new(Dummy("x")), TEST_WORLD_SEED)
            .unwrap();
        assert!(matches!(
            reg.add(&root, Box::new(Dummy("c")), TEST_WORLD_SEED),
            Err(ConductorError::PathConflict { .. })
        ));
    }
}

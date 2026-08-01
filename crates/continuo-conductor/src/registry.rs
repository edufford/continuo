use std::collections::BTreeMap;

use continuo_core::{Component, ComponentId, ComponentPath, SimTime};

use crate::error::ConductorError;
use crate::timing::StepTiming;

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
    fn note_child(&mut self, parent_path: &ComponentPath, child_name: &ComponentId) {
        let children = self.children.entry(parent_path.clone()).or_default();
        if !children.contains(child_name) {
            children.push(child_name.clone());
        }
    }

    fn is_internal_node(&self, path: &ComponentPath) -> bool {
        self.children.contains_key(path)
    }

    /// Forgets a departed child, so its id no longer counts as a declared
    /// sibling. Survivors keep their relative order — the only thing the
    /// visibility rule reads — and a later component may reuse the id,
    /// arriving at the end like any new sibling.
    ///
    /// **Recursive, upwards.** Forgetting the last child leaves its parent
    /// empty, and an empty node is not a node: it has to be forgotten from
    /// *its* parent in turn, on up until a node still has survivors or the
    /// world root is reached. So removing one leaf can retire a whole spine
    /// of composites that existed only to hold it, and `child_name` is not
    /// always a leaf — above the first step it is whatever branch just
    /// emptied.
    fn forget_child(&mut self, parent_path: &ComponentPath, child_name: &ComponentId) {
        let Some(children) = self.children.get_mut(parent_path) else {
            return;
        };
        // The forgetting itself. `retain` is a keep-filter, so the condition
        // is the survivors rather than the departure: everything that is
        // *not* `child_name` stays, in the order it was already in.
        children.retain(|c| c != child_name);
        // Survivors keep this node internal, and keep it a declared sibling
        // of its own siblings, so nothing above it changes. This is where
        // the walk up stops — and for the ordinary case, one leaf leaving a
        // composite that still has others, it stops right here.
        if !children.is_empty() {
            return;
        }
        // Emptied. The node stops being internal, so its path is free for a
        // leaf to take — and it stops being a declared sibling of its own
        // siblings, so a composite rebuilt here later arrives at the end of
        // that list like any new branch. Leaving it in place would restore
        // its old position instead, which is the disagreement between
        // arrival order and tree order that vacating slots exists to
        // prevent.
        self.children.remove(parent_path);
        // The pattern doubles as the other stopping condition: the world
        // root has no parent and no last segment, so the walk ends there
        // rather than trying to forget the root from something above it.
        if let (Some(grandparent_path), Some(child_name)) =
            (parent_path.parent(), parent_path.segments().last())
        {
            let child_name = child_name.clone();
            self.forget_child(&grandparent_path, &child_name);
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
    /// What this component's steps may cost in wall time, as declared when
    /// it joined.
    pub(crate) timing: StepTiming,
    /// How many of its steps have exceeded the soft half of that. Purely
    /// diagnostic — nothing in the run reads it.
    pub(crate) budget_misses: u64,
}

/// All registered leaves (indexed by declaration order — the deterministic
/// execution order within an instant) plus the tree.
///
/// A slot is `None` once its component has left. Vacating rather than
/// removing is what keeps every surviving index stable: indexes are the
/// execution order within an instant, so shifting them would silently
/// reorder components that had nothing to do with the departure.
// TODO(PLAN "Deferred"): vacated slots are never reclaimed, so this grows
// with *total* joins rather than with live components, and the due loop
// steps over the holes for the rest of the run. Harmless at demo scale;
// revisit for a long run that churns many actors.
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

    /// The declaration index of the component registered at `path`, if one
    /// is — for checking membership without removing anything.
    pub(crate) fn index_of(&self, path: &ComponentPath) -> Option<usize> {
        self.by_path.get(path).copied()
    }

    /// Mutable access to the component registered at `path`; `None` if
    /// nothing is. A map lookup rather than the direct indexing of
    /// [`Self::entry_mut`], for callers that hold the path and would
    /// otherwise have to carry an index alongside it.
    pub(crate) fn entry_mut_by_path(&mut self, path: &ComponentPath) -> Option<&mut Entry> {
        let index = self.index_of(path)?;

        // Return the entry at that slot; the index came from `by_path`, so
        // it is in range and live.
        self.entry_mut(index)
    }

    /// Every registered leaf strictly beneath `path`, in declaration order —
    /// what a composite means, given only leaves are ever registered.
    ///
    /// Empty when `path` is itself a leaf, or names nothing at all; the
    /// caller distinguishes those with [`Self::index_of`].
    ///
    /// Declaration order, not path order, because that is the order the
    /// removals must happen in to be reproducible: it is the order the
    /// components step in, and the order their departures reach the log.
    pub(crate) fn components_under(&self, path: &ComponentPath) -> Vec<ComponentPath> {
        let depth = path.segments().len();
        let mut found: Vec<(usize, ComponentPath)> = self
            .by_path
            .iter()
            .filter(|(candidate, _)| {
                candidate.segments().len() > depth && candidate.prefix(depth) == *path
            })
            .map(|(candidate, &index)| (index, candidate.clone()))
            .collect();
        found.sort_unstable_by_key(|(index, _)| *index);

        // Return the subtree's components, earliest-declared first.
        found.into_iter().map(|(_, path)| path).collect()
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
        if let (Some(parent_path), Some(child_name)) = (path.parent(), path.segments().last()) {
            self.tree.forget_child(&parent_path, child_name);
        }

        // Return the vacated index; its schedule entries are now stale.
        Some(index)
    }

    /// Registers a leaf under `parent_path`, creating intermediate tree nodes
    /// as needed. Returns the declaration index and full path.
    pub(crate) fn add(
        &mut self,
        parent_path: &ComponentPath,
        component: Box<dyn Component>,
        world_seed: u64,
        timing: StepTiming,
    ) -> Result<(usize, ComponentPath), ConductorError> {
        let path = parent_path.join(component.id());

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
            let node_parent_path = path.prefix(depth - 1);
            self.tree
                .note_child(&node_parent_path, &path.segments()[depth - 1]);
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
            timing,
            budget_misses: 0,
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

    /// Nor does any of them declare a step limit — the registry only stores
    /// what it is handed.
    const NO_LIMITS: StepTiming = StepTiming::unlimited();

    #[test]
    fn same_instant_rule() {
        let mut reg = Registry::default();
        let car1 = path("car1");
        let car2 = path("car2");
        reg.add(
            &car1,
            Box::new(Dummy("controller")),
            TEST_WORLD_SEED,
            NO_LIMITS,
        )
        .unwrap();
        reg.add(
            &car1,
            Box::new(Dummy("physics")),
            TEST_WORLD_SEED,
            NO_LIMITS,
        )
        .unwrap();
        reg.add(
            &car2,
            Box::new(Dummy("controller")),
            TEST_WORLD_SEED,
            NO_LIMITS,
        )
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
            .add(
                &car1,
                Box::new(Dummy("controller")),
                TEST_WORLD_SEED,
                NO_LIMITS,
            )
            .unwrap();
        let (physics, _) = reg
            .add(
                &car1,
                Box::new(Dummy("physics")),
                TEST_WORLD_SEED,
                NO_LIMITS,
            )
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
            .add(
                &car1,
                Box::new(Dummy("controller")),
                TEST_WORLD_SEED,
                NO_LIMITS,
            )
            .unwrap();
        reg.add(
            &car1,
            Box::new(Dummy("physics")),
            TEST_WORLD_SEED,
            NO_LIMITS,
        )
        .unwrap();
        // Declared first, so its same-instant output reaches the physics.
        assert!(
            reg.tree
                .releases_same_instant(&controller_path, &path("car1/physics"))
        );

        reg.remove(&controller_path);
        let (rejoined, _) = reg
            .add(
                &car1,
                Box::new(Dummy("controller")),
                TEST_WORLD_SEED,
                NO_LIMITS,
            )
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
        reg.add(&root, Box::new(Dummy("a")), TEST_WORLD_SEED, NO_LIMITS)
            .unwrap();
        // Duplicate leaf.
        assert!(matches!(
            reg.add(&root, Box::new(Dummy("a")), TEST_WORLD_SEED, NO_LIMITS),
            Err(ConductorError::DuplicatePath(_))
        ));
        // Leaf "a" cannot become a composite.
        assert!(matches!(
            reg.add(&path("a"), Box::new(Dummy("b")), TEST_WORLD_SEED, NO_LIMITS),
            Err(ConductorError::PathConflict { .. })
        ));
        // Composite "c" (via c/x) cannot become a leaf.
        reg.add(&path("c"), Box::new(Dummy("x")), TEST_WORLD_SEED, NO_LIMITS)
            .unwrap();
        assert!(matches!(
            reg.add(&root, Box::new(Dummy("c")), TEST_WORLD_SEED, NO_LIMITS),
            Err(ConductorError::PathConflict { .. })
        ));
    }

    /// Registers `id` under `parent_path`, returning the full path.
    fn add(reg: &mut Registry, parent_path: &str, id: &'static str) -> ComponentPath {
        let (_, path) = reg
            .add(
                &path(parent_path),
                Box::new(Dummy(id)),
                TEST_WORLD_SEED,
                NO_LIMITS,
            )
            .expect("registration succeeds");

        // Return where it landed.
        path
    }

    #[test]
    fn leaves_under_returns_a_subtree_in_declaration_order() {
        let mut reg = Registry::default();
        // Declared z before a, so declaration order and the `by_path`
        // BTreeMap's path order disagree. Declaration order is the one
        // removals must follow, because it is the order these would step.
        add(&mut reg, "car1", "z");
        add(&mut reg, "car1", "a");
        add(&mut reg, "car2", "physics");

        assert_eq!(
            reg.components_under(&path("car1")),
            vec![path("car1/z"), path("car1/a")],
            "declaration order, not alphabetical"
        );
        assert_eq!(
            reg.components_under(&path("car2")),
            vec![path("car2/physics")],
            "and only the subtree named"
        );
    }

    #[test]
    fn leaves_under_finds_nothing_under_a_leaf_or_a_stranger() {
        let mut reg = Registry::default();
        let physics = add(&mut reg, "car1", "physics");

        // Strict descendants only: a leaf is not under itself, which is
        // what lets a caller tell "this is a leaf" from "this is a
        // composite" by asking both questions.
        assert!(reg.components_under(&physics).is_empty());
        assert!(reg.components_under(&path("nobody")).is_empty());
    }

    #[test]
    fn leaves_under_reaches_through_nested_composites() {
        let mut reg = Registry::default();
        add(&mut reg, "car1/sensors", "imu");
        add(&mut reg, "car1", "physics");

        assert_eq!(
            reg.components_under(&path("car1")),
            vec![path("car1/sensors/imu"), path("car1/physics")],
            "depth is irrelevant; declaration order still decides"
        );
        assert_eq!(
            reg.components_under(&path("car1/sensors")),
            vec![path("car1/sensors/imu")]
        );
    }

    #[test]
    fn emptying_a_node_forgets_it_all_the_way_up() {
        // One leaf can retire a whole spine of composites that existed only
        // to hold it.
        let mut reg = Registry::default();
        let imu = add(&mut reg, "car1/sensors", "imu");
        assert!(reg.tree.is_internal_node(&path("car1")));
        assert!(reg.tree.is_internal_node(&path("car1/sensors")));

        reg.remove(&imu);

        assert!(!reg.tree.is_internal_node(&path("car1/sensors")));
        assert!(
            !reg.tree.is_internal_node(&path("car1")),
            "emptied in turn by its only child going"
        );
        assert!(
            reg.tree.children.is_empty(),
            "and the root forgot car1, so nothing is left declared anywhere"
        );
    }

    #[test]
    fn the_walk_up_stops_at_the_first_survivor() {
        let mut reg = Registry::default();
        let imu = add(&mut reg, "car1/sensors", "imu");
        add(&mut reg, "car1", "physics");

        reg.remove(&imu);

        assert!(
            !reg.tree.is_internal_node(&path("car1/sensors")),
            "the sensors node emptied"
        );
        assert!(
            reg.tree.is_internal_node(&path("car1")),
            "but physics survives, so car1 stays a node"
        );
        assert_eq!(
            reg.tree.children[&path("car1")],
            vec![ComponentId::new("physics").expect("valid id")],
            "and the emptied branch is gone from its child list"
        );
    }
}

use std::collections::BTreeMap;

use continuo_core::{Component, ComponentId, ComponentPath, SimTime};

use crate::error::ConductorError;

/// The component tree as data: node path → declared order of its children.
/// Declared order = registration order, and it is what "earlier sibling"
/// means in the visibility rule.
#[derive(Debug, Default)]
pub(crate) struct Tree {
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
        matches!((pub_order, sub_order), (Some(p), Some(s)) if p < s)
    }
}

/// One scheduled leaf component and its bookkeeping.
pub(crate) struct Entry {
    pub(crate) path: ComponentPath,
    pub(crate) component: Box<dyn Component>,
    pub(crate) last_step: Option<SimTime>,
    pub(crate) next_seq: u64,
}

/// All registered leaves (indexed by declaration order — the deterministic
/// execution order within an instant) plus the tree.
#[derive(Default)]
pub(crate) struct Registry {
    pub(crate) entries: Vec<Entry>,
    by_path: BTreeMap<ComponentPath, usize>,
    pub(crate) tree: Tree,
}

impl Registry {
    /// Registers a leaf under `parent`, creating intermediate tree nodes as
    /// needed. Returns the declaration index and full path.
    pub(crate) fn add(
        &mut self,
        parent: &ComponentPath,
        component: Box<dyn Component>,
    ) -> Result<(usize, ComponentPath), ConductorError> {
        let path = parent.child(component.id());

        if self.by_path.contains_key(&path) {
            return Err(ConductorError::DuplicatePath(path));
        }
        if self.tree.is_internal_node(&path) {
            return Err(ConductorError::PathConflict {
                existing: path.clone(),
                new: path,
            });
        }
        // No existing leaf may be an ancestor of the new path.
        for depth in 1..path.segments().len() {
            let ancestor = path.prefix(depth);
            if self.by_path.contains_key(&ancestor) {
                return Err(ConductorError::PathConflict {
                    existing: ancestor,
                    new: path,
                });
            }
        }

        for depth in 1..=path.segments().len() {
            let node_parent = path.prefix(depth - 1);
            self.tree
                .note_child(&node_parent, &path.segments()[depth - 1]);
        }

        let index = self.entries.len();
        self.by_path.insert(path.clone(), index);
        self.entries.push(Entry {
            path: path.clone(),
            component,
            last_step: None,
            next_seq: 0,
        });
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

    #[test]
    fn same_instant_rule() {
        let mut reg = Registry::default();
        let car1 = path("car1");
        let car2 = path("car2");
        reg.add(&car1, Box::new(Dummy("controller"))).unwrap();
        reg.add(&car1, Box::new(Dummy("physics"))).unwrap();
        reg.add(&car2, Box::new(Dummy("controller"))).unwrap();

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
    fn path_conflicts_rejected() {
        let mut reg = Registry::default();
        let root = ComponentPath::root();
        reg.add(&root, Box::new(Dummy("a"))).unwrap();
        // Duplicate leaf.
        assert!(matches!(
            reg.add(&root, Box::new(Dummy("a"))),
            Err(ConductorError::DuplicatePath(_))
        ));
        // Leaf "a" cannot become a composite.
        assert!(matches!(
            reg.add(&path("a"), Box::new(Dummy("b"))),
            Err(ConductorError::PathConflict { .. })
        ));
        // Composite "c" (via c/x) cannot become a leaf.
        reg.add(&path("c"), Box::new(Dummy("x"))).unwrap();
        assert!(matches!(
            reg.add(&root, Box::new(Dummy("c"))),
            Err(ConductorError::PathConflict { .. })
        ));
    }
}

//! Seed derivation: turning the one world seed into the per-component and
//! per-step seeds that every deterministic random stream starts from
//! (PLAN.md, Determinism).
//!
//! This sits between [`crate::hash`] and [`crate::random`] and belongs to
//! neither: it uses the hash to fold variable-length names (component
//! paths) down to 64 bits, and the generator's scrambler to combine two
//! 64-bit values into one that is unrelated to both. The result is a seed,
//! not a fingerprint. Nothing here feeds the determinism check, and
//! nothing in the determinism check feeds these.
//!
//! Why derive at all rather than hand every component the world seed:
//! adding, removing, or reordering components must not shift anyone else's
//! random stream, so each component's seed comes from its own identity.

use crate::hash::hash_bytes;
use crate::ids::ComponentPath;
use crate::random::{GOLDEN_GAMMA, RandomSplitMix64};

/// Combines two seeds into one, unrelated to either: spread `b` across all
/// 64 bits, fold it into `a`, and take one SplitMix64 scramble of the
/// result. Structured, near-consecutive inputs (paths of one actor, times
/// one nanosecond apart) still produce unrelated streams.
pub fn mix_seeds(a: u64, b: u64) -> u64 {
    // Return one scramble of the combined seeds.
    RandomSplitMix64::new(a ^ b.wrapping_mul(GOLDEN_GAMMA)).next_u64()
}

/// The per-component seed: `(world_seed, component_path)` per PLAN.md.
pub fn derive_component_seed(world_seed: u64, path: &ComponentPath) -> u64 {
    // Return the seed rooted at this component's place in the world.
    mix_seeds(world_seed, hash_bytes(path.to_string().as_bytes()))
}

/// The per-step seed: `(component_seed, sim time in nanoseconds)`, behind
/// [`StepCtx::step_random`](crate::StepCtx::step_random).
pub fn derive_step_seed(component_seed: u64, now_nanos: i64) -> u64 {
    // Return the seed for this component's stream at this instant.
    mix_seeds(component_seed, now_nanos as u64)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn component_seeds_differ_by_path_and_world_seed() {
        let p1 = ComponentPath::parse("car1/physics").unwrap();
        let p2 = ComponentPath::parse("car2/physics").unwrap();
        assert_ne!(derive_component_seed(1, &p1), derive_component_seed(1, &p2));
        assert_ne!(derive_component_seed(1, &p1), derive_component_seed(2, &p1));
        assert_eq!(derive_component_seed(1, &p1), derive_component_seed(1, &p1));
    }

    #[test]
    fn step_seeds_differ_by_the_smallest_time_step() {
        let component_seed = derive_component_seed(42, &ComponentPath::parse("car1").unwrap());
        assert_ne!(
            derive_step_seed(component_seed, 1_000_000_000),
            derive_step_seed(component_seed, 1_000_000_001)
        );
    }
}

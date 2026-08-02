//! Deterministic pseudo-random numbers for sim logic (milestone 2).
//!
//! Owned implementation for the same reason as the hash: bit identical on
//! every platform, toolchain, and version, forever. External RNG crates
//! explicitly do not promise stream stability across releases (e.g.
//! `rand`'s `SmallRng` documents that its algorithm may change).
//!
//! Why SplitMix64 specifically:
//!
//! - **Any seed is a good seed.** Its output is a one-to-one scramble of a
//!   simple counter (distinct states can never collapse into one), so there
//!   are no weak seeds, no zero-state lockup, and no warm-up draws needed,
//!   unlike the xorshift/xoshiro family, whose
//!   own authors recommend seeding *via SplitMix64* for exactly that
//!   reason. That matters here because seeds arrive from structured,
//!   correlated inputs (hashed paths, mixed sim times, world seed
//!   arithmetic), and consecutive-ish seeds must still yield unrelated
//!   streams. The same property makes it the scrambling primitive behind
//!   [`crate::seed`]'s derivation of child seeds from `(world_seed, path)`
//!   and `(component_seed, time)`, so one algorithm serves both roles.
//! - **Stateless-friendly**: a single u64 of state, so
//!   `StepCtx::step_random()`'s fresh-per-step streams cost nothing to
//!   construct and components that persist a stream store 8 bytes.
//! - **Small enough to be obviously correct and trivially portable**: a
//!   handful of shift/multiply constants, verified against the reference
//!   implementation's vectors (below), reimplementable identically in
//!   Python tooling if it ever needs to reproduce a stream.
//! - **Statistical quality is ample for simulation noise** (passes BigCrush
//!   in its 64-bit form); this is not a cryptographic generator, and
//!   sequential-correlation-sensitive Monte Carlo work would warrant an
//!   upgrade (a versioned change, like the hash).
//!
//! Seeding rules (PLAN.md, Determinism): every component's randomness
//! derives from `(world_seed, component_path)`, never OS entropy, never
//! wall time. See [`crate::seed`] for the derivation itself.

/// SplitMix64. Reference: Steele, Lea, and Flood, "Fast Splittable
/// Pseudorandom Number Generators" (OOPSLA 2014); constants as in the
/// public-domain reference implementation by Sebastiano Vigna
/// (<https://prng.di.unimi.it/splitmix64.c>).
#[derive(Debug, Clone)]
pub struct RandomSplitMix64 {
    state: u64,
}

impl RandomSplitMix64 {
    pub const fn new(seed: u64) -> Self {
        RandomSplitMix64 { state: seed }
    }

    pub fn next_u64(&mut self) -> u64 {
        self.state = self.state.wrapping_add(GOLDEN_GAMMA);
        let mut z = self.state;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);

        // Return the fully scrambled output for this state increment.
        z ^ (z >> 31)
    }

    /// Uniform in [0, 1) with 53 bits of precision.
    pub fn next_f64(&mut self) -> f64 {
        (self.next_u64() >> 11) as f64 * (1.0 / (1u64 << 53) as f64)
    }

    /// Uniform in [lo, hi).
    pub fn range_f64(&mut self, lo: f64, hi: f64) -> f64 {
        lo + self.next_f64() * (hi - lo)
    }
}

/// SplitMix64's state increment: the 64-bit odd approximation of the golden
/// ratio, chosen by the algorithm's authors so successive states are spread
/// as evenly as possible. Also used by [`crate::seed`] to spread one input
/// before combining it with another.
pub(crate) const GOLDEN_GAMMA: u64 = 0x9E37_79B9_7F4A_7C15;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn deterministic_stream() {
        let mut a = RandomSplitMix64::new(42);
        let mut b = RandomSplitMix64::new(42);
        for _ in 0..100 {
            assert_eq!(a.next_u64(), b.next_u64());
        }
    }

    #[test]
    fn splitmix64_reference_vector() {
        // First outputs of splitmix64 with seed 0, from the reference
        // implementation.
        let mut r = RandomSplitMix64::new(0);
        assert_eq!(r.next_u64(), 0xE220_A839_7B1D_CDAF);
        assert_eq!(r.next_u64(), 0x6E78_9E6A_A1B9_65F4);
        assert_eq!(r.next_u64(), 0x06C4_5D18_8009_454F);
    }

    #[test]
    fn f64_in_unit_interval() {
        let mut r = RandomSplitMix64::new(7);
        for _ in 0..1000 {
            let x = r.next_f64();
            assert!((0.0..1.0).contains(&x));
        }
    }
}

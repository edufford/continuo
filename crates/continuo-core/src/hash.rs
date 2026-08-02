//! Deterministic hashing for the per-tick determinism check (milestone 2).
//!
//! Implemented here rather than pulled in as a dependency so the hash
//! function - like time formatting and the RNG - is fully owned: stable
//! constants, identical on every platform and toolchain, forever.
//!
//! Why FNV-1a 64 specifically:
//!
//! - **The job is fingerprinting, not security or speed.** Divergence
//!   detection compares hashes of runs that are either identical (equal
//!   hashes) or different (any input difference scrambling the output at
//!   all catches it); nobody is crafting collisions, and the bytes hashed
//!   per tick are tiny, so cryptographic strength (SHA-2) and
//!   throughput-optimized designs (xxHash) buy nothing here.
//! - **Small enough to be obviously correct and trivially portable**: two
//!   constants and a byte loop - easy to audit against published test
//!   vectors (below), impossible to get subtly platform-dependent, and
//!   reimplementable in one line anywhere else that ever needs to check a
//!   digest (e.g. Python tooling reading event logs).
//! - **Byte-at-a-time streaming** fits the conductor's incremental
//!   absorption (paths, times, payloads, state) with no block/finalize
//!   bookkeeping, and chaining ticks is just resuming from the previous
//!   digest.
//!
//! If hash quality ever becomes a real concern (it shouldn't for
//! regression fingerprinting), swapping the algorithm is a versioned
//! event-log change, not an architectural one.

/// FNV-1a, 64-bit. Reference: Fowler/Noll/Vo,
/// <http://www.isthe.com/chongo/tech/comp/fnv/>
#[derive(Debug, Clone, Copy)]
pub struct HashFnv1a64(u64);

const FNV_OFFSET_BASIS: u64 = 0xcbf2_9ce4_8422_2325;
const FNV_PRIME: u64 = 0x0000_0100_0000_01b3;

impl HashFnv1a64 {
    pub const fn new() -> Self {
        HashFnv1a64(FNV_OFFSET_BASIS)
    }

    /// Resumes hashing from a previous hash value - used to chain the
    /// running world hash across ticks.
    pub const fn resume(hash: u64) -> Self {
        HashFnv1a64(hash)
    }

    pub fn write(&mut self, bytes: &[u8]) {
        let mut h = self.0;
        for &b in bytes {
            h ^= b as u64;
            h = h.wrapping_mul(FNV_PRIME);
        }
        self.0 = h;
    }

    pub fn write_u64(&mut self, value: u64) {
        self.write(&value.to_le_bytes());
    }

    pub fn write_i64(&mut self, value: i64) {
        self.write(&value.to_le_bytes());
    }

    pub const fn finish(self) -> u64 {
        self.0
    }
}

impl Default for HashFnv1a64 {
    fn default() -> Self {
        Self::new()
    }
}

/// One-shot convenience.
pub fn hash_bytes(bytes: &[u8]) -> u64 {
    let mut h = HashFnv1a64::new();
    h.write(bytes);

    // Return the hash of the full byte slice.
    h.finish()
}

/// Serde helpers serializing a `u64` hash as a fixed-width hex string
/// (`"cbf29ce484222325"`) - human-scannable in logs and immune to any
/// reader that treats JSON numbers as f64.
pub mod hex_u64 {
    use serde::{Deserialize, Deserializer, Serializer, de};

    pub fn serialize<S: Serializer>(value: &u64, serializer: S) -> Result<S::Ok, S::Error> {
        serializer.serialize_str(&format!("{value:016x}"))
    }

    pub fn deserialize<'de, D: Deserializer<'de>>(deserializer: D) -> Result<u64, D::Error> {
        let raw = String::deserialize(deserializer)?;

        // Return the hash value parsed back from its hex text.
        u64::from_str_radix(&raw, 16).map_err(de::Error::custom)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn known_vectors() {
        // Published FNV-1a 64 test vectors.
        assert_eq!(hash_bytes(b""), 0xcbf2_9ce4_8422_2325);
        assert_eq!(hash_bytes(b"a"), 0xaf63_dc4c_8601_ec8c);
        assert_eq!(hash_bytes(b"foobar"), 0x85944171f73967e8);
    }

    #[test]
    fn incremental_equals_one_shot() {
        let mut h = HashFnv1a64::new();
        h.write(b"foo");
        h.write(b"bar");
        assert_eq!(h.finish(), hash_bytes(b"foobar"));
    }

    #[test]
    fn resume_chains() {
        let first = hash_bytes(b"tick1");
        let mut chained = HashFnv1a64::resume(first);
        chained.write(b"tick2");
        let mut manual = HashFnv1a64::new();
        manual.write(b"tick1");
        manual.write(b"tick2");
        assert_eq!(chained.finish(), manual.finish());
    }

    #[test]
    fn hex_round_trip() {
        #[derive(serde::Serialize, serde::Deserialize)]
        struct H(#[serde(with = "hex_u64")] u64);
        let json = serde_json::to_string(&H(0xcbf2_9ce4_8422_2325)).unwrap();
        assert_eq!(json, r#""cbf29ce484222325""#);
        let back: H = serde_json::from_str(&json).unwrap();
        assert_eq!(back.0, 0xcbf2_9ce4_8422_2325);
    }
}

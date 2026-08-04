use std::fmt;

use serde::{Deserialize, Deserializer, Serialize, de};

use crate::error::CoreError;

/// Root chunk every simulation key sits under: `continuo/{world}/...`.
///
/// Here rather than with any one publisher because it is the namespace itself,
/// which nothing in particular owns. There is then one place to change if the
/// root ever moves, and an observer republishing elsewhere can say where the
/// simulation's keys start without spelling it out for itself.
///
/// Nothing needs to concatenate this by hand: [`KeyExpr::new_rooted`] builds keys
/// under it and owns the separator, so no call site has to remember one.
///
/// Tests that assert on whole key strings deliberately keep writing them out.
/// A test built from the same constant as the code would agree with it however
/// the constant changed, which is the opposite of what pinning a wire format is
/// for.
pub const KEY_ROOT: &str = "continuo";

/// A key expression following Zenoh keyexpr syntax: `/`-separated chunks,
/// where a chunk may be a literal, `*` (exactly one chunk), or `**` (any
/// number of chunks, including zero).
///
/// This is the minimal subset the in-process router needs; Zenoh's own
/// matcher takes over when the Zenoh transport lands (milestone 7).
// TODO(M7): validate against (or replace with) zenoh's keyexpr type so
// in-proc and distributed matching can never disagree.
#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize)]
#[serde(transparent)]
pub struct KeyExpr(String);

impl KeyExpr {
    /// A validated key expression from a **complete** key, taken exactly as
    /// given.
    ///
    /// Nothing is added: this is also the deserialization path, where a key
    /// arrives already whole. For building one under the simulation root, use
    /// [`Self::new_rooted`], which is what almost every publisher wants and
    /// which saves spelling [`KEY_ROOT`] out.
    pub fn new(expr: impl Into<String>) -> Result<Self, CoreError> {
        let expr = expr.into();
        let err = |reason: &str| CoreError::InvalidKeyExpr {
            expr: expr.clone(),
            reason: reason.to_string(),
        };
        if expr.is_empty() {
            return Err(err("must be non-empty"));
        }
        for chunk in expr.split('/') {
            match chunk {
                "" => return Err(err("empty chunk (leading, trailing, or double '/')")),
                "*" | "**" => {}
                literal if literal.contains(['*', '$', '?', '#']) => {
                    return Err(err("literal chunks may not contain wildcard characters"));
                }
                _ => {}
            }
        }

        // Return the validated key expression.
        Ok(KeyExpr(expr))
    }

    /// A key under the simulation root: `continuo/{path}`.
    ///
    /// The separator lives here so that no call site has to remember it.
    /// Concatenating [`KEY_ROOT`] by hand is how `continuodemo/actor/car1/pose`
    /// happens, which is a perfectly valid key expression that simply nothing
    /// subscribes to, so the mistake is silent.
    ///
    /// Separate from [`Self::new`] rather than folded into it because `new` is
    /// also the deserialization path, and a serialized key is already complete:
    /// rooting there would double the root on every round trip.
    pub fn new_rooted(path: impl fmt::Display) -> Result<Self, CoreError> {
        // Return the key, validated like any other.
        KeyExpr::new(format!("{KEY_ROOT}/{path}"))
    }

    pub fn as_str(&self) -> &str {
        &self.0
    }

    fn chunks(&self) -> impl Iterator<Item = &str> {
        self.0.split('/')
    }

    /// Whether this (possibly wildcarded) expression matches a concrete key.
    ///
    /// `key` is expected to be wildcard-free (which is true of every
    /// published key; wildcards live only in subscriptions).
    pub fn matches(&self, key: &KeyExpr) -> bool {
        let expr: Vec<&str> = self.chunks().collect();
        let key: Vec<&str> = key.chunks().collect();

        // Return whether the pattern chunks match all of the key's chunks.
        matches_chunks(&expr, &key)
    }
}

fn matches_chunks(expr: &[&str], key: &[&str]) -> bool {
    match expr.split_first() {
        None => key.is_empty(),
        Some((&"**", rest)) => {
            // `**` absorbs zero or more chunks.
            (0..=key.len()).any(|skip| matches_chunks(rest, &key[skip..]))
        }
        Some((&"*", rest)) => !key.is_empty() && matches_chunks(rest, &key[1..]),
        Some((literal, rest)) => key.first() == Some(literal) && matches_chunks(rest, &key[1..]),
    }
}

impl fmt::Display for KeyExpr {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(&self.0)
    }
}

impl<'de> Deserialize<'de> for KeyExpr {
    fn deserialize<D: Deserializer<'de>>(deserializer: D) -> Result<Self, D::Error> {
        let raw = String::deserialize(deserializer)?;
        KeyExpr::new(raw).map_err(de::Error::custom)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn kx(s: &str) -> KeyExpr {
        KeyExpr::new(s).unwrap()
    }

    #[test]
    fn validation() {
        assert!(KeyExpr::new("continuo/demo/actor/car1/pose").is_ok());
        assert!(KeyExpr::new("continuo/*/actor/**").is_ok());
        assert!(KeyExpr::new("").is_err());
        assert!(KeyExpr::new("/leading").is_err());
        assert!(KeyExpr::new("trailing/").is_err());
        assert!(KeyExpr::new("a//b").is_err());
        assert!(KeyExpr::new("a/b*c").is_err());
    }

    #[test]
    fn rooting() {
        // Written out rather than built from KEY_ROOT: a test assembled from
        // the same constant as the code would agree with it however the
        // constant changed.
        assert_eq!(
            KeyExpr::new_rooted("demo/actor/car1/pose")
                .unwrap()
                .as_str(),
            "continuo/demo/actor/car1/pose"
        );
        assert_eq!(
            KeyExpr::new_rooted("*/actor/*/pose").unwrap().as_str(),
            "continuo/*/actor/*/pose"
        );

        // The separator belongs to `new_rooted`, so passing one is a doubled
        // chunk and is rejected rather than quietly producing `continuo//...`.
        assert!(KeyExpr::new_rooted("/demo/actor").is_err());
    }

    #[test]
    fn rooting_is_not_folded_into_new() {
        // `new` is the deserialization path, so it has to take a complete key.
        // Rooting there would double the root on every round trip.
        let key = KeyExpr::new("continuo/demo/actor/car1/pose").unwrap();
        let round_tripped: KeyExpr =
            serde_json::from_str(&serde_json::to_string(&key).unwrap()).unwrap();

        assert_eq!(round_tripped, key);
    }

    #[test]
    fn literal_matching() {
        assert!(kx("a/b/c").matches(&kx("a/b/c")));
        assert!(!kx("a/b/c").matches(&kx("a/b")));
        assert!(!kx("a/b").matches(&kx("a/b/c")));
        assert!(!kx("a/b/c").matches(&kx("a/b/d")));
    }

    #[test]
    fn single_wildcard() {
        assert!(kx("a/*/c").matches(&kx("a/b/c")));
        assert!(!kx("a/*/c").matches(&kx("a/c")));
        assert!(!kx("a/*").matches(&kx("a/b/c")));
    }

    #[test]
    fn multi_wildcard() {
        assert!(kx("a/**").matches(&kx("a/b/c")));
        assert!(kx("a/**").matches(&kx("a")));
        assert!(kx("**/pose").matches(&kx("continuo/demo/actor/car1/pose")));
        assert!(kx("a/**/d").matches(&kx("a/b/c/d")));
        assert!(kx("a/**/d").matches(&kx("a/d")));
        assert!(!kx("a/**/d").matches(&kx("a/b/c")));
        assert!(kx("**").matches(&kx("anything/at/all")));
    }
}

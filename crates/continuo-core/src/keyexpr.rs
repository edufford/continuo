use std::fmt;

use serde::{Deserialize, Deserializer, Serialize, de};

use crate::error::CoreError;

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

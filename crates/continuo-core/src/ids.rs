use std::fmt;

use serde::{Deserialize, Deserializer, Serialize, de};

use crate::error::CoreError;

/// A single component name: one level of the component tree.
///
/// Non-empty, and free of `/` (the path separator) and key-expression
/// wildcard characters, since ids are embedded in both paths and key
/// expressions.
#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize)]
#[serde(transparent)]
pub struct ComponentId(String);

impl ComponentId {
    pub fn new(id: impl Into<String>) -> Result<Self, CoreError> {
        let id = id.into();
        if id.is_empty() || id.contains(['/', '*', '$', '?', '#']) {
            return Err(CoreError::InvalidComponentId(id));
        }
        Ok(ComponentId(id))
    }

    pub fn as_str(&self) -> &str {
        &self.0
    }
}

impl fmt::Display for ComponentId {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(&self.0)
    }
}

impl<'de> Deserialize<'de> for ComponentId {
    fn deserialize<D: Deserializer<'de>>(deserializer: D) -> Result<Self, D::Error> {
        let raw = String::deserialize(deserializer)?;
        ComponentId::new(raw).map_err(de::Error::custom)
    }
}

/// Absolute position of a component in the tree, e.g. `car1/physics`.
///
/// The world root is the empty path; scheduled components are always leaves
/// with a non-empty path. `Ord` is lexicographic over segments and provides
/// the deterministic sort key used in transports and registries.
#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord, Hash, Default, Serialize, Deserialize)]
#[serde(transparent)]
pub struct ComponentPath(Vec<ComponentId>);

impl ComponentPath {
    pub const fn root() -> Self {
        ComponentPath(Vec::new())
    }

    /// Parses a `/`-separated path. The empty string is the root.
    pub fn parse(path: &str) -> Result<Self, CoreError> {
        if path.is_empty() {
            return Ok(Self::root());
        }
        let segments = path
            .split('/')
            .map(ComponentId::new)
            .collect::<Result<Vec<_>, _>>()?;
        Ok(ComponentPath(segments))
    }

    pub fn segments(&self) -> &[ComponentId] {
        &self.0
    }

    pub fn is_root(&self) -> bool {
        self.0.is_empty()
    }

    /// Returns a new path with `id` appended, like `Path::join`.
    pub fn join(&self, id: ComponentId) -> ComponentPath {
        let mut segments = self.0.clone();
        segments.push(id);
        ComponentPath(segments)
    }

    pub fn parent(&self) -> Option<ComponentPath> {
        if self.0.is_empty() {
            None
        } else {
            Some(ComponentPath(self.0[..self.0.len() - 1].to_vec()))
        }
    }

    /// The first `len` segments as a path (`len` must not exceed the path
    /// length).
    pub fn prefix(&self, len: usize) -> ComponentPath {
        ComponentPath(self.0[..len].to_vec())
    }

    /// Number of leading segments shared with `other`.
    pub fn common_prefix_len(&self, other: &ComponentPath) -> usize {
        self.0
            .iter()
            .zip(other.0.iter())
            .take_while(|(a, b)| a == b)
            .count()
    }
}

impl fmt::Display for ComponentPath {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let mut first = true;
        for segment in &self.0 {
            if !first {
                f.write_str("/")?;
            }
            f.write_str(segment.as_str())?;
            first = false;
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn id_validation() {
        assert!(ComponentId::new("car1").is_ok());
        assert!(ComponentId::new("").is_err());
        assert!(ComponentId::new("a/b").is_err());
        assert!(ComponentId::new("a*").is_err());
    }

    #[test]
    fn path_parse_and_display() {
        let p = ComponentPath::parse("car1/physics").unwrap();
        assert_eq!(p.segments().len(), 2);
        assert_eq!(p.to_string(), "car1/physics");
        assert_eq!(ComponentPath::parse("").unwrap(), ComponentPath::root());
        assert!(ComponentPath::parse("a//b").is_err());
    }

    #[test]
    fn common_prefix() {
        let a = ComponentPath::parse("car1/physics").unwrap();
        let b = ComponentPath::parse("car1/controller").unwrap();
        let c = ComponentPath::parse("car2/physics").unwrap();
        assert_eq!(a.common_prefix_len(&b), 1);
        assert_eq!(a.common_prefix_len(&c), 0);
        assert_eq!(a.common_prefix_len(&a), 2);
    }
}

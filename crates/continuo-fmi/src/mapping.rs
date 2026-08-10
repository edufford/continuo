use continuo_core::{KeyExpr, SimDuration};
use serde_json::Value;

/// Which messages feed which of an FMU's variables, where its outputs go, and
/// how often it steps.
///
/// This is the whole configuration of an imported FMU, and it is data: adding
/// one to a world writes a mapping rather than a type. Built from Rust
/// literals today and from scenario files later, which is why every field is
/// public and nothing here is clever.
pub struct FmuMapping {
    /// How often the component steps. Checked against the FMU's
    /// `fixedInternalStepSize` where it declares one.
    pub period: SimDuration,
    /// Messages feeding the FMU's input variables.
    pub inputs: Vec<InputBinding>,
    /// The FMU's output variables, and where each publishes.
    pub outputs: Vec<OutputBinding>,
    /// Values set once during initialization, by variable name. Parameters
    /// live here, which is what lets one `.fmu` serve every car in a world
    /// with different numbers in each.
    pub initial_values: Vec<(String, Value)>,
}

/// One FMU input variable and the message elements that feed it.
///
/// An array is fed by one pointer per element rather than by a second
/// mechanism: a scalar carries one pointer and an array of dimension N
/// carries N, in the row-major order the standard specifies for a matrix.
/// The count is checked against the variable's resolved dimension when the
/// component is built, which is what stops a rebuilt FMU and a stale mapping
/// from drifting apart in silence.
pub struct InputBinding {
    /// The FMU's name for the variable, as `modelDescription.xml` spells it.
    pub variable: String,
    /// The key whose messages feed it.
    pub key: KeyExpr,
    /// One JSON Pointer per element, resolved against the decoded payload.
    pub pointers: Vec<String>,
    /// The value for an element whose pointer finds nothing in a message
    /// that did arrive. No message at all is a different thing and holds the
    /// previous value instead, which is what sample-and-hold means.
    ///
    /// Checked against the variable's declared type when the component is
    /// built, so a default the FMU could never accept fails there rather
    /// than at the first gap in the data.
    pub when_missing: Value,
}

impl InputBinding {
    /// Binds a variable to a key, deriving the pointer from the variable's
    /// name. The scalar case, and correct whenever the FMU was named after
    /// the payload it reads.
    pub fn new(variable: impl Into<String>, key: KeyExpr, when_missing: Value) -> Self {
        let variable = variable.into();
        let pointers = vec![pointer_from_name(&variable)];

        // Return a scalar binding reading the path its own name spells.
        InputBinding {
            variable,
            key,
            pointers,
            when_missing,
        }
    }

    /// Replaces the derived pointer with pointers written out, one per
    /// element. Anything an FMU does not name after the message it consumes
    /// needs this, which is every third-party FMU and every array.
    pub fn with_pointers<S: Into<String>>(mut self, pointers: impl IntoIterator<Item = S>) -> Self {
        self.pointers = pointers.into_iter().map(Into::into).collect();
        self
    }

    /// The value for each element, in declaration order, resolved against one
    /// decoded payload.
    ///
    /// Addresses the decoded value rather than the bytes it arrived as, so
    /// the wire format never reaches this crate.
    pub fn resolve<'a>(&'a self, payload: &'a Value) -> Vec<&'a Value> {
        self.pointers
            .iter()
            .map(|pointer| payload.pointer(pointer).unwrap_or(&self.when_missing))
            .collect()
    }
}

/// One FMU output variable and the key it publishes on.
///
/// The payload path derives from the variable's name, so an output named
/// `accel_cmd` publishes `{"accel_cmd": ...}`. Variables sharing a key merge
/// into one payload, which is how an FMU whose outputs are named
/// `position.x` and `position.y` publishes one nested object.
pub struct OutputBinding {
    /// The FMU's name for the variable, as `modelDescription.xml` spells it.
    pub variable: String,
    /// The key it publishes on.
    pub key: KeyExpr,
}

impl OutputBinding {
    pub fn new(variable: impl Into<String>, key: KeyExpr) -> Self {
        OutputBinding {
            variable: variable.into(),
            key,
        }
    }
}

/// The JSON Pointer an FMI structured name spells: `.` nests and `[i]`
/// indexes, so `position.x` addresses `/position/x` and `a[1][2]` addresses
/// `/a/1/2`.
///
/// FMI 3.0's structured naming convention and RFC 6901 describe the same
/// shape in different punctuation, which is why an FMU authored beside its
/// host can name its variables after the payloads they read and write no
/// pointers at all. It is a convenience rather than an assumption: a
/// third-party FMU names things in its own vocabulary, and that is what
/// [`InputBinding::with_pointers`] is for.
pub fn pointer_from_name(name: &str) -> String {
    let mut pointer = String::new();
    for segment in name.split(['.', '[']) {
        pointer.push('/');
        pointer.push_str(&escape(segment.strip_suffix(']').unwrap_or(segment)));
    }

    // Return the pointer the name spells.
    pointer
}

/// Pointers for one field of every element of an array payload, which is what
/// an FMU array input needs: `element_pointers("/detections", 3, "range")`
/// gives `/detections/0/range` through `/detections/2/range`.
///
/// `count` is the variable's dimension rather than the payload's length,
/// since the payload's length varies per message and the FMU's does not. A
/// message carrying fewer elements leaves the tail to
/// [`InputBinding::when_missing`], which is how a short radar scan reads as a
/// clear road rather than as an error.
pub fn element_pointers(array: &str, count: usize, field: &str) -> Vec<String> {
    let field = escape(field);

    // Return one pointer per element the variable declares.
    (0..count)
        .map(|index| format!("{array}/{index}/{field}"))
        .collect()
}

/// Escapes one JSON Pointer reference token per RFC 6901: `~` first, so the
/// tildes introduced by `/` are not escaped twice.
fn escape(token: &str) -> String {
    token.replace('~', "~0").replace('/', "~1")
}

#[cfg(test)]
mod tests {
    use serde_json::json;

    use super::*;

    fn binding<S: Into<String>>(
        pointers: impl IntoIterator<Item = S>,
        when_missing: Value,
    ) -> InputBinding {
        InputBinding::new("v", KeyExpr::new("continuo/w/x").unwrap(), when_missing)
            .with_pointers(pointers)
    }

    #[test]
    fn a_structured_name_spells_its_own_pointer() {
        assert_eq!(pointer_from_name("speed"), "/speed");
        assert_eq!(pointer_from_name("position.x"), "/position/x");
        assert_eq!(pointer_from_name("orientation.w"), "/orientation/w");
    }

    #[test]
    fn an_indexed_name_spells_an_indexed_pointer() {
        assert_eq!(pointer_from_name("a[1]"), "/a/1");
        assert_eq!(pointer_from_name("a[1][2]"), "/a/1/2");
        assert_eq!(pointer_from_name("m.a[1][2].b"), "/m/a/1/2/b");
    }

    #[test]
    fn a_name_containing_pointer_punctuation_is_escaped() {
        // RFC 6901 gives `~` and `/` meaning inside a token, and FMI puts no
        // such restriction on a variable name, so a name carrying either has
        // to survive the derivation rather than silently address elsewhere.
        assert_eq!(pointer_from_name("a/b"), "/a~1b");
        assert_eq!(pointer_from_name("a~b"), "/a~0b");
        assert_eq!(pointer_from_name("a~/b"), "/a~0~1b");
    }

    #[test]
    fn element_pointers_walk_one_field_across_the_array() {
        assert_eq!(
            element_pointers("/detections", 3, "range"),
            [
                "/detections/0/range",
                "/detections/1/range",
                "/detections/2/range"
            ]
        );
        assert_eq!(
            element_pointers("/detections", 0, "range"),
            [] as [String; 0]
        );
    }

    #[test]
    fn resolution_reads_a_decoded_value_in_declaration_order() {
        let payload = json!({"position": {"x": 1.0, "y": 2.0}, "speed": 3.0});
        let binding = binding(["/speed", "/position/x", "/position/y"], json!(0.0));
        assert_eq!(
            binding.resolve(&payload),
            [&json!(3.0), &json!(1.0), &json!(2.0)]
        );
    }

    #[test]
    fn a_short_payload_fills_the_tail_with_the_default() {
        // The radar's case: a scan carrying two cars feeds a variable sized
        // for four, and the empty slots have to read as a clear road.
        let payload = json!({"detections": [{"range": 10.0}, {"range": 20.0}]});
        let binding = binding(element_pointers("/detections", 4, "range"), json!(1e9));
        assert_eq!(
            binding.resolve(&payload),
            [&json!(10.0), &json!(20.0), &json!(1e9), &json!(1e9)]
        );
    }

    #[test]
    fn a_pointer_finding_nothing_uses_the_default_whatever_its_type() {
        // One field serves every FMI type, so the default for a boolean is a
        // boolean and the default for a string is a string.
        let payload = json!({"present": 1.0});
        assert_eq!(
            binding(["/absent"], json!(true)).resolve(&payload),
            [&json!(true)]
        );
        assert_eq!(
            binding(["/absent"], json!("idle")).resolve(&payload),
            [&json!("idle")]
        );
        assert_eq!(
            binding(["/absent"], json!(-7)).resolve(&payload),
            [&json!(-7)]
        );
    }

    #[test]
    fn a_derived_pointer_reads_the_payload_its_variable_is_named_for() {
        let payload = json!({"position": {"x": 4.5}});
        let binding = InputBinding::new(
            "position.x",
            KeyExpr::new("continuo/w/actor/car/pose").unwrap(),
            json!(0.0),
        );
        assert_eq!(binding.resolve(&payload), [&json!(4.5)]);
    }
}

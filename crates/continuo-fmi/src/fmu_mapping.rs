use continuo_core::{CoreError, KeyExpr, SimDuration};
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
    pub fmu_var_name: String,
    /// The key whose messages feed it.
    pub subscribed_key: KeyExpr,
    /// One JSON Pointer per element, resolved against the decoded payload.
    ///
    /// Flat whatever the variable's rank, and as long as the product of its
    /// dimensions: a matrix is a list in row-major order rather than a list
    /// of lists. [`json_pointers_for_array`] and
    /// [`json_pointers_for_dimensions`] write the repetitive cases.
    pub pointers: Vec<String>,
    /// The value for an element whose pointer finds nothing in a message that
    /// did arrive, and `None` if that must never happen.
    ///
    /// `None` is the default because a pointer reading a field the publisher
    /// always writes has no honest fallback: finding nothing means the
    /// mapping and the publisher disagree about the payload's shape, and
    /// substituting a value would feed the FMU a number nobody published,
    /// silently and for the rest of the run. It halts instead, naming the
    /// variable, the pointer and the key, which is the bargain
    /// [`CoreError::PayloadDecode`] already makes: the failure is a pure
    /// function of the mapping and the payload, so it reproduces at the
    /// identical instant on every machine rather than diverging.
    ///
    /// `Some` is for the shape where absence is itself part of the
    /// measurement: an array variable whose payload carries fewer elements
    /// than the variable declares, where the empty slots have a meaning the
    /// publisher never has to spell out. Holding the previous value cannot
    /// serve there, since a slot that stops being written would keep its last
    /// value forever, and an FMI array is set whole in one call anyway, so
    /// writing part of one is not a thing that exists.
    ///
    /// Nothing is logged when a default is used, since on that shape it is
    /// used most steps by design. The loud case is the one that halts.
    ///
    /// A `Some` value is checked against the variable's declared type when
    /// the component is built, so a default the FMU could never accept fails
    /// there rather than at the first gap in the data.
    ///
    /// [`CoreError::PayloadDecode`]: continuo_core::CoreError::PayloadDecode
    pub when_missing: Option<Value>,
}

impl InputBinding {
    /// Binds a variable to a key, deriving the pointer from the variable's
    /// name. The scalar case, and correct whenever the FMU was named after
    /// the payload it reads.
    pub fn new(fmu_var_name: impl Into<String>, subscribed_key: KeyExpr) -> Self {
        let fmu_var_name = fmu_var_name.into();
        let pointers = vec![json_pointer_from_name(&fmu_var_name)];

        // Return a scalar binding reading the path its own name spells, whose
        // pointer has to find something.
        InputBinding {
            fmu_var_name,
            subscribed_key,
            pointers,
            when_missing: None,
        }
    }

    /// Writes the pointers out instead of deriving them from the variable's
    /// name, one per element.
    ///
    /// Two cases need it. A scalar whose FMU spells the variable differently
    /// from the payload it reads, which is most FMUs written by anyone else,
    /// since a model names its ports for its own internals rather than for
    /// the messages of a host it has never heard of. And any array, since
    /// nothing derives `/detections/0/range` from a variable named `range`.
    pub fn with_pointers<S: Into<String>>(mut self, pointers: impl IntoIterator<Item = S>) -> Self {
        self.pointers = pointers.into_iter().map(Into::into).collect();
        self
    }

    /// Says an element may be absent, and what it reads as then. See
    /// [`InputBinding::when_missing`] for when that is honest and when it
    /// hides a wiring mistake.
    pub fn when_missing(mut self, value: Value) -> Self {
        self.when_missing = Some(value);
        self
    }

    /// The value for each element, in declaration order, resolved against one
    /// decoded payload.
    ///
    /// Addresses the decoded value rather than the bytes it arrived as, so
    /// the wire format never reaches this crate.
    pub fn resolve<'a>(&'a self, payload: &'a Value) -> Result<Vec<&'a Value>, CoreError> {
        self.pointers
            .iter()
            .map(|pointer| {
                payload
                    .pointer(pointer)
                    .or(self.when_missing.as_ref())
                    .ok_or_else(|| CoreError::ComponentFailure {
                        reason: format!(
                            "input {:?}: nothing at {:?} on {}, and no value declared for its absence",
                            self.fmu_var_name,
                            pointer,
                            self.subscribed_key.as_str()
                        ),
                    })
            })
            .collect()
    }
}

/// One FMU output variable, the key it publishes on, and where in that
/// message's payload its value lands.
///
/// The payload pointer derives from the variable's name, so an output named
/// `accel_cmd` publishes `{"accel_cmd": ...}`. Outputs sharing a key merge
/// into one payload, which is how an FMU whose outputs are named
/// `position.x` and `position.y` publishes one nested object.
///
/// An array variable publishes its whole value at that pointer as a JSON
/// array, and a multi-dimensional one as nested arrays, the outermost
/// dimension first. Nothing per element, because an output builds its own
/// payload rather than digging through one somebody else designed, and an
/// element of a payload under construction has no address until the payload
/// exists.
pub struct OutputBinding {
    /// The FMU's name for the variable, as `modelDescription.xml` spells it.
    pub fmu_var_name: String,
    /// The key it publishes on.
    pub published_key: KeyExpr,
    /// Where in the payload its value lands, as a JSON Pointer.
    ///
    /// Overridable for the same reason [`InputBinding::with_pointers`]
    /// exists, and just as necessary: an FMU names its ports for its own
    /// internals, while a world's message shapes are fixed by whoever
    /// already reads them. Without this, an FMU calling its output `a_out`
    /// could only publish `{"a_out": ...}` into a world whose consumers
    /// decode `accel_cmd`, and bridging the two would take a whole component
    /// that renamed one field.
    pub payload_pointer: String,
}

impl OutputBinding {
    /// Publishes a variable on a key, at the payload path its name spells.
    pub fn new(fmu_var_name: impl Into<String>, published_key: KeyExpr) -> Self {
        let fmu_var_name = fmu_var_name.into();
        let payload_pointer = json_pointer_from_name(&fmu_var_name);

        // Return a binding publishing where its own name spells.
        OutputBinding {
            fmu_var_name,
            published_key,
            payload_pointer,
        }
    }

    /// Lands the value somewhere other than where the variable's name spells.
    pub fn with_pointer(mut self, payload_pointer: impl Into<String>) -> Self {
        self.payload_pointer = payload_pointer.into();
        self
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
pub fn json_pointer_from_name(name: &str) -> String {
    let mut pointer = String::new();
    for segment in name.split(['.', '[']) {
        pointer.push('/');
        pointer.push_str(&escape_json_pointer_token(
            segment.strip_suffix(']').unwrap_or(segment),
        ));
    }

    // Return the pointer the name spells.
    pointer
}

/// Pointers reading one field out of every element of a JSON array.
///
/// For a payload of
/// `{"detections": [{"range": 8.0}, {"range": 20.0}, {"range": 31.0}]}`,
/// `json_pointers_for_array("/detections", 3, "range")` gives
/// `/detections/0/range`, `/detections/1/range` and `/detections/2/range`.
///
/// `count` is the FMU variable's dimension rather than the payload's length,
/// because the variable is a fixed size and a message is not. A message
/// carrying fewer elements leaves the tail pointers finding nothing, which
/// is what [`InputBinding::when_missing`] is for.
///
/// Use [`json_pointers_for_dimensions`] instead where the payload is nested
/// arrays of plain values rather than object dictionaries.
pub fn json_pointers_for_array(array: &str, count: usize, field: &str) -> Vec<String> {
    let field = escape_json_pointer_token(field);

    // Return one pointer per element the variable declares.
    (0..count)
        .map(|index| format!("{array}/{index}/{field}"))
        .collect()
}

/// Pointers for every element of an array or a matrix of plain values.
///
/// For a payload of `{"a": [[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]]}`,
/// `json_pointers_for_dimensions("/a", &[2, 3])` gives `/a/0/0`, `/a/0/1`,
/// `/a/0/2`, `/a/1/0`, `/a/1/1` and `/a/1/2`.
///
/// One flat list whatever the shape, in **row-major** order, so the last
/// index varies fastest. That is the order FMI 3.0 specifies for a
/// multi-dimensional variable's values, and getting it backwards transposes
/// the matrix. A transposed square matrix still runs and still publishes
/// numbers, so a test pins the order rather than this sentence.
///
/// An empty `dimensions` gives the prefix alone, which is the pointer to a
/// scalar.
pub fn json_pointers_for_dimensions(prefix: &str, dimensions: &[usize]) -> Vec<String> {
    let mut pointers = vec![prefix.to_string()];
    for &size in dimensions {
        pointers = pointers
            .iter()
            .flat_map(|pointer| (0..size).map(move |index| format!("{pointer}/{index}")))
            .collect();
    }

    // Return one pointer per element, the last index varying fastest.
    pointers
}

// TODO(PLAN "Scenario configuration"): this pair and `insert_at_pointer` are
// plain RFC 6901 rather than anything to do with FMI, so they would belong in
// core if a second crate ever addressed into a payload. Nothing does today,
// and reading is already served by `serde_json::Value::pointer`. What is
// missing there is writing, which only a component that builds a payload
// without a Rust type for it needs, so the trigger is a second such component
// rather than a second reader.
/// Escapes one JSON Pointer reference token per RFC 6901: `~` first, so the
/// tildes introduced by `/` are not escaped twice.
///
/// See [`unescape_json_pointer_token`] for the way back.
pub fn escape_json_pointer_token(token: &str) -> String {
    token.replace('~', "~0").replace('/', "~1")
}

/// Reverses [`escape_json_pointer_token`]: `~1` first, so a `~` that
/// unescaping just produced is not then read as introducing an escape of its
/// own.
pub fn unescape_json_pointer_token(token: &str) -> String {
    token.replace("~1", "/").replace("~0", "~")
}

/// Puts `value` into the message payload being built, at a JSON Pointer.
///
/// Objects rather than arrays, because a payload under construction has no
/// shape to read an index against. Only an FMU's own variable names produce
/// these paths, and a name is a name whatever it looks like.
pub(crate) fn insert_at_pointer(message_payload: &mut Value, pointer: &str, value: Value) {
    // A pointer starts with the separator, so splitting it gives an empty
    // first token standing for the whole document, which is where the walk
    // starts rather than a name to descend into.
    let tokens: Vec<String> = pointer
        .split('/')
        .skip(1)
        .map(unescape_json_pointer_token)
        .collect();

    let Some((last, parents)) = tokens.split_last() else {
        *message_payload = value;
        return;
    };

    let mut cursor = message_payload;
    for token in parents {
        if !cursor.is_object() {
            *cursor = Value::Object(serde_json::Map::new());
        }
        cursor = cursor
            .as_object_mut()
            .expect("just made an object")
            .entry(token.clone())
            .or_insert(Value::Null);
    }

    if !cursor.is_object() {
        *cursor = Value::Object(serde_json::Map::new());
    }
    cursor
        .as_object_mut()
        .expect("just made an object")
        .insert(last.clone(), value);
}

#[cfg(test)]
mod tests {
    use serde_json::json;

    use super::*;

    fn binding<S: Into<String>>(
        pointers: impl IntoIterator<Item = S>,
        when_missing: Value,
    ) -> InputBinding {
        InputBinding::new("v", KeyExpr::new("continuo/w/x").unwrap())
            .with_pointers(pointers)
            .when_missing(when_missing)
    }

    #[test]
    fn escaping_a_token_and_unescaping_it_gives_the_name_back() {
        for name in ["plain", "a/b", "a~b", "a~/b", "~", "/", "~0", "~1"] {
            let escaped = escape_json_pointer_token(name);
            assert_eq!(unescape_json_pointer_token(&escaped), name, "{name:?}");
        }
    }

    #[test]
    fn a_structured_name_spells_its_own_pointer() {
        assert_eq!(json_pointer_from_name("speed"), "/speed");
        assert_eq!(json_pointer_from_name("position.x"), "/position/x");
        assert_eq!(json_pointer_from_name("orientation.w"), "/orientation/w");
    }

    #[test]
    fn an_indexed_name_spells_an_indexed_pointer() {
        assert_eq!(json_pointer_from_name("a[1]"), "/a/1");
        assert_eq!(json_pointer_from_name("a[1][2]"), "/a/1/2");
        assert_eq!(json_pointer_from_name("m.a[1][2].b"), "/m/a/1/2/b");
    }

    #[test]
    fn a_name_containing_pointer_punctuation_is_escaped() {
        // RFC 6901 gives `~` and `/` meaning inside a token, and FMI puts no
        // such restriction on a variable name, so a name carrying either has
        // to survive the derivation rather than silently address elsewhere.
        assert_eq!(json_pointer_from_name("a/b"), "/a~1b");
        assert_eq!(json_pointer_from_name("a~b"), "/a~0b");
        assert_eq!(json_pointer_from_name("a~/b"), "/a~0~1b");
    }

    #[test]
    fn one_pointer_per_element_walks_one_field_across_the_array() {
        assert_eq!(
            json_pointers_for_array("/detections", 3, "range"),
            [
                "/detections/0/range",
                "/detections/1/range",
                "/detections/2/range"
            ]
        );
        assert_eq!(
            json_pointers_for_array("/detections", 0, "range"),
            [] as [String; 0]
        );
    }

    #[test]
    fn an_output_lands_where_its_name_spells_unless_told_otherwise() {
        let key = KeyExpr::new("continuo/w/actor/car/accel_cmd").unwrap();
        assert_eq!(
            OutputBinding::new("accel_cmd", key.clone()).payload_pointer,
            "/accel_cmd"
        );
        assert_eq!(
            OutputBinding::new("position.x", key.clone()).payload_pointer,
            "/position/x"
        );

        // An FMU names its ports for its own internals, so bridging its
        // vocabulary to the world's is a mapping's job rather than a
        // component's.
        assert_eq!(
            OutputBinding::new("a_out", key)
                .with_pointer("/accel_cmd")
                .payload_pointer,
            "/accel_cmd"
        );
    }

    #[test]
    fn a_variable_of_any_rank_binds_through_one_flat_row_major_list() {
        // The last index varies fastest, which is what FMI specifies and
        // what a transposed matrix would quietly violate while still
        // running.
        assert_eq!(json_pointers_for_dimensions("/a", &[]), ["/a"]);
        assert_eq!(
            json_pointers_for_dimensions("/a", &[3]),
            ["/a/0", "/a/1", "/a/2"]
        );
        assert_eq!(
            json_pointers_for_dimensions("/a", &[2, 3]),
            ["/a/0/0", "/a/0/1", "/a/0/2", "/a/1/0", "/a/1/1", "/a/1/2"]
        );
        assert_eq!(
            json_pointers_for_dimensions("/a", &[2, 2, 2]),
            [
                "/a/0/0/0", "/a/0/0/1", "/a/0/1/0", "/a/0/1/1", "/a/1/0/0", "/a/1/0/1", "/a/1/1/0",
                "/a/1/1/1"
            ]
        );
    }

    #[test]
    fn a_matrix_resolves_row_by_row() {
        let payload = json!({"a": [[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]]});
        let binding = binding(json_pointers_for_dimensions("/a", &[2, 3]), json!(0.0));
        assert_eq!(
            binding.resolve(&payload).unwrap(),
            [
                &json!(1.0),
                &json!(2.0),
                &json!(3.0),
                &json!(4.0),
                &json!(5.0),
                &json!(6.0)
            ]
        );
    }

    #[test]
    fn resolution_reads_a_decoded_value_in_declaration_order() {
        let payload = json!({"position": {"x": 1.0, "y": 2.0}, "speed": 3.0});
        let binding = binding(["/speed", "/position/x", "/position/y"], json!(0.0));
        assert_eq!(
            binding.resolve(&payload).unwrap(),
            [&json!(3.0), &json!(1.0), &json!(2.0)]
        );
    }

    #[test]
    fn a_short_payload_fills_the_tail_with_the_default() {
        // The radar's case: a scan carrying two cars feeds a variable sized
        // for four, and the empty slots have to read as a clear road.
        let payload = json!({"detections": [{"range": 10.0}, {"range": 20.0}]});
        let binding = binding(
            json_pointers_for_array("/detections", 4, "range"),
            json!(1e9),
        );
        assert_eq!(
            binding.resolve(&payload).unwrap(),
            [&json!(10.0), &json!(20.0), &json!(1e9), &json!(1e9)]
        );
    }

    #[test]
    fn a_pointer_finding_nothing_uses_the_default_whatever_its_type() {
        // One field serves every FMI type, so the default for a boolean is a
        // boolean and the default for a string is a string.
        let payload = json!({"present": 1.0});
        assert_eq!(
            binding(["/absent"], json!(true)).resolve(&payload).unwrap(),
            [&json!(true)]
        );
        assert_eq!(
            binding(["/absent"], json!("idle"))
                .resolve(&payload)
                .unwrap(),
            [&json!("idle")]
        );
        assert_eq!(
            binding(["/absent"], json!(-7)).resolve(&payload).unwrap(),
            [&json!(-7)]
        );
    }

    #[test]
    fn a_derived_pointer_reads_the_payload_its_variable_is_named_for() {
        let payload = json!({"position": {"x": 4.5}});
        let binding = InputBinding::new(
            "position.x",
            KeyExpr::new("continuo/w/actor/car/pose").unwrap(),
        );
        assert_eq!(binding.resolve(&payload).unwrap(), [&json!(4.5)]);
    }

    #[test]
    fn a_pointer_that_must_resolve_halts_naming_what_it_looked_for() {
        // The default, because a scalar reading a field that is always there
        // has no honest fallback: the mapping and the publisher disagree
        // about the payload's shape, and a substituted number would drive a
        // car from a pose nobody published.
        let payload = json!({"position": {"y": 4.5}});
        let binding = InputBinding::new(
            "position.x",
            KeyExpr::new("continuo/w/actor/car/pose").unwrap(),
        );

        let reason = binding.resolve(&payload).unwrap_err().to_string();
        assert!(reason.contains("position.x"), "{reason}");
        assert!(reason.contains("/position/x"), "{reason}");
        assert!(reason.contains("continuo/w/actor/car/pose"), "{reason}");
    }

    #[test]
    fn a_default_covers_only_the_elements_that_are_missing() {
        // One default serves every element, so a scan that skips a slot in
        // the middle is not a special case.
        let payload = json!({"detections": [{"range": 10.0}, {}, {"range": 30.0}]});
        let binding = binding(
            json_pointers_for_array("/detections", 4, "range"),
            json!(1e9),
        );
        assert_eq!(
            binding.resolve(&payload).unwrap(),
            [&json!(10.0), &json!(1e9), &json!(30.0), &json!(1e9)]
        );
    }
}

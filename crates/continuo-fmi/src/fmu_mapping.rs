use continuo_core::{KeyExpr, SimDuration};
use serde_json::Value;

use crate::error::FmuConstructionError;

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
/// A variable of any rank reads one element per value it holds, in the
/// row-major order the standard specifies for a matrix, and [`InputSource`]
/// says where in the payload to find them.
///
/// Building the component counts them against the variable's resolved
/// dimensions, which is what stops a rebuilt FMU and a stale mapping from
/// drifting apart in silence.
pub struct InputBinding {
    /// The FMU's name for the variable, as `modelDescription.xml` spells it.
    pub fmu_var_name: String,
    /// The key whose messages feed it.
    pub subscribed_key: KeyExpr,
    /// Where in the payload its elements are, and `None` to derive that from
    /// the variable's own name: `position.x` reads `/position/x`, and a `u`
    /// holding three values reads `/u/0`, `/u/1` and `/u/2`, since deriving
    /// walks every element of whatever rank the FMU declares.
    pub source: Option<InputSource>,
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

/// Where an input variable's elements are found in a message.
///
/// Two forms, told apart by shape: one pointer, or a list of them. A string
/// and a list are different kinds of JSON value, so the scenario file this
/// becomes needs no tag to keep them apart.
///
/// Writing no source at all is the third case, an absence rather than a
/// third shape. See [`InputBinding::source`] for what the name spells
/// instead.
///
/// ```json5
/// inputs: [
///   // A pattern, whose wildcards the FMU's dimensions expand.
///   { fmu_var: "range", message_key: ".../radar",
///     from: "/detections/*/range", when_missing: 1e9 },
///
///   // The elements written out, for a payload no one pattern reaches.
///   { fmu_var: "x0", message_key: ".../pose",
///     from: ["/position/x", "/position/y"] },
///
///   // No `from` at all, so the name spells the path: `/speed` for a scalar,
///   // and `/speed/*` for as many dimensions as the FMU declares.
///   { fmu_var: "speed", message_key: ".../pose" },
/// ]
/// ```
// TODO(PLAN "Scenario configuration"): that block is the shape to deserialize
// into, and it is written down here because the reasoning behind it is worth
// more than the syntax. Nothing parses it yet.
pub enum InputSource {
    /// One JSON Pointer, where a `*` token stands for every index of a
    /// dimension.
    ///
    /// One `*` per dimension: none for a scalar, `/u/*` for a vector,
    /// `/a/*/*` for a matrix, and `/detections/*/range` to take one field out
    /// of every element of an array of object dictionaries. The FMU's own
    /// dimensions supply the counts, so a mapping never writes a size the
    /// variable already declares, and a rebuilt FMU resizes its bindings with
    /// it.
    ///
    /// Expanded row-major, the order FMI specifies for a multi-dimensional
    /// variable's values: over a 2 by 3, `/a/*/*` gives `/a/0/0`, `/a/0/1`,
    /// `/a/0/2`, then `/a/1/0`, so the rightmost index runs through its whole
    /// range before the one left of it moves.
    ///
    /// Only a whole token counts, so `/a*b` addresses a key spelled `a*b`.
    /// What the syntax costs is a payload key spelled exactly `*`, which no
    /// escape can give back: RFC 6901 has none for this, and inventing one
    /// would be extending the RFC. Payload keys come from serde field names,
    /// and `*` is not a legal Rust identifier.
    Pattern(String),

    /// One JSON Pointer per element, written out.
    ///
    /// For the elements a single pattern cannot reach: values scattered
    /// across a payload rather than lying in one array, or an order the
    /// message does not carry. This is the one form that states a count of
    /// its own, and so the one the dimension check exists for.
    Pointers(Vec<String>),
}

/// A string reference is one [`Pattern`](InputSource::Pattern).
impl From<&str> for InputSource {
    fn from(pattern: &str) -> Self {
        InputSource::Pattern(pattern.to_string())
    }
}

/// A string is one [`Pattern`](InputSource::Pattern).
impl From<String> for InputSource {
    fn from(pattern: String) -> Self {
        InputSource::Pattern(pattern)
    }
}

/// An array is [`Pointers`](InputSource::Pointers), the elements written out.
impl<S: Into<String>, const N: usize> From<[S; N]> for InputSource {
    fn from(pointers: [S; N]) -> Self {
        InputSource::Pointers(pointers.into_iter().map(Into::into).collect())
    }
}

/// A `Vec` is [`Pointers`](InputSource::Pointers), the elements written out.
impl<S: Into<String>> From<Vec<S>> for InputSource {
    fn from(pointers: Vec<S>) -> Self {
        InputSource::Pointers(pointers.into_iter().map(Into::into).collect())
    }
}

impl InputBinding {
    /// Binds a variable to a key, reading the payload its own name spells.
    /// Correct whenever the FMU was named after the messages it consumes,
    /// whatever the variable's rank.
    pub fn new(fmu_var_name: impl Into<String>, subscribed_key: KeyExpr) -> Self {
        // Return a binding with no source yet: deriving one needs dimensions
        // the FMU has not been asked for.
        InputBinding {
            fmu_var_name: fmu_var_name.into(),
            subscribed_key,
            source: None,
            when_missing: None,
        }
    }

    /// Says where to read, instead of the path the variable's name spells.
    ///
    /// Takes either form of [`InputSource`], since what is written says
    /// which it is: `"/detections/*/range"` is a pattern, and
    /// `["/position/x", "/position/y"]` is the elements written out.
    ///
    /// Needed whenever the FMU's vocabulary is not the world's, which is most
    /// FMUs written by anyone else, and for any array gathered from somewhere
    /// other than an array of the same name.
    pub fn with_pointer(mut self, source: impl Into<InputSource>) -> Self {
        self.source = Some(source.into());
        self
    }

    /// Says an element may be absent, and what it reads as then. See
    /// [`InputBinding::when_missing`] for when that is honest and when it
    /// hides a wiring mistake.
    pub fn when_missing(mut self, value: Value) -> Self {
        self.when_missing = Some(value);
        self
    }

    /// One JSON Pointer per element the variable holds, in the order the FMU
    /// reads them.
    ///
    /// This is where the FMU's own sizes reach a mapping. A pattern's
    /// wildcards expand over them, and a derived source appends one wildcard
    /// per dimension, so neither repeats a number the variable already
    /// declares. Only a written-out list states a count of its own, which is
    /// what the caller checks against these dimensions.
    pub(crate) fn expand(&self, dimensions: &[usize]) -> Result<Vec<String>, FmuConstructionError> {
        let pattern = match &self.source {
            // Elements written out are already the answer, so this leaves
            // `expand` rather than binding a pattern to expand below.
            Some(InputSource::Pointers(pointers)) => return Ok(pointers.clone()),
            Some(InputSource::Pattern(pattern)) => pattern.clone(),
            // With nothing written, the name spells the path and a wildcard
            // per dimension walks whatever rank the FMU declares. That is an
            // ordinary pattern below, and one that cannot fail the count
            // check, having been built from the same count.
            None => {
                let mut derived = json_pointer_from_fmu_var_name(&self.fmu_var_name);
                for _ in dimensions {
                    derived.push_str("/*");
                }
                derived
            }
        };

        // A pointer's tokens are separated by `/`, and its leading `/` makes
        // the first token empty. That head is taken as it is, so only the
        // tokens after it can be wildcards, and splitting once here is what
        // keeps the count and the walk from disagreeing about which those are.
        let mut tokens = pattern.split('/').map(str::to_string);
        let head = tokens.next().unwrap_or_default();
        let tokens: Vec<String> = tokens.collect();

        // Each `*` walks one dimension, so a pattern needs exactly as many as
        // the variable has: none for a scalar, one for a vector, two for a
        // matrix, and N for a variable of N dimensions.
        let wildcards = tokens.iter().filter(|token| token.as_str() == "*").count();
        if wildcards != dimensions.len() {
            return Err(FmuConstructionError::Wildcard {
                variable: self.fmu_var_name.clone(),
                pattern,
                wildcards,
                dimensions: dimensions.to_vec(),
            });
        }

        // Walk it once, growing the list at each wildcard and extending every
        // pointer so far at each literal token. Taking the dimensions in
        // order, each in full, is what makes the result row-major: the last
        // wildcard is the one whose neighbors end up adjacent.
        let mut pointers = vec![head];
        let mut sizes = dimensions.iter();
        for token in tokens {
            if token == "*" {
                let size = *sizes
                    .next()
                    .expect("one wildcard per dimension, checked above");
                pointers = pointers
                    .iter()
                    .flat_map(|pointer| (0..size).map(move |index| format!("{pointer}/{index}")))
                    .collect();
            } else {
                for pointer in &mut pointers {
                    pointer.push('/');
                    pointer.push_str(&token);
                }
            }
        }

        // Return one pointer per element the pattern reaches.
        Ok(pointers)
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
    /// Overridable for the same reason [`InputBinding::with_pointer`] is: an
    /// FMU names its ports for its own internals, while a world's message
    /// shapes are fixed by whoever already reads them. Without it, an FMU
    /// calling its output `a_out` could only publish `{"a_out": ...}` into a
    /// world whose consumers decode `accel_cmd`.
    ///
    /// Every level it names is an object, so a JSON array in the payload can
    /// only come from an array variable published whole: `/accel/0` and
    /// `/accel/1` on two scalars write the keys `"0"` and `"1"` rather than
    /// two elements.
    pub payload_pointer: String,
}

impl OutputBinding {
    /// Publishes a variable on a key, at the payload path its name spells.
    pub fn new(fmu_var_name: impl Into<String>, published_key: KeyExpr) -> Self {
        let fmu_var_name = fmu_var_name.into();
        let payload_pointer = json_pointer_from_fmu_var_name(&fmu_var_name);

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

/// The JSON Pointer an FMI structured name spells, read as the standard's
/// own grammar writes one.
///
/// | name | pointer |
/// | ---- | ------- |
/// | `position.x` | `/position/x` |
/// | `pipe[3,4].T[14]` | `/pipe/3/4/T/14` |
/// | `robot.axis.'motor #234'` | `/robot/axis/motor #234` |
///
/// So `.` nests, one bracket group indexes as many axes as it lists commas,
/// and a quoted name is a single token however it is punctuated, since the
/// grammar lets one hold `.`, `[`, `]` and `,` alike. An index travels as it
/// was written, because FMI leaves it undefined whether the convention counts
/// from 0 or from 1, and only the payload can say.
///
/// FMI 3.0's structured naming convention and RFC 6901 describe the same
/// shape in different punctuation, which is why an FMU authored beside its
/// host can name its variables after the payloads they read and write no
/// pointers at all. It is a convenience rather than an assumption: a
/// third-party FMU names things in its own vocabulary, and that is what
/// [`InputBinding::with_pointer`] is for.
///
/// The FMU's `variableNamingConvention` is not consulted. Strictly it should
/// be, since the grammar above is what `structured` means and an FMU
/// declaring nothing is `flat`. But a dotted name means hierarchy in practice
/// wherever it appears, and plenty of exporters never set the attribute. An
/// FMU that really did mean a literal `a.b` gets a pointer that finds
/// nothing, which one `with_pointer` line corrects.
///
/// A name that does not parse is taken literally rather than refused, on the
/// same argument: the pointer it then spells finds nothing and halts naming
/// itself, which says more than a complaint about punctuation would.
pub fn json_pointer_from_fmu_var_name(fmu_var_name: &str) -> String {
    // A derivative names another variable rather than a path through a
    // payload, so there is nothing to nest and its punctuation is syntax.
    if fmu_var_name.starts_with("der(") {
        return format!("/{}", escape_json_pointer_token(fmu_var_name));
    }

    let mut tokens: Vec<String> = Vec::new();
    let mut token = String::new();
    let mut characters = fmu_var_name.chars();

    while let Some(character) = characters.next() {
        match character {
            // Quotes are the standard's way of admitting a name that would
            // otherwise read as punctuation, so what they enclose is taken
            // whole and the quotes themselves do not travel.
            '\'' => {
                while let Some(quoted) = characters.next() {
                    match quoted {
                        '\'' => break,
                        '\\' => token.push(unescape_modelica(characters.next())),
                        other => token.push(other),
                    }
                }
            }
            '.' => finish_token(&mut tokens, &mut token),
            '[' => {
                finish_token(&mut tokens, &mut token);

                // One group indexes every axis at this level, so `a[1,2]`
                // names two of them rather than a token spelled `1,2`.
                let mut index = String::new();
                for indexed in characters.by_ref() {
                    match indexed {
                        ',' => finish_token(&mut tokens, &mut index),
                        ']' => break,
                        other => index.push(other),
                    }
                }
                finish_token(&mut tokens, &mut index);
            }
            other => token.push(other),
        }
    }
    finish_token(&mut tokens, &mut token);

    // Return the pointer the name spells.
    tokens
        .iter()
        .map(|token| format!("/{}", escape_json_pointer_token(token)))
        .collect()
}

/// Finishes a token, if the name has actually put anything in one, and
/// clears the buffer for the next.
///
/// Nothing separates the `]` of an index from the `.` that may follow it, so
/// emptiness is how that pair is told apart from a name with a gap in it.
fn finish_token(tokens: &mut Vec<String>, token: &mut String) {
    if !token.is_empty() {
        tokens.push(std::mem::take(token));
    }
}

/// What one escape inside a quoted name stands for, per the Modelica string
/// escapes the naming grammar borrows.
///
/// `\'`, `\"`, `\?` and `\\` all stand for the character itself, which is
/// what the fallback covers. A backslash at the very end of a name escapes
/// nothing, so it is kept as itself, which is how the rest of the parser
/// treats a malformed name.
fn unescape_modelica(escaped: Option<char>) -> char {
    match escaped {
        Some('a') => '\u{7}',
        Some('b') => '\u{8}',
        Some('f') => '\u{c}',
        Some('n') => '\n',
        Some('r') => '\r',
        Some('t') => '\t',
        Some('v') => '\u{b}',
        Some(other) => other,
        None => '\\',
    }
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
/// Every level is an object, a numeric token included, so `/a/0` is the key
/// `"0"` rather than the first element of an array.
///
/// For a structured name like `a[1]` that is the only defensible reading.
/// FMI 3.0 says the structured naming convention "does not define if arrays
/// are 0-based or 1-based", so putting that index in a JSON array would mean
/// guessing whether `a[1]` is the first element or the second. Keeping it as
/// the key it was written with needs no such guess. It also handles the gaps
/// the standard allows, since "it might be that not all elements of an array
/// are present" and an array would have to fill them.
///
/// An FMU whose array does have a JSON array's shape says so by declaring a
/// dimension instead, and that variable publishes whole.
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
    use super::*;

    /// The parts of the payload a binding on `name` comes to against a
    /// variable of `dimensions`, with no FMU anywhere: expansion is a pure
    /// function of the pattern and those sizes.
    fn json_pointers(name: &str, source: Option<InputSource>, dimensions: &[usize]) -> Vec<String> {
        let mut binding = InputBinding::new(name, KeyExpr::new("continuo/w/x").unwrap());
        binding.source = source;
        binding.expand(dimensions).expect("expands")
    }

    /// Whatever [`InputBinding::with_pointer`] would make of it, which is how
    /// a test says a source the way a mapping does.
    fn source(written: impl Into<InputSource>) -> Option<InputSource> {
        Some(written.into())
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
        assert_eq!(json_pointer_from_fmu_var_name("speed"), "/speed");
        assert_eq!(json_pointer_from_fmu_var_name("position.x"), "/position/x");
        assert_eq!(
            json_pointer_from_fmu_var_name("orientation.w"),
            "/orientation/w"
        );
    }

    #[test]
    fn an_indexed_name_spells_one_token_per_axis() {
        // FMI writes a multi-dimensional index as one bracket group with
        // commas inside it, which the standard's own `der(pipe[3,4].T[14],2)`
        // example shows. A JSON Pointer wants one token per axis.
        assert_eq!(json_pointer_from_fmu_var_name("a[1]"), "/a/1");
        assert_eq!(json_pointer_from_fmu_var_name("a[1,2]"), "/a/1/2");
        assert_eq!(json_pointer_from_fmu_var_name("m.a[1,2].b"), "/m/a/1/2/b");
        assert_eq!(
            json_pointer_from_fmu_var_name("pipe[3,4].T[14]"),
            "/pipe/3/4/T/14"
        );

        // Chained brackets are not a form the grammar generates, and cost
        // nothing to accept.
        assert_eq!(json_pointer_from_fmu_var_name("a[1][2]"), "/a/1/2");
    }

    #[test]
    fn a_quoted_name_is_one_token_however_it_is_punctuated() {
        // A quoted name may hold any of the separators, so splitting on them
        // would spell a path out of somebody's variable name.
        assert_eq!(json_pointer_from_fmu_var_name("'a.b'"), "/a.b");
        assert_eq!(json_pointer_from_fmu_var_name("'a[1]'"), "/a[1]");
        assert_eq!(
            json_pointer_from_fmu_var_name("robot.axis.'motor #234'"),
            "/robot/axis/motor #234"
        );

        // The quotes are the standard's syntax rather than part of the name,
        // and what RFC 6901 reserves still needs escaping inside them.
        assert_eq!(json_pointer_from_fmu_var_name("'a/b'"), "/a~1b");
    }

    #[test]
    fn an_escape_inside_a_quoted_name_stands_for_its_character() {
        // Without this a `\'` would end the quote early, and the rest of the
        // name would be read as though it were syntax.
        assert_eq!(json_pointer_from_fmu_var_name(r"'a\'b'"), "/a'b");
        assert_eq!(json_pointer_from_fmu_var_name(r"'a\\b'"), "/a\\b");
        assert_eq!(json_pointer_from_fmu_var_name(r"'a\tb'"), "/a\tb");
    }

    #[test]
    fn a_derivative_name_stays_one_token() {
        // `der(x)` names another variable rather than a path through a
        // payload, so there is nothing to nest, and splitting on the
        // punctuation inside would spell a path out of the syntax.
        assert_eq!(json_pointer_from_fmu_var_name("der(x)"), "/der(x)");
        assert_eq!(
            json_pointer_from_fmu_var_name("der(pipe[3,4].T[14],2)"),
            "/der(pipe[3,4].T[14],2)"
        );
    }

    #[test]
    fn a_name_containing_pointer_punctuation_is_escaped() {
        // RFC 6901 gives `~` and `/` meaning inside a token, and FMI puts no
        // such restriction on a variable name, so a name carrying either has
        // to survive the derivation rather than silently address elsewhere.
        assert_eq!(json_pointer_from_fmu_var_name("a/b"), "/a~1b");
        assert_eq!(json_pointer_from_fmu_var_name("a~b"), "/a~0b");
        assert_eq!(json_pointer_from_fmu_var_name("a~/b"), "/a~0~1b");
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
    fn a_wildcard_walks_one_field_across_every_element() {
        // The radar's shape, and the reason a pattern exists: the payload is
        // a list of object dictionaries, the variable is a plain array, and
        // nothing but the FMU says how many elements to take.
        //
        // The scenario file this becomes:
        //
        //     { fmu_var: "range", message_key: ".../radar",
        //       from: "/detections/*/range", when_missing: 1e9 }
        let pattern = || source("/detections/*/range");
        assert_eq!(
            json_pointers("range", pattern(), &[3]),
            [
                "/detections/0/range",
                "/detections/1/range",
                "/detections/2/range"
            ]
        );

        // The same pattern against a variable the FMU declares differently,
        // which is the whole point of not writing the count down.
        assert_eq!(
            json_pointers("range", pattern(), &[1]),
            ["/detections/0/range"]
        );
    }

    #[test]
    fn a_wildcard_per_dimension_expands_row_major() {
        // The rightmost index runs through its whole range first, which is
        // what FMI specifies and what a transposed matrix would quietly
        // violate while still running.
        let pattern = |p: &str| source(p);
        assert_eq!(json_pointers("a", pattern("/a"), &[]), ["/a"]);
        assert_eq!(
            json_pointers("a", pattern("/a/*"), &[3]),
            ["/a/0", "/a/1", "/a/2"]
        );
        assert_eq!(
            json_pointers("a", pattern("/a/*/*"), &[2, 3]),
            ["/a/0/0", "/a/0/1", "/a/0/2", "/a/1/0", "/a/1/1", "/a/1/2"]
        );
        assert_eq!(
            json_pointers("a", pattern("/a/*/*/*"), &[2, 2, 2]),
            [
                "/a/0/0/0", "/a/0/0/1", "/a/0/1/0", "/a/0/1/1", "/a/1/0/0", "/a/1/0/1", "/a/1/1/0",
                "/a/1/1/1"
            ]
        );

        // A wildcard walks indices and never field names, so a second one
        // needs the payload's inner axis to be an array too. What binds here
        // is a variable declaring two dimensions, fed from
        //
        //     {"tracks": [{"lead": [1.0, 2.0]}, {"lead": [3.0, 4.0]}]}
        //
        // taking `tracks` as its outer dimension and each `lead` as the
        // inner. A payload whose inner axis is named fields instead wants one
        // variable per field, each of a single dimension.
        assert_eq!(
            json_pointers("a", pattern("/tracks/*/lead/*"), &[2, 2]),
            [
                "/tracks/0/lead/0",
                "/tracks/0/lead/1",
                "/tracks/1/lead/0",
                "/tracks/1/lead/1"
            ]
        );
    }

    #[test]
    fn a_name_alone_walks_every_element_of_whatever_it_names() {
        // With no source at all, which is what omitting one in a scenario
        // file will mean. A scalar reads the path its name spells and an
        // array reads every element of it, the FMU supplying the rank.
        //
        // The scenario file this becomes:
        //
        //     { fmu_var: "position.x", message_key: ".../pose" }
        assert_eq!(json_pointers("position.x", None, &[]), ["/position/x"]);
        assert_eq!(json_pointers("u", None, &[3]), ["/u/0", "/u/1", "/u/2"]);
        assert_eq!(
            json_pointers("u", None, &[2, 2]),
            ["/u/0/0", "/u/0/1", "/u/1/0", "/u/1/1"]
        );
    }

    #[test]
    fn only_a_whole_token_is_a_wildcard() {
        // What the syntax costs is a payload key spelled exactly `*`, and
        // nothing wider than that.
        let pattern = Some(InputSource::Pattern("/a*b/*/c*".to_string()));
        assert_eq!(
            json_pointers("v", pattern, &[2]),
            ["/a*b/0/c*", "/a*b/1/c*"]
        );
    }

    #[test]
    fn a_written_out_list_is_taken_exactly_as_it_is() {
        // The form for a payload no single pattern reaches, so nothing is
        // expanded and no wildcard is read.
        //
        // The scenario file this becomes, a list where the pattern form is a
        // string, which is what keeps the two apart with no tag to write:
        //
        //     { fmu_var: "x0", message_key: ".../pose",
        //       from: ["/position/x", "/position/y"] }
        let written = source(["/position/x", "/position/y"]);
        assert_eq!(
            json_pointers("x0", written, &[2]),
            ["/position/x", "/position/y"]
        );
    }

    #[test]
    fn a_pattern_whose_wildcards_miss_the_rank_fails_naming_both() {
        // The mistake worth catching, since `/u` against an array would
        // otherwise bind one element of a variable holding many.
        let mut binding =
            InputBinding::new("u", KeyExpr::new("continuo/w/x").unwrap()).with_pointer("/u");
        let reason = binding.expand(&[3]).unwrap_err().to_string();
        assert!(reason.contains("\"u\""), "{reason}");
        assert!(reason.contains("/u"), "{reason}");
        assert!(reason.contains('3'), "{reason}");

        // And the other way, where a pattern walks an axis the FMU does not
        // have.
        binding = binding.with_pointer("/u/*/*");
        let reason = binding.expand(&[3]).unwrap_err().to_string();
        assert!(reason.contains("/u/*/*"), "{reason}");
        assert!(reason.contains('2'), "{reason}");
    }
}

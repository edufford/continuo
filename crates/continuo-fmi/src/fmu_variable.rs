//! What a mapping resolves to against the FMU: each variable's value
//! reference, declared type and sizes, and the bindings that carry them.

use std::collections::BTreeMap;

use continuo_core::CoreError;
use fmi::fmi3::schema::{
    AbstractVariableTrait, ArrayableVariableTrait, Causality, Dimension, Fmi3ModelDescription,
    InitializableVariableTrait, Variable, VariableType,
};
use serde_json::Value;

use crate::error::FmuConstructionError;
use crate::fmu_mapping::{InputBinding, OutputBinding};

/// An input binding with its variable resolved against the FMU.
pub(crate) struct BoundInput {
    pub(crate) binding: InputBinding,
    pub(crate) variable: ResolvedVariable,
    /// One JSON Pointer per element, worked out once at construction from the
    /// binding's source and the sizes the FMU turned out to declare.
    json_pointers: Vec<String>,
}

impl BoundInput {
    /// Binds an input to the variable it feeds, working out every part of the
    /// payload it reads and checking they come to what the variable holds.
    ///
    /// The FMU is the authority on that count. A pattern cannot disagree with
    /// it, since its wildcards expand over those dimensions rather than
    /// repeating them, so what the check still catches is a list written out
    /// by hand: unchecked, a rebuilt FMU and a stale mapping drift apart and
    /// the model reads whatever the tail of the buffer held.
    pub(crate) fn new(
        binding: InputBinding,
        variable: ResolvedVariable,
    ) -> Result<Self, FmuConstructionError> {
        let json_pointers = binding.expand(&variable.dimensions)?;
        if json_pointers.len() != variable.len() {
            return Err(FmuConstructionError::Dimension {
                variable: variable.name.clone(),
                supplied: json_pointers.len(),
                expected: variable.len(),
                dimensions: variable.dimensions.clone(),
            });
        }

        // Return an input that knows every part of the payload it reads.
        Ok(BoundInput {
            binding,
            variable,
            json_pointers,
        })
    }

    /// The value for each element, in the order the FMU reads them, resolved
    /// against one decoded payload.
    ///
    /// Addresses the decoded value rather than the bytes it arrived as, so
    /// the wire format never reaches this crate.
    pub(crate) fn resolve<'a>(&'a self, payload: &'a Value) -> Result<Vec<&'a Value>, CoreError> {
        self.json_pointers
            .iter()
            .map(|json_pointer| {
                payload
                    .pointer(json_pointer)
                    .or(self.binding.when_missing.as_ref())
                    .ok_or_else(|| CoreError::ComponentFailure {
                        reason: format!(
                            "input {:?}: nothing at {:?} on {}, and no value declared for its absence",
                            self.binding.fmu_var_name,
                            json_pointer,
                            self.binding.subscribed_key.as_str()
                        ),
                    })
            })
            .collect()
    }
}

/// An output binding with its variable resolved against the FMU.
pub(crate) struct BoundOutput {
    pub(crate) binding: OutputBinding,
    pub(crate) variable: ResolvedVariable,
}

/// What a variable's name resolves to against the FMU.
///
/// A few fields copied out of the model description rather than the schema's
/// own variable, because neither way of keeping that one exists: `Variable`
/// is not `Clone`, so it cannot be owned, and borrowing it would make the
/// component self-referential, since the import holding the description is a
/// field beside the variables that would point into it.
///
/// `dimensions` is the part the schema could not supply in any case. It
/// declares a value reference where a structural parameter sizes an array,
/// and turning that into a number is most of what this type is for.
///
/// Always carries real sizes, which is why [`StructuralSizes`] is worked out
/// before anything is resolved. A half-built one would be indistinguishable
/// from a scalar, since both have no dimensions, and an array mistaken for a
/// scalar reads one value where it should read many.
pub(crate) struct ResolvedVariable {
    pub(crate) name: String,
    pub(crate) value_reference: u32,
    pub(crate) declared_type: VariableType,
    pub(crate) causality: Causality,
    /// The size of each dimension, outermost first, and empty for a scalar.
    /// Resolved rather than read: a dimension may name a structural parameter
    /// instead of stating a number, so these are the sizes this instance
    /// actually has rather than the ones the XML declares.
    pub(crate) dimensions: Vec<usize>,
}

impl ResolvedVariable {
    /// How many values this variable holds: the product of its dimensions,
    /// and one for a scalar.
    pub(crate) fn len(&self) -> usize {
        self.dimensions.iter().product::<usize>().max(1)
    }

    /// Whether this is one of the structural parameters that size the FMU's
    /// arrays, and so has to be written before initialization begins.
    pub(crate) fn is_structural(&self) -> bool {
        self.causality == Causality::StructuralParameter
    }
}

/// What the FMU says about a variable the mapping names, sized for this
/// instance.
pub(crate) fn resolve_fmu_var(
    description: &Fmi3ModelDescription,
    name: &str,
    sizes: &StructuralSizes,
) -> Result<ResolvedVariable, FmuConstructionError> {
    let declared = description
        .model_variables
        .find_by_name(name)
        .ok_or_else(|| FmuConstructionError::UnknownVariable {
            variable: name.to_string(),
            available: description
                .model_variables
                .iter_abstract()
                .map(|variable| variable.name().to_string())
                .collect(),
        })?;

    let dimensions = declared_dimensions(description, name)
        .iter()
        .map(|dimension| match dimension {
            Dimension::Fixed(size) => Ok(*size as usize),
            Dimension::Variable(value_reference) => sizes
                .by_value_reference
                .get(value_reference)
                .copied()
                .ok_or_else(|| FmuConstructionError::UnresolvedDimension {
                    variable: name.to_string(),
                    value_reference: *value_reference,
                }),
        })
        .collect::<Result<Vec<_>, _>>()?;

    // Return the variable in the form every read and write takes.
    Ok(ResolvedVariable {
        name: name.to_string(),
        value_reference: declared.value_reference(),
        declared_type: declared.data_type(),
        causality: declared.causality(),
        dimensions,
    })
}

/// The effective size of every dimension that names a structural parameter.
///
/// A dimension is either a constant or a reference to a variable's value, and
/// where it is a reference the value in force is the mapping's if it set one
/// and the FMU's declared start otherwise. Reading the XML alone would size
/// an array by numbers the mapping has already overridden.
pub(crate) struct StructuralSizes {
    /// Keyed by the structural parameter's value reference, which is how a
    /// `Dimension` names one, holding the size in force for this instance.
    pub(crate) by_value_reference: BTreeMap<u32, usize>,
}

impl StructuralSizes {
    /// `initial_values` is the mapping's own list, in full: the entries that
    /// name a structural parameter are picked out here, since this has to run
    /// before any variable can be resolved.
    pub(crate) fn new(
        description: &Fmi3ModelDescription,
        initial_values: &[(String, Value)],
    ) -> Result<Self, FmuConstructionError> {
        let mut by_value_reference = BTreeMap::new();

        // What the FMU itself says first: every structural parameter's
        // `start` attribute in `modelDescription.xml`. FMI requires one to be
        // UInt64 wherever a dimension names it, which is what makes matching
        // that one arm enough.
        for declaration in &description.model_variables.variables {
            if let Variable::UInt64(parameter) = declaration
                && parameter.causality() == Causality::StructuralParameter
                && let Some(start) = parameter.start().and_then(|start| start.first())
            {
                by_value_reference.insert(parameter.value_reference, *start as usize);
            }
        }

        // Then the mapping's own values, overwriting a start rather than
        // competing with it. Entries naming anything else are skipped here
        // and resolved later, which is where an unknown name is reported.
        for (name, value) in initial_values {
            let Some(variable) = description.model_variables.find_by_name(name) else {
                continue;
            };
            if variable.causality() != Causality::StructuralParameter {
                continue;
            }

            let size = value
                .as_u64()
                .ok_or_else(|| FmuConstructionError::StructuralParameter {
                    variable: name.clone(),
                    value: value.to_string(),
                })?;
            by_value_reference.insert(variable.value_reference(), size as usize);
        }

        // Return the sizes this instance runs with.
        Ok(StructuralSizes { by_value_reference })
    }
}

impl ResolvedVariable {
    /// Turns a value the mapping wrote into the flat, row-major list an FMI
    /// call takes. The reverse of [`ResolvedVariable::shape`].
    ///
    /// Descends only as far as the variable has dimensions, so a matrix may
    /// be written either as arrays of arrays or as one flat list. The
    /// standard spells a matrix flat in a `start` attribute, so accepting
    /// both costs nothing.
    pub(crate) fn flatten<'a>(&self, value: &'a Value) -> Vec<&'a Value> {
        // Recursive, one level per dimension, and anything at the bottom is a
        // value rather than something to descend into.
        fn walk<'a>(value: &'a Value, depth: usize, flat: &mut Vec<&'a Value>) {
            match value {
                Value::Array(elements) if depth > 0 => {
                    for element in elements {
                        walk(element, depth - 1, flat);
                    }
                }
                leaf => flat.push(leaf),
            }
        }

        let mut flat = Vec::new();
        walk(value, self.dimensions.len(), &mut flat);

        // Return the values in the order the FMU reads them.
        flat
    }

    /// Nests a flat, row-major list of values back into the shape the
    /// variable declares: a scalar as itself, a vector as an array, a matrix
    /// as arrays of arrays.
    pub(crate) fn shape(&self, values: Vec<Value>) -> Value {
        if self.dimensions.is_empty() {
            return values.into_iter().next().unwrap_or(Value::Null);
        }

        // Build the nesting from the inside out. Each pass takes the current
        // flat list and groups it into arrays of one dimension's size, so a
        // 2 by 3 matrix goes from six values to two arrays of three. The
        // outermost dimension is skipped because the return wraps it.
        //
        // Innermost first is what makes this row-major: the last index is the
        // one whose neighbors are adjacent in the flat list.
        let mut level = values;
        for size in self.dimensions.iter().skip(1).rev() {
            level = level
                .chunks(*size)
                .map(|chunk| Value::Array(chunk.to_vec()))
                .collect();
        }

        Value::Array(level)
    }
}

/// The dimensions `modelDescription.xml` declares for a variable, before any
/// structural parameter is resolved.
///
/// Matched over the variable enum because `dimensions` belongs to the
/// arrayable trait rather than the abstract one, and a name lookup hands back
/// the abstract view.
fn declared_dimensions<'a>(description: &'a Fmi3ModelDescription, name: &str) -> &'a [Dimension] {
    const NONE: &[Dimension] = &[];

    macro_rules! arms {
        ($variable:expr, $($case:ident),*) => {
            match $variable {
                $(Variable::$case(var) => (var.name(), var.dimensions()),)*
                Variable::Clock(var) => (var.name(), NONE),
            }
        };
    }

    description
        .model_variables
        .variables
        .iter()
        .find_map(|variable| {
            let (declared_name, dimensions) = arms!(
                variable, Int8, UInt8, Int16, UInt16, Int32, UInt32, Int64, UInt64, Float32,
                Float64, Boolean, String, Binary
            );
            (declared_name == name).then_some(dimensions)
        })
        .unwrap_or(NONE)
}

#[cfg(test)]
mod tests {
    use continuo_core::KeyExpr;
    use serde_json::json;

    use super::*;
    use crate::fmu_mapping::InputSource;

    /// A binding bound to a variable of `dimensions`, with no FMU anywhere:
    /// binding and resolution are pure functions of the mapping and a decoded
    /// value, which is what keeps the wire format out of this crate.
    fn bind(
        binding: InputBinding,
        dimensions: Vec<usize>,
    ) -> Result<BoundInput, FmuConstructionError> {
        let variable = ResolvedVariable {
            name: binding.fmu_var_name.clone(),
            value_reference: 0,
            declared_type: VariableType::FmiFloat64,
            causality: Causality::Input,
            dimensions,
        };
        BoundInput::new(binding, variable)
    }

    /// A binding on `v` reading `source`, with a default for an element that
    /// is not there.
    fn reading(source: impl Into<InputSource>, when_missing: Value) -> InputBinding {
        InputBinding::new("v", KeyExpr::new("continuo/w/x").unwrap())
            .with_pointer(source)
            .when_missing(when_missing)
    }

    #[test]
    fn a_matrix_resolves_row_by_row() {
        let payload = json!({"a": [[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]]});
        let bound = bind(reading("/a/*/*", json!(0.0)), vec![2, 3]).expect("binds");
        assert_eq!(
            bound.resolve(&payload).unwrap(),
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
        let binding = reading(["/speed", "/position/x", "/position/y"], json!(0.0));
        let bound = bind(binding, vec![3]).expect("binds");
        assert_eq!(
            bound.resolve(&payload).unwrap(),
            [&json!(3.0), &json!(1.0), &json!(2.0)]
        );
    }

    #[test]
    fn a_short_payload_fills_the_tail_with_the_default() {
        // The radar's case: a scan carrying two cars feeds a variable sized
        // for four, and the empty slots have to read as a clear road. The
        // pattern is the same one either way, since the size that differs is
        // the FMU's rather than the mapping's.
        let payload = json!({"detections": [{"range": 10.0}, {"range": 20.0}]});
        let bound = bind(reading("/detections/*/range", json!(1e9)), vec![4]).expect("binds");
        assert_eq!(
            bound.resolve(&payload).unwrap(),
            [&json!(10.0), &json!(20.0), &json!(1e9), &json!(1e9)]
        );
    }

    #[test]
    fn a_default_covers_only_the_elements_that_are_missing() {
        // One default serves every element, so a scan that skips a slot in
        // the middle is not a special case.
        let payload = json!({"detections": [{"range": 10.0}, {}, {"range": 30.0}]});
        let bound = bind(reading("/detections/*/range", json!(1e9)), vec![4]).expect("binds");
        assert_eq!(
            bound.resolve(&payload).unwrap(),
            [&json!(10.0), &json!(1e9), &json!(30.0), &json!(1e9)]
        );
    }

    #[test]
    fn a_pointer_finding_nothing_uses_the_default_whatever_its_type() {
        // One field serves every FMI type, so the default for a boolean is a
        // boolean and the default for a string is a string.
        let payload = json!({"present": 1.0});
        for default in [json!(true), json!("idle"), json!(-7)] {
            let bound = bind(reading(["/absent"], default.clone()), Vec::new()).expect("binds");
            assert_eq!(bound.resolve(&payload).unwrap(), [&default]);
        }
    }

    #[test]
    fn a_derived_pointer_reads_the_payload_its_variable_is_named_for() {
        let payload = json!({"position": {"x": 4.5}});
        let binding = InputBinding::new(
            "position.x",
            KeyExpr::new("continuo/w/actor/car/pose").unwrap(),
        );
        let bound = bind(binding, Vec::new()).expect("binds");
        assert_eq!(bound.resolve(&payload).unwrap(), [&json!(4.5)]);
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
        let bound = bind(binding, Vec::new()).expect("binds");

        let reason = bound.resolve(&payload).unwrap_err().to_string();
        assert!(reason.contains("position.x"), "{reason}");
        assert!(reason.contains("/position/x"), "{reason}");
        assert!(reason.contains("continuo/w/actor/car/pose"), "{reason}");
    }

    #[test]
    fn a_written_out_list_that_misses_the_count_does_not_bind() {
        // The one form that states a size of its own, and so the only one
        // that can disagree with the FMU about it.
        let binding = reading(["/a/0", "/a/1"], json!(0.0));
        let reason = match bind(binding, vec![3]) {
            Err(error) => error.to_string(),
            Ok(_) => panic!("two pointers should not bind a three-value variable"),
        };
        assert!(reason.contains("\"v\""), "{reason}");
        assert!(reason.contains('3'), "{reason}");
        assert!(reason.contains('2'), "{reason}");
    }
}

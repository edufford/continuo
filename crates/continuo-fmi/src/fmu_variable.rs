//! What a mapping resolves to against the FMU: each variable's value
//! reference, declared type and sizes, and the bindings that carry them.

use std::collections::BTreeMap;

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
        // one whose neighbours are adjacent in the flat list.
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

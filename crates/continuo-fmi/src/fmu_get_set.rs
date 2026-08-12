//! Reading and writing an FMU's variables, dispatched on the type each one
//! declares.
//!
//! A mapping never names a type. The FMU does, in `modelDescription.xml`, and
//! repeating it in the mapping would be a second source of truth that could
//! disagree with the binary it points at.

use std::ffi::CString;

use continuo_core::{ComponentId, CoreError};
use fmi::fmi3::GetSet as _;
use fmi::fmi3::instance::InstanceCS;
use fmi::fmi3::schema::VariableType;
use serde_json::Value;

use crate::convert;
use crate::error::{step_failure, unbound_clock};
use crate::fmu_variable::{BoundInput, BoundOutput, ResolvedVariable};

/// How much room to give an FMU for one Binary value.
///
/// `fmi3GetBinary` writes into a buffer the caller sizes, and
/// `modelDescription.xml` need not say how large a value will be, so there is
/// nothing honest to read. A megabyte is far past anything a variable
/// carrying configuration or a small blob would use.
// TODO(PLAN "Deferred"): an FMU wanting more than this is the large-payload
// item rather than a constant to raise, since a value that size should not be
// travelling base64 inside a JSON string in the first place.
const MAX_BINARY: usize = 1 << 20;

/// Sets one input variable, dispatching on the type the FMU declares.
///
/// A free function rather than a method so the instance and the bindings can
/// be borrowed at the same time, which `&mut self` does not allow.
pub(crate) fn set_input_var(
    instance: &mut InstanceCS,
    id: &ComponentId,
    input: &BoundInput,
    values: &[&Value],
) -> Result<(), CoreError> {
    set_values(instance, id, &input.variable, values)
}

/// Writes one variable, whatever its shape.
///
/// A variable holding many values is still one call into the FMU, carrying
/// all of them at once, so an array is not a special case here.
pub(crate) fn set_values(
    instance: &mut InstanceCS,
    id: &ComponentId,
    variable: &ResolvedVariable,
    values: &[&Value],
) -> Result<(), CoreError> {
    let name = variable.name.as_str();
    let references = [variable.value_reference];

    macro_rules! set_with {
        ($convert:path, $setter:ident) => {{
            let converted = values
                .iter()
                .map(|value| $convert(value, name))
                .collect::<Result<Vec<_>, _>>()?;
            instance
                .$setter(&references, &converted)
                .map(|_| ())
                .map_err(|source| step_failure(id, name, stringify!($setter), &source))
        }};
    }

    match variable.declared_type {
        VariableType::FmiFloat64 => set_with!(convert::to_fmi_f64, set_float64),
        VariableType::FmiFloat32 => set_with!(convert::to_fmi_f32, set_float32),
        VariableType::FmiInt8 => set_with!(convert::to_fmi_i8, set_int8),
        VariableType::FmiInt16 => set_with!(convert::to_fmi_i16, set_int16),
        VariableType::FmiInt32 => set_with!(convert::to_fmi_i32, set_int32),
        VariableType::FmiInt64 => set_with!(convert::to_fmi_i64, set_int64),
        VariableType::FmiUInt8 => set_with!(convert::to_fmi_u8, set_uint8),
        VariableType::FmiUInt16 => set_with!(convert::to_fmi_u16, set_uint16),
        VariableType::FmiUInt32 => set_with!(convert::to_fmi_u32, set_uint32),
        VariableType::FmiUInt64 => set_with!(convert::to_fmi_u64, set_uint64),
        VariableType::FmiBoolean => set_with!(convert::to_fmi_bool, set_boolean),
        VariableType::FmiString => {
            let converted = values
                .iter()
                .map(|value| convert::to_fmi_string(value, name))
                .collect::<Result<Vec<_>, _>>()?;
            instance
                .set_string(&references, &converted)
                .map_err(|source| step_failure(id, name, "set_string", &source))
        }
        VariableType::FmiBinary => {
            let converted = values
                .iter()
                .map(|value| convert::to_fmi_binary(value, name))
                .collect::<Result<Vec<_>, _>>()?;
            let borrowed: Vec<&[u8]> = converted.iter().map(Vec::as_slice).collect();
            instance
                .set_binary(&references, &borrowed)
                .map_err(|source| step_failure(id, name, "set_binary", &source))
        }
        VariableType::FmiClock => Err(unbound_clock(name)),
    }
}

/// Reads one output variable, dispatching on the type the FMU declares.
pub(crate) fn get_output_var(
    instance: &mut InstanceCS,
    id: &ComponentId,
    output: &BoundOutput,
) -> Result<Value, CoreError> {
    let variable = &output.variable;
    let name = variable.name.as_str();
    let references = [variable.value_reference];
    let count = variable.len();

    macro_rules! get_with {
        // `$zero` fills a buffer the FMU writes into, so it is a size rather
        // than a fallback: whatever it holds is overwritten by the get, and
        // a failed get returns before the value is read.
        ($convert:path, $getter:ident, $zero:expr) => {{
            let mut values = vec![$zero; count];
            instance
                .$getter(&references, &mut values)
                .map_err(|source| step_failure(id, name, stringify!($getter), &source))?;
            let converted = values
                .into_iter()
                .map(|value| $convert(value, name))
                .collect::<Result<Vec<_>, _>>()?;
            Ok(variable.shape(converted))
        }};
    }

    match variable.declared_type {
        VariableType::FmiFloat64 => get_with!(convert::from_fmi_f64, get_float64, 0.0),
        VariableType::FmiFloat32 => get_with!(convert::from_fmi_f32, get_float32, 0.0),
        VariableType::FmiInt8 => get_with!(convert::from_fmi_i8, get_int8, 0),
        VariableType::FmiInt16 => get_with!(convert::from_fmi_i16, get_int16, 0),
        VariableType::FmiInt32 => get_with!(convert::from_fmi_i32, get_int32, 0),
        VariableType::FmiInt64 => get_with!(convert::from_fmi_i64, get_int64, 0),
        VariableType::FmiUInt8 => get_with!(convert::from_fmi_u8, get_uint8, 0),
        VariableType::FmiUInt16 => get_with!(convert::from_fmi_u16, get_uint16, 0),
        VariableType::FmiUInt32 => get_with!(convert::from_fmi_u32, get_uint32, 0),
        VariableType::FmiUInt64 => get_with!(convert::from_fmi_u64, get_uint64, 0),
        VariableType::FmiBoolean => get_with!(convert::from_fmi_bool, get_boolean, false),
        VariableType::FmiString => {
            // What the FMU hands back is valid only until the next call
            // on this instance. `get_string` copies it into these owned
            // `CString`s before returning, and `from_fmi_string` copies again
            // into an owned `Value`, so nothing here holds a borrow into FMU
            // memory. Getting that wrong reads as intermittent corruption
            // rather than as a failure.
            let mut values = vec![CString::default(); count];
            instance
                .get_string(&references, &mut values)
                .map_err(|source| step_failure(id, name, "get_string", &source))?;
            let converted = values
                .iter()
                .map(|value| convert::from_fmi_string(value, name))
                .collect::<Result<Vec<_>, _>>()?;
            Ok(variable.shape(converted))
        }
        VariableType::FmiBinary => {
            let mut buffers = vec![vec![0u8; MAX_BINARY]; count];
            let mut slices: Vec<&mut [u8]> = buffers.iter_mut().map(Vec::as_mut_slice).collect();
            let sizes = instance
                .get_binary(&references, &mut slices)
                .map_err(|source| step_failure(id, name, "get_binary", &source))?;
            // A size of zero is a length rather than a failure: an FMU may
            // hold an empty Binary, which encodes as the empty string and
            // decodes back to no bytes.
            let converted = buffers
                .iter()
                .zip(sizes.iter().chain(std::iter::repeat(&0)))
                .map(|(buffer, size)| convert::from_fmi_binary(&buffer[..*size], name))
                .collect::<Result<Vec<_>, _>>()?;
            Ok(variable.shape(converted))
        }
        VariableType::FmiClock => Err(unbound_clock(name)),
    }
}

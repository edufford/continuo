//! Values crossing between a payload and an FMU variable.
//!
//! Every conversion is checked against the type the FMU declares, and a value
//! that does not fit halts rather than wrapping, saturating or rounding. An
//! Int8 handed 999 is a wiring mistake, and the run that hides it is worse
//! than the run that stops.
//!
//! Integers never travel through `f64`. Above 2^53 that loses digits in
//! silence, and `serde_json` carries big integers exactly (the workspace
//! turns on `arbitrary_precision`), so the round trip is lossless only if the
//! integer path stays integral end to end. Every conversion here goes through
//! `i128`, which holds every FMI integer type with room to spare, and the
//! range check falls out of `try_into`.

use std::ffi::CString;

use continuo_core::CoreError;
use serde_json::Value;

/// Fails naming the variable, what it is, and what it was handed.
///
/// The variable name is the useful half: a mapping points at an FMU built
/// elsewhere, so "which of the forty numbers was wrong" is the question, and
/// the value alone does not answer it.
fn mismatch(variable: &str, wanted: &str, value: &Value) -> CoreError {
    CoreError::ComponentFailure {
        reason: format!("variable {variable:?} is {wanted}, and {value} is not a valid one"),
    }
}

/// A JSON number as an integer, whatever its width and sign.
///
/// `i128` because it is the one type that holds every FMI integer, `Int64`
/// and `UInt64` alike, so the eight conversions differ only in the range
/// `try_into` checks. Returns `None` for anything that is not a whole number,
/// so `3.0` fed to an `Int32` fails as a type mismatch rather than quietly
/// truncating to `3`.
fn as_integer(value: &Value) -> Option<i128> {
    match value.as_i64() {
        Some(signed) => Some(i128::from(signed)),
        None => value.as_u64().map(i128::from),
    }
}

macro_rules! integer_conversions {
    ($($to:ident, $from:ident, $ty:ty, $fmi_name:literal;)*) => {
        $(
            #[doc = concat!("A payload value as an FMI ", $fmi_name, ".")]
            pub fn $to(value: &Value, variable: &str) -> Result<$ty, CoreError> {
                as_integer(value)
                    .and_then(|integer| <$ty>::try_from(integer).ok())
                    .ok_or_else(|| mismatch(variable, concat!("an ", $fmi_name), value))
            }

            #[doc = concat!("An FMI ", $fmi_name, " as a payload value.")]
            pub fn $from(value: $ty, _variable: &str) -> Result<Value, CoreError> {
                Ok(Value::from(value))
            }
        )*
    };
}

integer_conversions! {
    to_i8, from_i8, i8, "Int8";
    to_i16, from_i16, i16, "Int16";
    to_i32, from_i32, i32, "Int32";
    to_i64, from_i64, i64, "Int64";
    to_u8, from_u8, u8, "UInt8";
    to_u16, from_u16, u16, "UInt16";
    to_u32, from_u32, u32, "UInt32";
    to_u64, from_u64, u64, "UInt64";
}

/// A payload value as an FMI Float64. Whole numbers are accepted, since JSON
/// writes `3` and `3.0` for the same quantity and only the integer types care
/// which.
pub fn to_f64(value: &Value, variable: &str) -> Result<f64, CoreError> {
    value
        .as_f64()
        .ok_or_else(|| mismatch(variable, "a Float64", value))
}

/// A payload value as an FMI Float32.
///
/// Precision loss is accepted and range overflow is not: narrowing 0.1
/// changes the digits after the fifteenth, which is what asking for a
/// Float32 means, while narrowing 1e300 produces infinity, which is a
/// different number rather than a rounder one.
pub fn to_f32(value: &Value, variable: &str) -> Result<f32, CoreError> {
    let wide = to_f64(value, variable)?;
    let narrow = wide as f32;
    if narrow.is_finite() || !wide.is_finite() {
        Ok(narrow)
    } else {
        Err(mismatch(variable, "a Float32", value))
    }
}

/// An FMI Float64 as a payload value.
///
/// Non-finite fails here rather than at the publish guard, because this is
/// where the variable's name is still in hand. JSON writes `NaN` and `±inf`
/// as `null`, which decodes nowhere, so a diverging FMU would otherwise
/// surface as a decode failure at some other component, at a later instant,
/// with nothing pointing back at the model that produced it.
pub fn from_f64(value: f64, variable: &str) -> Result<Value, CoreError> {
    if value.is_finite() {
        Ok(Value::from(value))
    } else {
        Err(CoreError::ComponentFailure {
            reason: format!("variable {variable:?} produced {value}, which no payload can carry"),
        })
    }
}

/// An FMI Float32 as a payload value, widened losslessly.
pub fn from_f32(value: f32, variable: &str) -> Result<Value, CoreError> {
    from_f64(f64::from(value), variable)
}

/// A payload value as an FMI Boolean.
///
/// A flag rather than a number, so `1` is not `true` here. FMI declares a
/// Boolean where it means one, `fmi3Boolean` is a C `bool`, and JSON has the
/// literals, so nothing along the way needs an integer to stand in.
pub fn to_bool(value: &Value, variable: &str) -> Result<bool, CoreError> {
    value
        .as_bool()
        .ok_or_else(|| mismatch(variable, "a Boolean", value))
}

/// An FMI Boolean as a payload value.
pub fn from_bool(value: bool, _variable: &str) -> Result<Value, CoreError> {
    Ok(Value::Bool(value))
}

/// A payload value as an FMI String.
///
/// An interior NUL fails, since a C string ends at the first one and passing
/// it on would hand the FMU a silently shortened value. JSON can carry a NUL
/// as an escape, so this is reachable from ordinary data rather than only
/// from a mistake.
pub fn to_cstring(value: &Value, variable: &str) -> Result<CString, CoreError> {
    let text = value
        .as_str()
        .ok_or_else(|| mismatch(variable, "a String", value))?;

    CString::new(text).map_err(|error| CoreError::ComponentFailure {
        reason: format!(
            "variable {variable:?} is a String, and this one carries a NUL at byte {}, \
             which cannot cross into C without being cut short there",
            error.nul_position()
        ),
    })
}

/// An FMI String as a payload value.
///
/// Fails on bytes that are not UTF-8. FMI 3.0 requires a String to be UTF-8,
/// so this is an FMU breaking the standard rather than a limitation, and it
/// is worth saying so out loud instead of substituting replacement
/// characters.
pub fn from_cstring(value: &CString, variable: &str) -> Result<Value, CoreError> {
    value
        .to_str()
        .map(|text| Value::String(text.to_string()))
        .map_err(|_| CoreError::ComponentFailure {
            reason: format!(
                "variable {variable:?} is a String, and this FMU returned bytes that are not \
                 UTF-8, which FMI 3.0 requires"
            ),
        })
}

#[cfg(test)]
mod tests {
    use serde_json::json;

    use super::*;

    #[test]
    fn an_integer_larger_than_a_float_can_hold_survives_the_round_trip() {
        // The 2^53 trap. Every digit has to survive, so this fails outright
        // if anything on the path routes an integer through an f64.
        let big = 9_007_199_254_740_993_i64;
        assert_eq!(big as f64 as i64, 9_007_199_254_740_992, "premise");

        let out = from_i64(to_i64(&json!(big), "v").unwrap(), "v").unwrap();
        assert_eq!(out, json!(big));
        assert_eq!(out.to_string(), big.to_string());

        let huge = u64::MAX;
        assert_eq!(to_u64(&json!(huge), "v").unwrap(), huge);
    }

    #[test]
    fn a_value_outside_a_variables_range_halts_naming_the_variable_and_type() {
        let reason = to_i8(&json!(999), "gear").unwrap_err().to_string();
        assert!(reason.contains("gear"), "{reason}");
        assert!(reason.contains("Int8"), "{reason}");
        assert!(reason.contains("999"), "{reason}");

        assert!(to_u8(&json!(-1), "v").is_err());
        assert!(to_i32(&json!(i64::from(i32::MAX) + 1), "v").is_err());
        assert!(to_u64(&json!(-1), "v").is_err());
    }

    #[test]
    fn a_fractional_number_is_not_an_integer() {
        // `3.0` fed to an Int32 is a type mismatch rather than a rounding
        // question, so nothing here has to decide which way to round.
        assert!(to_i32(&json!(3.0), "v").is_err());
        assert!(to_i32(&json!(2.5), "v").is_err());
        assert_eq!(to_i32(&json!(3), "v").unwrap(), 3);
    }

    #[test]
    fn a_float_takes_a_whole_number_as_readily_as_a_fractional_one() {
        assert_eq!(to_f64(&json!(3), "v").unwrap(), 3.0);
        assert_eq!(to_f64(&json!(3.5), "v").unwrap(), 3.5);
    }

    #[test]
    fn a_float32_loses_precision_but_not_magnitude() {
        // Narrowing 0.1 changes digits, which is what asking for a Float32
        // means. Narrowing 1e300 produces infinity, which is a different
        // number rather than a rounder one.
        assert_eq!(to_f32(&json!(0.1), "v").unwrap(), 0.1_f32);
        assert!(to_f32(&json!(1e300), "v").is_err());
        assert!(to_f32(&json!(-1e300), "v").is_err());
    }

    #[test]
    fn a_non_finite_output_halts_naming_the_variable() {
        let reason = from_f64(f64::NAN, "height").unwrap_err().to_string();
        assert!(reason.contains("height"), "{reason}");
        assert!(from_f64(f64::INFINITY, "v").is_err());
        assert!(from_f32(f32::NEG_INFINITY, "v").is_err());
    }

    #[test]
    fn a_boolean_is_a_flag_and_not_a_number() {
        assert!(to_bool(&json!(true), "v").unwrap());
        assert!(to_bool(&json!(1), "v").is_err());
        assert!(to_bool(&json!("true"), "v").is_err());
        assert_eq!(from_bool(false, "v").unwrap(), json!(false));
    }

    #[test]
    fn a_string_containing_an_interior_nul_halts_rather_than_truncating() {
        let reason = to_cstring(&json!("ab\u{0}cd"), "label")
            .unwrap_err()
            .to_string();
        assert!(reason.contains("label"), "{reason}");
        assert!(reason.contains('2'), "{reason}");

        assert_eq!(to_cstring(&json!("abcd"), "v").unwrap().as_bytes(), b"abcd");
    }

    #[test]
    fn a_non_utf8_string_from_an_fmu_halts_naming_the_variable() {
        let invalid = CString::new(vec![0xff, 0xfe]).unwrap();
        let reason = from_cstring(&invalid, "label").unwrap_err().to_string();
        assert!(reason.contains("label"), "{reason}");
        assert!(reason.contains("UTF-8"), "{reason}");

        let valid = CString::new("hello").unwrap();
        assert_eq!(from_cstring(&valid, "v").unwrap(), json!("hello"));
    }

    #[test]
    fn an_integer_output_publishes_as_an_integer() {
        // `3` and `3.0` are different bytes and so different hashes, and the
        // world hash is taken over published bytes.
        assert_eq!(from_i32(3, "v").unwrap().to_string(), "3");
        assert_eq!(from_u8(3, "v").unwrap().to_string(), "3");
        assert_eq!(from_f64(3.0, "v").unwrap().to_string(), "3.0");
    }
}

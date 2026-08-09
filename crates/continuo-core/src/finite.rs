//! Finding non-finite floats in a value, before it reaches the wire.
//!
//! `serde_json` writes `NaN` and `±inf` as `null` rather than failing, so a
//! component publishing one emits a payload that decodes nowhere. The failure
//! then surfaces at whichever consumer reads it next, which is a different
//! component, at a later instant, with nothing pointing back at the arithmetic
//! that produced it.
//!
//! Neither obvious hook can catch it. A custom [`serde_json::ser::Formatter`]
//! never sees the float: `serialize_f64` routes non-finite values to
//! `write_null` directly, so the formatter is handed the same call an
//! `Option::None` produces. `serde_json::to_value` collapses both to
//! `Value::Null` for the same reason. Distinguishing them means looking at the
//! value rather than at anything JSON made of it.
//!
//! So this walks the value with a [`Serializer`] that writes nothing. Every
//! method is a no-op except the two float ones, which is why the volume of code
//! below carries so little logic. It reports through the error channel, which
//! is what stops the walk at the first offender rather than visiting the rest
//! of a value already known to be bad.

use std::fmt::{self, Display};

use serde::{Serialize, Serializer, ser};

/// A non-finite float found in a value, and where it was.
#[derive(Debug, Clone, PartialEq)]
pub(crate) struct NonFinite {
    /// Dotted field path, such as `position.x`, or `waypoints[2].y` inside a
    /// sequence. Empty when the published value is itself a bare float.
    pub path: String,

    /// The offending value, kept for the message: `NaN`, `inf`, and `-inf`
    /// are three different mistakes and the fix differs.
    pub value: f64,
}

impl Display for NonFinite {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        if self.path.is_empty() {
            write!(f, "{}", self.value)
        } else {
            write!(f, "{} at {}", self.value, self.path)
        }
    }
}

/// The first non-finite float in `value`, or `None` if every float is finite.
///
/// Callers can skip this entirely when the serialized payload holds no `null`,
/// since a non-finite float always writes one. See [`crate::StepCtx::publish`].
pub(crate) fn find_non_finite<T: Serialize + ?Sized>(value: &T) -> Option<NonFinite> {
    let mut finder = Finder { path: Vec::new() };
    match value.serialize(&mut finder) {
        Err(Found::NonFinite(found)) => Some(found),
        // A value this walk cannot serialize is not this function's business.
        // `publish` has already serialized it for real by the time it asks, so
        // anything unserializable failed there with a better message.
        Ok(()) | Err(Found::Other(_)) => None,
    }
}

/// Where the walk currently is, as the path is built and unwound.
enum Segment {
    Field(&'static str),
    Index(usize),
}

struct Finder {
    path: Vec<Segment>,
}

impl Finder {
    /// The current position, rendered the way a reader would write it.
    fn path(&self) -> String {
        let mut out = String::new();
        for segment in &self.path {
            match segment {
                Segment::Field(name) => {
                    if !out.is_empty() {
                        out.push('.');
                    }
                    out.push_str(name);
                }
                Segment::Index(index) => {
                    out.push('[');
                    out.push_str(&index.to_string());
                    out.push(']');
                }
            }
        }
        out
    }

    /// Accepts a float, or reports it with the path it was found at.
    fn check(&self, value: f64) -> Result<(), Found> {
        if value.is_finite() {
            return Ok(());
        }

        // Return the first one found, which ends the walk.
        Err(Found::NonFinite(NonFinite {
            path: self.path(),
            value,
        }))
    }
}

/// The walk's error channel, which is how a find is reported.
#[derive(Debug)]
enum Found {
    NonFinite(NonFinite),

    /// Whatever `serde` reports through [`ser::Error::custom`]. Nothing here
    /// produces one, but the trait requires the constructor to exist.
    Other(String),
}

impl Display for Found {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Found::NonFinite(found) => write!(f, "non-finite float {found}"),
            Found::Other(message) => f.write_str(message),
        }
    }
}

impl std::error::Error for Found {}

impl ser::Error for Found {
    fn custom<T: Display>(message: T) -> Self {
        Found::Other(message.to_string())
    }
}

/// Every method writes nothing. The two float ones are the whole point; the
/// rest exist because [`Serializer`] requires them, and either recurse into
/// what might contain a float or accept a value that cannot.
impl Serializer for &mut Finder {
    type Ok = ();
    type Error = Found;
    type SerializeSeq = Self;
    type SerializeTuple = Self;
    type SerializeTupleStruct = Self;
    type SerializeTupleVariant = Self;
    type SerializeMap = Self;
    type SerializeStruct = Self;
    type SerializeStructVariant = Self;

    fn serialize_f64(self, value: f64) -> Result<(), Found> {
        self.check(value)
    }

    // Widened rather than checked as an `f32`, so one code path decides what
    // finite means. The widening is exact and cannot turn a finite value
    // non-finite or the reverse.
    fn serialize_f32(self, value: f32) -> Result<(), Found> {
        self.check(value as f64)
    }

    fn serialize_bool(self, _: bool) -> Result<(), Found> {
        Ok(())
    }
    fn serialize_i8(self, _: i8) -> Result<(), Found> {
        Ok(())
    }
    fn serialize_i16(self, _: i16) -> Result<(), Found> {
        Ok(())
    }
    fn serialize_i32(self, _: i32) -> Result<(), Found> {
        Ok(())
    }
    fn serialize_i64(self, _: i64) -> Result<(), Found> {
        Ok(())
    }
    fn serialize_u8(self, _: u8) -> Result<(), Found> {
        Ok(())
    }
    fn serialize_u16(self, _: u16) -> Result<(), Found> {
        Ok(())
    }
    fn serialize_u32(self, _: u32) -> Result<(), Found> {
        Ok(())
    }
    fn serialize_u64(self, _: u64) -> Result<(), Found> {
        Ok(())
    }
    fn serialize_char(self, _: char) -> Result<(), Found> {
        Ok(())
    }
    fn serialize_str(self, _: &str) -> Result<(), Found> {
        Ok(())
    }
    fn serialize_bytes(self, _: &[u8]) -> Result<(), Found> {
        Ok(())
    }
    fn serialize_none(self) -> Result<(), Found> {
        Ok(())
    }
    fn serialize_unit(self) -> Result<(), Found> {
        Ok(())
    }
    fn serialize_unit_struct(self, _: &'static str) -> Result<(), Found> {
        Ok(())
    }
    fn serialize_unit_variant(self, _: &'static str, _: u32, _: &'static str) -> Result<(), Found> {
        Ok(())
    }

    fn serialize_some<T>(self, value: &T) -> Result<(), Found>
    where
        T: Serialize + ?Sized,
    {
        value.serialize(self)
    }

    fn serialize_newtype_struct<T>(self, _: &'static str, value: &T) -> Result<(), Found>
    where
        T: Serialize + ?Sized,
    {
        value.serialize(self)
    }

    // A newtype variant contributes its name to the path, since two variants
    // of one enum are different places to have gone wrong.
    fn serialize_newtype_variant<T>(
        self,
        _: &'static str,
        _: u32,
        variant: &'static str,
        value: &T,
    ) -> Result<(), Found>
    where
        T: Serialize + ?Sized,
    {
        self.path.push(Segment::Field(variant));
        let result = value.serialize(&mut *self);
        self.path.pop();
        result
    }

    fn serialize_seq(self, _: Option<usize>) -> Result<Self, Found> {
        // Indices are pushed per element, so start with a slot to overwrite.
        self.path.push(Segment::Index(0));
        Ok(self)
    }
    fn serialize_tuple(self, len: usize) -> Result<Self, Found> {
        self.serialize_seq(Some(len))
    }
    fn serialize_tuple_struct(self, _: &'static str, len: usize) -> Result<Self, Found> {
        self.serialize_seq(Some(len))
    }
    fn serialize_tuple_variant(
        self,
        _: &'static str,
        _: u32,
        variant: &'static str,
        len: usize,
    ) -> Result<Self, Found> {
        self.path.push(Segment::Field(variant));
        self.serialize_seq(Some(len))
    }
    fn serialize_map(self, _: Option<usize>) -> Result<Self, Found> {
        Ok(self)
    }
    fn serialize_struct(self, _: &'static str, _: usize) -> Result<Self, Found> {
        Ok(self)
    }
    fn serialize_struct_variant(
        self,
        _: &'static str,
        _: u32,
        variant: &'static str,
        _: usize,
    ) -> Result<Self, Found> {
        self.path.push(Segment::Field(variant));
        Ok(self)
    }
}

/// Walks one element of a sequence, keeping the index in the path current.
fn serialize_indexed<T>(finder: &mut Finder, value: &T) -> Result<(), Found>
where
    T: Serialize + ?Sized,
{
    let result = value.serialize(&mut *finder);
    if let Some(Segment::Index(index)) = finder.path.last_mut() {
        *index += 1;
    }

    // Return the element's verdict, having advanced regardless.
    result
}

impl ser::SerializeSeq for &mut Finder {
    type Ok = ();
    type Error = Found;

    fn serialize_element<T>(&mut self, value: &T) -> Result<(), Found>
    where
        T: Serialize + ?Sized,
    {
        serialize_indexed(self, value)
    }

    fn end(self) -> Result<(), Found> {
        self.path.pop();
        Ok(())
    }
}

impl ser::SerializeTuple for &mut Finder {
    type Ok = ();
    type Error = Found;

    fn serialize_element<T>(&mut self, value: &T) -> Result<(), Found>
    where
        T: Serialize + ?Sized,
    {
        serialize_indexed(self, value)
    }

    fn end(self) -> Result<(), Found> {
        self.path.pop();
        Ok(())
    }
}

impl ser::SerializeTupleStruct for &mut Finder {
    type Ok = ();
    type Error = Found;

    fn serialize_field<T>(&mut self, value: &T) -> Result<(), Found>
    where
        T: Serialize + ?Sized,
    {
        serialize_indexed(self, value)
    }

    fn end(self) -> Result<(), Found> {
        self.path.pop();
        Ok(())
    }
}

impl ser::SerializeTupleVariant for &mut Finder {
    type Ok = ();
    type Error = Found;

    fn serialize_field<T>(&mut self, value: &T) -> Result<(), Found>
    where
        T: Serialize + ?Sized,
    {
        serialize_indexed(self, value)
    }

    fn end(self) -> Result<(), Found> {
        // Both the index and the variant name.
        self.path.pop();
        self.path.pop();
        Ok(())
    }
}

impl ser::SerializeMap for &mut Finder {
    type Ok = ();
    type Error = Found;

    // A key is not a place a float can hide in JSON, which turns every key
    // into a string, so only the value is walked.
    fn serialize_key<T>(&mut self, _: &T) -> Result<(), Found>
    where
        T: Serialize + ?Sized,
    {
        Ok(())
    }

    fn serialize_value<T>(&mut self, value: &T) -> Result<(), Found>
    where
        T: Serialize + ?Sized,
    {
        value.serialize(&mut **self)
    }

    fn end(self) -> Result<(), Found> {
        Ok(())
    }
}

impl ser::SerializeStruct for &mut Finder {
    type Ok = ();
    type Error = Found;

    fn serialize_field<T>(&mut self, name: &'static str, value: &T) -> Result<(), Found>
    where
        T: Serialize + ?Sized,
    {
        self.path.push(Segment::Field(name));
        let result = value.serialize(&mut **self);
        self.path.pop();
        result
    }

    fn end(self) -> Result<(), Found> {
        Ok(())
    }
}

impl ser::SerializeStructVariant for &mut Finder {
    type Ok = ();
    type Error = Found;

    fn serialize_field<T>(&mut self, name: &'static str, value: &T) -> Result<(), Found>
    where
        T: Serialize + ?Sized,
    {
        self.path.push(Segment::Field(name));
        let result = value.serialize(&mut **self);
        self.path.pop();
        result
    }

    fn end(self) -> Result<(), Found> {
        // The variant name pushed when the walk entered it.
        self.path.pop();
        Ok(())
    }
}

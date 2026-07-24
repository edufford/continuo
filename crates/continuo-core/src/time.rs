use std::fmt;
use std::ops::{Add, AddAssign, Mul, Neg, Sub, SubAssign};
use std::str::FromStr;

use serde::{Deserialize, Deserializer, Serialize, Serializer, de};

use crate::error::CoreError;

const NANOS_PER_SEC: i64 = 1_000_000_000;

/// Absolute simulation time: integer nanoseconds since the start of the run.
///
/// On the wire this is decimal seconds with at most 9 fractional digits,
/// formatted and parsed through pure integer math (never `f64`), so the
/// representation is exact and the serialized bytes are canonical
/// (trailing zeros trimmed, at least one fractional digit: `1.5`, `2.0`,
/// `0.033333333`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Default)]
pub struct SimTime(i64);

/// A span of simulation time: integer nanoseconds, same wire format as
/// [`SimTime`].
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Default)]
pub struct SimDuration(i64);

impl SimTime {
    pub const ZERO: SimTime = SimTime(0);

    pub const fn from_nanos(ns: i64) -> Self {
        SimTime(ns)
    }

    pub const fn from_micros(us: i64) -> Self {
        SimTime(us * 1_000)
    }

    pub const fn from_millis(ms: i64) -> Self {
        SimTime(ms * 1_000_000)
    }

    pub const fn from_secs(s: i64) -> Self {
        SimTime(s * NANOS_PER_SEC)
    }

    pub const fn as_nanos(self) -> i64 {
        self.0
    }

    /// Rounds to the nearest nanosecond. Only for values entering the system
    /// from float computations; scheduling comparisons stay integer.
    pub fn from_secs_f64(secs: f64) -> Self {
        SimTime((secs * NANOS_PER_SEC as f64).round() as i64)
    }

    pub fn as_secs_f64(self) -> f64 {
        self.0 as f64 / NANOS_PER_SEC as f64
    }

    /// Canonical decimal-seconds form used on the wire.
    pub fn to_canonical_string(self) -> String {
        format_ns(self.0)
    }
}

impl SimDuration {
    pub const ZERO: SimDuration = SimDuration(0);

    pub const fn from_nanos(ns: i64) -> Self {
        SimDuration(ns)
    }

    pub const fn from_micros(us: i64) -> Self {
        SimDuration(us * 1_000)
    }

    pub const fn from_millis(ms: i64) -> Self {
        SimDuration(ms * 1_000_000)
    }

    pub const fn from_secs(s: i64) -> Self {
        SimDuration(s * NANOS_PER_SEC)
    }

    pub const fn as_nanos(self) -> i64 {
        self.0
    }

    /// Rounds to the nearest nanosecond (see [`SimTime::from_secs_f64`]).
    pub fn from_secs_f64(secs: f64) -> Self {
        SimDuration((secs * NANOS_PER_SEC as f64).round() as i64)
    }

    pub fn as_secs_f64(self) -> f64 {
        self.0 as f64 / NANOS_PER_SEC as f64
    }

    pub fn to_canonical_string(self) -> String {
        format_ns(self.0)
    }
}

impl Add<SimDuration> for SimTime {
    type Output = SimTime;
    fn add(self, rhs: SimDuration) -> SimTime {
        SimTime(self.0 + rhs.0)
    }
}

impl AddAssign<SimDuration> for SimTime {
    fn add_assign(&mut self, rhs: SimDuration) {
        self.0 += rhs.0;
    }
}

impl Sub<SimDuration> for SimTime {
    type Output = SimTime;
    fn sub(self, rhs: SimDuration) -> SimTime {
        SimTime(self.0 - rhs.0)
    }
}

impl Sub for SimTime {
    type Output = SimDuration;
    fn sub(self, rhs: SimTime) -> SimDuration {
        SimDuration(self.0 - rhs.0)
    }
}

impl Add for SimDuration {
    type Output = SimDuration;
    fn add(self, rhs: SimDuration) -> SimDuration {
        SimDuration(self.0 + rhs.0)
    }
}

impl AddAssign for SimDuration {
    fn add_assign(&mut self, rhs: SimDuration) {
        self.0 += rhs.0;
    }
}

impl Sub for SimDuration {
    type Output = SimDuration;
    fn sub(self, rhs: SimDuration) -> SimDuration {
        SimDuration(self.0 - rhs.0)
    }
}

impl SubAssign for SimDuration {
    fn sub_assign(&mut self, rhs: SimDuration) {
        self.0 -= rhs.0;
    }
}

impl Mul<i64> for SimDuration {
    type Output = SimDuration;
    fn mul(self, rhs: i64) -> SimDuration {
        SimDuration(self.0 * rhs)
    }
}

impl Neg for SimDuration {
    type Output = SimDuration;
    fn neg(self) -> SimDuration {
        SimDuration(-self.0)
    }
}

impl fmt::Display for SimTime {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(&format_ns(self.0))
    }
}

impl fmt::Display for SimDuration {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(&format_ns(self.0))
    }
}

impl FromStr for SimTime {
    type Err = CoreError;
    fn from_str(s: &str) -> Result<Self, Self::Err> {
        parse_ns(s).map(SimTime)
    }
}

impl FromStr for SimDuration {
    type Err = CoreError;
    fn from_str(s: &str) -> Result<Self, Self::Err> {
        parse_ns(s).map(SimDuration)
    }
}

/// Formats integer nanoseconds as canonical decimal seconds: trailing zeros
/// trimmed, at least one fractional digit.
fn format_ns(ns: i64) -> String {
    let sign = if ns < 0 { "-" } else { "" };
    let abs = ns.unsigned_abs();
    let secs = abs / NANOS_PER_SEC as u64;
    let frac = abs % NANOS_PER_SEC as u64;
    let mut frac_str = format!("{frac:09}");
    while frac_str.len() > 1 && frac_str.ends_with('0') {
        frac_str.pop();
    }

    // Return the canonical decimal-seconds text, e.g. "1.5" or "0.033333333".
    format!("{sign}{secs}.{frac_str}")
}

/// Parses decimal seconds into integer nanoseconds via pure integer math.
///
/// Accepts `[-]digits[.digits]` with at most 9 fractional digits. Exponents
/// are rejected: they never appear in canonical output, and refusing them
/// keeps the parser trivially exact.
fn parse_ns(s: &str) -> Result<i64, CoreError> {
    let err = |reason: &str| CoreError::TimeParse {
        value: s.to_string(),
        reason: reason.to_string(),
    };

    let (negative, body) = match s.strip_prefix('-') {
        Some(rest) => (true, rest),
        None => (false, s),
    };

    let (int_part, frac_part) = match body.split_once('.') {
        Some((i, f)) => (i, f),
        None => (body, ""),
    };

    if int_part.is_empty() || !int_part.bytes().all(|b| b.is_ascii_digit()) {
        return Err(err("expected decimal digits before the point"));
    }
    if body.contains('.')
        && (frac_part.is_empty() || !frac_part.bytes().all(|b| b.is_ascii_digit()))
    {
        return Err(err("expected decimal digits after the point"));
    }
    if frac_part.len() > 9 {
        return Err(err(
            "more than 9 fractional digits (sub-nanosecond precision is not representable)",
        ));
    }

    let secs: i128 = int_part.parse().map_err(|_| err("integer part overflow"))?;
    let mut frac: i128 = if frac_part.is_empty() {
        0
    } else {
        frac_part
            .parse()
            .map_err(|_| err("fractional part overflow"))?
    };
    for _ in 0..(9 - frac_part.len()) {
        frac *= 10;
    }

    let magnitude = secs * NANOS_PER_SEC as i128 + frac;
    let signed = if negative { -magnitude } else { magnitude };

    // Return the parsed time as integer nanoseconds, or an overflow error.
    i64::try_from(signed).map_err(|_| err("out of range for 64-bit nanoseconds"))
}

fn serialize_ns<S: Serializer>(ns: i64, serializer: S) -> Result<S::Ok, S::Error> {
    // Route through serde_json::Number so serde_json (with the
    // arbitrary_precision feature) emits the canonical string as an exact
    // number token instead of an f64.
    let canonical = format_ns(ns);
    let number = serde_json::Number::from_str(&canonical)
        .map_err(|e| serde::ser::Error::custom(format!("canonical time not a JSON number: {e}")))?;

    // Return the exact decimal-seconds token, serialized as a JSON number.
    number.serialize(serializer)
}

fn deserialize_ns<'de, D: Deserializer<'de>>(deserializer: D) -> Result<i64, D::Error> {
    let number = serde_json::Number::deserialize(deserializer)?;

    // Return integer nanoseconds parsed from the number's source text (with
    // arbitrary_precision, to_string() is the exact token).
    parse_ns(&number.to_string()).map_err(de::Error::custom)
}

impl Serialize for SimTime {
    fn serialize<S: Serializer>(&self, serializer: S) -> Result<S::Ok, S::Error> {
        serialize_ns(self.0, serializer)
    }
}

impl<'de> Deserialize<'de> for SimTime {
    fn deserialize<D: Deserializer<'de>>(deserializer: D) -> Result<Self, D::Error> {
        deserialize_ns(deserializer).map(SimTime)
    }
}

impl Serialize for SimDuration {
    fn serialize<S: Serializer>(&self, serializer: S) -> Result<S::Ok, S::Error> {
        serialize_ns(self.0, serializer)
    }
}

impl<'de> Deserialize<'de> for SimDuration {
    fn deserialize<D: Deserializer<'de>>(deserializer: D) -> Result<Self, D::Error> {
        deserialize_ns(deserializer).map(SimDuration)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn canonical_formatting() {
        assert_eq!(SimTime::from_nanos(0).to_canonical_string(), "0.0");
        assert_eq!(
            SimTime::from_nanos(1_500_000_000).to_canonical_string(),
            "1.5"
        );
        assert_eq!(
            SimTime::from_nanos(2_000_000_000).to_canonical_string(),
            "2.0"
        );
        assert_eq!(
            SimTime::from_nanos(33_333_333).to_canonical_string(),
            "0.033333333"
        );
        assert_eq!(SimTime::from_nanos(1).to_canonical_string(), "0.000000001");
        assert_eq!(
            SimDuration::from_nanos(-1_500_000_000).to_canonical_string(),
            "-1.5"
        );
    }

    #[test]
    fn parse_round_trip() {
        for ns in [
            0i64,
            1,
            999_999_999,
            1_000_000_000,
            1_500_000_000,
            33_333_333,
            -42_000_000_001,
            i64::MAX,
            i64::MIN + 1,
        ] {
            let s = format_ns(ns);
            assert_eq!(parse_ns(&s).unwrap(), ns, "round-trip failed for {s}");
        }
    }

    #[test]
    fn parse_accepts_bare_integers() {
        assert_eq!(
            "2".parse::<SimTime>().unwrap(),
            SimTime::from_nanos(2_000_000_000)
        );
        assert_eq!(
            "-3".parse::<SimDuration>().unwrap(),
            SimDuration::from_secs(-3)
        );
    }

    #[test]
    fn parse_rejects_bad_input() {
        assert!(parse_ns("1.0000000001").is_err()); // 10 fractional digits
        assert!(parse_ns("1e9").is_err());
        assert!(parse_ns("1.").is_err());
        assert!(parse_ns(".5").is_err());
        assert!(parse_ns("").is_err());
        assert!(parse_ns("abc").is_err());
        assert!(parse_ns("99999999999999999999.0").is_err()); // overflow
    }

    #[test]
    fn serde_json_round_trip_is_exact_and_canonical() {
        let t = SimTime::from_nanos(1_234_567_891);
        let json = serde_json::to_string(&t).unwrap();
        assert_eq!(json, "1.234567891");
        let back: SimTime = serde_json::from_str(&json).unwrap();
        assert_eq!(back, t);

        let whole = SimTime::from_nanos(2_000_000_000);
        assert_eq!(serde_json::to_string(&whole).unwrap(), "2.0");
    }

    #[test]
    fn from_secs_f64_rounds_to_nearest_ns() {
        assert_eq!(
            SimTime::from_secs_f64(0.1),
            SimTime::from_nanos(100_000_000)
        );
        assert_eq!(
            SimDuration::from_secs_f64(1.0 / 30.0),
            SimDuration::from_nanos(33_333_333)
        );
    }

    #[test]
    fn arithmetic() {
        let t = SimTime::from_nanos(5_000_000_000);
        let d = SimDuration::from_millis(1500);
        assert_eq!(t + d, SimTime::from_nanos(6_500_000_000));
        assert_eq!(t - d, SimTime::from_nanos(3_500_000_000));
        assert_eq!(t + d - t, d);
        assert_eq!(d * 2, SimDuration::from_secs(3));
    }
}

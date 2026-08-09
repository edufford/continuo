//! Rejecting non-finite floats at the publisher.
//!
//! Two things are under test and they pull in opposite directions. A `NaN` or
//! an infinity must be refused wherever it is nested, because JSON writes it as
//! `null` and it would decode nowhere. And a payload that legitimately contains
//! `null`, which is what `Option::None` serializes to, must still publish,
//! because the cheap pre-check that decides whether to look closer keys on
//! exactly those four bytes.

use continuo_core::{CoreError, KeyExpr, Quat, SimTime, StepCtx, Vec3};
use serde::Serialize;

/// Publishes `value` through a real `StepCtx` and returns what came back.
///
/// Through the context rather than calling the walk directly, since what has
/// to hold is that a component cannot get a non-finite float onto the wire,
/// not merely that a function can spot one.
fn publish<T: Serialize>(value: &T) -> Result<(), CoreError> {
    let mut ctx = StepCtx::new(SimTime::ZERO, None, "demo", 0, Vec::new());
    ctx.publish(KeyExpr::new("test/payload").expect("valid key"), value)
}

/// The message a rejection carries, for asserting on what it names.
fn rejection(result: Result<(), CoreError>) -> String {
    match result {
        Err(error @ CoreError::NonFiniteFloat { .. }) => error.to_string(),
        other => panic!("expected a non-finite rejection, got {other:?}"),
    }
}

#[derive(Serialize)]
struct Nested {
    label: String,
    position: Vec3,
    orientation: Quat,
}

#[derive(Serialize)]
struct Optional {
    name: Option<String>,
    reading: Option<f64>,
    position: Vec3,
}

#[derive(Serialize)]
struct WithSequence {
    waypoints: Vec<Vec3>,
}

fn finite_vec3() -> Vec3 {
    Vec3 {
        x: 1.0,
        y: 2.0,
        z: 3.0,
    }
}

fn finite_quat() -> Quat {
    Quat {
        w: 1.0,
        x: 0.0,
        y: 0.0,
        z: 0.0,
    }
}

#[test]
fn every_kind_of_non_finite_is_refused() {
    // Three different mistakes with three different causes: a zero over zero,
    // an overflow, and its negative. All three are `null` on the wire, so all
    // three have to be caught here.
    for value in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
        let mut position = finite_vec3();
        position.y = value;
        let message = rejection(publish(&Nested {
            label: "ego".to_string(),
            position,
            orientation: finite_quat(),
        }));

        assert!(
            message.contains("position.y"),
            "the message has to name the field, since finding it is the point: {message}"
        );
        assert!(
            message.contains("test/payload"),
            "and the key, since a component publishes more than one: {message}"
        );
    }
}

#[test]
fn the_value_is_named_so_the_three_causes_are_told_apart() {
    // `NaN` from a zero over zero and an infinity from an overflow are
    // different bugs, and a message saying only "non-finite" would send a
    // reader to look for the wrong one.
    let cases = [
        (f64::NAN, "NaN"),
        (f64::INFINITY, "inf"),
        (f64::NEG_INFINITY, "-inf"),
    ];
    for (value, expected) in cases {
        let message = rejection(publish(&value));
        assert!(
            message.contains(expected),
            "expected {expected:?} in the message, got {message}"
        );
    }
}

#[test]
fn a_non_finite_inside_a_sequence_names_its_index() {
    // A path of `waypoints[1].z` is the difference between reading one number
    // and reading a list to find which one moved.
    let mut second = finite_vec3();
    second.z = f64::NAN;
    let message = rejection(publish(&WithSequence {
        waypoints: vec![finite_vec3(), second, finite_vec3()],
    }));

    assert!(
        message.contains("waypoints[1].z"),
        "the index has to survive into the path: {message}"
    );
}

#[test]
fn a_bare_float_is_refused_without_a_path_to_name() {
    // Nothing containing it, so there is no field to point at, and the message
    // must not invent one or read as though a field were missing.
    let message = rejection(publish(&f64::NAN));

    assert!(message.contains("NaN"), "{message}");
    assert!(
        !message.contains(" at "),
        "a bare value has nowhere to be, so the message should not say where: {message}"
    );
}

#[test]
fn an_f32_is_refused_as_readily_as_an_f64() {
    // The check widens to `f64`, so this is really asserting that the widening
    // happens rather than the `f32` slipping past unchecked.
    #[derive(Serialize)]
    struct Reading {
        temperature: f32,
    }

    let message = rejection(publish(&Reading {
        temperature: f32::NAN,
    }));
    assert!(message.contains("temperature"), "{message}");
}

#[test]
fn a_payload_whose_floats_are_finite_publishes() {
    publish(&Nested {
        label: "ego".to_string(),
        position: finite_vec3(),
        orientation: finite_quat(),
    })
    .expect("every float is finite");
}

#[test]
fn a_none_still_publishes_even_though_it_writes_null() {
    // The guard runs only when the payload holds `null`, and `None` writes one.
    // So this is the case that would break if the walk confused the two, which
    // is exactly what `serde_json` itself does: a formatter sees the same call
    // for both, which is why the walk looks at the value instead.
    publish(&Optional {
        name: None,
        reading: None,
        position: finite_vec3(),
    })
    .expect("a missing value is not a broken one");
}

#[test]
fn the_word_null_inside_a_string_still_publishes() {
    // The cheap pre-check scans bytes, so a string mentioning null trips it and
    // costs a walk. The walk finds nothing, which is the point: the fast path
    // is allowed to be wrong about *whether to look*, never about the answer.
    #[derive(Serialize)]
    struct Labelled {
        label: String,
        position: Vec3,
    }

    publish(&Labelled {
        label: "nullable sensor".to_string(),
        position: finite_vec3(),
    })
    .expect("a string is not a float");
}

#[test]
fn a_non_finite_hidden_behind_a_none_is_still_found() {
    // Both in one payload, so the walk has to pass over the `None` and keep
    // going rather than stopping at the first `null` it accounts for.
    let mut position = finite_vec3();
    position.x = f64::INFINITY;
    let message = rejection(publish(&Optional {
        name: None,
        reading: None,
        position,
    }));

    assert!(message.contains("position.x"), "{message}");
}

#[test]
fn a_non_finite_inside_a_some_is_found() {
    // `Some` is transparent on the wire, so it must be transparent to the walk.
    let message = rejection(publish(&Optional {
        name: Some("ego".to_string()),
        reading: Some(f64::NAN),
        position: finite_vec3(),
    }));

    assert!(message.contains("reading"), "{message}");
}

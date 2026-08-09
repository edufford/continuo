//! One tick's contents cannot be read two ways.
//!
//! The tick hash absorbs a run of fields with nothing between them, so where
//! two variable-length fields sit next to each other, moving a byte from the
//! end of one to the start of the next produces the identical byte stream.
//! Two different worlds then hash alike, and a divergence that only moves a
//! boundary goes unseen, which is the one thing a fingerprint must not do.
//!
//! A component contributes
//! `path | next_due | [key | seq | payload]* | state?`, and the pairs that
//! touch are: a payload and the following message's key, a payload and the
//! state after it, and one component's last field and the next component's
//! path. Only the middle one was ever separated, by a `b"|state|"` marker.
//!
//! Each test below builds two worlds whose bytes concatenate identically and
//! requires their hashes to differ.

use continuo_conductor::{Conductor, ConductorConfig, Pacing, WORLD_LEVEL};
use continuo_core::{Component, ComponentId, CoreError, KeyExpr, SimDuration, SimTime, StepCtx};
use continuo_transport::InProcTransport;

/// Publishes a fixed set of `(key, payload)` pairs once, then holds fixed
/// state, so a test can move a byte from one field into the next and change
/// nothing else.
struct ScriptedComponent {
    id: &'static str,
    published: Vec<(&'static str, &'static [u8])>,
    state: Option<&'static [u8]>,
    done: bool,
}

impl Component for ScriptedComponent {
    fn id(&self) -> ComponentId {
        ComponentId::new(self.id).expect("valid id")
    }

    fn subscriptions(&self) -> Vec<KeyExpr> {
        Vec::new()
    }

    fn step(&mut self, ctx: &mut StepCtx) -> Result<SimTime, CoreError> {
        if !self.done {
            self.done = true;
            for (key, payload) in &self.published {
                // Raw pre-serialized bytes, so a test controls the payload
                // down to the byte rather than through a shape `serde_json`
                // chooses. Still valid JSON, which is what a real payload is.
                let raw: Box<serde_json::value::RawValue> =
                    serde_json::from_slice(payload).expect("the test payload is valid JSON");
                ctx.publish(KeyExpr::new(*key).expect("valid key"), &raw)?;
            }
        }

        // Return the next due time, past the end of the runs below.
        Ok(ctx.now() + SimDuration::from_millis(10))
    }

    fn state_bytes(&self) -> Option<Vec<u8>> {
        self.state.map(<[u8]>::to_vec)
    }
}

/// The world hash after one tick of the components given, in the order given.
fn world_hash(components: Vec<ScriptedComponent>) -> u64 {
    let config = ConductorConfig {
        world_name: "boundary-test".into(),
        world_seed: 42,
        pacing: Pacing::FreeRun,
    };
    let mut conductor =
        Conductor::new(config, InProcTransport::new()).expect("free-run config is always accepted");
    for component in components {
        conductor
            .add_component(WORLD_LEVEL, Box::new(component))
            .expect("registration succeeds");
    }
    conductor.step_once().expect("one tick runs");

    // Return the fingerprint of that single tick.
    conductor.world_hash()
}

/// A component publishing one payload on one key, holding optional state.
fn one_message_component(
    id: &'static str,
    payload: &'static [u8],
    state: Option<&'static [u8]>,
) -> ScriptedComponent {
    ScriptedComponent {
        id,
        published: vec![("k", payload)],
        state,
        done: false,
    }
}

#[test]
fn a_payload_and_the_next_key_have_a_boundary() {
    // Nothing separated these. The second message's key follows the first
    // message's payload directly, so `12` then key `k` and `1` then key `2k`
    // absorb the same bytes.
    let wide_payload = ScriptedComponent {
        id: "a",
        published: vec![("first", b"12"), ("k", b"0")],
        state: None,
        done: false,
    };
    let wide_key = ScriptedComponent {
        id: "a",
        published: vec![("first", b"1"), ("2k", b"0")],
        state: None,
        done: false,
    };
    assert_ne!(world_hash(vec![wide_payload]), world_hash(vec![wide_key]));
}

#[test]
fn one_components_last_field_and_the_next_components_path_have_a_boundary() {
    // Nothing separated these either. The next component's path follows the
    // previous component's last field directly, so payload `12` then path `b`
    // and payload `1` then path `2b` absorb the same bytes. Registration order
    // is step order, so `a` goes first in both.
    assert_ne!(
        world_hash(vec![
            one_message_component("a", b"12", None),
            one_message_component("b", b"0", None)
        ]),
        world_hash(vec![
            one_message_component("a", b"1", None),
            one_message_component("2b", b"0", None)
        ]),
    );
}

#[test]
fn a_payload_and_the_state_after_it_have_a_boundary() {
    // The one boundary the old marker did separate. Kept because the property
    // has to hold whatever separates it, and the marker is gone.
    assert_ne!(
        world_hash(vec![one_message_component("a", b"12", Some(b"3"))]),
        world_hash(vec![one_message_component("a", b"1", Some(b"23"))]),
    );
}

#[test]
fn absent_state_is_not_empty_state() {
    // A length is written even for nothing, so `Some(&[])` contributes eight
    // zero bytes where `None` contributes none. A component that started
    // reporting empty state would otherwise be invisible.
    assert_ne!(
        world_hash(vec![one_message_component("a", b"1", None)]),
        world_hash(vec![one_message_component("a", b"1", Some(b""))]),
    );
}

#[test]
fn the_same_world_still_hashes_the_same() {
    // The control. The length prefixes are only worth anything if they left
    // determinism alone.
    assert_eq!(
        world_hash(vec![one_message_component("a", b"12", Some(b"3"))]),
        world_hash(vec![one_message_component("a", b"12", Some(b"3"))]),
    );
}

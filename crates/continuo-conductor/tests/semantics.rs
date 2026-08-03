//! Scheduling and visibility semantics of the conductor, exercised with
//! probe components that record what they observe.

use std::sync::{Arc, Mutex};

use continuo_conductor::{Conductor, ConductorConfig, ConductorError, Pacing, WORLD_LEVEL};
use continuo_core::{Component, ComponentId, KeyExpr, SimDuration, SimTime, StepCtx};
use continuo_transport::InProcTransport;

/// What one probe observed at one of its steps.
#[derive(Debug, Clone, PartialEq)]
struct Observation {
    now: SimTime,
    /// (publisher, seq, message time) for each released inbox message.
    inbox: Vec<(String, u64, SimTime)>,
}

type ObservationLog = Arc<Mutex<Vec<(String, Observation)>>>;

/// A probe: fixed period, optional publication each step, records its inbox.
struct Probe {
    id: &'static str,
    period: SimDuration,
    subscribe: Option<&'static str>,
    publish: Option<&'static str>,
    observations: ObservationLog,
}

impl Component for Probe {
    fn id(&self) -> ComponentId {
        ComponentId::new(self.id).unwrap()
    }

    fn subscriptions(&self) -> Vec<KeyExpr> {
        self.subscribe
            .map(|k| vec![KeyExpr::new(k).unwrap()])
            .unwrap_or_default()
    }

    fn step(&mut self, ctx: &mut StepCtx) -> SimTime {
        let observation = Observation {
            now: ctx.now(),
            inbox: ctx
                .inbox()
                .iter()
                .map(|m| (m.publisher.to_string(), m.seq, m.time))
                .collect(),
        };
        self.observations
            .lock()
            .unwrap()
            .push((self.id.to_string(), observation));
        if let Some(key) = self.publish {
            ctx.publish(KeyExpr::new(key).unwrap(), &ctx.now().to_canonical_string())
                .unwrap();
        }

        // Return the next due time, one period from now.
        ctx.now() + self.period
    }
}

/// A sim-time duration, in milliseconds.
fn dur_ms(n: i64) -> SimDuration {
    SimDuration::from_millis(n)
}

/// A sim-time instant, in milliseconds.
fn t_sim_ms(n: i64) -> SimTime {
    SimTime::from_millis(n)
}

fn new_conductor() -> Conductor<InProcTransport> {
    Conductor::new(
        ConductorConfig {
            world_name: "test".into(),
            world_seed: 0,
            pacing: Pacing::FreeRun,
        },
        InProcTransport::new(),
    )
    .unwrap()
}

#[test]
fn advances_to_earliest_due_and_orders_by_declaration() {
    let observations: ObservationLog = Default::default();
    let mut c = new_conductor();
    // Declared order: a then b, but different periods.
    c.add_component(
        WORLD_LEVEL,
        Box::new(Probe {
            id: "a",
            period: dur_ms(3),
            subscribe: None,
            publish: None,
            observations: observations.clone(),
        }),
    )
    .unwrap();
    c.add_component(
        WORLD_LEVEL,
        Box::new(Probe {
            id: "b",
            period: dur_ms(2),
            subscribe: None,
            publish: None,
            observations: observations.clone(),
        }),
    )
    .unwrap();

    c.run_until(t_sim_ms(6)).unwrap();

    let steps: Vec<(String, SimTime)> = observations
        .lock()
        .unwrap()
        .iter()
        .map(|(id, o)| (id.clone(), o.now))
        .collect();
    assert_eq!(
        steps,
        vec![
            ("a".into(), t_sim_ms(0)), // both due at 0: declaration order
            ("b".into(), t_sim_ms(0)),
            ("b".into(), t_sim_ms(2)),
            ("a".into(), t_sim_ms(3)),
            ("b".into(), t_sim_ms(4)),
            ("a".into(), t_sim_ms(6)), // both due at 6: declaration order again
            ("b".into(), t_sim_ms(6)),
        ]
    );
}

#[test]
fn strict_advance_guard_rejects_non_advancing_next_due() {
    let observations: ObservationLog = Default::default();
    let mut c = new_conductor();
    c.add_component(
        WORLD_LEVEL,
        Box::new(Probe {
            id: "stuck",
            period: dur_ms(0),
            subscribe: None,
            publish: None,
            observations,
        }),
    )
    .unwrap();

    let err = c.run_until(t_sim_ms(1)).unwrap_err();
    assert!(
        matches!(err, ConductorError::ScheduleViolation { .. }),
        "got {err:?}"
    );
}

#[test]
fn same_instant_delivery_to_later_sibling_within_composite() {
    let observations: ObservationLog = Default::default();
    let mut c = new_conductor();
    c.add_component(
        "actor",
        Box::new(Probe {
            id: "producer",
            period: dur_ms(10),
            subscribe: None,
            publish: Some("test/data"),
            observations: observations.clone(),
        }),
    )
    .unwrap();
    c.add_component(
        "actor",
        Box::new(Probe {
            id: "consumer",
            period: dur_ms(10),
            subscribe: Some("test/data"),
            publish: None,
            observations: observations.clone(),
        }),
    )
    .unwrap();

    c.run_until(t_sim_ms(10)).unwrap();

    let observations = observations.lock().unwrap();
    let consumer: Vec<&Observation> = observations
        .iter()
        .filter(|(id, _)| id == "consumer")
        .map(|(_, o)| o)
        .collect();
    // t=0: the producer's t=0 message arrives same-instant (earlier sibling).
    assert_eq!(consumer[0].now, t_sim_ms(0));
    assert_eq!(
        consumer[0].inbox,
        vec![("actor/producer".to_string(), 0, t_sim_ms(0))]
    );
    // t=10ms: only the t=10ms message (t=0 was already delivered).
    assert_eq!(
        consumer[1].inbox,
        vec![("actor/producer".to_string(), 1, t_sim_ms(10))]
    );
}

#[test]
fn cross_actor_same_instant_is_deferred_to_next_step() {
    let observations: ObservationLog = Default::default();
    let mut c = new_conductor();
    // Two world-level actors, co-scheduled every 10 ms.
    c.add_component(
        WORLD_LEVEL,
        Box::new(Probe {
            id: "producer",
            period: dur_ms(10),
            subscribe: None,
            publish: Some("test/data"),
            observations: observations.clone(),
        }),
    )
    .unwrap();
    c.add_component(
        WORLD_LEVEL,
        Box::new(Probe {
            id: "consumer",
            period: dur_ms(10),
            subscribe: Some("test/data"),
            publish: None,
            observations: observations.clone(),
        }),
    )
    .unwrap();

    c.run_until(t_sim_ms(20)).unwrap();

    let observations = observations.lock().unwrap();
    let consumer: Vec<&Observation> = observations
        .iter()
        .filter(|(id, _)| id == "consumer")
        .map(|(_, o)| o)
        .collect();
    // t=0: nothing, since the producer's t=0 message is same-instant and
    // cross-actor.
    assert_eq!(consumer[0].inbox, vec![]);
    // t=10ms: exactly the t=0 message; the t=10ms one is again deferred.
    assert_eq!(
        consumer[1].inbox,
        vec![("producer".to_string(), 0, t_sim_ms(0))]
    );
    // t=20ms: the t=10ms message.
    assert_eq!(
        consumer[2].inbox,
        vec![("producer".to_string(), 1, t_sim_ms(10))]
    );
}

#[test]
fn slow_consumer_receives_accumulated_messages_in_order() {
    let observations: ObservationLog = Default::default();
    let mut c = new_conductor();
    c.add_component(
        WORLD_LEVEL,
        Box::new(Probe {
            id: "fast",
            period: dur_ms(10),
            subscribe: None,
            publish: Some("test/data"),
            observations: observations.clone(),
        }),
    )
    .unwrap();
    c.add_component(
        WORLD_LEVEL,
        Box::new(Probe {
            id: "slow",
            period: dur_ms(35),
            subscribe: Some("test/data"),
            publish: None,
            observations: observations.clone(),
        }),
    )
    .unwrap();

    c.run_until(t_sim_ms(70)).unwrap();

    let observations = observations.lock().unwrap();
    let slow: Vec<&Observation> = observations
        .iter()
        .filter(|(id, _)| id == "slow")
        .map(|(_, o)| o)
        .collect();
    // Steps at 0, 35, 70. At t=35: messages from 0,10,20,30 (all < 35).
    let times: Vec<SimTime> = slow[1].inbox.iter().map(|(_, _, t)| *t).collect();
    assert_eq!(
        times,
        vec![t_sim_ms(0), t_sim_ms(10), t_sim_ms(20), t_sim_ms(30)]
    );
    // At t=70: messages from 40,50,60 (t=70 deferred, cross-actor).
    let times: Vec<SimTime> = slow[2].inbox.iter().map(|(_, _, t)| *t).collect();
    assert_eq!(times, vec![t_sim_ms(40), t_sim_ms(50), t_sim_ms(60)]);
}

#[test]
fn real_time_pacing_is_accepted() {
    // Pacing is a construction-time mode switch now, not an error path; its
    // timing behavior is covered in tests/pacing.rs.
    let conductor = Conductor::new(
        ConductorConfig {
            world_name: "test".into(),
            world_seed: 0,
            pacing: Pacing::real_time(),
        },
        InProcTransport::new(),
    )
    .expect("real-time pacing is supported");
    assert_eq!(conductor.overrun_reanchor_count(), 0);
}

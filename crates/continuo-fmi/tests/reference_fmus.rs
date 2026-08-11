//! The importer against somebody else's FMUs.
//!
//! Driven through `Component::step` directly rather than through a conductor,
//! since what is under test is the adapter's own lifecycle: which FMI calls
//! happen on which step, and what comes back out. The conductor has its own
//! suites for scheduling and delivery.

use continuo_core::{Component, ComponentPath, KeyExpr, Message, SimDuration, SimTime, StepCtx};
use continuo_fmi::{
    FmuComponent, FmuMapping, InputBinding, OutputBinding, fixture_path,
    json_pointers_for_dimensions,
};
use serde_json::{Value, json};

const WORLD: &str = "continuo/test";

fn key(name: &str) -> KeyExpr {
    KeyExpr::new(format!("{WORLD}/{name}")).unwrap()
}

fn empty_mapping(period_ms: u64) -> FmuMapping {
    FmuMapping {
        period: SimDuration::from_millis(period_ms as i64),
        inputs: Vec::new(),
        outputs: Vec::new(),
        initial_values: Vec::new(),
    }
}

/// Routes `fmi`'s `log` output into `tracing`, so an FMU's own diagnostics
/// are visible under `--nocapture`. Without it, a model refusing a call says
/// only "Error", and the reason it printed goes nowhere.
fn show_fmu_logs() {
    tracing_subscriber::fmt()
        .with_max_level(tracing::Level::DEBUG)
        .with_test_writer()
        .try_init()
        .ok();
}

/// Steps a component once and returns what it published, keyed by key.
fn step(component: &mut FmuComponent, now: SimTime, inbox: Vec<Message>) -> Vec<(String, Value)> {
    show_fmu_logs();
    let mut ctx = StepCtx::new(now, None, WORLD, 0, inbox);
    component.step(&mut ctx).expect("step");

    // Return each published payload decoded, since what matters is the value
    // rather than the bytes.
    ctx.take_outbox()
        .into_iter()
        .map(|(key, payload)| {
            (
                key.as_str().to_string(),
                serde_json::from_slice(&payload).expect("published payload decodes"),
            )
        })
        .collect()
}

/// One message on `key` carrying `payload`, as a publisher would send it.
fn message(key_name: &str, payload: Value) -> Message {
    Message {
        key: key(key_name),
        publisher: ComponentPath::parse("other").unwrap(),
        seq: 0,
        sim_time: SimTime::ZERO,
        payload: serde_json::to_vec(&payload).unwrap(),
    }
}

#[test]
fn a_reference_fmu_loads_instantiates_and_steps() {
    let mut mapping = empty_mapping(10);
    mapping.outputs = vec![OutputBinding::new("h", key("ball"))];
    let mut ball = FmuComponent::new("ball", fixture_path("BouncingBall"), mapping).expect("build");

    // The first step initializes and publishes what the FMU starts holding,
    // which for a ball is the height it was dropped from.
    let published = step(&mut ball, SimTime::ZERO, Vec::new());
    let start = published[0].1["h"].as_f64().unwrap();
    assert_eq!(published[0].0, "continuo/test/ball");
    assert!(start > 0.9, "starts near 1 m: {start}");

    // Then it falls, which is the whole of what this fixture does.
    let mut previous = start;
    for tick in 1..40 {
        let now = SimTime::from_millis(tick * 10);
        let height = step(&mut ball, now, Vec::new())[0].1["h"].as_f64().unwrap();
        assert!(height < previous, "still falling at {now}: {height}");
        previous = height;
    }
    assert!(previous < 0.3, "fell most of the way: {previous}");
}

#[test]
fn inputs_reach_the_fmu_and_outputs_carry_them_back() {
    let mut mapping = empty_mapping(100);
    mapping.inputs =
        vec![InputBinding::new("Float64_continuous_input", key("in")).with_pointers(["/value"])];
    mapping.outputs = vec![OutputBinding::new("Float64_continuous_output", key("out"))];
    let mut fmu =
        FmuComponent::new("feedthrough", fixture_path("Feedthrough"), mapping).expect("build");

    step(
        &mut fmu,
        SimTime::ZERO,
        vec![message("in", json!({"value": 4.5}))],
    );
    let published = step(
        &mut fmu,
        SimTime::from_millis(100),
        vec![message("in", json!({"value": 7.25}))],
    );

    assert_eq!(published.len(), 1);
    assert_eq!(published[0].0, "continuo/test/out");
    assert_eq!(published[0].1["Float64_continuous_output"], json!(7.25));
}

#[test]
fn an_input_set_during_initialization_survives_to_the_first_output() {
    // One half of the question the plan left open: whether a value set
    // between EnterInitializationMode and ExitInitializationMode is still
    // there afterwards. Feedthrough passes its input straight to its output,
    // so the first published value answers it.
    let mut mapping = empty_mapping(100);
    mapping.inputs =
        vec![InputBinding::new("Float64_continuous_input", key("in")).with_pointers(["/value"])];
    mapping.outputs = vec![OutputBinding::new("Float64_continuous_output", key("out"))];
    let mut fmu =
        FmuComponent::new("feedthrough", fixture_path("Feedthrough"), mapping).expect("build");

    let published = step(
        &mut fmu,
        SimTime::ZERO,
        vec![message("in", json!({"value": 3.5}))],
    );
    assert_eq!(
        published[0].1["Float64_continuous_output"],
        json!(3.5),
        "a value set during initialization reached the first output"
    );
}

#[test]
fn outputs_route_to_their_declared_keys() {
    // Two outputs on two keys give two messages; two on one key give one
    // payload carrying both, which is what lets an FMU publish a pose.
    let mut mapping = empty_mapping(100);
    mapping.outputs = vec![
        OutputBinding::new("Float64_continuous_output", key("a")),
        OutputBinding::new("Float64_discrete_output", key("b")),
    ];
    let mut fmu =
        FmuComponent::new("feedthrough", fixture_path("Feedthrough"), mapping).expect("build");
    let published = step(&mut fmu, SimTime::ZERO, Vec::new());
    assert_eq!(published.len(), 2);
    assert_eq!(published[0].0, "continuo/test/a");
    assert_eq!(published[1].0, "continuo/test/b");

    let mut mapping = empty_mapping(100);
    mapping.outputs = vec![
        OutputBinding::new("Float64_continuous_output", key("both")).with_pointer("/position/x"),
        OutputBinding::new("Float64_discrete_output", key("both")).with_pointer("/position/y"),
    ];
    let mut fmu =
        FmuComponent::new("feedthrough", fixture_path("Feedthrough"), mapping).expect("build");
    let published = step(&mut fmu, SimTime::ZERO, Vec::new());
    assert_eq!(published.len(), 1, "one key, one message");
    assert!(published[0].1["position"]["x"].is_number());
    assert!(published[0].1["position"]["y"].is_number());
}

#[test]
fn an_unknown_variable_name_fails_at_construction_naming_the_alternatives() {
    let mut mapping = empty_mapping(100);
    mapping.outputs = vec![OutputBinding::new("no_such_variable", key("out"))];

    let reason = match FmuComponent::new("feedthrough", fixture_path("Feedthrough"), mapping) {
        Err(error) => error.to_string(),
        Ok(_) => panic!("an unknown variable name should not build"),
    };
    assert!(reason.contains("no_such_variable"), "{reason}");
    assert!(reason.contains("Float64_continuous_output"), "{reason}");
}

#[test]
fn a_period_that_is_not_a_multiple_of_the_fixed_internal_step_size_fails() {
    // Feedthrough declares a 0.1 s internal step, so 100 ms lands and 30 ms
    // does not: an FMU stepping internally at its own size would answer from
    // an instant other than the one asked for.
    assert!(FmuComponent::new("ok", fixture_path("Feedthrough"), empty_mapping(100)).is_ok());
    assert!(FmuComponent::new("ok", fixture_path("Feedthrough"), empty_mapping(200)).is_ok());

    let reason = match FmuComponent::new("bad", fixture_path("Feedthrough"), empty_mapping(30)) {
        Err(error) => error.to_string(),
        Ok(_) => panic!("a period off the internal grid should not build"),
    };
    assert!(reason.contains("0.03"), "{reason}");
    assert!(reason.contains("0.1"), "{reason}");
}

#[test]
#[ignore = "fmi 0.8.0 hands over a resource path with no trailing separator"]
fn an_fmu_reads_its_own_resource_files() {
    // The only thing that checks the resource path handed to
    // fmi3InstantiateCoSimulation. FMI 3.0 changed that argument from 2.0's
    // file URI to a plain path, and this FMU reads `resources/y.txt` during
    // initialization, so `y` is simply wrong if the path is.
    //
    // Ignored because it fails, and it fails upstream rather than here. FMI
    // 3.0 requires that path to carry a trailing separator, and
    // `Fmi3Import::canonical_resource_path_string` does not append one, so
    // the FMU builds a path ending in `resourcesy.txt` and cannot open it. The crate's own
    // doc comment states the requirement the code then misses, and `main`
    // reads the same way as the released 0.8.0.
    //
    // Upstream, where the doc comment states the requirement and the six
    // lines under it miss it:
    // <https://github.com/jondo2010/rust-fmi/blob/v0.8.0/fmi/src/fmi3/import.rs#L66-L76>
    //
    // Kept rather than deleted: it is the only test that would notice, and
    // running it is how we will know the fix landed.
    let mut mapping = empty_mapping(1000);
    mapping.outputs = vec![OutputBinding::new("y", key("resource"))];
    let mut fmu = FmuComponent::new("resource", fixture_path("Resource"), mapping).expect("build");

    let published = step(&mut fmu, SimTime::ZERO, Vec::new());
    assert_eq!(published[0].1["y"], json!(1), "read from resources/y.txt");
}

#[test]
fn the_extraction_directory_is_reachable_and_holds_the_fmu() {
    // `tempfile` throws away its cleanup errors on drop, so a Windows delete
    // that fails because the library is still loaded leaves a directory
    // behind in silence. Knowing the path is what makes that diagnosable.
    let fmu =
        FmuComponent::new("ball", fixture_path("BouncingBall"), empty_mapping(10)).expect("build");
    let extracted = fmu.extracted_path().to_path_buf();
    assert!(
        extracted.join("modelDescription.xml").is_file(),
        "{extracted:?}"
    );

    drop(fmu);
    assert!(
        !extracted.exists(),
        "the instance drops before the directory it was loaded from: {extracted:?}"
    );
}

#[test]
fn a_missing_message_holds_the_previous_value() {
    // Sample and hold: a step with nothing on a bound key does not write that
    // variable, so the FMU keeps what it had rather than reading a default.
    let mut mapping = empty_mapping(100);
    mapping.inputs =
        vec![InputBinding::new("Float64_continuous_input", key("in")).with_pointers(["/value"])];
    mapping.outputs = vec![OutputBinding::new("Float64_continuous_output", key("out"))];
    let mut fmu =
        FmuComponent::new("feedthrough", fixture_path("Feedthrough"), mapping).expect("build");

    step(
        &mut fmu,
        SimTime::ZERO,
        vec![message("in", json!({"value": 9.0}))],
    );
    let published = step(&mut fmu, SimTime::from_millis(100), Vec::new());
    assert_eq!(published[0].1["Float64_continuous_output"], json!(9.0));
}

#[test]
fn only_the_newest_message_on_a_key_is_applied() {
    // FMI's own input semantics: an FMU sees the value at the step boundary
    // rather than every intermediate one, so the older message is not applied
    // and overwritten, it is never read at all.
    let mut mapping = empty_mapping(100);
    mapping.inputs =
        vec![InputBinding::new("Float64_continuous_input", key("in")).with_pointers(["/value"])];
    mapping.outputs = vec![OutputBinding::new("Float64_continuous_output", key("out"))];
    let mut fmu =
        FmuComponent::new("feedthrough", fixture_path("Feedthrough"), mapping).expect("build");

    let published = step(
        &mut fmu,
        SimTime::ZERO,
        vec![
            message("in", json!({"value": 1.0})),
            message("in", json!({"value": 2.0})),
        ],
    );
    assert_eq!(published[0].1["Float64_continuous_output"], json!(2.0));
}

#[test]
fn two_identical_fmu_runs_publish_identically() {
    let run = || {
        let mut mapping = empty_mapping(10);
        mapping.outputs = vec![OutputBinding::new("h", key("ball"))];
        let mut ball =
            FmuComponent::new("ball", fixture_path("BouncingBall"), mapping).expect("build");

        (0..50)
            .map(|tick| {
                let now = SimTime::from_millis(tick * 10);
                step(&mut ball, now, Vec::new())[0].1.to_string()
            })
            .collect::<Vec<_>>()
    };

    assert_eq!(run(), run());
}

#[test]
fn an_fmu_handles_its_own_events_when_event_mode_is_off() {
    // Instantiated with `event_mode_used = false`, so an FMU must deal with
    // its own events inside a step rather than asking to be taken into event
    // mode. A bouncing ball is the case with an event in it: the bounce is a
    // state event at a time nothing predicted, and the height has to come
    // back up without the adapter doing anything about it.
    let mut mapping = empty_mapping(10);
    mapping.outputs = vec![OutputBinding::new("h", key("ball"))];
    let mut ball = FmuComponent::new("ball", fixture_path("BouncingBall"), mapping).expect("build");

    let heights: Vec<f64> = (0..100)
        .map(|tick| {
            let now = SimTime::from_millis(tick * 10);
            step(&mut ball, now, Vec::new())[0].1["h"].as_f64().unwrap()
        })
        .collect();

    let lowest = heights.iter().cloned().fold(f64::MAX, f64::min);
    let after = heights.iter().skip_while(|h| **h > lowest).cloned();
    assert!(
        after.clone().any(|h| h > lowest + 0.01),
        "bounced without the adapter entering event mode: lowest {lowest}"
    );
}

/// StateSpace computes `y = C x + D u` over matrices sized by the structural
/// parameters `m`, `n` and `r`, all declared with a start of 3.
///
/// These tests drive the `D u` half, the direct feedthrough, because it shows
/// on the first step and needs no state. The `C x` half is unreachable here,
/// and not for want of trying: this fixture's state cannot be set through
/// co-simulation at all.
///
/// Its `x0` is inert. The model's `setStartValues` copies `x = x0` and then
/// assigns `x = 0` on the next line, and both have been there since the
/// commit that added the model. The copy could never have worked anyway,
/// since `setStartValues` runs only at instantiate and reset, while an
/// importer writes parameters later, in Initialization Mode. Setting `x`
/// directly is refused too: the model allows it only in Continuous Time Mode
/// or Event Mode, which an importer with event mode off never enters.
///
/// Every matrix is set explicitly. FMI requires an array to be written again
/// after a structural parameter changes its size, and these all change size
/// together.
fn state_space(sizes: &[(&str, u64)], d: Value) -> FmuMapping {
    let width = sizes
        .iter()
        .find(|(name, _)| *name == "n")
        .map_or(3, |(_, size)| *size as usize);
    let zeros = json!(vec![vec![0.0; width]; width]);

    let mut mapping = empty_mapping(1000);
    mapping.initial_values = sizes
        .iter()
        .map(|(name, size)| ((*name).to_string(), json!(size)))
        .chain([
            ("A".to_string(), zeros.clone()),
            ("B".to_string(), zeros.clone()),
            ("C".to_string(), zeros),
            ("D".to_string(), d),
        ])
        .collect();
    mapping.outputs = vec![OutputBinding::new("y", key("y"))];
    mapping
}

/// An identity matrix of `size`, written as nested arrays.
fn identity(size: usize) -> Value {
    json!(
        (0..size)
            .map(|row| (0..size)
                .map(|column| if row == column { 1.0 } else { 0.0 })
                .collect::<Vec<_>>())
            .collect::<Vec<_>>()
    )
}

#[test]
fn an_array_sized_by_a_structural_parameter_binds_at_its_declared_start() {
    // No size set, so they are the XML's own: m = n = r = 3, and `u` takes
    // three values.
    let mut mapping = state_space(&[], identity(3));
    mapping.inputs = vec![
        InputBinding::new("u", key("u")).with_pointers(json_pointers_for_dimensions("/u", &[3])),
    ];
    let mut fmu = FmuComponent::new("ss", fixture_path("StateSpace"), mapping).expect("build");

    let published = step(
        &mut fmu,
        SimTime::ZERO,
        vec![message("u", json!({"u": [1.0, 2.0, 3.0]}))],
    );
    assert_eq!(
        published[0].1["y"],
        json!([1.0, 2.0, 3.0]),
        "with D the identity, y is u, three values wide"
    );
}

#[test]
fn a_structural_parameter_set_at_construction_resizes_its_arrays() {
    // Two rather than three, which is only possible if configuration mode ran
    // and the dimensions were resolved from the mapping rather than from the
    // XML. Binding `u` with two pointers would fail construction otherwise,
    // and `y` would come back three wide.
    let mut mapping = state_space(&[("m", 2), ("n", 2), ("r", 2)], identity(2));
    mapping.inputs = vec![
        InputBinding::new("u", key("u")).with_pointers(json_pointers_for_dimensions("/u", &[2])),
    ];
    let mut fmu = FmuComponent::new("ss", fixture_path("StateSpace"), mapping).expect("build");

    let published = step(
        &mut fmu,
        SimTime::ZERO,
        vec![message("u", json!({"u": [4.0, 5.0]}))],
    );
    assert_eq!(
        published[0].1["y"],
        json!([4.0, 5.0]),
        "every array resized together with the parameter that sizes them"
    );
}

#[test]
fn an_array_binding_whose_pointer_count_misses_the_dimension_fails_at_construction() {
    // The check that stops a rebuilt FMU and a stale mapping from drifting
    // apart in silence, where the FMU would otherwise read whatever the tail
    // of the buffer held.
    let mut mapping = state_space(&[], identity(3));
    mapping.inputs = vec![
        InputBinding::new("u", key("u")).with_pointers(json_pointers_for_dimensions("/u", &[2])),
    ];

    let reason = match FmuComponent::new("ss", fixture_path("StateSpace"), mapping) {
        Err(error) => error.to_string(),
        Ok(_) => panic!("two pointers should not bind a three-value variable"),
    };
    assert!(reason.contains("\"u\""), "{reason}");
    assert!(reason.contains('3'), "{reason}");
    assert!(reason.contains('2'), "{reason}");
}

#[test]
fn a_matrix_binds_row_major() {
    // A transposed matrix still runs and still publishes numbers, so the
    // order is pinned by a value rather than left to a comment. `D` is
    // asymmetric here, and `y = D u` reads it out directly.
    let run = |d: Value| {
        let mut mapping = state_space(&[("m", 2), ("n", 2), ("r", 2)], d);
        mapping.inputs = vec![
            InputBinding::new("u", key("u"))
                .with_pointers(json_pointers_for_dimensions("/u", &[2])),
        ];
        let mut fmu = FmuComponent::new("ss", fixture_path("StateSpace"), mapping).expect("build");
        step(
            &mut fmu,
            SimTime::ZERO,
            vec![message("u", json!({"u": [3.0, 7.0]}))],
        )[0]
        .1["y"]
            .clone()
    };

    // Row-major means the outer array is the first index, so this D takes the
    // second element of u into the first element of y.
    assert_eq!(run(json!([[0.0, 1.0], [0.0, 0.0]])), json!([7.0, 0.0]));
    assert_eq!(run(json!([[0.0, 0.0], [1.0, 0.0]])), json!([0.0, 3.0]));

    // The same matrix written flat, which is how the standard spells one in a
    // start attribute, has to mean the same thing.
    assert_eq!(run(json!([0.0, 1.0, 0.0, 0.0])), json!([7.0, 0.0]));
}

#[test]
fn an_array_output_publishes_as_an_array() {
    // The shape an output takes is the FMU's own, so a vector publishes as a
    // JSON array at the pointer its name derives rather than as N messages.
    let mapping = state_space(&[], identity(3));
    let mut fmu = FmuComponent::new("ss", fixture_path("StateSpace"), mapping).expect("build");

    let published = step(&mut fmu, SimTime::ZERO, Vec::new());
    assert!(published[0].1["y"].is_array(), "{}", published[0].1);
    assert_eq!(published[0].1["y"].as_array().unwrap().len(), 3);
}

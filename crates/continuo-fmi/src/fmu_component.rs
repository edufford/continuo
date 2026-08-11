use std::collections::BTreeMap;
use std::ffi::CString;
use std::path::{Path, PathBuf};

use continuo_core::{
    Component, ComponentId, CoreError, KeyExpr, Message, SimDuration, SimTime, StepCtx,
};
use fmi::fmi3::instance::InstanceCS;
use fmi::fmi3::schema::{
    AbstractVariableTrait, ArrayableVariableTrait, Causality, Dimension, Fmi3ModelDescription,
    InitializableVariableTrait, Variable, VariableType,
};
use fmi::fmi3::{CoSimulation, Common, Fmi3Model, GetSet, import::Fmi3Import};
use fmi::traits::FmiImport;
use serde_json::Value;

use crate::convert;
use crate::error::FmuConstructionError;
use crate::mapping::{FmuMapping, InputBinding, OutputBinding, unescape_json_pointer_token};

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

/// An imported FMI 3.0 Co-Simulation FMU, running as a component.
///
/// One concrete type for every FMU there will ever be, because an FMU is data
/// here: a `.fmu` path plus an [`FmuMapping`]. Anything else would mean
/// recompiling the host to add a model, which is the thing a standard that
/// ships models as binaries exists to avoid.
pub struct FmuComponent {
    id: ComponentId,
    /// Declared before `import`, which is load-bearing rather than stylistic.
    /// Fields drop in declaration order, the instance holds the loaded shared
    /// library, and the import owns the temporary directory that library came
    /// out of. Windows refuses to delete a directory holding a loaded DLL, so
    /// the other order leaves one behind per instance per run.
    instance: InstanceCS,
    import: Fmi3Import,
    inputs: Vec<BoundInput>,
    outputs: Vec<BoundOutput>,
    subscriptions: Vec<KeyExpr>,
    period: SimDuration,
    /// Variables the mapping gives a starting value. Held until the first
    /// step, which is where Initialization Mode happens.
    vars_to_initialize: Vec<(ResolvedVariable, Value)>,
    /// When this component last stepped, and `None` until it has. The FMU's
    /// own clock: `fmi3DoStep` is told the point it steps from, so the
    /// adapter has to remember where it left the model.
    last_step: Option<SimTime>,
}

/// An input binding with its variable resolved against the FMU.
struct BoundInput {
    binding: InputBinding,
    variable: ResolvedVariable,
}

/// An output binding with its variable resolved against the FMU.
struct BoundOutput {
    binding: OutputBinding,
    variable: ResolvedVariable,
}

/// What a variable's name resolves to once the FMU has been read.
struct ResolvedVariable {
    name: String,
    value_reference: u32,
    declared_type: VariableType,
    /// The size of each dimension, outermost first, and empty for a scalar.
    /// Resolved rather than read: a dimension may name a structural parameter
    /// instead of stating a number, so these are the sizes this instance
    /// actually has rather than the ones the XML declares.
    dimensions: Vec<usize>,
}

impl ResolvedVariable {
    /// How many values this variable holds: the product of its dimensions,
    /// and one for a scalar.
    fn len(&self) -> usize {
        self.dimensions.iter().product::<usize>().max(1)
    }
}

impl FmuComponent {
    /// Imports a `.fmu` and instantiates it for co-simulation.
    ///
    /// Everything the mapping asserts about the FMU is checked here rather
    /// than at the first step: that each named variable exists, and that the
    /// period lands on the FMU's own step size. A mapping is data written
    /// beside a binary built elsewhere, so no compiler ever compares the two,
    /// and construction is the last moment before a run is under way.
    pub fn new(
        id: &str,
        fmu_path: impl AsRef<Path>,
        mapping: FmuMapping,
    ) -> Result<Self, FmuConstructionError> {
        let fmu_path = fmu_path.as_ref();
        let id = ComponentId::new(id)?;

        let import: Fmi3Import =
            fmi::import::from_path(fmu_path).map_err(|source| FmuConstructionError::Import {
                path: fmu_path.to_path_buf(),
                source,
            })?;

        let description = import.model_description();
        let co_simulation = description.co_simulation.as_ref().ok_or_else(|| {
            FmuConstructionError::NotCoSimulation {
                model_name: description.model_name.clone(),
            }
        })?;
        if let Some(internal) = co_simulation.fixed_internal_step_size {
            check_period(mapping.period, internal, &id)?;
        }

        let FmuMapping {
            period,
            inputs,
            outputs,
            initial_values,
        } = mapping;

        // Structural parameters are the ones that decide how large the arrays
        // are, so they are set first, in their own mode, and everything after
        // this reads the sizes they chose rather than the ones the XML
        // declares.
        let mut structural_vars = Vec::new();
        let mut vars_to_initialize = Vec::new();
        for (name, value) in initial_values {
            let variable = resolve_fmu_var(description, &name)?;
            if variable.is_structural(description) {
                structural_vars.push((variable, value));
            } else {
                vars_to_initialize.push((variable, value));
            }
        }

        let structural_sizes = StructuralSizes::new(description, &structural_vars)?;

        // Only now can the rest be sized, since a structural parameter set
        // above may have changed how many values a variable holds.
        let vars_to_initialize = vars_to_initialize
            .into_iter()
            .map(|(variable, value)| {
                Ok((
                    variable.with_dimensions(description, &structural_sizes)?,
                    value,
                ))
            })
            .collect::<Result<Vec<_>, FmuConstructionError>>()?;

        let inputs = inputs
            .into_iter()
            .map(|binding| {
                let variable = resolve_fmu_var(description, &binding.fmu_var_name)?
                    .with_dimensions(description, &structural_sizes)?;

                // The pointer count is the mapping's claim about how large
                // this variable is, and the FMU is the authority. Left
                // unchecked, a rebuilt FMU and a stale mapping drift apart
                // and the FMU reads whatever the tail of the buffer held.
                if binding.pointers.len() != variable.len() {
                    return Err(FmuConstructionError::Dimension {
                        variable: variable.name.clone(),
                        supplied: binding.pointers.len(),
                        expected: variable.len(),
                        dimensions: variable.dimensions.clone(),
                    });
                }
                Ok(BoundInput { binding, variable })
            })
            .collect::<Result<Vec<_>, FmuConstructionError>>()?;
        let outputs = outputs
            .into_iter()
            .map(|binding| {
                let variable = resolve_fmu_var(description, &binding.fmu_var_name)?
                    .with_dimensions(description, &structural_sizes)?;
                Ok(BoundOutput { binding, variable })
            })
            .collect::<Result<Vec<_>, FmuConstructionError>>()?;

        // One subscription per distinct key, since two variables reading one
        // message must not have it delivered twice.
        let mut subscriptions: Vec<KeyExpr> = inputs
            .iter()
            .map(|input| input.binding.subscribed_key.clone())
            .collect();
        subscriptions.sort();
        subscriptions.dedup();

        tracing::debug!(
            component = id.as_str(),
            source = %fmu_path.display(),
            extracted_to = %import.archive_path().display(),
            "imported FMU"
        );

        let instance = import
            .instantiate_cs(
                id.as_str(),
                // `visible`: an FMU here has no user to interact with.
                false,
                // `logging_on`: a model's own diagnostics are the only
                // account of why it refused a call, and they reach `tracing`
                // through `log`.
                true,
                // `event_mode_used`: off, so initialization ends in Step Mode
                // and an FMU handles its own events inside a step. Turning it
                // on is a milestone of its own.
                false,
                // `early_return_allowed`: follows from event mode being off.
                false,
                // `required_intermediate_variables`: none, since nothing here
                // reads an FMU part way through a step.
                &[],
            )
            .map_err(|source| FmuConstructionError::Instantiate {
                instance_name: id.to_string(),
                source,
            })?;

        let mut component = FmuComponent {
            id,
            instance,
            import,
            inputs,
            outputs,
            subscriptions,
            period,
            vars_to_initialize,
            last_step: None,
        };

        // Configuration Mode is the only state where a structural parameter
        // may be written, and it comes before initialization. Entering it for
        // nothing would be a call an FMU need not implement, so an FMU with
        // no structural parameters in its mapping never sees it.
        if !structural_vars.is_empty() {
            component
                .instance
                .enter_configuration_mode()
                .map_err(|source| FmuConstructionError::Configure {
                    instance_name: component.id.to_string(),
                    reason: format!("{source:?}"),
                })?;
            component.set_initial_values(&structural_vars)?;
            component
                .instance
                .exit_configuration_mode()
                .map_err(|source| FmuConstructionError::Configure {
                    instance_name: component.id.to_string(),
                    reason: format!("{source:?}"),
                })?;
        }

        // Return a component whose instance drops before the directory its
        // library was loaded from.
        Ok(component)
    }

    /// Writes name-and-value pairs from the mapping straight into the FMU.
    ///
    /// The values are the mapping's own rather than a message's, so there is
    /// no payload to resolve against and no pointer involved: each is written
    /// whole, as an array if the variable is one.
    fn set_initial_values(
        &mut self,
        initial_vals: &[(ResolvedVariable, Value)],
    ) -> Result<(), FmuConstructionError> {
        for (variable, value) in initial_vals {
            let elements = variable.flatten(value);
            if elements.len() != variable.len() {
                return Err(FmuConstructionError::Dimension {
                    variable: variable.name.clone(),
                    supplied: elements.len(),
                    expected: variable.len(),
                    dimensions: variable.dimensions.clone(),
                });
            }
            set_values(&mut self.instance, &self.id, variable, &elements).map_err(|source| {
                FmuConstructionError::InitialValue {
                    variable: variable.name.clone(),
                    reason: source.to_string(),
                }
            })?;
        }

        Ok(())
    }

    /// Where this FMU was extracted to, a temporary directory the importer
    /// chose.
    ///
    /// Worth exposing because `tempfile` discards its cleanup errors on drop:
    /// a Windows delete that fails because the library is still loaded leaves
    /// the directory behind and says nothing, and knowing the path is what
    /// makes that diagnosable at all.
    pub fn extracted_path(&self) -> &Path {
        self.import.archive_path()
    }

    /// Feeds the newest message on each bound key into the FMU.
    ///
    /// Only the newest, and older messages on that key are never decoded at
    /// all. That is FMI's own input semantics, where an FMU sees the value at
    /// the step boundary rather than every intermediate one, and it matches
    /// the sample and hold a plant already does. A key with no message this
    /// step is not written, so the FMU keeps what it had.
    fn apply_inbox(&mut self, inbox: &[Message]) -> Result<(), CoreError> {
        let mut newest: BTreeMap<&str, &Message> = BTreeMap::new();
        for message in inbox.iter().rev() {
            newest.entry(message.key.as_str()).or_insert(message);
        }

        // Decode each key once, however many variables read out of it.
        let mut payloads: BTreeMap<&str, Value> = BTreeMap::new();
        for (key, message) in newest {
            payloads.insert(key, message.decode()?);
        }

        for input in &self.inputs {
            let Some(payload) = payloads.get(input.binding.subscribed_key.as_str()) else {
                continue;
            };
            let values = input.binding.resolve(payload)?;
            set_input_var(&mut self.instance, &self.id, input, &values)?;
        }

        Ok(())
    }

    /// Reads every output and publishes it, one message per key.
    ///
    /// Outputs sharing a key merge into one payload, so an FMU naming its
    /// outputs `position.x` and `position.y` publishes one nested object
    /// rather than two messages that each overwrite the other.
    fn publish_outputs(&mut self, ctx: &mut StepCtx) -> Result<(), CoreError> {
        let mut payloads: Vec<(KeyExpr, Value)> = Vec::new();
        for output in &self.outputs {
            let value = get_output_var(&mut self.instance, &self.id, output)?;
            let key = &output.binding.published_key;

            let slot = match payloads.iter_mut().find(|(existing, _)| existing == key) {
                Some((_, payload)) => payload,
                None => {
                    payloads.push((key.clone(), Value::Null));
                    &mut payloads.last_mut().expect("just pushed").1
                }
            };
            insert_at_pointer(slot, &output.binding.payload_pointer, value);
        }

        for (key, payload) in payloads {
            ctx.publish(key, &payload)?;
        }

        Ok(())
    }
}

impl Component for FmuComponent {
    fn id(&self) -> ComponentId {
        self.id.clone()
    }

    fn subscriptions(&self) -> Vec<KeyExpr> {
        self.subscriptions.clone()
    }

    fn step(&mut self, ctx: &mut StepCtx) -> Result<SimTime, CoreError> {
        let now = ctx.now();

        match self.last_step {
            // Nothing to step across yet, so the first step initializes and
            // publishes what the FMU starts out holding. Initialization ends
            // in Step Mode, since this instance declared it does not handle
            // events.
            None => {
                self.instance
                    .enter_initialization_mode(None, now.as_secs_f64(), None)
                    .map_err(|source| failed(&self.id, "", "enter_initialization_mode", &source))?;

                // The mapping's values first, then the inbox, so a message
                // arriving at instant zero wins over a start value written
                // for the case where none does.
                let vars_to_initialize = std::mem::take(&mut self.vars_to_initialize);
                self.set_initial_values(&vars_to_initialize)
                    .map_err(|error| CoreError::ComponentFailure {
                        reason: error.to_string(),
                    })?;
                self.apply_inbox(ctx.inbox())?;
                self.instance
                    .exit_initialization_mode()
                    .map_err(|source| failed(&self.id, "", "exit_initialization_mode", &source))?;
            }
            Some(last) => {
                self.apply_inbox(ctx.inbox())?;

                let mut event_handling_needed = false;
                let mut terminate_simulation = false;
                let mut early_return = false;
                let mut last_successful_time = 0.0;
                self.instance
                    .do_step(
                        last.as_secs_f64(),
                        (now - last).as_secs_f64(),
                        // `no_set_fmu_state_prior_to_current_point`: this
                        // adapter never rewinds an FMU, so it promises not
                        // to, which lets a model discard what it would
                        // otherwise keep in order to be rewound.
                        true,
                        &mut event_handling_needed,
                        &mut terminate_simulation,
                        &mut early_return,
                        &mut last_successful_time,
                    )
                    .map_err(|source| failed(&self.id, "", "do_step", &source))?;

                // An FMU asking to stop is a halt rather than a suggestion.
                // It happens before any output is read, so a failed step
                // publishes nothing, which is what the conductor does with a
                // failed step's outbox anyway.
                if terminate_simulation {
                    return Err(CoreError::ComponentFailure {
                        reason: format!(
                            "FMU instance {:?} asked to terminate the simulation",
                            self.id.as_str()
                        ),
                    });
                }

                // The other two flags are refused rather than ignored, since
                // continuing past either would publish values from an instant
                // nobody asked about.
                //
                // `early_return` says the FMU stopped short of the requested
                // end. It cannot arrive while `early_return_allowed` is
                // false, so seeing it means the FMU broke its side of the
                // agreement, and the outputs belong to `last_successful_time`
                // rather than to now. That value is only meaningful when this
                // flag is set, which is why nothing reads it otherwise.
                if early_return {
                    return Err(CoreError::ComponentFailure {
                        reason: format!(
                            "FMU instance {:?} stopped early at {last_successful_time} s, not {} s",
                            self.id.as_str(),
                            now.as_secs_f64()
                        ),
                    });
                }

                // TODO(PLAN "Deferred"): `event_handling_needed` is where
                // event mode would begin. With it switched off an FMU handles
                // its own events inside a step and must not ask, so the flag
                // arriving means the model expects a mode this adapter does
                // not enter, and its state is not what the next step assumes.
                if event_handling_needed {
                    return Err(CoreError::ComponentFailure {
                        reason: format!(
                            "FMU instance {:?} asked for event mode, which it was instantiated without",
                            self.id.as_str()
                        ),
                    });
                }
            }
        }

        self.publish_outputs(ctx)?;
        self.last_step = Some(now);

        // Return the next instant this FMU is due.
        Ok(now + self.period)
    }
}

/// Sets one input variable, dispatching on the type the FMU declares.
///
/// A free function rather than a method so the instance and the bindings can
/// be borrowed at the same time, which `&mut self` does not allow.
fn set_input_var(
    instance: &mut InstanceCS,
    id: &ComponentId,
    input: &BoundInput,
    values: &[&Value],
) -> Result<(), CoreError> {
    set_values(instance, id, &input.variable, values)
}

/// Writes one variable, whatever its shape.
///
/// An array is one call with `nValues = N` against a single value reference,
/// which is what the C API's separate reference and value counts are for, so
/// nothing here treats an array as a special case.
fn set_values(
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
                .map_err(|source| failed(id, name, stringify!($setter), &source))
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
                .map_err(|source| failed(id, name, "set_string", &source))
        }
        VariableType::FmiBinary => {
            let converted = values
                .iter()
                .map(|value| convert::to_fmi_binary(value, name))
                .collect::<Result<Vec<_>, _>>()?;
            let borrowed: Vec<&[u8]> = converted.iter().map(Vec::as_slice).collect();
            instance
                .set_binary(&references, &borrowed)
                .map_err(|source| failed(id, name, "set_binary", &source))
        }
        VariableType::FmiClock => Err(unbound_clock(name)),
    }
}

/// Reads one output variable, dispatching on the type the FMU declares.
fn get_output_var(
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
                .map_err(|source| failed(id, name, stringify!($getter), &source))?;
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
                .map_err(|source| failed(id, name, "get_string", &source))?;
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
                .map_err(|source| failed(id, name, "get_binary", &source))?;
            let converted = buffers
                .iter()
                .zip(sizes.iter().chain(std::iter::repeat(&0)))
                // Zero is a length rather than a failure: an FMU may hold an
                // empty Binary, and that encodes as the empty string, which
                // decodes back to no bytes.
                .map(|(buffer, size)| convert::from_fmi_binary(&buffer[..*size], name))
                .collect::<Result<Vec<_>, _>>()?;
            Ok(variable.shape(converted))
        }
        VariableType::FmiClock => Err(unbound_clock(name)),
    }
}

/// A Clock is a scheduling concept rather than data, and it belongs with the
/// event mode this adapter switches off.
// TODO(PLAN "Deferred"): binding Clock is part of taking on event mode, and
// the larger part: clocked FMUs add the interval and shift APIs on top of the
// mode itself, which is why an FMU with plain state events needs none of it.
fn unbound_clock(variable: &str) -> CoreError {
    CoreError::ComponentFailure {
        reason: format!("variable {variable:?} is a Clock, which this adapter does not bind"),
    }
}

/// A failed call into the FMU, named so the halt says which instance and
/// which call.
fn failed(id: &ComponentId, variable: &str, call: &str, source: &dyn std::fmt::Debug) -> CoreError {
    let about = if variable.is_empty() {
        String::new()
    } else {
        format!(" for variable {variable:?}")
    };

    // Return a halt naming the instance and the call that refused.
    CoreError::ComponentFailure {
        reason: format!(
            "FMU instance {:?} refused {call}{about}: {source:?}",
            id.as_str()
        ),
    }
}

/// What the FMU says about a variable the mapping names.
///
/// Dimensions are left empty here and filled by
/// [`ResolvedVariable::with_dimensions`], because resolving them needs the
/// structural parameters settled first.
fn resolve_fmu_var(
    description: &Fmi3ModelDescription,
    name: &str,
) -> Result<ResolvedVariable, FmuConstructionError> {
    description
        .model_variables
        .find_by_name(name)
        .map(|variable| ResolvedVariable {
            name: name.to_string(),
            value_reference: variable.value_reference(),
            declared_type: variable.data_type(),
            dimensions: Vec::new(),
        })
        .ok_or_else(|| FmuConstructionError::UnknownVariable {
            variable: name.to_string(),
            available: description
                .model_variables
                .iter_abstract()
                .map(|variable| variable.name().to_string())
                .collect(),
        })
}

/// The effective size of every dimension that names a structural parameter.
///
/// A dimension is either a constant or a reference to a variable's value, and
/// where it is a reference the value in force is the mapping's if it set one
/// and the FMU's declared start otherwise. Reading the XML alone would size
/// an array by numbers the mapping has already overridden.
struct StructuralSizes {
    by_value_reference: BTreeMap<u32, usize>,
}

impl StructuralSizes {
    fn new(
        description: &Fmi3ModelDescription,
        structural: &[(ResolvedVariable, Value)],
    ) -> Result<Self, FmuConstructionError> {
        let mut by_value_reference = BTreeMap::new();

        // Declared starts first, so a mapping's value replaces one rather
        // than competing with it.
        for variable in &description.model_variables.variables {
            if let Variable::UInt64(declared) = variable
                && declared.causality() == Causality::StructuralParameter
                && let Some(start) = declared.start().and_then(|start| start.first())
            {
                by_value_reference.insert(declared.value_reference, *start as usize);
            }
        }

        for (variable, value) in structural {
            let size = value
                .as_u64()
                .ok_or_else(|| FmuConstructionError::StructuralParameter {
                    variable: variable.name.clone(),
                    value: value.to_string(),
                })?;
            by_value_reference.insert(variable.value_reference, size as usize);
        }

        // Return the sizes this instance runs with.
        Ok(StructuralSizes { by_value_reference })
    }
}

impl ResolvedVariable {
    /// Whether this variable is one of the structural parameters that size
    /// the arrays, and so has to be written before initialization begins.
    fn is_structural(&self, description: &Fmi3ModelDescription) -> bool {
        description
            .model_variables
            .find_by_name(&self.name)
            .is_some_and(|variable| variable.causality() == Causality::StructuralParameter)
    }

    /// Fills in the dimensions this instance actually has.
    fn with_dimensions(
        mut self,
        description: &Fmi3ModelDescription,
        sizes: &StructuralSizes,
    ) -> Result<Self, FmuConstructionError> {
        self.dimensions = declared_dimensions(description, &self.name)
            .iter()
            .map(|dimension| match dimension {
                Dimension::Fixed(size) => Ok(*size as usize),
                Dimension::Variable(value_reference) => sizes
                    .by_value_reference
                    .get(value_reference)
                    .copied()
                    .ok_or_else(|| FmuConstructionError::UnresolvedDimension {
                        variable: self.name.clone(),
                        value_reference: *value_reference,
                    }),
            })
            .collect::<Result<Vec<_>, _>>()?;

        Ok(self)
    }

    /// Reads a value the mapping wrote into the flat, row-major list an FMI
    /// call takes, which is [`ResolvedVariable::shape`] backwards.
    ///
    /// Nesting to the variable's own depth, so a matrix may be written either
    /// as arrays of arrays or as one flat list. The standard itself spells a
    /// matrix flat in a `start` attribute, so accepting both costs nothing
    /// and refusing one would be arbitrary.
    fn flatten<'a>(&self, value: &'a Value) -> Vec<&'a Value> {
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
    fn shape(&self, values: Vec<Value>) -> Value {
        if self.dimensions.is_empty() {
            return values.into_iter().next().unwrap_or(Value::Null);
        }

        // Fold from the innermost dimension outward, chunking the flat list
        // each time, which is what row-major order means.
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

/// Fails unless the mapping's period lands on the FMU's own step size.
///
/// An FMU declaring `fixedInternalStepSize` advances internally in steps of
/// that size whatever it is asked for, so a period that is not a whole number
/// of them reads values from an instant other than the one the caller means.
/// Better to say so than to run a world quietly a fraction of a step out.
fn check_period(
    period: SimDuration,
    fixed_internal_step_size: f64,
    id: &ComponentId,
) -> Result<(), FmuConstructionError> {
    // Compared as whole nanoseconds rather than as floats, so "a multiple of"
    // is exact and no tolerance has to be chosen. `modelDescription.xml`
    // writes the FMU's step as a decimal, and rounding that to the nearest
    // nanosecond is what sim time does with every other duration, so the two
    // land on the same grid by construction.
    let internal_ns = SimDuration::from_secs_f64(fixed_internal_step_size).as_nanos();
    let period_ns = period.as_nanos();
    if internal_ns > 0 && period_ns >= internal_ns && period_ns % internal_ns == 0 {
        Ok(())
    } else {
        Err(FmuConstructionError::Period {
            instance_name: id.to_string(),
            period: period.as_secs_f64(),
            fixed_internal_step_size,
        })
    }
}

/// Puts `value` into the message payload being built, at a JSON Pointer.
///
/// Objects rather than arrays, because a payload under construction has no
/// shape to read an index against. Only an FMU's own variable names produce
/// these paths, and a name is a name whatever it looks like.
fn insert_at_pointer(message_payload: &mut Value, pointer: &str, value: Value) {
    // A pointer starts with the separator, so splitting it gives an empty
    // first token standing for the whole document, which is where the walk
    // starts rather than a name to descend into.
    let tokens: Vec<String> = pointer
        .split('/')
        .skip(1)
        .map(unescape_json_pointer_token)
        .collect();

    let Some((last, parents)) = tokens.split_last() else {
        *message_payload = value;
        return;
    };

    let mut cursor = message_payload;
    for token in parents {
        if !cursor.is_object() {
            *cursor = Value::Object(serde_json::Map::new());
        }
        cursor = cursor
            .as_object_mut()
            .expect("just made an object")
            .entry(token.clone())
            .or_insert(Value::Null);
    }

    if !cursor.is_object() {
        *cursor = Value::Object(serde_json::Map::new());
    }
    cursor
        .as_object_mut()
        .expect("just made an object")
        .insert(last.clone(), value);
}

/// Where a vendored reference FMU lives, for tests and examples.
pub fn fixture_path(name: &str) -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("tests/fixtures")
        .join(format!("{name}.fmu"))
}

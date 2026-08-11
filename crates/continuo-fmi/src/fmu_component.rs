//! An imported FMU's lifecycle: what happens at construction, and what
//! happens on each step.

use std::collections::BTreeMap;
use std::path::Path;

use continuo_core::{
    Component, ComponentId, CoreError, KeyExpr, Message, SimDuration, SimTime, StepCtx,
};
use fmi::fmi3::instance::InstanceCS;
use fmi::fmi3::{CoSimulation, Common, Fmi3Model, import::Fmi3Import};
use fmi::traits::FmiImport;
use serde_json::Value;

use crate::error::{FmuConstructionError, step_failure};
use crate::fmu_get_set::{get_output_var, set_input_var, set_values};
use crate::fmu_mapping::{FmuMapping, insert_at_pointer};
use crate::fmu_variable::{
    BoundInput, BoundOutput, ResolvedVariable, StructuralSizes, resolve_fmu_var,
};

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

        // The sizes come first, before any variable is resolved, because a
        // structural parameter the mapping sets decides how many values the
        // arrays hold. Resolving against the XML's own numbers would size
        // them by figures the mapping has already replaced.
        let structural_sizes = StructuralSizes::new(description, &initial_values)?;

        // Structural parameters are written in their own mode, ahead of
        // initialization, so they are kept apart from the rest here.
        let mut structural_vars = Vec::new();
        let mut vars_to_initialize = Vec::new();
        for (name, value) in initial_values {
            let variable = resolve_fmu_var(description, &name, &structural_sizes)?;
            if variable.is_structural() {
                structural_vars.push((variable, value));
            } else {
                vars_to_initialize.push((variable, value));
            }
        }

        let inputs = inputs
            .into_iter()
            .map(|binding| {
                let variable =
                    resolve_fmu_var(description, &binding.fmu_var_name, &structural_sizes)?;

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
                let variable =
                    resolve_fmu_var(description, &binding.fmu_var_name, &structural_sizes)?;
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

        // Return a component sized by its structural parameters and ready to
        // initialize, which it does on its first step.
        Ok(component)
    }

    /// Writes values the mapping wrote out straight into the FMU.
    ///
    /// The mapping supplies these itself rather than binding them to a
    /// message, so there is no payload to resolve and no pointer involved.
    /// Each variable is written whole, as an array where it is one.
    ///
    /// Used twice, in the two modes the standard allows: structural
    /// parameters during configuration, and everything else during
    /// initialization.
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

    /// An FMU is opaque to the per-tick hash, so it is covered in
    /// output-hash mode: everything it publishes is hashed, and divergence
    /// shows the first time a changed internal value reaches an output.
    ///
    /// PLAN.md's determinism rules anticipated state-hash mode for an FMU
    /// declaring `canSerializeFMUState`, and all four vendored fixtures
    /// declare it. That flag turns out to be the wrong question, which is
    /// the more interesting of the two reasons this returns `None`.
    ///
    /// **Nothing promises that serialized state is a fingerprint.** FMI 3.0
    /// documents the flag as meaning those functions are supported, and
    /// `fmi3SerializeFMUState` as copying the referenced data into a byte
    /// vector. Neither says what the bytes contain, and nothing says equal
    /// states serialize to equal bytes. So byte stability cannot be assumed
    /// from any FMU, and can only be established one FMU at a time by
    /// measuring, which is why it belongs in a mapping rather than keyed off
    /// a capability.
    ///
    /// The reference FMUs show the pessimistic case is real, and they are
    /// published by the same body that wrote the standard. Serialization
    /// there is a `memcpy` of the whole `ModelInstance` struct, which holds
    /// an instance
    /// name pointer, a `componentEnvironment`, and five callback pointers
    /// including the logger, which points back into this binary. Those are
    /// addresses, differing between runs of one program on one machine, and
    /// the padding between fields is never written.
    ///
    /// Restoring is unaffected, which is what makes the two capabilities
    /// different rather than one of them broken. The standard's own example
    /// for serializing is storing to a file and restarting from it later, so
    /// surviving a process is the intent, and a conforming FMU deals with
    /// its own pointers. How is its business: the reference FMUs copy field
    /// by field when state is set back and skip every pointer, leaving the
    /// live instance its own callbacks and names. So the surplus bytes are
    /// ignored by the only consumer that reads them, and bytes nobody reads
    /// are free to be anything.
    ///
    /// The second reason is upstream, and would matter only if the first
    /// were solved: `fmi` 0.8.0 wraps no serialization call and disables
    /// `get_fmu_state` with `#[cfg(false)]`, and `Instance` keeps its
    /// library handle and instance pointer private, so the raw bindings
    /// cannot be reached either.
    ///
    /// One design note, since it is not obvious until you try. This method
    /// takes `&self`, while every call on an FMU instance takes `&mut self`,
    /// plain getters included: a get in FMI can run model code to compute an
    /// output, and taking a state allocates. So the bytes would have to be
    /// captured during `step` and cached. That costs nothing over
    /// serializing on demand, since the conductor asks every stepped
    /// component every step, so `&self` is not the obstacle either.
    // TODO(PLAN "Deferred"): an FMU whose serialization is a stable function
    // of its state could join the hash directly, so divergence is caught when
    // it happens rather than when it surfaces. A mapping opt-in rather than
    // the opt-out the plan expected, since stability is never promised and so
    // has to be measured per FMU. It needs `fmi` to expose the state calls
    // first: a flag alone cannot be honored while `Instance` keeps its
    // library handle and pointer private.
    //
    // There is a middle option that needs no upstream change, if this is ever
    // wanted sooner: hash every variable the FMU declares rather than only
    // the mapped outputs, through the gets already here. Stronger than
    // output-hash, since locals like StateSpace's `x` and `der(x)` are
    // exposed and mapped nowhere, and weaker than state-hash, since anything
    // genuinely hidden stays hidden. Costs one get per declared variable per
    // step, and is not a pure observation on an FMU whose getters compute.
    fn state_bytes(&self) -> Option<Vec<u8>> {
        None
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
                    .map_err(|source| {
                        step_failure(&self.id, "", "enter_initialization_mode", &source)
                    })?;

                // The mapping's values first, then the inbox, so a message
                // arriving at instant zero wins over a start value written
                // for the case where none does.
                let vars_to_initialize = std::mem::take(&mut self.vars_to_initialize);
                self.set_initial_values(&vars_to_initialize)
                    .map_err(|error| CoreError::ComponentFailure {
                        reason: error.to_string(),
                    })?;
                self.apply_inbox(ctx.inbox())?;
                self.instance.exit_initialization_mode().map_err(|source| {
                    step_failure(&self.id, "", "exit_initialization_mode", &source)
                })?;
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
                    .map_err(|source| step_failure(&self.id, "", "do_step", &source))?;

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

use std::path::PathBuf;

use continuo_core::{ComponentId, CoreError};
use thiserror::Error;

/// Why an FMU could not be made into a component.
///
/// Named for when it happens rather than for what it wraps, because that is
/// the whole distinction: everything here is a wiring mistake that fails
/// before a run starts. Once a component is running, `Component::step` may
/// return nothing but [`CoreError`], so every step-time failure travels as
/// [`CoreError::ComponentFailure`] with a reason naming the instance and the
/// call, and this type never grows a variant for one.
///
/// [`CoreError`]: continuo_core::CoreError
/// [`CoreError::ComponentFailure`]: continuo_core::CoreError::ComponentFailure
#[derive(Debug, Error)]
pub enum FmuConstructionError {
    /// The `.fmu` could not be read: no such file, not a zip, no
    /// `modelDescription.xml` inside it, or one that does not parse.
    #[error("cannot import FMU from {path:?}: {source}")]
    Import {
        path: PathBuf,
        #[source]
        source: fmi::Error,
    },

    /// The FMU imported but would not hand back a co-simulation instance.
    /// Loading the shared library lands here too, which is where a `.fmu`
    /// built for other platforms than this one shows up.
    #[error("cannot instantiate {instance_name:?} for co-simulation: {source}")]
    Instantiate {
        instance_name: String,
        #[source]
        source: fmi::Error,
    },

    /// The mapping names a variable the FMU does not declare.
    ///
    /// Lists what the FMU declares, because the usual cause is a rename on
    /// one side of a boundary the compiler cannot see across: the mapping is
    /// data, the FMU is a binary built elsewhere, and nothing checks the two
    /// agree until this moment.
    #[error("FMU declares no variable {variable:?}; it declares {}", available.join(", "))]
    UnknownVariable {
        variable: String,
        available: Vec<String>,
    },

    /// The id this component would register under is not a legal one: empty,
    /// or carrying a `/` or a wildcard.
    ///
    /// Forwarded rather than restated, because [`ComponentId`] owns that rule
    /// and its message already names the offending id. `transparent` passes
    /// both the text and the source through, so a caller printing the chain
    /// does not see one failure written out twice.
    ///
    /// [`ComponentId`]: continuo_core::ComponentId
    #[error(transparent)]
    Id(#[from] continuo_core::CoreError),

    /// The mapping's period does not land on the FMU's own step size.
    ///
    /// An FMU declaring `fixedInternalStepSize` steps internally at that size
    /// whatever it is asked for, so a period that is not a whole number of
    /// them reads values from an instant other than the one the caller means.
    #[error(
        "period {period} s is not a multiple of the {fixed_internal_step_size} s step {instance_name:?} takes"
    )]
    Period {
        instance_name: String,
        period: f64,
        fixed_internal_step_size: f64,
    },

    /// The mapping supplies a different number of values than the variable
    /// holds.
    ///
    /// The FMU is the authority on that count, and the mapping is a claim
    /// about it. Unchecked, a rebuilt FMU and a stale mapping drift apart and
    /// the model reads whatever the tail of the buffer held.
    #[error(
        "variable {variable:?} holds {expected} values {dimensions:?}, and the mapping supplies {supplied}"
    )]
    Dimension {
        variable: String,
        /// How many the mapping supplies, counting JSON Pointers where it
        /// binds a message and values where it writes them out.
        supplied: usize,
        /// How many the variable holds, the product of its dimensions.
        expected: usize,
        dimensions: Vec<usize>,
    },

    /// A dimension names a variable whose value is not known.
    #[error(
        "variable {variable:?} is sized by value reference {value_reference}, which has no value"
    )]
    UnresolvedDimension {
        variable: String,
        value_reference: u32,
    },

    /// A structural parameter was given something that is not a size.
    #[error("structural parameter {variable:?} sizes an array, and {value} is not a count")]
    StructuralParameter { variable: String, value: String },

    /// The FMU refused to enter or leave Configuration Mode, which is the
    /// only state where a structural parameter may be written.
    #[error("instance {instance_name:?} refused configuration mode: {reason}")]
    Configure {
        instance_name: String,
        reason: String,
    },

    /// The FMU refused a value the mapping asked to set before the run.
    #[error("initial value for {variable:?}: {reason}")]
    InitialValue { variable: String, reason: String },

    /// The FMU ships no co-simulation interface, so it cannot be stepped by a
    /// conductor at all. Model exchange FMUs need a solver, which is a
    /// different thing to build than an adapter.
    #[error("FMU {model_name:?} declares no co-simulation interface")]
    NotCoSimulation { model_name: String },
}

/// A failed call into the FMU, named so the halt says which instance and
/// which call.
pub(crate) fn step_failure(
    id: &ComponentId,
    variable: &str,
    call: &str,
    source: &dyn std::fmt::Debug,
) -> CoreError {
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

/// A Clock is a scheduling concept rather than data, and it belongs with the
/// event mode this adapter switches off.
// TODO(PLAN "Deferred"): binding Clock is part of taking on event mode, and
// the larger part: clocked FMUs add the interval and shift APIs on top of the
// mode itself, which is why an FMU with plain state events needs none of it.
pub(crate) fn unbound_clock(variable: &str) -> CoreError {
    CoreError::ComponentFailure {
        reason: format!("variable {variable:?} is a Clock, which this adapter does not bind"),
    }
}

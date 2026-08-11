use std::path::PathBuf;

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

    /// The component id is not a legal one.
    #[error("{0}")]
    Id(#[source] continuo_core::CoreError),

    /// The mapping's period does not land on the FMU's own step size.
    ///
    /// An FMU declaring `fixedInternalStepSize` steps internally at that size
    /// whatever it is asked for, so a period that is not a whole number of
    /// them reads values from an instant other than the one the caller means.
    #[error(
        "instance {instance_name:?} would step every {period} s, which is not a whole number \
         of the {fixed_internal_step_size} s steps this FMU takes internally"
    )]
    Period {
        instance_name: String,
        period: f64,
        fixed_internal_step_size: f64,
    },

    /// The FMU ships no co-simulation interface, so it cannot be stepped by a
    /// conductor at all. Model exchange FMUs need a solver, which is a
    /// different thing to build than an adapter.
    #[error("FMU {model_name:?} declares no co-simulation interface")]
    NotCoSimulation { model_name: String },
}

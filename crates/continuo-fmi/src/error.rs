use std::path::PathBuf;

use thiserror::Error;

/// Why an FMU could not be made into a component.
///
/// Construction only. Once a component is running, `Component::step` may
/// return nothing but [`CoreError`], so every step-time failure travels as
/// [`CoreError::ComponentFailure`] with a reason naming the instance and the
/// call. The split is the useful one: everything here is a wiring mistake
/// that fails before a run starts, while a step-time failure halts a run in
/// progress.
///
/// [`CoreError`]: continuo_core::CoreError
/// [`CoreError::ComponentFailure`]: continuo_core::CoreError::ComponentFailure
#[derive(Debug, Error)]
pub enum FmuError {
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

    /// The FMU ships no co-simulation interface, so it cannot be stepped by a
    /// conductor at all. Model exchange FMUs need a solver, which is a
    /// different thing to build than an adapter.
    #[error("FMU {model_name:?} declares no co-simulation interface")]
    NotCoSimulation { model_name: String },
}

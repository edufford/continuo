//! A `.fmu` extracted once and shared by every instance built from it.

use std::path::{Path, PathBuf};
use std::sync::Arc;

use fmi::fmi3::import::Fmi3Import;
use fmi::traits::FmiImport;

use crate::error::FmuConstructionError;

/// A `.fmu` extracted once to a temporary directory, from which any number
/// of [`FmuComponent`](crate::FmuComponent)s are built, so only the first
/// loads its library.
///
/// An FMU declaring `canBeInstantiatedOnlyOncePerProcess` cannot share its
/// library, so it is extracted again for each instance. Cloning shares the
/// extraction, and the directory is deleted when the last clone and instance
/// are gone.
#[derive(Clone)]
pub struct FmuExtraction {
    source: PathBuf,
    fmi3_import: Arc<Fmi3Import>,
}

impl FmuExtraction {
    /// Extracts the `.fmu` file at `fmu_path` and returns the extraction.
    pub fn extract(fmu_path: impl AsRef<Path>) -> Result<Self, FmuConstructionError> {
        let source = fmu_path.as_ref().to_path_buf();
        let fmi3_import =
            fmi::import::from_path(&source).map_err(|error| FmuConstructionError::Import {
                path: source.clone(),
                source: error,
            })?;

        // Return the extraction, ready to build instances from.
        Ok(FmuExtraction {
            source,
            fmi3_import: Arc::new(fmi3_import),
        })
    }

    /// The file path to the `.fmu`.
    pub fn source(&self) -> &Path {
        &self.source
    }

    /// The import a new instance loads its library from: this one's, or a
    /// fresh extraction for an FMU allowing one instance per process.
    pub(crate) fn fmi3_import_for_instance(&self) -> Result<Arc<Fmi3Import>, FmuConstructionError> {
        let once_per_process = self
            .fmi3_import
            .model_description()
            .co_simulation
            .as_ref()
            .and_then(|co_simulation| co_simulation.can_be_instantiated_only_once_per_process)
            .unwrap_or(false);

        // Return a separate extraction for an FMU that cannot share one.
        if once_per_process {
            Ok(FmuExtraction::extract(&self.source)?.fmi3_import)
        } else {
            Ok(Arc::clone(&self.fmi3_import))
        }
    }
}

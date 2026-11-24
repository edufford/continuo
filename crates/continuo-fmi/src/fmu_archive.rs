//! A `.fmu` extracted once and shared by every instance built from it.

use std::path::{Path, PathBuf};
use std::sync::Arc;

use fmi::fmi3::import::Fmi3Import;
use fmi::traits::FmiImport;

use crate::error::FmuConstructionError;

/// A `.fmu` extracted to a temporary directory, from which any number of
/// [`FmuComponent`](crate::FmuComponent)s can be built.
///
/// Every instance built from one archive loads the shared library from the
/// same file, so the operating system loads it once and later instances only
/// add a reference to it. On Windows a first load takes 75 to 93 ms and a
/// later one 65 us, which is the difference between a run that stalls as
/// each car joins and one that does not. Instances then share the library's
/// statics, which FMI allows unless the FMU declares
/// `canBeInstantiatedOnlyOncePerProcess`; such an FMU gets an extraction of
/// its own per instance.
///
/// Cloning is cheap and shares the extraction. The temporary directory is
/// deleted once the last clone and the last instance built from it are gone.
#[derive(Clone)]
pub struct FmuArchive {
    source: PathBuf,
    import: Arc<Fmi3Import>,
}

impl FmuArchive {
    /// Extracts `fmu_path` and reads its model description.
    pub fn open(fmu_path: impl AsRef<Path>) -> Result<Self, FmuConstructionError> {
        let source = fmu_path.as_ref().to_path_buf();
        let import = Arc::new(extract(&source)?);

        // Return the archive, extracted and ready to build instances from.
        Ok(FmuArchive { source, import })
    }

    /// The `.fmu` this archive was extracted from.
    pub fn source(&self) -> &Path {
        &self.source
    }

    /// The extraction an instance loads its library from: this archive's own,
    /// or for an FMU that may be instantiated only once per process, a new
    /// one for that instance alone.
    pub(crate) fn import_for_instance(&self) -> Result<Arc<Fmi3Import>, FmuConstructionError> {
        let once_per_process = self
            .import
            .model_description()
            .co_simulation
            .as_ref()
            .and_then(|co_simulation| co_simulation.can_be_instantiated_only_once_per_process)
            .unwrap_or(false);

        // Return a separate extraction for an FMU that cannot share one.
        if once_per_process {
            Ok(Arc::new(extract(&self.source)?))
        } else {
            Ok(Arc::clone(&self.import))
        }
    }
}

fn extract(fmu_path: &Path) -> Result<Fmi3Import, FmuConstructionError> {
    fmi::import::from_path(fmu_path).map_err(|source| FmuConstructionError::Import {
        path: fmu_path.to_path_buf(),
        source,
    })
}

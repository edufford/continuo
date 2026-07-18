/// Conductor configuration.
#[derive(Debug, Clone)]
pub struct ConductorConfig {
    /// World name, used in key expressions (`continuo/{world}/...`).
    pub world: String,
    /// `false` = free-run (as fast as possible); `true` = 1× real-time.
    /// Real-time pacing arrives in milestone 3 — until then `true` is
    /// rejected at construction.
    pub real_time_pacing: bool,
}

impl Default for ConductorConfig {
    fn default() -> Self {
        ConductorConfig {
            world: "world".to_string(),
            real_time_pacing: false,
        }
    }
}

/// Conductor configuration.
#[derive(Debug, Clone)]
pub struct ConductorConfig {
    /// World name, used in key expressions (`continuo/{world}/...`).
    pub world: String,
    /// World seed: the root of every component's deterministic randomness
    /// (per-component seeds derive from `(seed, component_path)`) and the
    /// starting point of the running world hash. Same seed + same scenario
    /// => identical runs.
    pub seed: u64,
    /// `false` = free-run (as fast as possible); `true` = 1× real-time.
    /// Real-time pacing arrives in milestone 3 — until then `true` is
    /// rejected at construction.
    pub real_time_pacing: bool,
}

impl Default for ConductorConfig {
    fn default() -> Self {
        ConductorConfig {
            world: "world".to_string(),
            seed: 0,
            real_time_pacing: false,
        }
    }
}

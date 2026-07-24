/// Conductor configuration.
#[derive(Debug, Clone)]
pub struct ConductorConfig {
    /// World name: fills the world segment of every key expression
    /// (`continuo/{world}/...`).
    pub world_name: String,
    /// World seed: the root of every component's deterministic randomness
    /// (per-component seeds derive from `(world_seed, component_path)`) and
    /// the starting point of the running world hash. Same seed + same
    /// scenario => identical runs.
    pub world_seed: u64,
    /// `false` = free-run (as fast as possible); `true` = 1× real-time.
    /// Real-time pacing arrives in milestone 3 — until then `true` is
    /// rejected at construction.
    pub real_time_pacing: bool,
}

impl Default for ConductorConfig {
    fn default() -> Self {
        ConductorConfig {
            world_name: "world".to_string(),
            world_seed: 0,
            real_time_pacing: false,
        }
    }
}

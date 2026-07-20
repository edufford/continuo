use thiserror::Error;

/// Errors produced by continuo-core types.
#[derive(Debug, Error)]
pub enum CoreError {
    #[error(
        "invalid component id {0:?}: must be non-empty and contain no '/' or wildcard characters"
    )]
    InvalidComponentId(String),

    #[error("invalid key expression {expr:?}: {reason}")]
    InvalidKeyExpr { expr: String, reason: String },

    #[error("invalid time value {value:?}: {reason}")]
    TimeParse { value: String, reason: String },

    #[error("failed to serialize payload for key {key:?}: {source}")]
    PayloadSerialize {
        key: String,
        #[source]
        source: serde_json::Error,
    },
}

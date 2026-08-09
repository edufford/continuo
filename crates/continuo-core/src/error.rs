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

    /// Rejected at the publisher, because JSON cannot carry it. `serde_json`
    /// writes `NaN` and `±inf` as `null`, which decodes nowhere, so letting one
    /// through would surface as a decode failure at some later consumer with
    /// nothing pointing back at the arithmetic that produced it.
    #[error("non-finite float {found} in payload for key {key:?}")]
    NonFiniteFloat {
        key: String,
        /// The value and where it was, such as `NaN at position.x`.
        found: String,
    },
}

//! `waddle._core` — the PyO3 shim over `waddle-runtime`.
//!
//! Hollow frontend: this module marshals between Python and core; every
//! claim/lease/handoff/timeline decision is made in waddle-core exactly
//! once. If an `if` about claims appears here, that is a defect.

use pyo3::prelude::*;
use pyo3::types::PyFrozenSet;

mod convert;
mod episode;
mod session;
mod verbs;

/// Which connected transports this extension was compiled with — the ONLY
/// feature-detection surface the Python layer may branch on (it decides
/// which kwargs it can honor and what to say when it cannot; everything
/// behind them stays core's). Names match the cargo features one-for-one:
/// `"grpc"` (the tonic control transport), `"livekit"` (the media plane).
fn built_features() -> Vec<&'static str> {
    [
        cfg!(feature = "grpc").then_some("grpc"),
        cfg!(feature = "livekit").then_some("livekit"),
    ]
    .into_iter()
    .flatten()
    .collect()
}

/// Parse a canonical proto3 JSON `waddle.v0.RobotDescription` and validate
/// it against the domain layer. Raises `ValueError` with the underlying
/// message on either failure.
#[pyfunction]
fn validate_robot_json(json: &str) -> PyResult<()> {
    let robot = convert::parse_robot_json(json)?;
    waddle_types::RobotDescription::try_from(&robot)
        .map(|_| ())
        .map_err(|e| pyo3::exceptions::PyValueError::new_err(e.to_string()))
}

/// Decode a compiled `waddle.v0.RobotDescription` exactly as core does and
/// hand back core's own canonical proto3 JSON of what it *understood*.
///
/// Decoding tolerates unknown fields on purpose (append-only schema
/// evolution), so validation alone can never prove a key Python compiled
/// landed anywhere: a misspelled one is dropped in silence and validates
/// fine. Round-tripping is the only way to see it — a key that survives is
/// a field, a key that vanishes was never one.
#[pyfunction]
fn robot_json_roundtrip(json: &str) -> PyResult<String> {
    let robot = convert::parse_robot_json(json)?;
    waddle_sidecar::json::message_to_json("waddle.v0.RobotDescription", &robot)
        .map_err(|e| pyo3::exceptions::PyValueError::new_err(e.to_string()))
}

#[pymodule]
fn _core(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_class::<session::PySession>()?;
    m.add_class::<session::AgentResult>()?;
    m.add_class::<episode::PyEpisode>()?;
    m.add_class::<episode::GateInfo>()?;
    m.add_class::<verbs::PyChunk>()?;
    m.add_function(wrap_pyfunction!(session::create_session, m)?)?;
    m.add_function(wrap_pyfunction!(validate_robot_json, m)?)?;
    m.add_function(wrap_pyfunction!(robot_json_roundtrip, m)?)?;
    m.add("__version__", env!("CARGO_PKG_VERSION"))?;
    m.add("FEATURES", PyFrozenSet::new(m.py(), built_features())?)?;
    Ok(())
}

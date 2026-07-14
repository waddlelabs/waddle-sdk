//! waddle-controlplane — the client side of the `ControlPlane` service.
//!
//! Session registration and grant negotiation, reconnect with backoff,
//! offline event buffering (bounded, drop-oldest, replay in order), and the
//! N11 heartbeat (proxy signals, never live verb latencies).
//!
//! The transport is a trait; the tested default is the in-memory
//! [`transport::InMemoryTransport`] with a scriptable server. The real tonic
//! gRPC transport is deferred behind the `tonic-transport` feature.
//! Everything here is synchronous (std channels + one client thread); the
//! runtime owns any async.

pub mod backoff;
pub mod buffer;
pub mod client;
pub mod heartbeat;
pub mod negotiate;
pub mod transport;

pub use backoff::Backoff;
pub use buffer::OfflineBuffer;
pub use client::{ClientConfig, ControlPlaneClient, PlaneEvent};
pub use heartbeat::{HeartbeatTracker, HostLoad};
pub use transport::{ClientMsg, ControlConn, ControlTransport, InMemoryTransport, ServerMsg};

#[derive(Debug, thiserror::Error)]
pub enum PlaneError {
    #[error("transport: {0}")]
    Transport(String),
    #[error("the control plane client is shut down")]
    Shutdown,
}

#[cfg(feature = "tonic-transport")]
pub mod tonic_transport {
    //! Typed stub: the live gRPC transport is a named-trigger deferral.

    use super::{ControlTransport, PlaneError};
    use std::sync::Arc;

    pub fn connect(_endpoint: &str) -> Result<Arc<dyn ControlTransport>, PlaneError> {
        Err(PlaneError::Transport(
            "tonic transport is a stub until the gRPC integration milestone".into(),
        ))
    }
}

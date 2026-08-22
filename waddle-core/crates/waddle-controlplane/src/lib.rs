//! waddle-controlplane — the client side of the `ControlPlane` service.
//!
//! Session registration and grant negotiation, reconnect with backoff,
//! offline event buffering (bounded, drop-oldest, replay in order), and the
//! N11 heartbeat (proxy signals, never live verb latencies).
//!
//! One classification — [`ClientMsg::is_droppable`] — decides what may be
//! shed, and it binds twice: while the plane is offline nothing droppable
//! enters the [`buffer`], and while the plane is connected but not draining
//! nothing droppable queues without bound inside a transport ([`inflight`]).
//! Its sibling — [`ClientMsg::connection_scoped_flag`] — decides what may
//! cross a connection boundary: a message whose content is legal only under
//! a negotiated feature flag ([`flags`]) belongs to the connection that
//! accepted it and never survives a partition.
//!
//! The transport is a trait; the tested default is the in-memory
//! [`transport::InMemoryTransport`] with a scriptable server. The real tonic
//! gRPC transport lives in [`grpc`] behind the `tonic-transport` feature
//! (tokio stays confined to its dedicated worker thread). Everything on the
//! public surface is synchronous (std channels + one client thread).

pub mod backoff;
pub mod buffer;
pub mod client;
pub mod flags;
pub mod heartbeat;
pub mod inflight;
pub mod negotiate;
pub mod transport;

pub use backoff::Backoff;
pub use buffer::OfflineBuffer;
pub use client::{ClientConfig, ControlPlaneClient, PlaneEvent};
pub use heartbeat::{HeartbeatTracker, HostLoad};
pub use inflight::{DEFAULT_INFLIGHT_CAP, Inflight, InflightLimit};
pub use transport::{
    ClientMsg, ControlConn, ControlTransport, InMemoryTransport, RegistrationRejection, ServerMsg,
};

#[derive(Debug, thiserror::Error)]
pub enum PlaneError {
    #[error("transport: {0}")]
    Transport(String),
    #[error("the control plane client is shut down")]
    Shutdown,
}

#[cfg(feature = "tonic-transport")]
pub mod grpc;

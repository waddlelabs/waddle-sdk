//! waddle-codecs — dialect codecs between upstream wire formats and the
//! Waddle protocol types (amendments N4/N15).
//!
//! A *codec* translates one upstream ecosystem's wire format (its "dialect")
//! into `waddle_types::pb` messages and back. Codecs are **independently
//! versioned** (N4): they track upstream wire formats on their own release
//! cadence, decoupled from the workspace crates, so an upstream format bump
//! never forces a Waddle release (or vice versa).
//!
//! Layering (CI-checked): this crate depends only on `waddle-types` plus
//! serde-adjacent leaf crates. No tokio, no threads, no I/O, no clocks, no
//! randomness — `cargo tree -p waddle-codecs -e normal` must never contain
//! tokio or the fsm/gate/runtime crates.
//!
//! The write-path rules this crate enforces:
//!
//! - **No floating "latest" (N15).** [`Registry`] resolution never picks the
//!   newest matching version. A lookup succeeds only when it resolves
//!   unambiguously — exact pins always do; ranges only when a single
//!   certified codec matches.
//! - **Mandatory certification.** A codec is never returned by lookup until
//!   [`Registry::certify`] has run its round-trip fixtures green.

pub mod descriptor;
pub mod dialects;
pub mod registry;
pub mod signing;
pub mod traits;

pub use descriptor::{CodecDescriptor, DescriptorError};
pub use dialects::lerobot_async::LerobotAsyncCodec;
pub use dialects::openpi::OpenPiCodec;
pub use registry::{CertFixtures, CertReport, Registry, RegistryError};
pub use signing::{InsecureAcceptAll, Sha256ContentPin, SignatureError, SignatureVerifier};
pub use traits::{Codec, CodecCaps, CodecError, ObsFrame};

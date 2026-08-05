//! The feature-flag names this crate classifies by (protocol registry:
//! `waddle-protocol/docs/VERSIONING.md` §3), named ONCE — the runtime
//! re-exports these rather than spelling them again.
//!
//! They live here because this is where a flag becomes a fact: the SDK
//! declares them on its `RegisterRequest` and the plane answers with
//! `RegisterResponse.accepted_feature_flags`, per connection, at every
//! registration. Everything downstream — which behaviors a session may
//! exhibit, and which messages may cross a given connection
//! ([`crate::ClientMsg::connection_scoped_flag`]) — keys off that answer.
//! A flag no message here is classified by (e.g. `waddle.v0.agent`, which
//! gates plane→SDK routing only) stays where it is used.
//!
//! A flag string is WIRE: renaming one of these constants renames nothing on
//! the wire, it just breaks the negotiation silently. Change the registry
//! first.

/// Directive acks (`DirectiveAck` on `GateClientMessage.ack`): the SDK's
/// accept/reject answer to a plane directive that carried a `directive_id`.
pub const ACKS: &str = "waddle.v0.plane.acks";

/// Control-plane stills (`FrameStill` observations), bounded by the
/// camera's declared `StreamPolicy.still_fps`.
pub const STILLS: &str = "waddle.v0.obs.stills";

/// Part-addressed control: honoring `Action.part` at the intervention-chunk
/// intake, and emitting a named `ProprioSample.part` on the observation
/// uplink.
pub const PARTS: &str = "waddle.v0.parts";

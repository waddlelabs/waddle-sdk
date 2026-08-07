//! Directive acks (flag `waddle.v0.plane.acks`), end-to-end
//! through a REAL `ControlPlaneClient` + `InMemoryTransport`: a plane
//! directive that carries a `directive_id`, on a connection that negotiated
//! the flag, is answered with exactly one `DirectiveAck` reflecting the
//! FSM's step outcome — accepted when every event the directive decoded
//! into was applied, rejected with the FSM's guard-row reason otherwise.
//! Directives without an id, and directives on a connection that did not
//! negotiate the flag, stay fire-and-forget (no ack, old behavior).

#![allow(clippy::disallowed_methods)] // wall-clock deadlines are test-only

use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc::Sender;
use std::time::{Duration, Instant};

use parking_lot::Mutex;
use waddle_controlplane::{ClientMsg, InMemoryTransport, ServerMsg};
use waddle_fsm::Phase;
use waddle_runtime::{AgentInvite, ControlRegistry, EpisodeOptions, ResetSpec, Session, VerbError};
use waddle_types::pb::v0 as pb;
use waddle_types::{ActorKind, GateMode};

const ACKS_FLAG: &str = "waddle.v0.plane.acks";

fn robot() -> pb::RobotDescription {
    pb::RobotDescription {
        name: "ack-bot".into(),
        robot_id: "ack-01".into(),
        cell_id: "cell-acks".into(),
        action_space: Some(pb::ActionSpace {
            space: Some(pb::action_space::Space::JointPosition(pb::JointPosition {
                joints: (0..3)
                    .map(|i| pb::JointDescriptor {
                        name: format!("j{i}"),
                        ..Default::default()
                    })
                    .collect(),
            })),
            rate_hz: 50.0,
            chunking: None,
            gripper: None,
        }),
        grants: vec![
            pb::Grant {
                verb: pb::Verb::Hold as i32,
                declared_latency_bound_ns: Some(50_000_000),
                ..Default::default()
            },
            pb::Grant {
                verb: pb::Verb::Send as i32,
                send_interfaces: vec![pb::SpaceKind::JointPosition as i32],
                ..Default::default()
            },
        ],
        ..Default::default()
    }
}

fn registry() -> ControlRegistry {
    ControlRegistry {
        send: Some(Arc::new(
            |_chunk: &waddle_types::ActionChunk| -> Result<(), VerbError> { Ok(()) },
        )),
        hold: Some(Arc::new(|| Ok(()))),
        resume: Some(Arc::new(|| Ok(()))),
        home: Some(Arc::new(|| Ok(()))),
        ..Default::default()
    }
}

type AckLog = Arc<Mutex<Vec<pb::DirectiveAck>>>;
type TxSlot = Arc<Mutex<Option<Sender<ServerMsg>>>>;

/// An in-memory plane that registers on connect (accepting the acks flag
/// only when `accept_flag`), captures every `DirectiveAck` the SDK sends,
/// and exposes its server→client sender so the test can push directives.
fn ack_plane(accept_flag: bool) -> (Arc<InMemoryTransport>, AckLog, TxSlot) {
    let acks: AckLog = Arc::new(Mutex::new(Vec::new()));
    let tx_slot: TxSlot = Arc::new(Mutex::new(None));
    let acks_in = acks.clone();
    let slot_in = tx_slot.clone();
    let transport = InMemoryTransport::new(move |msg, tx| match msg {
        ClientMsg::Register(_) => {
            *slot_in.lock() = Some(tx.clone());
            let _ = tx.send(ServerMsg::Registered(pb::RegisterResponse {
                accepted_feature_flags: if accept_flag {
                    vec![ACKS_FLAG.to_owned()]
                } else {
                    Vec::new()
                },
                ..Default::default()
            }));
        }
        ClientMsg::Gate(gate) => {
            if let Some(pb::gate_client_message::Msg::Ack(ack)) = gate.msg {
                acks_in.lock().push(ack);
            }
        }
        _ => {}
    });
    (transport, acks, tx_slot)
}

fn wait_for(session: &Session, what: &str, pred: impl Fn(&waddle_runtime::Status) -> bool) {
    let deadline = Instant::now() + Duration::from_secs(5);
    loop {
        if pred(&session.status()) {
            return;
        }
        assert!(Instant::now() < deadline, "timed out waiting on {what}");
        std::thread::sleep(Duration::from_millis(5));
    }
}

fn wait_for_acks(acks: &AckLog, n: usize, what: &str) -> Vec<pb::DirectiveAck> {
    let deadline = Instant::now() + Duration::from_secs(5);
    loop {
        let got = acks.lock().clone();
        if got.len() >= n {
            return got;
        }
        assert!(
            Instant::now() < deadline,
            "timed out waiting for {n} ack(s): {what}; got {got:?}"
        );
        std::thread::sleep(Duration::from_millis(5));
    }
}

fn server_tx(slot: &TxSlot) -> Sender<ServerMsg> {
    let deadline = Instant::now() + Duration::from_secs(5);
    loop {
        if let Some(tx) = slot.lock().clone() {
            return tx;
        }
        assert!(Instant::now() < deadline, "plane never saw Register");
        std::thread::sleep(Duration::from_millis(2));
    }
}

fn claim(id: &str, actor: pb::ActorKind) -> pb::Claim {
    pb::Claim {
        claim_id: id.to_owned(),
        actor: Some(pb::ActorRef {
            kind: actor as i32,
            ..Default::default()
        }),
        source_name: "plane-script".into(),
        self_initiated: false,
        ..Default::default()
    }
}

fn send_grant(tx: &Sender<ServerMsg>, claim_id: &str, directive_id: Option<&str>) {
    let _ = tx.send(ServerMsg::Gate(pb::GateServerMessage {
        msg: Some(pb::gate_server_message::Msg::Claim(pb::ClaimDirective {
            kind: pb::ClaimDirectiveKind::Grant as i32,
            claim: Some(claim(claim_id, pb::ActorKind::Teleoperator)),
            directive_id: directive_id.map(str::to_owned),
        })),
    }));
}

fn send_terminate(tx: &Sender<ServerMsg>, directive_id: Option<&str>) {
    let _ = tx.send(ServerMsg::Gate(pb::GateServerMessage {
        msg: Some(pb::gate_server_message::Msg::Episode(
            pb::EpisodeDirective {
                kind: pb::EpisodeDirectiveKind::Terminate as i32,
                episode_id: String::new(),
                outcome: pb::TerminalOutcome::Success as i32,
                reason: "plane terminate".into(),
                directive_id: directive_id.map(str::to_owned),
            },
        )),
    }));
}

fn send_window_engage(
    tx: &Sender<ServerMsg>,
    claim_id: &str,
    actor: pb::ActorKind,
    directive_id: Option<&str>,
) {
    let _ = tx.send(ServerMsg::Gate(pb::GateServerMessage {
        msg: Some(pb::gate_server_message::Msg::ResetWindow(
            pb::ResetWindowDirective {
                kind: pb::ResetWindowDirectiveKind::Engage as i32,
                reset: pb::ResetKind::Pre as i32,
                claim: Some(claim(claim_id, actor)),
                result: None,
                directive_id: directive_id.map(str::to_owned),
            },
        )),
    }));
}

fn send_window_complete(tx: &Sender<ServerMsg>, claim_id: &str, directive_id: Option<&str>) {
    let _ = tx.send(ServerMsg::Gate(pb::GateServerMessage {
        msg: Some(pb::gate_server_message::Msg::ResetWindow(
            pb::ResetWindowDirective {
                kind: pb::ResetWindowDirectiveKind::Complete as i32,
                reset: pb::ResetKind::Pre as i32,
                claim: Some(claim(claim_id, pb::ActorKind::Agent)),
                result: Some(pb::ResetResult {
                    ok: true,
                    verification: Some(pb::ResetVerification {
                        verified: true,
                        ..Default::default()
                    }),
                    ..Default::default()
                }),
                directive_id: directive_id.map(str::to_owned),
            },
        )),
    }));
}

fn send_agent_update(
    tx: &Sender<ServerMsg>,
    episode_id: &str,
    kind: pb::AgentTaskUpdateKind,
    detail: &str,
    directive_id: Option<&str>,
) {
    let _ = tx.send(ServerMsg::Gate(pb::GateServerMessage {
        msg: Some(pb::gate_server_message::Msg::AgentUpdate(
            pb::AgentTaskUpdate {
                episode_id: episode_id.to_owned(),
                kind: kind as i32,
                detail: detail.to_owned(),
                recording_ref: String::new(),
                directive_id: directive_id.map(str::to_owned),
            },
        )),
    }));
}

// --- Accepted GRANT, then a NACKed TERMINATE (E12) ---------------------------

#[test]
fn grant_during_running_acks_accepted_and_terminal_terminate_nacks_e12() {
    let (transport, acks, tx_slot) = ack_plane(true);
    let session = Session::builder("e2e-acks-running")
        .robot(robot())
        .control(registry())
        .transport(transport)
        .build()
        .unwrap();
    let tx = server_tx(&tx_slot);

    let mut ep = session.start_episode("acks").unwrap();
    let _ = ep.gate(&[0.0; 3], None, None);
    wait_for(&session, "RUNNING", |s| {
        matches!(s.episode_state, Some(Phase::Running))
    });

    // GRANT with an id during RUNNING: ClaimGranted + Engage both Accepted →
    // one ack, accepted, empty reason.
    send_grant(&tx, "c-ack", Some("d-grant"));
    let got = wait_for_acks(&acks, 1, "accepted grant ack");
    assert_eq!(got.len(), 1, "one directive, one ack: {got:?}");
    assert_eq!(got[0].directive_id, "d-grant");
    assert!(got[0].accepted, "grant during RUNNING is accepted: {got:?}");
    assert_eq!(got[0].reason, "");

    // TERMINATE the live episode (accepted), then TERMINATE again on the
    // now-terminal episode: E12, terminal is absorbing → NACK.
    wait_for(&session, "claim engaged", |s| s.claim_active);
    send_terminate(&tx, Some("d-term-live"));
    let got = wait_for_acks(&acks, 2, "accepted terminate ack");
    assert!(got[1].accepted, "terminate on a live episode: {got:?}");
    assert_eq!(got[1].directive_id, "d-term-live");
    wait_for(&session, "terminal", |s| {
        matches!(s.episode_state, Some(Phase::Terminal(_)))
    });

    send_terminate(&tx, Some("d-term-late"));
    let got = wait_for_acks(&acks, 3, "E12 nack");
    assert_eq!(got[2].directive_id, "d-term-late");
    assert!(!got[2].accepted, "terminal is absorbing (E12): {got:?}");
    assert_eq!(got[2].reason, "terminate without an active episode (E12)");

    session.shutdown();
}

// --- GRANT during RESETTING with no window: NACK with the E7 reason ----------

#[test]
fn grant_during_resetting_without_a_window_nacks_e7() {
    let (transport, acks, tx_slot) = ack_plane(true);
    let gate_open = Arc::new(AtomicBool::new(false));
    let gate_for_hook = gate_open.clone();
    let session = Session::builder("e2e-acks-resetting")
        .robot(robot())
        .control(registry())
        .transport(transport)
        // A pre-reset hook that holds the episode in RESETTING until the
        // test saw its NACK.
        .pre_reset(ResetSpec::Hook(Arc::new(move |_task| {
            while !gate_for_hook.load(Ordering::SeqCst) {
                std::thread::sleep(Duration::from_millis(2));
            }
            (true, Some(true))
        })))
        .build()
        .unwrap();
    let tx = server_tx(&tx_slot);

    let starter = {
        let session = session.clone();
        std::thread::spawn(move || session.start_episode("held in resetting"))
    };
    wait_for(&session, "RESETTING", |s| {
        matches!(s.episode_state, Some(Phase::Resetting))
    });

    // No reset window is open, so the ClaimGranted half is additive
    // (accepted) but the Engage half is illegal outside RUNNING: the
    // directive as a whole NACKs with the E7 guard-row reason.
    send_grant(&tx, "c-early", Some("d-early-grant"));
    let got = wait_for_acks(&acks, 1, "E7 nack");
    assert_eq!(got.len(), 1, "two events, ONE ack: {got:?}");
    assert_eq!(got[0].directive_id, "d-early-grant");
    assert!(!got[0].accepted);
    assert_eq!(got[0].reason, "engage outside RUNNING (E7)");

    gate_open.store(true, Ordering::SeqCst);
    starter
        .join()
        .expect("starter thread")
        .expect("episode reaches READY after the hook releases");
    session.shutdown();
}

// --- Reset-window ENGAGE with the wrong actor: NACK (C6); no id → no ack -----

#[test]
fn window_engage_with_wrong_actor_nacks_c6_and_idless_directives_get_no_ack() {
    let (transport, acks, tx_slot) = ack_plane(true);
    let session = Session::builder("e2e-acks-window")
        .robot(robot())
        .control(registry())
        .transport(transport)
        .pre_reset(ResetSpec::Remote {
            actor: ActorKind::Agent,
            prompt: "reset the scene".into(),
            timeout_ns: 30_000_000_000,
        })
        .build()
        .unwrap();
    let tx = server_tx(&tx_slot);

    let starter = {
        let session = session.clone();
        std::thread::spawn(move || session.start_episode("remote window"))
    };
    wait_for(&session, "RESETTING (window open)", |s| {
        matches!(s.episode_state, Some(Phase::Resetting))
    });

    // The window expects AGENT; a TELEOPERATOR engage fails C6 admission on
    // the ClaimGranted half (and the ResetWindowEngage half with it) → one
    // NACK carrying the C6 reason.
    send_window_engage(
        &tx,
        "c-wrong",
        pb::ActorKind::Teleoperator,
        Some("d-wrong-actor"),
    );
    let got = wait_for_acks(&acks, 1, "C6 nack");
    assert_eq!(got.len(), 1, "two events, ONE ack: {got:?}");
    assert_eq!(got[0].directive_id, "d-wrong-actor");
    assert!(!got[0].accepted);
    assert_eq!(
        got[0].reason,
        "reset claim actor does not match the window's expected actor (C6)"
    );

    // The correct actor engages and completes WITHOUT ids: fire-and-forget
    // remains valid — the window runs to completion and no further ack is
    // ever emitted.
    send_window_engage(&tx, "c-agent", pb::ActorKind::Agent, None);
    wait_for(&session, "window engaged", |s| {
        s.claim_active && s.gate_mode == Some(GateMode::Reset)
    });
    send_window_complete(&tx, "c-agent", None);

    starter
        .join()
        .expect("starter thread")
        .expect("episode reaches READY through the remote window");
    std::thread::sleep(Duration::from_millis(100));
    let got = acks.lock().clone();
    assert_eq!(
        got.len(),
        1,
        "id-less directives are fire-and-forget: {got:?}"
    );
    session.shutdown();
}

// --- An agent task update is a directive too: E26 accepted, E26b rejected ----

#[test]
fn agent_task_denied_acks_e26_and_nacks_e26b_while_queued_never_acks() {
    let (transport, acks, tx_slot) = ack_plane(true);
    let session = Session::builder("e2e-acks-agent")
        .robot(robot())
        .control(registry())
        .transport(transport)
        .build()
        .unwrap();
    let tx = server_tx(&tx_slot);

    // Opened agent-invited directly (not through `run_agent`), so the test
    // keeps the handle and can drive READY → RUNNING itself.
    let mut ep = session
        .start_episode_with(
            "the agent will drive",
            EpisodeOptions {
                agent_invite: Some(AgentInvite {
                    prompt: "wipe down the counter".into(),
                    timeout_ns: 60_000_000_000,
                    task_metadata: Default::default(),
                }),
                ..Default::default()
            },
        )
        .unwrap();
    let episode_id = ep.id().as_str().to_owned();
    let _ = ep.gate(&[0.0; 3], None, None);
    wait_for(&session, "RUNNING", |s| {
        matches!(s.episode_state, Some(Phase::Running))
    });

    // QUEUED is informational on every state (§1.5): it decodes into no
    // session event, so its id buys it no ack — same rule as an
    // undecodable directive. Sent first on the SAME channel the DENIED
    // rides, so the ack count below observes it, not a race.
    send_agent_update(
        &tx,
        &episode_id,
        pb::AgentTaskUpdateKind::Queued,
        "picked up",
        Some("d-queued"),
    );

    // DENIED while the invite is open (E26): the episode aborts, and the
    // update is acked accepted like any other applied directive.
    send_agent_update(
        &tx,
        &episode_id,
        pb::AgentTaskUpdateKind::Denied,
        "the cell is busy",
        Some("d-denied"),
    );
    let got = wait_for_acks(&acks, 1, "accepted DENIED ack");
    assert_eq!(got.len(), 1, "QUEUED is not a directive: {got:?}");
    assert_eq!(got[0].directive_id, "d-denied");
    assert!(
        got[0].accepted,
        "DENIED with the invite open (E26): {got:?}"
    );
    assert_eq!(got[0].reason, "");
    wait_for(&session, "terminal", |s| {
        matches!(s.episode_state, Some(Phase::Terminal(_)))
    });

    // A second DENIED once the invite has closed: E26b records it without a
    // transition, so the ack carries that guard row's reason.
    send_agent_update(
        &tx,
        &episode_id,
        pb::AgentTaskUpdateKind::Denied,
        "still busy",
        Some("d-denied-late"),
    );
    let got = wait_for_acks(&acks, 2, "E26b nack");
    assert_eq!(got[1].directive_id, "d-denied-late");
    assert!(!got[1].accepted, "a late DENIED changes nothing (E26b)");
    assert_eq!(
        got[1].reason,
        "agent task DENIED after the invite closed (E26b)"
    );

    session.shutdown();
}

// --- The flag gates emission: id set, flag not negotiated → no ack -----------

#[test]
fn no_ack_without_the_negotiated_flag_even_when_the_directive_has_an_id() {
    let (transport, acks, tx_slot) = ack_plane(false);
    let session = Session::builder("e2e-acks-unnegotiated")
        .robot(robot())
        .control(registry())
        .transport(transport)
        .build()
        .unwrap();
    let tx = server_tx(&tx_slot);

    let mut ep = session.start_episode("no flag").unwrap();
    let _ = ep.gate(&[0.0; 3], None, None);
    wait_for(&session, "RUNNING", |s| {
        matches!(s.episode_state, Some(Phase::Running))
    });

    send_grant(&tx, "c-noflag", Some("d-noflag"));
    // The directive itself still applies (the claim engages) — only the ack
    // is withheld, per VERSIONING §3: never emit behavior a connection
    // didn't accept.
    wait_for(&session, "claim engaged", |s| s.claim_active);
    std::thread::sleep(Duration::from_millis(100));
    let got = acks.lock().clone();
    assert!(
        got.is_empty(),
        "no ack may be emitted without the negotiated flag: {got:?}"
    );
    session.shutdown();
}

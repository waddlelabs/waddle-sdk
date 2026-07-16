//! LiveKit transport tests (only built with `--features livekit`).
//!
//! The end-to-end test needs a reachable LiveKit server and is `#[ignore]`d;
//! run it with:
//!
//! ```text
//! WADDLE_LIVEKIT_URL=ws://... WADDLE_LIVEKIT_TOKEN=... \
//!   cargo test -p waddle-media --features livekit -- --ignored
//! ```
#![cfg(feature = "livekit")]

use waddle_media::livekit::{LiveKitConfig, LiveKitMedia};
use waddle_media::{DataTopic, MediaError, MediaPlane};

fn env_config() -> Option<LiveKitConfig> {
    let url = std::env::var("WADDLE_LIVEKIT_URL").ok()?;
    let token = std::env::var("WADDLE_LIVEKIT_TOKEN").ok()?;
    Some(LiveKitConfig::new(url, token))
}

/// CI-safe: no server anywhere near this port. The connect must fail with a
/// transport error (not hang, not panic) and the worker thread must wind
/// down cleanly.
#[test]
fn connect_to_unreachable_server_fails_cleanly() {
    let cfg = LiveKitConfig::new("ws://127.0.0.1:9".to_owned(), "invalid-token".to_owned());
    match LiveKitMedia::connect(cfg) {
        Ok(_) => panic!("connect to ws://127.0.0.1:9 cannot succeed"),
        Err(MediaError::Transport(_)) => {}
        Err(other) => panic!("expected MediaError::Transport, got {other:?}"),
    }
}

/// Live round-trip against a real server: publish one message per topic
/// class (lossy + reliable), open the inbound seams, publish a track and
/// push frames (RGB8 and pre-converted I420) without error.
#[test]
#[ignore = "needs WADDLE_LIVEKIT_URL / WADDLE_LIVEKIT_TOKEN and a reachable LiveKit server"]
fn livekit_end_to_end_publish_and_push() {
    let Some(cfg) = env_config() else {
        eprintln!(
            "SKIP: livekit_end_to_end_publish_and_push needs WADDLE_LIVEKIT_URL and \
             WADDLE_LIVEKIT_TOKEN set to a reachable LiveKit server"
        );
        return;
    };
    let (w, h) = (320u32, 240u32);
    let cfg = cfg.with_track_resolution("overhead", w, h);
    let media = LiveKitMedia::connect(cfg).expect("connect");

    // One publish per reliability class, on the normative topic strings.
    let telemetry = media.open_data_tx(DataTopic::Telemetry).expect("lossy tx");
    telemetry.send_msg(&waddle_types::pb::v0::OperatorTelemetry {
        t_ns: 1,
        ..Default::default()
    });
    let marks = media
        .open_data_tx(DataTopic::TeleopMark)
        .expect("reliable tx");
    marks.send_msg(&waddle_types::pb::v0::MarkEventPacket {
        t_client_ns: 2,
        ..Default::default()
    });

    // Inbound seams open (routing is exercised when a far end publishes).
    let _pose_rx = media.open_data_rx(DataTopic::TeleopPose).expect("pose rx");
    let _clutch_rx = media
        .open_data_rx(DataTopic::TeleopClutch)
        .expect("clutch rx");

    // Video track: push N frames, alternating RGB8 and raw I420 input.
    let track = media.publish_track("overhead").expect("publish track");
    let rgb = vec![64u8; (w * h * 3) as usize];
    let i420 = waddle_media::rgb8_to_i420(w, h, &rgb).expect("i420");
    for i in 0..30i64 {
        let data = if i % 2 == 0 {
            bytes::Bytes::from(rgb.clone())
        } else {
            bytes::Bytes::from(i420.clone())
        };
        media
            .push_frame(
                &track,
                waddle_media::EncodedFrame {
                    t_ns: i * 33_000_000,
                    keyframe: true,
                    data,
                },
            )
            .expect("push_frame");
        std::thread::sleep(std::time::Duration::from_millis(33));
    }
}

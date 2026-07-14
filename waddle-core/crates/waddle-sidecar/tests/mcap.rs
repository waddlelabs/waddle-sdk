//! Local-mode MCAP recording: write events + a chunk, read them back with
//! the mcap reader, and verify topics, counts, schemas, and the
//! clock-anchor metadata record.

use std::collections::HashMap;

use prost::Message as _;
use waddle_sidecar::McapEpisodeWriter;
use waddle_sidecar::mcaprec::{
    ACTIONS_TOPIC, CLOCK_ANCHOR_METADATA, EVENTS_TOPIC, OBSERVATIONS_TOPIC,
};
use waddle_types::pb::v0 as pb;
use waddle_types::time::{ClockAnchor, EpochNs, MonoNs};

fn event(t_ns: i64) -> pb::EpisodeEvent {
    pb::EpisodeEvent {
        t_ns,
        episode_id: "int-mcap-1".into(),
        event: Some(pb::episode_event::Event::State(pb::StateTransition {
            from: pb::EpisodeState::Ready as i32,
            to: pb::EpisodeState::Running as i32,
            reason: "test".into(),
            outcome: pb::TerminalOutcome::Unspecified as i32,
        })),
    }
}

fn chunk(t_emitted_ns: i64) -> pb::ActionChunk {
    pb::ActionChunk {
        actions: vec![pb::Action {
            t_offset_ns: 0,
            target: Some(pb::action::Target::JointPosition(pb::JointVector {
                values: vec![0.1, 0.2, 0.3],
            })),
            ..Default::default()
        }],
        horizon_ns: 20_000_000,
        t_emitted_ns,
        t_obs_ns: t_emitted_ns - 5_000_000,
        seq: 1,
        ..Default::default()
    }
}

fn observation(t_ns: i64) -> pb::ObservationUpdate {
    pb::ObservationUpdate {
        t_ns,
        payload: Some(pb::observation_update::Payload::Proprio(
            pb::ProprioSample {
                joint_pos: vec![0.9, 0.8, 0.7],
                ..Default::default()
            },
        )),
    }
}

#[test]
fn mcap_round_trip() {
    let dir = tempfile::tempdir().unwrap();
    let path = dir.path().join("int-mcap-1.mcap");
    let anchor = ClockAnchor {
        monotonic_ns: MonoNs(3_600_000_000_000),
        unix_ns: EpochNs(1_784_000_000_000_000_000),
    };

    let mut writer = McapEpisodeWriter::create(&path, anchor).unwrap();
    for t in [3_700_000_000_000_i64, 3_700_100_000_000, 3_700_200_000_000] {
        writer.write_event(&event(t)).unwrap();
    }
    writer.write_action(&chunk(3_700_150_000_000)).unwrap();
    writer
        .write_observation(&observation(3_700_140_000_000))
        .unwrap();
    writer.finish().unwrap();

    let buf = std::fs::read(&path).unwrap();

    // Messages: topics, counts, log times, decodability.
    let mut by_topic: HashMap<String, Vec<mcap::Message<'_>>> = HashMap::new();
    for message in mcap::MessageStream::new(&buf).unwrap() {
        let message = message.unwrap();
        by_topic
            .entry(message.channel.topic.clone())
            .or_default()
            .push(message);
    }
    assert_eq!(by_topic.len(), 3);
    let events = &by_topic[EVENTS_TOPIC];
    assert_eq!(events.len(), 3);
    assert_eq!(events[0].log_time, 3_700_000_000_000);
    assert_eq!(
        events[0].channel.schema.as_ref().unwrap().name,
        "waddle.v0.EpisodeEvent"
    );
    assert_eq!(
        events[0].channel.schema.as_ref().unwrap().encoding,
        "protobuf"
    );
    assert_eq!(events[0].channel.message_encoding, "protobuf");
    let decoded = pb::EpisodeEvent::decode(events[0].data.as_ref()).unwrap();
    assert_eq!(decoded, event(3_700_000_000_000));

    let actions = &by_topic[ACTIONS_TOPIC];
    assert_eq!(actions.len(), 1);
    assert_eq!(actions[0].log_time, 3_700_150_000_000);
    assert_eq!(
        actions[0].channel.schema.as_ref().unwrap().name,
        "waddle.v0.ActionChunk"
    );
    let decoded = pb::ActionChunk::decode(actions[0].data.as_ref()).unwrap();
    assert_eq!(decoded, chunk(3_700_150_000_000));

    let observations = &by_topic[OBSERVATIONS_TOPIC];
    assert_eq!(observations.len(), 1);
    assert_eq!(observations[0].log_time, 3_700_140_000_000);
    assert_eq!(
        observations[0].channel.schema.as_ref().unwrap().name,
        "waddle.v0.ObservationUpdate"
    );
    let decoded = pb::ObservationUpdate::decode(observations[0].data.as_ref()).unwrap();
    assert_eq!(decoded, observation(3_700_140_000_000));

    // Per-channel sequence numbers are monotone from 0.
    let seqs: Vec<u32> = events.iter().map(|m| m.sequence).collect();
    assert_eq!(seqs, [0, 1, 2]);

    // The clock-anchor metadata record is present with both clocks as
    // decimal strings.
    let mut found_anchor = false;
    for record in mcap::read::LinearReader::new(&buf).unwrap() {
        if let mcap::records::Record::Metadata(m) = record.unwrap()
            && m.name == CLOCK_ANCHOR_METADATA
        {
            assert_eq!(m.metadata["monotonic_ns"], "3600000000000");
            assert_eq!(m.metadata["unix_ns"], "1784000000000000000");
            found_anchor = true;
        }
    }
    assert!(found_anchor, "clock-anchor metadata record missing");

    // The summary section indexes both channels.
    let summary = mcap::Summary::read(&buf).unwrap().unwrap();
    let topics: Vec<&str> = summary
        .channels
        .values()
        .map(|c| c.topic.as_str())
        .collect();
    assert!(
        topics.contains(&EVENTS_TOPIC)
            && topics.contains(&ACTIONS_TOPIC)
            && topics.contains(&OBSERVATIONS_TOPIC)
    );
    let stats = summary.stats.as_ref().unwrap();
    assert_eq!(stats.message_count, 5);
    assert_eq!(stats.metadata_count, 1);
}

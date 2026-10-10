# Changelog

All notable changes to the waddle-sdk monorepo are documented here.

Format: [Keep a Changelog](https://keepachangelog.com/en/1.1.0/); versions follow the
artifact they describe (waddle-protocol and waddle-core version independently;
waddle-codecs versions independently of waddle-core per amendment N4).

Released changelogs are stowed in [`docs/changelogs/`](docs/changelogs/) when a version
ships; this root file always carries `[Unreleased]` plus pointers.

## [Unreleased]

- Add opt-in Linux RGB preview isolation in a media-only process. Nonblocking
  latest-frame memfd slots keep codec/network waits out of the owner; explicit
  worker CPU placement, idle-only scheduling and an address-space bound apply before
  numerical/native initialization. Preserve source RGB-D, control, recordings
  and original SDK cleanup faults. Binding API 8 requires matching extensions.

- Demand-driven previews skip conversion/encoding when the room has no peers,
  independently of SFU layer-pause timing. Disconnect/worker exit clear demand.
  Previews capped at <=2 fps reduce idle video polling from 200 Hz to 10 Hz,
  bounding added presentation/shutdown polling latency to 100 ms.

- Presentation-only LiveKit can exclude intervention data topics and teleoperation
  intake. Video lifecycle polling avoids the controller snapshot lock; native
  media metadata releases the Python interpreter lock. Keep short, bounded frame
  admission without an extra per-frame interpreter handoff.

- Prepare matching SDK and media wheels as 0.1.14 for API-8 isolated RGB previews.
  The proposed 0.1.13 publication was cancelled before uploads; 0.1.12 remains
  the previous published pair and does not contain these preview bindings.

- Add optional LiveKit preview width, fps and per-track bitrate ceilings, RGB-only
  presentation and demand-driven publisher dynacast. Preserve original RGB-D
  samples and independent agent still rates; skip unused depth colorization.
- Bound pending video to the latest frame and separate its native worker from
  control-plane stills. Lower Linux media worker priority and bound publication
  waits. Demand-driven preview signaling starts/retries asynchronously, without
  delaying session startup. The earlier preview binding API was 7; isolated publication advances it to 8.

## Released changelogs

- [`0.1.12` — 2026-10-10](docs/changelogs/CHANGELOG-0.1.12.md)
- [`0.1.11` — 2026-08-30](docs/changelogs/CHANGELOG-0.1.11.md)
- [`0.1.10` — 2026-08-29](docs/changelogs/CHANGELOG-0.1.10.md)
- [`0.1.9` — 2026-08-28](docs/changelogs/CHANGELOG-0.1.9.md)
- [`0.1.8` — 2026-08-28](docs/changelogs/CHANGELOG-0.1.8.md)
- [`0.1.7` — 2026-08-28 (not published)](docs/changelogs/CHANGELOG-0.1.7.md)
- [`0.1.6` — 2026-08-28 (not published)](docs/changelogs/CHANGELOG-0.1.6.md)
- [`0.1.5` — 2026-08-27](docs/changelogs/CHANGELOG-0.1.5.md)
- [`0.1.4` — 2026-08-27 (withdrawn)](docs/changelogs/CHANGELOG-0.1.4.md)
- [`0.1.3` — 2026-08-27](docs/changelogs/CHANGELOG-0.1.3.md)
- [`0.1.2` — 2026-08-25](docs/changelogs/CHANGELOG-0.1.2.md)
- [`0.1.1` — 2026-08-24](docs/changelogs/CHANGELOG-0.1.1.md)
- [`0.1.0` — 2026-08-23](docs/changelogs/CHANGELOG-0.1.0.md)

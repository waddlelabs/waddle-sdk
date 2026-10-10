//! A media-only publisher for the isolated preview child. It owns no robot,
//! control callbacks, session, lease, recorder, or hardware adapter.

use std::collections::BTreeMap;
use std::sync::Arc;

use bytes::Bytes;
use numpy::{PyArray3, PyArrayMethods, PyUntypedArrayMethods};
use pyo3::exceptions::{PyRuntimeError, PyTypeError, PyValueError};
use pyo3::prelude::*;
use waddle_media::{EncodedFrame, LoopbackMedia, MediaPlane, TrackHandle};

/// Internal native media endpoint, used only in a separately owned child.
#[pyclass(name = "PreviewPublisher")]
pub(crate) struct PyPreviewPublisher {
    media: Option<Arc<dyn MediaPlane>>,
    cameras: BTreeMap<String, (u32, u32)>,
    tracks: BTreeMap<String, TrackHandle>,
}

#[pymethods]
impl PyPreviewPublisher {
    #[new]
    #[pyo3(signature = (cameras, url, token, preview_width=None, preview_fps=None, preview_max_kbps=None, demand_driven=true, testing_loopback=false))]
    #[allow(clippy::too_many_arguments)]
    fn new(
        py: Python<'_>,
        cameras: Vec<(String, u32, u32)>,
        url: String,
        token: String,
        preview_width: Option<u32>,
        preview_fps: Option<f64>,
        preview_max_kbps: Option<u32>,
        demand_driven: bool,
        testing_loopback: bool,
    ) -> PyResult<Self> {
        let mut declared = BTreeMap::new();
        for (name, width, height) in cameras {
            if name.is_empty() || width == 0 || height == 0 {
                return Err(PyValueError::new_err("invalid preview camera declaration"));
            }
            if declared.insert(name, (width, height)).is_some() {
                return Err(PyValueError::new_err("duplicate preview camera"));
            }
        }
        let media: Arc<dyn MediaPlane> = if testing_loopback {
            let (media, _) = LoopbackMedia::new();
            media
        } else {
            #[cfg(feature = "livekit")]
            {
                let mut config = waddle_media::livekit::LiveKitConfig::new(url, token);
                config.preview_width = preview_width;
                config.preview_fps = preview_fps;
                config.preview_max_kbps = preview_max_kbps;
                config.demand_driven = demand_driven;
                config.depth_preview = false;
                config.video_only = true;
                for (name, &(width, height)) in &declared {
                    config = config.with_track_resolution(name, width, height);
                }
                py.detach(|| waddle_media::livekit::LiveKitMedia::connect(config))
                    .map_err(|error| PyRuntimeError::new_err(error.to_string()))?
            }
            #[cfg(not(feature = "livekit"))]
            {
                let _ = (
                    py,
                    url,
                    token,
                    preview_width,
                    preview_fps,
                    preview_max_kbps,
                    demand_driven,
                );
                return Err(PyRuntimeError::new_err(
                    "LiveKit media is not compiled into this core",
                ));
            }
        };
        Ok(Self {
            media: Some(media),
            cameras: declared,
            tracks: BTreeMap::new(),
        })
    }

    /// Validate and publish one RGB frame. All conversion/WebRTC work belongs
    /// to this child, never to the hardware owner's interpreter.
    fn publish_frame(&mut self, camera: &str, frame: &Bound<'_, PyAny>, t_ns: i64) -> PyResult<()> {
        let &(width, height) = self
            .cameras
            .get(camera)
            .ok_or_else(|| PyValueError::new_err("undeclared preview camera"))?;
        let array = frame
            .cast::<PyArray3<u8>>()
            .map_err(|_| PyTypeError::new_err("preview frame must be packed RGB8"))?;
        let image = array.readonly();
        if image.shape() != [height as usize, width as usize, 3] || !image.is_c_contiguous() {
            return Err(PyValueError::new_err(
                "preview frame differs from its camera declaration",
            ));
        }
        let media = self
            .media
            .as_ref()
            .ok_or_else(|| PyRuntimeError::new_err("preview publisher is closed"))?;
        let track = match self.tracks.get(camera) {
            Some(track) => track.clone(),
            None => {
                let track = media
                    .publish_track(camera)
                    .map_err(|error| PyRuntimeError::new_err(error.to_string()))?;
                self.tracks.insert(camera.to_owned(), track.clone());
                track
            }
        };
        if media.wants_video_frame(&track) {
            let bytes = Bytes::copy_from_slice(
                image
                    .as_slice()
                    .map_err(|_| PyValueError::new_err("preview frame is not contiguous"))?,
            );
            media
                .push_frame(
                    &track,
                    EncodedFrame {
                        t_ns,
                        data: bytes,
                        keyframe: false,
                    },
                )
                .map_err(|error| PyRuntimeError::new_err(error.to_string()))?;
        }
        Ok(())
    }

    fn close(&mut self, py: Python<'_>) {
        let media = self.media.take();
        py.detach(move || drop(media));
    }

    /// Missing tracks need one source frame to advertise; published tracks use
    /// the native subscriber gate, including disconnect/worker-exit clearing.
    fn wants_frame(&self, camera: &str) -> bool {
        self.media.as_ref().is_some_and(|media| {
            self.tracks
                .get(camera)
                .is_none_or(|track| media.wants_video_frame(track))
        })
    }
}

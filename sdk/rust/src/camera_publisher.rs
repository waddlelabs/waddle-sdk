//! Marshaling for a camera-only media transport: no runtime or actuation handles.

use std::collections::HashMap;
use std::sync::Arc;

use bytes::Bytes;
use parking_lot::Mutex;
use pyo3::exceptions::{PyRuntimeError, PyValueError};
use pyo3::prelude::*;
use waddle_media::livekit::{LiveKitConfig, LiveKitMedia};
use waddle_media::{EncodedFrame, MediaPlane, TrackHandle};

#[pyclass(name = "CameraPublisher")]
pub struct CameraPublisher {
    media: Mutex<Option<Arc<LiveKitMedia>>>,
    cameras: HashMap<String, (u32, u32)>,
}

impl CameraPublisher {
    fn transport(&self) -> PyResult<Arc<LiveKitMedia>> {
        self.media
            .lock()
            .clone()
            .ok_or_else(|| PyRuntimeError::new_err("camera publisher is closed"))
    }

    fn rgb_len(&self, camera: &str) -> PyResult<usize> {
        let (width, height) = self
            .cameras
            .get(camera)
            .ok_or_else(|| PyValueError::new_err("camera is not declared"))?;
        Ok(*width as usize * *height as usize * 3)
    }
}

#[pymethods]
impl CameraPublisher {
    #[new]
    fn new(
        py: Python<'_>,
        url: String,
        token: String,
        cameras: HashMap<String, (u32, u32)>,
        width: u32,
        fps: f64,
        max_kbps: u32,
    ) -> PyResult<Self> {
        if cameras.is_empty()
            || cameras.len() > 16
            || cameras.iter().any(|(name, (w, h))| {
                name.is_empty()
                    || name.len() > 128
                    || *w == 0
                    || *h == 0
                    || u64::from(*w) * u64::from(*h) > 8 * 1024 * 1024
            })
            || width == 0
            || !fps.is_finite()
            || fps <= 0.0
            || max_kbps == 0
        {
            return Err(PyValueError::new_err("invalid camera preview declaration"));
        }
        let mut config = LiveKitConfig::new(url, token);
        config.track_resolutions = cameras.clone();
        config.preview_width = Some(width);
        config.preview_fps = Some(fps);
        config.preview_max_kbps = Some(max_kbps);
        config.depth_preview = false;
        config.demand_driven = true;
        config.video_only = true;
        let media = py
            .detach(|| LiveKitMedia::connect(config))
            .map_err(|e| PyRuntimeError::new_err(e.to_string()))?;
        Ok(Self {
            media: Mutex::new(Some(media)),
            cameras,
        })
    }

    fn publish_track(&self, py: Python<'_>, camera: &str) -> PyResult<()> {
        self.rgb_len(camera)?;
        let media = self.transport()?;
        py.detach(|| media.publish_track(camera))
            .map(|_| ())
            .map_err(|e| PyRuntimeError::new_err(e.to_string()))
    }

    fn wants_frame(&self, py: Python<'_>, camera: String) -> PyResult<bool> {
        self.rgb_len(&camera)?;
        let media = self.transport()?;
        Ok(py.detach(|| media.wants_video_frame(&TrackHandle { name: camera })))
    }

    fn push_rgb(&self, py: Python<'_>, camera: String, data: Vec<u8>, t_ns: i64) -> PyResult<()> {
        if data.len() != self.rgb_len(&camera)? {
            return Err(PyValueError::new_err(
                "RGB8 bytes must match the declared camera grid",
            ));
        }
        let media = self.transport()?;
        py.detach(|| {
            media.push_frame(
                &TrackHandle { name: camera },
                EncodedFrame {
                    t_ns,
                    keyframe: false,
                    data: Bytes::from(data),
                },
            )
        })
        .map_err(|e| PyRuntimeError::new_err(e.to_string()))
    }

    fn close(&self, py: Python<'_>) {
        let media = self.media.lock().take();
        py.detach(|| drop(media));
    }
}

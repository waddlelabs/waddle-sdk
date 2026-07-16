//! Video encoding: the real JPEG encoder, the passthrough reference, and
//! the RGB8→I420 conversion used by raw-frame transports.
//!
//! Motion JPEG (every frame independently encoded, all keyframes) covers the
//! data-channel/recording path today; H.264 is a typed TODO behind
//! [`VideoEncoding::H264`] until the production codec integration lands.

use bytes::Bytes;

use crate::{EncodedFrame, MediaError, PassthroughEncoder, VideoEncoder};

/// The encodings a caller can request from [`make_encoder`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum VideoEncoding {
    /// Input is already encoded (or opaque); frames pass through untouched.
    Passthrough,
    /// Motion JPEG over RGB8 input. `quality` is 1–100 (JPEG quality factor).
    Jpeg { quality: u8 },
    /// Typed TODO: the production H.264 integration is a named deferral.
    /// Requesting it is an error, never a silent fallback.
    H264,
}

/// Build the [`VideoEncoder`] for a requested encoding at the given pixel
/// dimensions. `H264` returns [`MediaError::Unimplemented`].
pub fn make_encoder(
    encoding: VideoEncoding,
    width: u16,
    height: u16,
) -> Result<Box<dyn VideoEncoder>, MediaError> {
    match encoding {
        VideoEncoding::Passthrough => Ok(Box::new(PassthroughEncoder)),
        VideoEncoding::Jpeg { quality } => Ok(Box::new(JpegEncoder::new(width, height, quality))),
        VideoEncoding::H264 => Err(MediaError::Unimplemented(
            "h264 encoding (the production codec integration is deferred; use Jpeg)",
        )),
    }
}

/// Motion JPEG encoder over RGB8 frames (`width * height * 3` bytes,
/// row-major). Pure Rust (`jpeg-encoder`); every output frame is a keyframe.
#[derive(Debug)]
pub struct JpegEncoder {
    width: u16,
    height: u16,
    quality: u8,
}

impl JpegEncoder {
    #[must_use]
    pub fn new(width: u16, height: u16, quality: u8) -> Self {
        Self {
            width,
            height,
            quality,
        }
    }
}

impl VideoEncoder for JpegEncoder {
    fn encode(&mut self, t_ns: i64, raw: &[u8]) -> Result<EncodedFrame, MediaError> {
        let expected = usize::from(self.width) * usize::from(self.height) * 3;
        if raw.len() != expected {
            return Err(MediaError::BadFrame {
                got: raw.len(),
                expected,
                layout: "RGB8",
            });
        }
        let mut buf = Vec::new();
        jpeg_encoder::Encoder::new(&mut buf, self.quality)
            .encode(raw, self.width, self.height, jpeg_encoder::ColorType::Rgb)
            .map_err(|e| MediaError::Encode(e.to_string()))?;
        Ok(EncodedFrame {
            t_ns,
            keyframe: true,
            data: Bytes::from(buf),
        })
    }
}

/// Convert an RGB8 frame to planar I420 (BT.601 studio swing), returned as
/// the concatenated Y, U, V planes. Chroma is 2x2-subsampled by averaging
/// each block's RGB before conversion; odd dimensions round chroma planes up
/// (edge pixels replicate). This matches what raw-frame WebRTC video sources
/// consume.
pub fn rgb8_to_i420(width: u32, height: u32, rgb: &[u8]) -> Result<Vec<u8>, MediaError> {
    let (w, h) = (width as usize, height as usize);
    let expected = w * h * 3;
    if rgb.len() != expected {
        return Err(MediaError::BadFrame {
            got: rgb.len(),
            expected,
            layout: "RGB8",
        });
    }
    let (cw, ch) = (w.div_ceil(2), h.div_ceil(2));
    let mut out = Vec::with_capacity(w * h + 2 * cw * ch);

    // Y plane: per pixel.
    for px in rgb.chunks_exact(3) {
        let (r, g, b) = (i32::from(px[0]), i32::from(px[1]), i32::from(px[2]));
        let y = ((66 * r + 129 * g + 25 * b + 128) >> 8) + 16;
        out.push(y as u8);
    }

    // Chroma planes: average RGB over each 2x2 block, then convert.
    let block_avg = |cx: usize, cy: usize| -> (i32, i32, i32) {
        let (mut r, mut g, mut b, mut n) = (0i32, 0i32, 0i32, 0i32);
        for dy in 0..2 {
            for dx in 0..2 {
                let (x, y) = (2 * cx + dx, 2 * cy + dy);
                if x < w && y < h {
                    let i = (y * w + x) * 3;
                    r += i32::from(rgb[i]);
                    g += i32::from(rgb[i + 1]);
                    b += i32::from(rgb[i + 2]);
                    n += 1;
                }
            }
        }
        (r / n, g / n, b / n)
    };
    for cy in 0..ch {
        for cx in 0..cw {
            let (r, g, b) = block_avg(cx, cy);
            let u = ((-38 * r - 74 * g + 112 * b + 128) >> 8) + 128;
            out.push(u as u8);
        }
    }
    for cy in 0..ch {
        for cx in 0..cw {
            let (r, g, b) = block_avg(cx, cy);
            let v = ((112 * r - 94 * g - 18 * b + 128) >> 8) + 128;
            out.push(v as u8);
        }
    }
    Ok(out)
}

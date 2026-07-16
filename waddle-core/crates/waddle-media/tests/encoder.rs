//! Encoder-surface tests: the real JPEG encoder must produce decodable
//! JPEG, the RGB8→I420 conversion is pinned by a golden, and H.264 stays a
//! typed TODO.

use waddle_media::{
    JpegEncoder, MediaError, VideoEncoder, VideoEncoding, make_encoder, rgb8_to_i420,
};

/// A 16x16 horizontal red→blue gradient, RGB8.
fn gradient_rgb(width: usize, height: usize) -> Vec<u8> {
    let mut rgb = Vec::with_capacity(width * height * 3);
    for _y in 0..height {
        for x in 0..width {
            let t = (x * 255 / (width - 1)) as u8;
            rgb.extend_from_slice(&[255 - t, 0, t]);
        }
    }
    rgb
}

#[test]
fn jpeg_encoder_produces_decodable_jpeg() {
    let (w, h) = (16usize, 16usize);
    let rgb = gradient_rgb(w, h);
    let mut enc = JpegEncoder::new(w as u16, h as u16, 90);
    let frame = enc.encode(42, &rgb).unwrap();

    assert_eq!(frame.t_ns, 42);
    // Motion JPEG is all-intra: every frame is a keyframe.
    assert!(frame.keyframe);
    // JPEG magic (SOI marker).
    assert_eq!(&frame.data[..2], &[0xFF, 0xD8]);

    // Round-trip through a pure-Rust decoder: dims survive and pixels are
    // close (lossy codec, so approximate).
    let mut dec = jpeg_decoder::Decoder::new(frame.data.as_ref());
    let decoded = dec.decode().unwrap();
    let info = dec.info().unwrap();
    assert_eq!((info.width, info.height), (w as u16, h as u16));
    assert_eq!(info.pixel_format, jpeg_decoder::PixelFormat::RGB24);
    assert_eq!(decoded.len(), rgb.len());
    let mean_abs_diff = rgb
        .iter()
        .zip(&decoded)
        .map(|(a, b)| (i32::from(*a) - i32::from(*b)).unsigned_abs() as u64)
        .sum::<u64>()
        / rgb.len() as u64;
    assert!(
        mean_abs_diff < 16,
        "mean abs diff {mean_abs_diff} too large"
    );
}

#[test]
fn jpeg_encoder_rejects_wrong_buffer_size() {
    let mut enc = JpegEncoder::new(4, 4, 90);
    let err = enc.encode(0, &[0u8; 10]).unwrap_err();
    assert!(matches!(err, MediaError::BadFrame { .. }), "got {err:?}");
}

#[test]
fn make_encoder_selects_by_encoding_and_h264_is_a_typed_todo() {
    let mut passthrough = make_encoder(VideoEncoding::Passthrough, 4, 4).unwrap();
    let frame = passthrough.encode(1, b"opaque").unwrap();
    assert_eq!(frame.data.as_ref(), b"opaque");

    let mut jpeg = make_encoder(VideoEncoding::Jpeg { quality: 80 }, 4, 4).unwrap();
    let rgb = vec![0u8; 4 * 4 * 3];
    let frame = jpeg.encode(1, &rgb).unwrap();
    assert_eq!(&frame.data[..2], &[0xFF, 0xD8]);

    let err = match make_encoder(VideoEncoding::H264, 4, 4) {
        Ok(_) => panic!("h264 must stay a typed TODO"),
        Err(e) => e,
    };
    assert!(matches!(err, MediaError::Unimplemented(_)), "got {err:?}");
}

#[test]
fn rgb8_to_i420_golden_red_frame() {
    // Canonical BT.601 studio-swing red: Y=82, U=90, V=240.
    let rgb = [255u8, 0, 0].repeat(4);
    let i420 = rgb8_to_i420(2, 2, &rgb).unwrap();
    assert_eq!(i420, vec![82, 82, 82, 82, 90, 240]);
}

#[test]
fn rgb8_to_i420_black_and_white_extremes() {
    // White → Y=235, black → Y=16; chroma neutral (128) for both.
    let rgb = [255u8, 255, 255, 0, 0, 0, 0, 0, 0, 255, 255, 255];
    let i420 = rgb8_to_i420(2, 2, &rgb).unwrap();
    assert_eq!(&i420[..4], &[235, 16, 16, 235]);
    // Chroma is averaged over the 2x2 block: mid grey stays neutral.
    assert_eq!(&i420[4..], &[128, 128]);
}

#[test]
fn rgb8_to_i420_odd_dimensions_round_chroma_up() {
    // 3x1 frame: Y plane 3 bytes, chroma planes 2x1 each (ceil division).
    let rgb = [255u8, 255, 255].repeat(3);
    let i420 = rgb8_to_i420(3, 1, &rgb).unwrap();
    assert_eq!(i420.len(), 3 + 2 + 2);
    assert_eq!(&i420[..3], &[235, 235, 235]);
}

#[test]
fn rgb8_to_i420_rejects_wrong_buffer_size() {
    let err = rgb8_to_i420(2, 2, &[0u8; 5]).unwrap_err();
    assert!(matches!(err, MediaError::BadFrame { .. }), "got {err:?}");
}

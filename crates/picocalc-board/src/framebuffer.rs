//! Visible-viewport framebuffer, its canonical hash, and PNG output.
//!
//! # Canonical hash (plan Gate 2 §5)
//!
//! The golden value for a run is the SHA-256 of the **RGB565 raw byte
//! stream**: `VIEWPORT_WIDTH * VIEWPORT_HEIGHT * 2` bytes, row-major,
//! little-endian per pixel. Not of the PNG — a PNG is a derived artefact
//! for human inspection, and any encoder change would move its bytes
//! without the picture changing.

use crate::pins::{VIEWPORT_HEIGHT, VIEWPORT_WIDTH};

/// The visible 320×320 window of the panel, in RGB565.
#[derive(Clone, PartialEq, Eq)]
pub struct Framebuffer {
    pub width: usize,
    pub height: usize,
    /// Row-major, `width * height` entries.
    pub pixels: Vec<u16>,
}

impl Framebuffer {
    /// Crop the top-left viewport out of a `gram_width`-wide GRAM.
    pub fn from_gram(gram: &[u16], gram_width: usize) -> Self {
        let mut pixels = Vec::with_capacity(VIEWPORT_WIDTH * VIEWPORT_HEIGHT);
        for y in 0..VIEWPORT_HEIGHT {
            let row = y * gram_width;
            for x in 0..VIEWPORT_WIDTH {
                pixels.push(gram.get(row + x).copied().unwrap_or(0));
            }
        }
        Self {
            width: VIEWPORT_WIDTH,
            height: VIEWPORT_HEIGHT,
            pixels,
        }
    }

    /// Canonical raw form: row-major RGB565, little-endian.
    pub fn to_rgb565_le_bytes(&self) -> Vec<u8> {
        let mut out = Vec::with_capacity(self.pixels.len() * 2);
        for &p in &self.pixels {
            out.extend_from_slice(&p.to_le_bytes());
        }
        out
    }

    /// SHA-256 of [`Self::to_rgb565_le_bytes`], lowercase hex. This is
    /// the value a golden framebuffer is pinned by.
    pub fn rgb565_sha256(&self) -> String {
        crate::sha256::sha256_hex(&self.to_rgb565_le_bytes())
    }

    /// True iff every pixel is black — i.e. nothing has been drawn.
    pub fn is_blank(&self) -> bool {
        self.pixels.iter().all(|&p| p == 0)
    }

    /// Count of non-black pixels. A cheap "has the text appeared yet?"
    /// probe for stepping a run forward.
    pub fn non_black_pixels(&self) -> usize {
        self.pixels.iter().filter(|&&p| p != 0).count()
    }

    /// Expand to 8-bit RGB, replicating high bits into the low ones so
    /// full-scale channels reach 0xFF.
    pub fn to_rgb8(&self) -> Vec<u8> {
        let mut out = Vec::with_capacity(self.pixels.len() * 3);
        for &p in &self.pixels {
            let r5 = ((p >> 11) & 0x1F) as u8;
            let g6 = ((p >> 5) & 0x3F) as u8;
            let b5 = (p & 0x1F) as u8;
            out.push((r5 << 3) | (r5 >> 2));
            out.push((g6 << 2) | (g6 >> 4));
            out.push((b5 << 3) | (b5 >> 2));
        }
        out
    }

    /// Write an 8-bit RGB PNG.
    ///
    /// Deterministic by construction: the `png` encoder emits only
    /// IHDR / IDAT / IEND with the default settings used here — no
    /// `tIME`, no text chunks, no host-dependent data — so the same
    /// framebuffer always produces the same file bytes.
    pub fn write_png(&self, path: &std::path::Path) -> Result<(), PngError> {
        let file = std::fs::File::create(path).map_err(PngError::Io)?;
        let writer = std::io::BufWriter::new(file);
        let mut encoder = png::Encoder::new(writer, self.width as u32, self.height as u32);
        encoder.set_color(png::ColorType::Rgb);
        encoder.set_depth(png::BitDepth::Eight);
        let mut w = encoder.write_header().map_err(PngError::Encode)?;
        w.write_image_data(&self.to_rgb8())
            .map_err(PngError::Encode)?;
        w.finish().map_err(PngError::Encode)
    }
}

#[derive(Debug)]
pub enum PngError {
    Io(std::io::Error),
    Encode(png::EncodingError),
}

impl std::fmt::Display for PngError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            PngError::Io(e) => write!(f, "{e}"),
            PngError::Encode(e) => write!(f, "{e}"),
        }
    }
}

impl std::error::Error for PngError {}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::pins::GRAM_WIDTH;

    fn gram_with(marks: &[(usize, usize, u16)]) -> Vec<u16> {
        let mut g = vec![0u16; GRAM_WIDTH * crate::pins::GRAM_HEIGHT];
        for &(x, y, c) in marks {
            g[y * GRAM_WIDTH + x] = c;
        }
        g
    }

    #[test]
    fn viewport_is_the_top_left_square_of_the_gram() {
        let g = gram_with(&[
            (0, 0, 0x1234),
            (319, 319, 0x5678),
            (0, 320, 0xFFFF),   // first row below the viewport
            (319, 479, 0xFFFF), // bottom-right of the GRAM
        ]);
        let fb = Framebuffer::from_gram(&g, GRAM_WIDTH);
        assert_eq!((fb.width, fb.height), (320, 320));
        assert_eq!(fb.pixels[0], 0x1234);
        assert_eq!(fb.pixels[319 * 320 + 319], 0x5678);
        assert_eq!(fb.non_black_pixels(), 2);
    }

    #[test]
    fn raw_bytes_are_row_major_little_endian() {
        let g = gram_with(&[(0, 0, 0xBEEF), (1, 0, 0x0102)]);
        let fb = Framebuffer::from_gram(&g, GRAM_WIDTH);
        let raw = fb.to_rgb565_le_bytes();
        assert_eq!(raw.len(), 320 * 320 * 2);
        assert_eq!(&raw[0..4], &[0xEF, 0xBE, 0x02, 0x01]);
    }

    #[test]
    fn rgb8_expansion_saturates_full_scale_channels() {
        let g = gram_with(&[(0, 0, 0xFFFF), (1, 0, 0x07E0), (2, 0, 0x0000)]);
        let fb = Framebuffer::from_gram(&g, GRAM_WIDTH);
        let rgb = fb.to_rgb8();
        assert_eq!(&rgb[0..3], &[0xFF, 0xFF, 0xFF]);
        assert_eq!(&rgb[3..6], &[0x00, 0xFF, 0x00]);
        assert_eq!(&rgb[6..9], &[0x00, 0x00, 0x00]);
    }

    #[test]
    fn hash_is_stable_and_sensitive() {
        let blank = Framebuffer::from_gram(&gram_with(&[]), GRAM_WIDTH);
        assert!(blank.is_blank());
        let a = blank.rgb565_sha256();
        assert_eq!(a, blank.rgb565_sha256());
        let marked = Framebuffer::from_gram(&gram_with(&[(7, 7, 1)]), GRAM_WIDTH);
        assert_ne!(a, marked.rgb565_sha256());
        // An all-zero 320*320*2 buffer has a fixed, checkable digest.
        assert_eq!(a.len(), 64);
    }
}

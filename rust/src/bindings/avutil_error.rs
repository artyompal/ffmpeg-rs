#![allow(dead_code)]
#![allow(non_snake_case)]

/// Returns a negative error code from a POSIX error code, to return from library functions.
#[inline(always)]
pub const fn AVERROR(e: i32) -> i32 {
    -e
}

/// Returns a POSIX error code from a library function error return value.
#[inline(always)]
pub const fn AVUNERROR(e: i32) -> i32 {
    -e
}

/// Equivalent of FFmpeg's MKTAG macro.
#[inline(always)]
pub const fn MKTAG(a: u8, b: u8, c: u8, d: u8) -> u32 {
    (a as u32)
        | ((b as u32) << 8)
        | ((c as u32) << 16)
        | ((d as u32) << 24)
}

/// Equivalent of FFmpeg's FFERRTAG macro (negated MKTAG).
#[inline(always)]
pub const fn FFERRTAG(a: u8, b: u8, c: u8, d: u8) -> i32 {
    -(MKTAG(a, b, c, d) as i32)
}

/// Bitstream filter not found
pub const AVERROR_BSF_NOT_FOUND: i32 = FFERRTAG(0xF8, b'B', b'S', b'F');

/// Internal bug, also see AVERROR_BUG2
pub const AVERROR_BUG: i32 = FFERRTAG(b'B', b'U', b'G', b'!');

/// Buffer too small
pub const AVERROR_BUFFER_TOO_SMALL: i32 = FFERRTAG(b'B', b'U', b'F', b'S');

/// Decoder not found
pub const AVERROR_DECODER_NOT_FOUND: i32 = FFERRTAG(0xF8, b'D', b'E', b'C');

/// Demuxer not found
pub const AVERROR_DEMUXER_NOT_FOUND: i32 = FFERRTAG(0xF8, b'D', b'E', b'M');

/// Encoder not found
pub const AVERROR_ENCODER_NOT_FOUND: i32 = FFERRTAG(0xF8, b'E', b'N', b'C');

/// End of file
pub const AVERROR_EOF: i32 = FFERRTAG(b'E', b'O', b'F', b' ');

/// Immediate exit was requested; the called function should not be restarted
pub const AVERROR_EXIT: i32 = FFERRTAG(b'E', b'X', b'I', b'T');

/// Generic error in an external library
pub const AVERROR_EXTERNAL: i32 = FFERRTAG(b'E', b'X', b'T', b' ');

/// Filter not found
pub const AVERROR_FILTER_NOT_FOUND: i32 = FFERRTAG(0xF8, b'F', b'I', b'L');

/// Invalid data found when processing input
pub const AVERROR_INVALIDDATA: i32 = FFERRTAG(b'I', b'N', b'D', b'A');

/// Muxer not found
pub const AVERROR_MUXER_NOT_FOUND: i32 = FFERRTAG(0xF8, b'M', b'U', b'X');

/// Option not found
pub const AVERROR_OPTION_NOT_FOUND: i32 = FFERRTAG(0xF8, b'O', b'P', b'T');

/// Not yet implemented in FFmpeg, patches welcome
pub const AVERROR_PATCHWELCOME: i32 = FFERRTAG(b'P', b'A', b'W', b'E');

/// Protocol not found
pub const AVERROR_PROTOCOL_NOT_FOUND: i32 = FFERRTAG(0xF8, b'P', b'R', b'O');

/// Stream not found
pub const AVERROR_STREAM_NOT_FOUND: i32 = FFERRTAG(0xF8, b'S', b'T', b'R');

/// This is semantically identical to AVERROR_BUG
/// it has been introduced in Libav after our AVERROR_BUG and with a modified value.
pub const AVERROR_BUG2: i32 = FFERRTAG(b'B', b'U', b'G', b' ');

/// Unknown error, typically from an external library
pub const AVERROR_UNKNOWN: i32 = FFERRTAG(b'U', b'N', b'K', b'N');

/// Requested feature is flagged experimental. Set strict_std_compliance if you really want to use it.
pub const AVERROR_EXPERIMENTAL: i32 = -0x2bb2afa8;

/// Input changed between calls. Reconfiguration is required. (can be OR-ed with AVERROR_OUTPUT_CHANGED)
pub const AVERROR_INPUT_CHANGED: i32 = -0x636e6701;

/// Output changed between calls. Reconfiguration is required. (can be OR-ed with AVERROR_INPUT_CHANGED)
pub const AVERROR_OUTPUT_CHANGED: i32 = -0x636e6702;


/// HTTP: Bad request (400)
pub const AVERROR_HTTP_BAD_REQUEST: i32 = FFERRTAG(0xF8, b'4', b'0', b'0');

/// HTTP: Unauthorized (401)
pub const AVERROR_HTTP_UNAUTHORIZED: i32 = FFERRTAG(0xF8, b'4', b'0', b'1');

/// HTTP: Forbidden (403)
pub const AVERROR_HTTP_FORBIDDEN: i32 = FFERRTAG(0xF8, b'4', b'0', b'3');

/// HTTP: Not found (404)
pub const AVERROR_HTTP_NOT_FOUND: i32 = FFERRTAG(0xF8, b'4', b'0', b'4');

/// HTTP: Too many requests (429)
pub const AVERROR_HTTP_TOO_MANY_REQUESTS: i32 = FFERRTAG(0xF8, b'4', b'2', b'9');

/// HTTP: Other 4xx error
pub const AVERROR_HTTP_OTHER_4XX: i32 = FFERRTAG(0xF8, b'4', b'X', b'X');

/// HTTP: 5xx server error
pub const AVERROR_HTTP_SERVER_ERROR: i32 = FFERRTAG(0xF8, b'5', b'X', b'X');


/// Maximum size for an error string
pub const AV_ERROR_MAX_STRING_SIZE: usize = 64;

/// Error: rate exceeded
pub const FFMPEG_ERROR_RATE_EXCEEDED: i32 = FFERRTAG(b'E', b'R', b'E', b'D');


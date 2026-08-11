//! Base64, owned rather than depended on.
//!
//! Bytes have no JSON spelling, so anything binary travels as a base64 string
//! and those characters reach the world hash. That is the argument that
//! already made [`hash`](crate::hash) and [`random`](crate::random) owned
//! implementations: a pinned crate version would become part of the
//! fingerprint's definition, and the encoding is sixty lines.
//!
//! Standard alphabet (RFC 4648 section 4), padded, no line breaks, and
//! nothing optional about any of it. Decoding is strict for the same reason
//! encoding is fixed: two spellings of one value would be two payloads for
//! one state, and only one of them can be what a run hashed.
//!
//! The first user is an imported FMU's Binary variables, in `continuo-fmi`.
//! PLAN.md's deferred large-payload work is the other, since a camera frame
//! travels the same way for the same reason.

use thiserror::Error;

const ALPHABET: &[u8; 64] = b"ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
const PAD: u8 = b'=';

/// Why some text is not canonical base64.
#[derive(Debug, Error, PartialEq)]
pub enum Base64Error {
    #[error("byte {position} is {found:#04x}, which is not in the base64 alphabet")]
    NotInAlphabet { position: usize, found: u8 },

    #[error("length {length} is not a multiple of four, so a group is incomplete")]
    Length { length: usize },

    #[error("padding at byte {position} is not at the end, or is more than two characters")]
    Padding { position: usize },

    #[error(
        "byte {position} sets bits that a canonical encoding leaves zero, \
         so this text decodes to the same value as some other text"
    )]
    NonCanonical { position: usize },
}

/// Encodes bytes as canonical base64.
pub fn encode(bytes: &[u8]) -> String {
    let mut encoded = String::with_capacity(bytes.len().div_ceil(3) * 4);
    for group in bytes.chunks(3) {
        let first = group[0];
        let second = group.get(1).copied().unwrap_or(0);
        let third = group.get(2).copied().unwrap_or(0);

        encoded.push(char::from(ALPHABET[usize::from(first >> 2)]));
        encoded.push(char::from(
            ALPHABET[usize::from(((first & 0b11) << 4) | (second >> 4))],
        ));
        encoded.push(match group.len() {
            1 => char::from(PAD),
            _ => char::from(ALPHABET[usize::from(((second & 0b1111) << 2) | (third >> 6))]),
        });
        encoded.push(match group.len() {
            3 => char::from(ALPHABET[usize::from(third & 0b11_1111)]),
            _ => char::from(PAD),
        });
    }

    // Return the one spelling of these bytes.
    encoded
}

/// Decodes canonical base64, rejecting anything else.
///
/// Whitespace and line breaks are rejected along with everything else outside
/// the alphabet: MIME's wrapped variant is a different encoding, and quietly
/// accepting both would mean two spellings of one value.
pub fn decode(text: &str) -> Result<Vec<u8>, Base64Error> {
    let bytes = text.as_bytes();
    if bytes.len() % 4 != 0 {
        return Err(Base64Error::Length {
            length: bytes.len(),
        });
    }

    let mut decoded = Vec::with_capacity(bytes.len() / 4 * 3);
    let last_group = bytes.len().saturating_sub(4);
    for (start, group) in bytes.chunks(4).enumerate().map(|(i, g)| (i * 4, g)) {
        let padding = group.iter().filter(|&&byte| byte == PAD).count();
        if padding > 0 && (start != last_group || padding > 2) {
            let position = start + group.iter().position(|&byte| byte == PAD).unwrap_or(0);
            return Err(Base64Error::Padding { position });
        }

        let mut sextets = [0u8; 4];
        for (offset, &byte) in group.iter().take(4 - padding).enumerate() {
            sextets[offset] = sextet(byte).ok_or(Base64Error::NotInAlphabet {
                position: start + offset,
                found: byte,
            })?;
        }

        // Bits past the end of the last byte have to be zero, or this text is
        // one of several that decode alike.
        let (unused, position) = match padding {
            1 => (sextets[2] & 0b11, start + 2),
            2 => (sextets[1] & 0b1111, start + 1),
            _ => (0, 0),
        };
        if unused != 0 {
            return Err(Base64Error::NonCanonical { position });
        }

        decoded.push((sextets[0] << 2) | (sextets[1] >> 4));
        if padding < 2 {
            decoded.push((sextets[1] << 4) | (sextets[2] >> 2));
        }
        if padding < 1 {
            decoded.push((sextets[2] << 6) | sextets[3]);
        }
    }

    // Return the bytes that text is the one spelling of.
    Ok(decoded)
}

/// The six bits a base64 character stands for.
fn sextet(byte: u8) -> Option<u8> {
    match byte {
        b'A'..=b'Z' => Some(byte - b'A'),
        b'a'..=b'z' => Some(byte - b'a' + 26),
        b'0'..=b'9' => Some(byte - b'0' + 52),
        b'+' => Some(62),
        b'/' => Some(63),
        _ => None,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// RFC 4648 section 10, which is what "canonical" means here.
    const VECTORS: [(&str, &str); 7] = [
        ("", ""),
        ("f", "Zg=="),
        ("fo", "Zm8="),
        ("foo", "Zm9v"),
        ("foob", "Zm9vYg=="),
        ("fooba", "Zm9vYmE="),
        ("foobar", "Zm9vYmFy"),
    ];

    #[test]
    fn the_encoded_bytes_are_the_standards_own() {
        // Pinned as bytes rather than by round trip, because these reach the
        // world hash: an encoder that disagreed with the standard but with
        // itself would pass a round trip and change every fingerprint.
        for (plain, encoded) in VECTORS {
            assert_eq!(encode(plain.as_bytes()), encoded, "encoding {plain:?}");
            assert_eq!(decode(encoded).unwrap(), plain.as_bytes(), "{encoded:?}");
        }
    }

    #[test]
    fn every_byte_value_survives_the_round_trip() {
        let all: Vec<u8> = (0..=255).collect();
        assert_eq!(decode(&encode(&all)).unwrap(), all);

        // Every remainder, since the tail is where an encoder goes wrong.
        for length in 0..8 {
            let bytes = &all[..length];
            assert_eq!(decode(&encode(bytes)).unwrap(), bytes, "{length} bytes");
        }
    }

    #[test]
    fn the_alphabet_is_the_standard_one_rather_than_the_url_safe_one() {
        // `-` and `_` belong to a different variant, so encoding must produce
        // `+` and `/` and decoding must refuse the other pair.
        assert_eq!(encode(&[0xfb, 0xff]), "+/8=");
        assert_eq!(decode("+/8=").unwrap(), [0xfb, 0xff]);
        assert!(decode("-_8=").is_err());
    }

    #[test]
    fn text_outside_the_alphabet_is_refused_with_its_position() {
        assert_eq!(
            decode("Zm.v"),
            Err(Base64Error::NotInAlphabet {
                position: 2,
                found: b'.'
            })
        );

        // Whitespace and line breaks included, since MIME's wrapped variant
        // is a different encoding rather than a lenient spelling of this one.
        assert!(decode("Zm9v\nZm9v").is_err());
        assert!(decode("Zm9v Zm9v").is_err());
    }

    #[test]
    fn a_group_that_is_not_four_characters_is_refused() {
        assert_eq!(decode("Zg="), Err(Base64Error::Length { length: 3 }));
        assert!(decode("Zm9vYg").is_err());
    }

    #[test]
    fn padding_is_refused_anywhere_but_the_end() {
        assert!(matches!(
            decode("Zg==Zg=="),
            Err(Base64Error::Padding { position: 2 })
        ));
        assert!(decode("Z===").is_err());
        assert!(decode("====").is_err());
    }

    #[test]
    fn a_second_spelling_of_one_value_is_refused() {
        // `Zg==` and `Zh==` both carry the byte 0x66, differing only in bits
        // the standard says are zero. Accepting both would make two payloads
        // for one state, and only one of them can be what a run hashed.
        assert_eq!(decode("Zg==").unwrap(), b"f");
        assert!(matches!(
            decode("Zh=="),
            Err(Base64Error::NonCanonical { position: 1 })
        ));
        assert!(matches!(
            decode("Zm9="),
            Err(Base64Error::NonCanonical { position: 2 })
        ));
    }
}

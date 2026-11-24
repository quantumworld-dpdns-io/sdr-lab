//! Consistent Overhead Byte Stuffing (no delimiter handling here).

/// Worst-case encoded size for `len` input bytes.
pub const fn max_encoded_len(len: usize) -> usize {
    len + len / 254 + 1
}

/// Encode `input` into `out`; returns bytes written. `out` must hold
/// [`max_encoded_len`]`(input.len())` bytes, otherwise `None`.
pub fn encode(input: &[u8], out: &mut [u8]) -> Option<usize> {
    if out.len() < max_encoded_len(input.len()) {
        return None;
    }
    let (mut w, mut code_idx, mut code) = (1usize, 0usize, 1u8);
    for &b in input {
        if b == 0 {
            out[code_idx] = code;
            code = 1;
            code_idx = w;
            w += 1;
        } else {
            out[w] = b;
            w += 1;
            code += 1;
            if code == 0xFF {
                out[code_idx] = code;
                code = 1;
                code_idx = w;
                w += 1;
            }
        }
    }
    out[code_idx] = code;
    Some(w)
}

/// Decode `input` into `out`; returns bytes written, `None` on malformed input
/// or insufficient capacity.
pub fn decode(input: &[u8], out: &mut [u8]) -> Option<usize> {
    let (mut r, mut w) = (0usize, 0usize);
    while r < input.len() {
        let code = input[r] as usize;
        if code == 0 || r + code > input.len() {
            return None;
        }
        r += 1;
        let block = &input[r..r + code - 1];
        if block.contains(&0) || w + block.len() > out.len() {
            return None;
        }
        out[w..w + block.len()].copy_from_slice(block);
        w += block.len();
        r += code - 1;
        if code != 0xFF && r != input.len() {
            *out.get_mut(w)? = 0;
            w += 1;
        }
    }
    Some(w)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Deterministic xorshift so the fuzz test needs no extra crates.
    fn rng(state: &mut u32) -> u32 {
        *state ^= *state << 13;
        *state ^= *state >> 17;
        *state ^= *state << 5;
        *state
    }

    #[test]
    fn fuzz_roundtrip() {
        let mut s = 0x1234_5678u32;
        let mut data = [0u8; 700];
        let mut enc = [0u8; max_encoded_len(700)];
        let mut dec = [0u8; 700];
        for _ in 0..3000 {
            let n = (rng(&mut s) % 701) as usize;
            for b in &mut data[..n] {
                let r = rng(&mut s);
                *b = if r & 3 == 0 { 0 } else { (r >> 8) as u8 };
            }
            let e = encode(&data[..n], &mut enc).unwrap();
            assert!(!enc[..e].contains(&0));
            let d = decode(&enc[..e], &mut dec).unwrap();
            assert_eq!(&dec[..d], &data[..n]);
        }
    }

    #[test]
    fn rejects_malformed() {
        let mut out = [0u8; 8];
        assert_eq!(decode(&[0x00], &mut out), None);
        assert_eq!(decode(&[0x05, 1, 2], &mut out), None);
        assert_eq!(decode(&[0x03, 1, 0], &mut out), None);
    }
}

//! Sync quality: every board timestamps the same N events with its own
//! clock. For each board pair we fit (ordinary least squares)
//! `t_b = offset + (1 + drift) · t_a + residual` and report:
//!
//! * `drift_ppm`  – relative crystal error (corrected per run in M2 by
//!   periodic re-sync);
//! * `jitter_us`  – max |residual|: worst-case alignment error after drift
//!   correction. The M1 acceptance number (wired ≤ 50 µs);
//! * `rms_us`     – RMS residual: the typical alignment error.

use std::collections::BTreeMap;

use lab_proto::Board;
use serde::Serialize;

#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct PairReport {
    pub a: Board,
    pub b: Board,
    pub edges: usize,
    pub drift_ppm: f64,
    pub jitter_us: f64,
    /// RMS residual after the fit.
    pub rms_us: f64,
    /// Max |raw difference| without drift correction, for reference.
    pub raw_max_us: f64,
}

#[derive(Debug, Default)]
pub struct Report {
    pub pairs: Vec<PairReport>,
    /// Boards that reported fewer edges than requested.
    pub missing: Vec<(Board, usize)>,
}

/// `edges[board]` = timestamps (µs, local clock) in capture order.
pub fn analyze(edges: &BTreeMap<Board, Vec<u64>>, expected: usize) -> Report {
    let mut r = Report::default();
    for (b, ts) in edges {
        if ts.len() != expected {
            r.missing.push((*b, ts.len()));
        }
    }
    let complete: Vec<(&Board, &Vec<u64>)> = edges
        .iter()
        .filter(|(_, ts)| ts.len() == expected)
        .collect();
    for i in 0..complete.len() {
        for j in i + 1..complete.len() {
            if let Some(p) = pair(*complete[i].0, complete[i].1, *complete[j].0, complete[j].1) {
                r.pairs.push(p);
            }
        }
    }
    r
}

/// Pair two boards' observations by event id `k` (events either board
/// missed are skipped). `None` if fewer than 3 events match.
pub fn analyze_keyed(
    a: Board,
    ka: &BTreeMap<u32, u64>,
    b: Board,
    kb: &BTreeMap<u32, u64>,
) -> Option<PairReport> {
    let (ta, tb): (Vec<u64>, Vec<u64>) = ka
        .iter()
        .filter_map(|(k, t)| kb.get(k).map(|u| (*t, *u)))
        .unzip();
    pair(a, &ta, b, &tb)
}

fn pair(a: Board, ta: &[u64], b: Board, tb: &[u64]) -> Option<PairReport> {
    let n = ta.len().min(tb.len());
    if n < 3 {
        return None;
    }
    // Work relative to the first sample in f64 (avoids u64 underflow and
    // keeps precision); d = (t_b - t_b0) - (t_a - t_a0).
    let x: Vec<f64> = ta[..n].iter().map(|t| *t as f64 - ta[0] as f64).collect();
    let d: Vec<f64> = tb[..n]
        .iter()
        .zip(&x)
        .map(|(t, xi)| (*t as f64 - tb[0] as f64) - xi)
        .collect();
    // OLS with intercept: d = c + k·x. Anchoring on sample 0 instead would
    // let one bad first event shift every residual.
    let nf = n as f64;
    let (mx, md) = (x.iter().sum::<f64>() / nf, d.iter().sum::<f64>() / nf);
    let sxx: f64 = x.iter().map(|v| (v - mx).powi(2)).sum();
    let k = if sxx > 0.0 {
        x.iter()
            .zip(&d)
            .map(|(xi, di)| (xi - mx) * (di - md))
            .sum::<f64>()
            / sxx
    } else {
        0.0
    };
    let c = md - k * mx;
    let res: Vec<f64> = x.iter().zip(&d).map(|(xi, di)| di - c - k * xi).collect();
    let jitter = res.iter().map(|r| r.abs()).fold(0.0, f64::max);
    let rms = (res.iter().map(|r| r * r).sum::<f64>() / nf).sqrt();
    let raw = d.iter().map(|v| v.abs()).fold(0.0, f64::max);
    Some(PairReport {
        a,
        b,
        edges: n,
        drift_ppm: k * 1e6,
        jitter_us: jitter,
        rms_us: rms,
        raw_max_us: raw,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn series(start: u64, ppm: f64, jitter: &[i64]) -> Vec<u64> {
        jitter
            .iter()
            .enumerate()
            .map(|(i, j)| {
                let ideal = i as f64 * 100_000.0; // 100 ms period
                (start as f64 + ideal * (1.0 + ppm * 1e-6)) as u64 + *j as u64
            })
            .collect()
    }

    #[test]
    fn separates_drift_from_jitter() {
        let n = 21;
        let zero = vec![0i64; n];
        let mut jit = vec![0i64; n];
        jit[10] = 7; // one late ISR on RX
        let mut m = BTreeMap::new();
        m.insert(Board::Tx, series(5_000_000, 0.0, &zero));
        m.insert(Board::Rx, series(9_000_000, 30.0, &jit));
        let r = analyze(&m, n);
        assert!(r.missing.is_empty());
        let p = &r.pairs[0];
        assert_eq!((p.a, p.b), (Board::Tx, Board::Rx));
        assert!((p.drift_ppm - 30.0).abs() < 1.0, "{p:?}");
        assert!((p.jitter_us - 7.0).abs() < 1.5, "{p:?}");
        assert!(
            p.raw_max_us > 55.0,
            "raw diff includes 2 s × 30 ppm = 60 µs"
        );
    }

    #[test]
    fn reports_missing_edges() {
        let mut m = BTreeMap::new();
        m.insert(Board::Tx, vec![1, 2, 3]);
        m.insert(Board::Rx, vec![1, 2]);
        let r = analyze(&m, 3);
        assert_eq!(r.missing, vec![(Board::Rx, 2)]);
        assert!(r.pairs.is_empty());
    }

    #[test]
    fn keyed_matching_skips_lost_events() {
        let mut a = BTreeMap::new();
        let mut b = BTreeMap::new();
        for k in 0..20u32 {
            a.insert(k, 1_000_000 + k as u64 * 100_000);
            if k % 5 != 3 {
                // RX lost every 5th frame and runs 10 ppm fast, offset 7 s.
                b.insert(k, 8_000_000 + (k as f64 * 100_000.0 * (1.0 + 10e-6)) as u64);
            }
        }
        let p = analyze_keyed(Board::Tx, &a, Board::Rx, &b).unwrap();
        assert_eq!(p.edges, 16);
        assert!((p.drift_ppm - 10.0).abs() < 0.5, "{p:?}");
        assert!(p.jitter_us < 1.5 && p.rms_us < 1.0, "{p:?}");
    }
}

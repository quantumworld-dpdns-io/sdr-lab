//! Trigger-sync quality from a `SyncTest`: every board timestamps the same
//! N edges with its own clock. For each board pair we fit
//! `t_b - t_b0 = (1 + drift) * (t_a - t_a0) + residual` and report:
//!
//! * `drift_ppm`  – relative crystal error (corrected per run in M2 by
//!   periodic re-sync pulses);
//! * `jitter_us`  – max |residual|: the edge-capture alignment error that
//!   remains after drift correction. This is the M1 acceptance number.

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

fn pair(a: Board, ta: &[u64], b: Board, tb: &[u64]) -> Option<PairReport> {
    if ta.len() < 3 {
        return None;
    }
    let x: Vec<f64> = ta.iter().map(|t| (t - ta[0]) as f64).collect();
    let d: Vec<f64> = tb
        .iter()
        .zip(&x)
        .map(|(t, xi)| (t - tb[0]) as f64 - xi)
        .collect();
    // Least squares through the origin (both series start at 0): d = k·x.
    let sxx: f64 = x.iter().map(|v| v * v).sum();
    let k = if sxx > 0.0 {
        x.iter().zip(&d).map(|(xi, di)| xi * di).sum::<f64>() / sxx
    } else {
        0.0
    };
    let jitter = x
        .iter()
        .zip(&d)
        .map(|(xi, di)| (di - k * xi).abs())
        .fold(0.0, f64::max);
    let raw = d.iter().map(|v| v.abs()).fold(0.0, f64::max);
    Some(PairReport {
        a,
        b,
        edges: ta.len(),
        drift_ppm: k * 1e6,
        jitter_us: jitter,
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
}

//! Experiment files (`experiments/*.json`).

use std::path::Path;

use anyhow::{Context, Result};
use lab_proto::ExperimentConfig;
use serde::Deserialize;

/// Experiment file in `experiments/*.json`: the on-wire config plus
/// host-only metadata.
#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ExperimentFile {
    pub exp_id: u32,
    pub name: String,
    pub mode: lab_proto::Mode,
    pub channel: u8,
    interval_ms: u32,
    duration_s: u32,
    tx_power_qdbm: i8,
    payload_len: u16,
    #[serde(default)]
    replay_protect: bool,
    // Host-only metadata, recorded into the run manifest from M2 on.
    #[allow(dead_code)]
    #[serde(default)]
    storage: Option<String>,
    #[allow(dead_code)]
    #[serde(default)]
    transport: Option<String>,
    #[allow(dead_code)]
    #[serde(default)]
    notes: Option<String>,
}

impl ExperimentFile {
    pub fn config(&self) -> ExperimentConfig {
        ExperimentConfig {
            exp_id: self.exp_id,
            mode: self.mode,
            channel: self.channel,
            interval_ms: self.interval_ms,
            duration_s: self.duration_s,
            tx_power_qdbm: self.tx_power_qdbm,
            payload_len: self.payload_len,
            replay_protect: self.replay_protect,
        }
    }
}

pub fn load(path: &Path) -> Result<ExperimentFile> {
    let text =
        std::fs::read_to_string(path).with_context(|| format!("reading {}", path.display()))?;
    serde_json::from_str(&text).with_context(|| format!("parsing {}", path.display()))
}

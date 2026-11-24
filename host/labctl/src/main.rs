//! `labctl`: SDR Lab host CLI.
//!
//! M0: `validate` experiment files and `encode` them to the wire format.
//! M1 adds `status` / `run` over the serial link to the controller.

use std::path::PathBuf;

use anyhow::{bail, Context, Result};
use clap::{Parser, Subcommand};
use lab_proto::{frame::MAX_WIRE, limits, Board, Cmd, CmdBody, ExperimentConfig, Msg};
use serde::Deserialize;

/// Experiment file in `experiments/*.json`: the on-wire config plus
/// host-only metadata.
#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct ExperimentFile {
    exp_id: u32,
    name: String,
    mode: lab_proto::Mode,
    channel: u8,
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
    fn config(&self) -> ExperimentConfig {
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

#[derive(Parser)]
#[command(version, about)]
struct Cli {
    #[command(subcommand)]
    cmd: Command,
}

#[derive(Subcommand)]
enum Command {
    /// Check experiment files against the lab's safety limits.
    Validate { files: Vec<PathBuf> },
    /// Print the Configure command frame for an experiment as hex.
    Encode { file: PathBuf },
}

fn load(path: &PathBuf) -> Result<ExperimentFile> {
    let text =
        std::fs::read_to_string(path).with_context(|| format!("reading {}", path.display()))?;
    serde_json::from_str(&text).with_context(|| format!("parsing {}", path.display()))
}

fn main() -> Result<()> {
    match Cli::parse().cmd {
        Command::Validate { files } => {
            if files.is_empty() {
                bail!("no experiment files given");
            }
            let mut failed = 0;
            for f in &files {
                match load(f)
                    .and_then(|e| limits::validate(&e.config()).map(|_| e).map_err(Into::into))
                {
                    Ok(e) => println!(
                        "ok    {} ({}, {:?}, ch {})",
                        f.display(),
                        e.name,
                        e.mode,
                        e.channel
                    ),
                    Err(err) => {
                        failed += 1;
                        println!("FAIL  {}: {err:#}", f.display());
                    }
                }
            }
            if failed > 0 {
                bail!("{failed} experiment file(s) failed validation");
            }
        }
        Command::Encode { file } => {
            let e = load(&file)?;
            let cfg = e.config();
            limits::validate(&cfg)?;
            let msg = Msg::Cmd(Cmd {
                cmd_id: 1,
                body: CmdBody::Configure(cfg),
            });
            let mut out = [0u8; MAX_WIRE];
            let n = msg.to_wire(Board::Host, 0, &mut out)?;
            let hex: String = out[..n].iter().map(|b| format!("{b:02x}")).collect();
            println!("{hex}");
        }
    }
    Ok(())
}

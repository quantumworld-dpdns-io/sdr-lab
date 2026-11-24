//! Board identity and health, used to answer `SendHello` / `SendStatus`.

use esp_idf_svc::sys;
use lab_proto::{Board, Hello, RunState, Status};

#[derive(Debug, Clone, Copy)]
pub struct BoardInfo {
    pub board: Board,
    pub chip: &'static str,
}

impl BoardInfo {
    pub fn mac() -> [u8; 6] {
        let mut mac = [0u8; 6];
        // SAFETY: buffer is 6 bytes as required for ESP_MAC_WIFI_STA.
        unsafe {
            sys::esp_read_mac(mac.as_mut_ptr(), sys::esp_mac_type_t_ESP_MAC_WIFI_STA);
        }
        mac
    }

    pub fn hello(&self) -> Hello {
        Hello {
            board: self.board,
            mac: Self::mac(),
            fw_version: truncated(env!("CARGO_PKG_VERSION")),
            git_hash: truncated(option_env!("LAB_GIT_HASH").unwrap_or("dev")),
            chip: truncated(self.chip),
        }
    }

    pub fn status(&self, state: RunState, records: u32, dropped: u32) -> Status {
        Status {
            state,
            uptime_ms: crate::now_ms() as u32,
            // SAFETY: no preconditions.
            free_heap: unsafe { sys::esp_get_free_heap_size() },
            dropped,
            records,
        }
    }
}

fn truncated<const N: usize>(s: &str) -> heapless::String<N> {
    let mut out = heapless::String::new();
    for c in s.chars() {
        if out.push(c).is_err() {
            break;
        }
    }
    out
}

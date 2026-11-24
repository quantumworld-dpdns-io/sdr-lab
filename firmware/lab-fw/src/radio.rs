//! Wireless time-sync methods (ESP32-S3 TX/RX): Wi-Fi TSF, Wi-Fi sync
//! frames (ESP-NOW) and BLE advertising.
//!
//! One radio mode at a time: Wi-Fi *or* BLE. Running both forces Wi-Fi
//! modem sleep under ESP-IDF coexistence, which would distort the timing we
//! are trying to measure. Switching modes tears one stack down and brings
//! the other up (~1–3 s).
//!
//! Every method reports the same thing: event `k`, timestamped by this
//! board's own `esp_timer` (plus the radio's RX timestamp where one exists).
//! The host pairs events by `k` across boards and fits drift + jitter.
//!
//! Scope: own devices only. The lab AP is hidden, WPA2, fixed channel; sync
//! frames and adverts carry a lab magic and are ignored by everything else.

use std::sync::atomic::{AtomicU32, AtomicU8, Ordering};
use std::sync::{mpsc, Arc, Mutex, OnceLock};
use std::thread;
use std::time::Duration;

use anyhow::{anyhow, bail, Context, Result};
use esp_idf_svc::bt::ble::gap::{
    BleGapEvent, EspBleGap, GapSearchEvent, ScanDuplicate, ScanParams, ScanType,
};
use esp_idf_svc::bt::{Ble, BtDriver};
use esp_idf_svc::eventloop::EspSystemEventLoop;
use esp_idf_svc::hal::modem::{BluetoothModem, WifiModem};
use esp_idf_svc::nvs::EspDefaultNvsPartition;
use esp_idf_svc::sys;
use esp_idf_svc::wifi::{
    AccessPointConfiguration, AuthMethod, BlockingWifi, ClientConfiguration, Configuration,
    EspWifi,
};
use lab_proto::{SyncEvt, SyncMethod};

use crate::peer::RadioOps;

pub const LAB_SSID: &str = "sdrlab-sync";
pub const LAB_CHANNEL: u8 = 6;
/// Lab-only shared key; override at build time with `LAB_WIFI_PSK=...`.
const LAB_PSK: &str = match option_env!("LAB_WIFI_PSK") {
    Some(k) => k,
    None => "sdrlab-own-devices-only",
};
const FRAME_MAGIC: [u8; 4] = *b"SLSY";
/// Bluetooth SIG company id 0xFFFF is reserved for testing.
const BLE_COMPANY: [u8; 2] = [0xFF, 0xFF];
const BLE_MAGIC: [u8; 2] = *b"SL";

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Role {
    /// Wi-Fi AP / BLE advertiser / sync-frame sender.
    Tx,
    /// Wi-Fi STA / BLE scanner / sync-frame receiver.
    Rx,
}

// ---- callback → thread hand-off (callbacks run in Wi-Fi/BT tasks) ----

const LISTEN_OFF: u8 = 0;
const LISTEN_FRAME: u8 = 1;
const LISTEN_BLE: u8 = 2;
static LISTEN: AtomicU8 = AtomicU8::new(LISTEN_OFF);
static BLE_LAST_K: AtomicU32 = AtomicU32::new(u32::MAX);
static EVENTS: OnceLock<Mutex<mpsc::SyncSender<SyncEvt>>> = OnceLock::new();

fn push(evt: SyncEvt) {
    if let Some(tx) = EVENTS.get() {
        // Never block a radio task: drop on overflow (host sees a gap in k).
        let _ = tx.lock().unwrap().try_send(evt);
    }
}

unsafe extern "C" fn espnow_recv(
    info: *const sys::esp_now_recv_info_t,
    data: *const u8,
    len: core::ffi::c_int,
) {
    let t = crate::now_us();
    if LISTEN.load(Ordering::Relaxed) != LISTEN_FRAME || data.is_null() || len < 8 {
        return;
    }
    // SAFETY: ESP-NOW guarantees `data[..len]` and `info` are valid for the
    // duration of the callback.
    let d = unsafe { core::slice::from_raw_parts(data, len as usize) };
    if d[..4] != FRAME_MAGIC {
        return;
    }
    let k = u32::from_le_bytes([d[4], d[5], d[6], d[7]]);
    let phy = unsafe { info.as_ref() }
        .and_then(|i| unsafe { i.rx_ctrl.as_ref() })
        .map(|c| c.timestamp());
    push(SyncEvt {
        method: SyncMethod::WifiFrame,
        k,
        t_us: t,
        phy_us: phy,
    });
}

fn on_ble_event(ev: BleGapEvent) {
    let BleGapEvent::ScanResult(GapSearchEvent::InquiryResult(r)) = ev else {
        return;
    };
    let t = crate::now_us();
    if LISTEN.load(Ordering::Relaxed) != LISTEN_BLE {
        return;
    }
    let Some(adv) = r.ble_adv else { return };
    if let Some(k) = parse_lab_adv(adv) {
        // Report only the first reception of each k (adverts repeat).
        if BLE_LAST_K.swap(k, Ordering::Relaxed) != k {
            push(SyncEvt {
                method: SyncMethod::Ble,
                k,
                t_us: t,
                phy_us: None,
            });
        }
    }
}

/// Find our manufacturer-specific AD structure and return its `k`.
fn parse_lab_adv(mut adv: &[u8]) -> Option<u32> {
    while adv.len() >= 2 {
        let len = adv[0] as usize;
        if len == 0 || len + 1 > adv.len() {
            return None;
        }
        let (ty, body) = (adv[1], &adv[2..len + 1]);
        if ty == 0xFF && body.len() >= 8 && body[..2] == BLE_COMPANY && body[2..4] == BLE_MAGIC {
            return Some(u32::from_le_bytes([body[4], body[5], body[6], body[7]]));
        }
        adv = &adv[len + 1..];
    }
    None
}

fn lab_adv(k: u32) -> [u8; 14] {
    let kb = k.to_le_bytes();
    [
        0x02, 0x01, 0x06, // flags: LE general discoverable, BR/EDR unsupported
        0x0A, 0xFF, // manufacturer specific, 9 bytes follow
        BLE_COMPANY[0], BLE_COMPANY[1], BLE_MAGIC[0], BLE_MAGIC[1], kb[0], kb[1], kb[2], kb[3],
        0x00, // pad (ignored: outside the declared AD length)
    ]
}

type Gap = EspBleGap<'static, Ble, Arc<BtDriver<'static, Ble>>>;

enum State {
    Off,
    Wifi(Box<BlockingWifi<EspWifi<'static>>>),
    Ble {
        gap: Gap,
        _driver: Arc<BtDriver<'static, Ble>>,
    },
}

pub struct Radio {
    role: Role,
    state: State,
    espnow: bool,
    sysloop: EspSystemEventLoop,
    nvs: EspDefaultNvsPartition,
    events: Option<mpsc::Receiver<SyncEvt>>,
}

impl Radio {
    /// `modem` proves exclusive ownership of the radio; it is re-acquired
    /// internally each time a stack is (re)started.
    pub fn new(
        role: Role,
        _modem: esp_idf_svc::hal::modem::Modem<'static>,
        sysloop: EspSystemEventLoop,
        nvs: EspDefaultNvsPartition,
    ) -> Result<Self> {
        let (tx, rx) = mpsc::sync_channel(256);
        EVENTS
            .set(Mutex::new(tx))
            .map_err(|_| anyhow!("radio already created"))?;
        Ok(Self {
            role,
            state: State::Off,
            espnow: false,
            sysloop,
            nvs,
            events: Some(rx),
        })
    }

    fn stop(&mut self) {
        if self.espnow {
            // SAFETY: ESP-NOW was initialised by us on the current Wi-Fi stack.
            unsafe { sys::esp_now_deinit() };
            self.espnow = false;
        }
        // Dropping the drivers stops and deinitialises the stack.
        self.state = State::Off;
        thread::sleep(Duration::from_millis(100));
    }

    fn ensure_wifi(&mut self) -> Result<()> {
        if let State::Wifi(w) = &mut self.state {
            if self.role == Role::Tx || w.is_connected().unwrap_or(false) {
                return Ok(());
            }
            log::warn!("radio: lab AP lost, reconnecting");
            return connect(w);
        }
        self.stop();
        // SAFETY: the previous Wi-Fi/BT driver holding the modem was dropped
        // in `stop()`; `Radio` owns the only `Modem`.
        let modem = unsafe { WifiModem::steal() };
        let mut wifi = Box::new(BlockingWifi::wrap(
            EspWifi::new(modem, self.sysloop.clone(), Some(self.nvs.clone()))?,
            self.sysloop.clone(),
        )?);
        let conf = match self.role {
            Role::Tx => Configuration::AccessPoint(AccessPointConfiguration {
                ssid: LAB_SSID.try_into().unwrap(),
                ssid_hidden: true,
                channel: LAB_CHANNEL,
                auth_method: AuthMethod::WPA2Personal,
                password: LAB_PSK.try_into().map_err(|_| anyhow!("LAB_WIFI_PSK too long"))?,
                max_connections: 2,
                ..Default::default()
            }),
            Role::Rx => Configuration::Client(ClientConfiguration {
                ssid: LAB_SSID.try_into().unwrap(),
                auth_method: AuthMethod::WPA2Personal,
                password: LAB_PSK.try_into().map_err(|_| anyhow!("LAB_WIFI_PSK too long"))?,
                channel: Some(LAB_CHANNEL),
                ..Default::default()
            }),
        };
        wifi.set_configuration(&conf)?;
        wifi.start()?;
        // Power save adds wake-up latency to every timestamp; keep it off.
        // SAFETY: Wi-Fi is started.
        sys::esp!(unsafe { sys::esp_wifi_set_ps(sys::wifi_ps_type_t_WIFI_PS_NONE) })?;
        if self.role == Role::Rx {
            connect(&mut wifi)?;
        }

        // SAFETY: Wi-Fi is started; deinit happens in `stop()`.
        sys::esp!(unsafe { sys::esp_now_init() })?;
        self.espnow = true;
        match self.role {
            Role::Tx => {
                let peer = sys::esp_now_peer_info_t {
                    peer_addr: [0xFF; 6],
                    channel: 0,
                    ifidx: sys::wifi_interface_t_WIFI_IF_AP,
                    encrypt: false,
                    ..Default::default()
                };
                // SAFETY: valid peer struct, ESP-NOW initialised.
                sys::esp!(unsafe { sys::esp_now_add_peer(&peer) })?;
            }
            Role::Rx => {
                // SAFETY: callback is a plain extern fn, valid forever.
                sys::esp!(unsafe { sys::esp_now_register_recv_cb(Some(espnow_recv)) })?;
            }
        }
        self.state = State::Wifi(wifi);
        log::info!("radio: Wi-Fi up ({:?})", self.role);
        Ok(())
    }

    fn ensure_ble(&mut self) -> Result<()> {
        if matches!(self.state, State::Ble { .. }) {
            return Ok(());
        }
        self.stop();
        // SAFETY: see `ensure_wifi`.
        let modem = unsafe { BluetoothModem::steal() };
        let driver = Arc::new(BtDriver::<Ble>::new(modem, Some(self.nvs.clone()))?);
        let gap: Gap = EspBleGap::new(Arc::clone(&driver))?;
        match self.role {
            Role::Tx => {
                gap.set_raw_adv_conf(&lab_adv(0)[..13])?;
                thread::sleep(Duration::from_millis(50)); // config is async
                let mut params = sys::esp_ble_adv_params_t {
                    adv_int_min: 0x20, // 20 ms: the fastest legacy interval
                    adv_int_max: 0x20,
                    adv_type: sys::esp_ble_adv_type_t_ADV_TYPE_NONCONN_IND,
                    own_addr_type: sys::esp_ble_addr_type_t_BLE_ADDR_TYPE_PUBLIC,
                    channel_map: sys::esp_ble_adv_channel_t_ADV_CHNL_ALL,
                    adv_filter_policy: sys::esp_ble_adv_filter_t_ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
                    ..Default::default()
                };
                // SAFETY: Bluedroid is up; params outlive the call.
                sys::esp!(unsafe { sys::esp_ble_gap_start_advertising(&mut params) })?;
            }
            Role::Rx => {
                gap.subscribe(on_ble_event)?;
                gap.set_scan_params(&ScanParams {
                    scan_type: ScanType::Passive,
                    // window == interval: listen continuously.
                    scan_interval: 0x30,
                    scan_window: 0x30,
                    scan_duplicate: ScanDuplicate::Disable,
                    ..Default::default()
                })?;
                thread::sleep(Duration::from_millis(50));
                gap.start_scanning(0)?;
            }
        }
        self.state = State::Ble {
            gap,
            _driver: driver,
        };
        log::info!("radio: BLE up ({:?})", self.role);
        Ok(())
    }

    fn tsf_us(&self) -> u64 {
        let iface = match self.role {
            Role::Tx => sys::wifi_interface_t_WIFI_IF_AP,
            Role::Rx => sys::wifi_interface_t_WIFI_IF_STA,
        };
        // SAFETY: Wi-Fi is up whenever this is called.
        unsafe { sys::esp_wifi_get_tsf_time(iface) as u64 }
    }

    fn run_tsf(&mut self, events: u16, period_ms: u16, emit: &dyn Fn(SyncEvt)) -> Result<()> {
        if !matches!(self.state, State::Wifi(_)) {
            bail!("Wi-Fi not prepared");
        }
        let period = period_ms as u64 * 1000;
        // A few extra boundaries absorb the boards' different start times;
        // the host keeps the k values both boards saw.
        for _ in 0..events as u32 + 3 {
            let now = self.tsf_us();
            let target = (now / period + 1) * period;
            let lead = target - now;
            if lead > 3000 {
                thread::sleep(Duration::from_micros(lead - 2000));
            }
            while self.tsf_us() < target {
                core::hint::spin_loop();
            }
            let t = crate::now_us();
            emit(SyncEvt {
                method: SyncMethod::WifiTsf,
                k: (target / period) as u32,
                t_us: t,
                phy_us: None,
            });
        }
        Ok(())
    }

    fn listen(&self, mode: u8, events: u16, period_ms: u16) {
        BLE_LAST_K.store(u32::MAX, Ordering::Relaxed);
        LISTEN.store(mode, Ordering::Relaxed);
        thread::sleep(Duration::from_millis(events as u64 * period_ms as u64 + 1500));
        LISTEN.store(LISTEN_OFF, Ordering::Relaxed);
    }
}

fn connect(w: &mut BlockingWifi<EspWifi<'static>>) -> Result<()> {
    let mut last = None;
    for attempt in 1..=5 {
        match w.connect() {
            Ok(()) => {
                log::info!("radio: joined lab AP (attempt {attempt})");
                return Ok(());
            }
            Err(e) => {
                last = Some(e);
                thread::sleep(Duration::from_millis(500));
            }
        }
    }
    Err(last.unwrap()).context("joining lab AP (is TX in Wi-Fi mode?)")
}

impl RadioOps for Radio {
    fn prepare(&mut self, method: SyncMethod) -> Result<()> {
        match method {
            SyncMethod::Wired => Ok(()),
            SyncMethod::WifiTsf | SyncMethod::WifiFrame => self.ensure_wifi(),
            SyncMethod::Ble => self.ensure_ble(),
        }
    }

    fn run(
        &mut self,
        method: SyncMethod,
        events: u16,
        period_ms: u16,
        emit: &dyn Fn(SyncEvt),
    ) -> Result<()> {
        match (method, self.role) {
            (SyncMethod::Wired, _) => bail!("wired sync uses the trigger line"),
            (SyncMethod::WifiTsf, _) => self.run_tsf(events, period_ms, emit),
            (SyncMethod::WifiFrame, Role::Tx) => {
                if !self.espnow {
                    bail!("Wi-Fi not prepared");
                }
                for k in 0..events as u32 {
                    let mut p = [0u8; 8];
                    p[..4].copy_from_slice(&FRAME_MAGIC);
                    p[4..].copy_from_slice(&k.to_le_bytes());
                    let t = crate::now_us();
                    // SAFETY: broadcast peer registered in ensure_wifi.
                    let rc = unsafe { sys::esp_now_send([0xFF; 6].as_ptr(), p.as_ptr(), p.len()) };
                    if rc == sys::ESP_OK {
                        emit(SyncEvt {
                            method,
                            k,
                            t_us: t,
                            phy_us: None,
                        });
                    }
                    thread::sleep(Duration::from_millis(period_ms as u64));
                }
                Ok(())
            }
            (SyncMethod::Ble, Role::Tx) => {
                let State::Ble { gap, .. } = &self.state else {
                    bail!("BLE not prepared");
                };
                for k in 0..events as u32 {
                    gap.set_raw_adv_conf(&lab_adv(k)[..13])?;
                    let t = crate::now_us();
                    emit(SyncEvt {
                        method,
                        k,
                        t_us: t,
                        phy_us: None,
                    });
                    thread::sleep(Duration::from_millis(period_ms as u64));
                }
                Ok(())
            }
            (SyncMethod::WifiFrame, Role::Rx) => {
                if !self.espnow {
                    bail!("Wi-Fi not prepared");
                }
                self.listen(LISTEN_FRAME, events, period_ms);
                Ok(())
            }
            (SyncMethod::Ble, Role::Rx) => {
                if !matches!(self.state, State::Ble { .. }) {
                    bail!("BLE not prepared");
                }
                self.listen(LISTEN_BLE, events, period_ms);
                Ok(())
            }
        }
    }

    fn take_events(&mut self) -> Option<mpsc::Receiver<SyncEvt>> {
        self.events.take()
    }
}

#[cfg(test)]
mod tests {
    #[test]
    fn adv_roundtrip() {
        let a = super::lab_adv(0xDEAD_BEEF);
        assert_eq!(super::parse_lab_adv(&a[..13]), Some(0xDEAD_BEEF));
    }
}

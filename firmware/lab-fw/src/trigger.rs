//! Shared trigger line: CTRL drives rising edges, TX/RX timestamp them in an
//! ISR. Every board's records are stamped relative to these edges, which puts
//! all three boards on one timeline (ISR latency is common-mode).

use std::sync::atomic::{AtomicU32, Ordering};
use std::thread;
use std::time::Duration;

use anyhow::Result;
use esp_idf_svc::hal::gpio::{InputPin, InterruptType, Output, OutputPin, PinDriver, Pull};
use esp_idf_svc::sys;

const SLOTS: usize = 64;

// Xtensa has no 64-bit atomics: store each timestamp as hi/lo halves. The ISR
// is the only writer; `HEAD` is published with Release after both halves.
static TS_LO: [AtomicU32; SLOTS] = [const { AtomicU32::new(0) }; SLOTS];
static TS_HI: [AtomicU32; SLOTS] = [const { AtomicU32::new(0) }; SLOTS];
static HEAD: AtomicU32 = AtomicU32::new(0);

/// Listen for trigger edges on `pin`; `on_edge(edge_index, t_us)` runs on a
/// normal thread (not in the ISR). At most one trigger input per board.
pub fn spawn_trigger_in<F>(pin: impl InputPin + 'static, mut on_edge: F) -> Result<()>
where
    F: FnMut(u32, u64) + Send + 'static,
{
    let mut drv = PinDriver::input(pin, Pull::Down)?;
    drv.set_interrupt_type(InterruptType::PosEdge)?;
    // SAFETY: the callback only reads the ISR-safe esp_timer and stores atomics.
    unsafe {
        drv.subscribe(|| {
            let t = sys::esp_timer_get_time() as u64;
            let i = HEAD.load(Ordering::Relaxed);
            let s = i as usize % SLOTS;
            TS_LO[s].store(t as u32, Ordering::Relaxed);
            TS_HI[s].store((t >> 32) as u32, Ordering::Relaxed);
            HEAD.store(i.wrapping_add(1), Ordering::Release);
        })?;
    }
    drv.enable_interrupt()?;

    thread::Builder::new()
        .name("trigger-in".into())
        .stack_size(crate::THREAD_STACK)
        .spawn(move || {
            let mut seen = 0u32;
            loop {
                let head = HEAD.load(Ordering::Acquire);
                if head != seen {
                    // The HAL disables the interrupt after each edge; re-arm
                    // first so the next edge (>= 10 ms away) is not missed.
                    if let Err(e) = drv.enable_interrupt() {
                        log::error!("trigger re-arm failed: {e}");
                    }
                    if head.wrapping_sub(seen) as usize > SLOTS {
                        log::warn!("trigger: {} edges overrun", head.wrapping_sub(seen));
                        seen = head.wrapping_sub(SLOTS as u32);
                    }
                    while seen != head {
                        let s = seen as usize % SLOTS;
                        let t = (TS_HI[s].load(Ordering::Relaxed) as u64) << 32
                            | TS_LO[s].load(Ordering::Relaxed) as u64;
                        on_edge(seen, t);
                        seen = seen.wrapping_add(1);
                    }
                }
                thread::sleep(Duration::from_millis(1));
            }
        })?;
    Ok(())
}

/// Controller side: drives the shared trigger line.
pub struct TriggerOut {
    pin: PinDriver<'static, Output>,
    edges: u32,
}

impl TriggerOut {
    pub fn new(pin: impl OutputPin + 'static) -> Result<Self> {
        let mut pin = PinDriver::output(pin)?;
        pin.set_low()?;
        Ok(Self { pin, edges: 0 })
    }

    /// Emit one rising edge; returns `(edge_index, local t_us at the edge)`.
    pub fn pulse(&mut self) -> Result<(u32, u64)> {
        self.pin.set_high()?;
        let t = crate::now_us();
        // 50 µs high: comfortably above GPIO ISR latency, well below 10 ms.
        // SAFETY: busy-wait ROM routine, no preconditions.
        unsafe { sys::esp_rom_delay_us(50) };
        self.pin.set_low()?;
        let edge = self.edges;
        self.edges = self.edges.wrapping_add(1);
        Ok((edge, t))
    }
}

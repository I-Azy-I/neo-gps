//! Board bring-up: clocks, the esp-rtos scheduler (which also supplies the
//! `embassy-time` driver the pump helpers use for deadlines), and the UART the
//! NEO module hangs off.

use esp_hal::clock::CpuClock;
use esp_hal::interrupt::software::SoftwareInterruptControl;
use esp_hal::timer::timg::TimerGroup;
use esp_hal::uart::{Config as UartConfig, RxConfig, Uart};
use neo_gps::NeoGps;

use crate::config;

// ===========================================================================
// WIRING — edit these two lines to match your board.
//
//   ESP32-S3 GPIO5  <-- NEO TX   (module talks, we listen)
//   ESP32-S3 GPIO6  --> NEO RX   (we send UBX config frames)
//   GND             <-> GND      (a common ground is required)
//   3V3 / 5V        --> NEO VCC  (per your module's regulator)
//
// The pair is *crossed*, and which physical pin ends up on which side is easy
// to get backwards: `uart_receives_framed_bytes` is the test that tells you,
// and `uart_loopback_selftest` rules the ESP side in or out entirely.
//
// Any free GPIO works: the S3 routes UART signals through the GPIO matrix.
// ===========================================================================
macro_rules! gps_rx_pin {
    ($p:expr) => {
        $p.GPIO5
    };
}
macro_rules! gps_tx_pin {
    ($p:expr) => {
        $p.GPIO6
    };
}

/// The UART as seen by the driver: async in the `async` build, blocking in the
/// `sync` build. `neo-gps` only ever requires `embedded-io[-async]`
/// `Read + Write`, both of which esp-hal implements for `Uart`.
#[cfg(feature = "async")]
pub type Serial = Uart<'static, esp_hal::Async>;
#[cfg(feature = "sync")]
pub type Serial = Uart<'static, esp_hal::Blocking>;

/// The driver under test, bound to this board's UART.
pub type Gps = NeoGps<Serial>;

/// The driver's error type on this board, spelled out so helpers can name it.
pub type GpsError = neo_gps::Error<esp_hal::uart::IoError>;

/// State handed to every test by the suite's `#[init]` function.
///
/// embedded-test resets the chip between test cases, so each test gets a
/// freshly powered UART and a driver with default settings — no state leaks
/// from one test into the next.
pub struct Ctx {
    pub gps: Gps,
}

/// Move the *host* UART to a new baud rate, keeping the same peripheral.
///
/// The driver cannot do this itself: it only ever holds a byte stream, so
/// after `set_baud` the module has moved and this side has not. Give the UART
/// back, retune it, and wrap it again.
///
/// Flush first. `write_all` returning does not mean the last byte has been
/// shifted out, and retuning the divisor with a `CFG-PRT` frame still in the
/// FIFO would send its tail at the wrong rate.
pub fn rebuild_at_baud(gps: Gps, baud: u32) -> Gps {
    let mut uart = gps.free();
    uart.flush().expect("UART flush before retuning");
    uart.apply_config(
        &UartConfig::default()
            .with_baudrate(baud)
            .with_rx(RxConfig::default().with_fifo_full_threshold(64)),
    )
    .expect("UART rejected the new baud rate");
    NeoGps::new(uart)
}

/// Bring up the chip and return a driver attached to the module.
///
/// Called once per test case (embedded-test re-runs `#[init]` after each
/// reset), so it must be idempotent from a cold start — it is: every step
/// consumes a peripheral singleton taken fresh from `esp_hal::init`.
pub fn init() -> Ctx {
    let p = esp_hal::init(esp_hal::Config::default().with_cpu_clock(CpuClock::max()));

    // esp-rtos owns the timer that backs `embassy_time`, which the pump
    // helpers use for their deadlines, and drives async tests' executor.
    let timg0 = TimerGroup::new(p.TIMG0);
    let sw_int = SoftwareInterruptControl::new(p.SW_INTERRUPT);
    esp_rtos::start(timg0.timer0, sw_int.software_interrupt0);

    let config = UartConfig::default()
        .with_baudrate(config::BAUD)
        // Wake the driver once it can fill its 64-byte read buffer instead of
        // at the 120-byte default, so a whole NMEA sentence does not sit in
        // the FIFO waiting on the idle timeout.
        .with_rx(RxConfig::default().with_fifo_full_threshold(64));

    let uart = Uart::new(p.UART1, config)
        .expect("UART1 config rejected (check NEO_BAUD)")
        .with_rx(gps_rx_pin!(p))
        .with_tx(gps_tx_pin!(p));

    #[cfg(feature = "async")]
    let uart = uart.into_async();

    log::info!(
        "neo-gps hardware suite: {} build, {} baud",
        crate::MODE,
        config::BAUD
    );

    Ctx {
        gps: NeoGps::new(uart),
    }
}

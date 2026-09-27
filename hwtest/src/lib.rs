//! Shared harness for the on-target hardware-in-the-loop test suite.
//!
//! The tests themselves live in [`tests/hardware.rs`](../tests/hardware.rs);
//! this crate holds everything they need that is *not* a test: board bring-up
//! ([`board`]), the build-time configuration knobs ([`config`]), and the
//! event-pump helpers ([`pump`]).
//!
//! ## Why a separate crate
//!
//! `neo-gps` itself is target-agnostic and its own unit tests run on the host.
//! These tests are the opposite: they need a real UART wired to a real module,
//! so they are built for `xtensa-esp32s3-none-elf`, flashed by `probe-rs`, and
//! run on the MCU. Keeping them in a standalone workspace means a plain
//! `cargo test` at the repo root never tries to build ESP-specific code.
//!
//! ## Sync and async in one suite
//!
//! The driver's `sync` and `async` features are mutually exclusive, so the
//! suite is compiled twice — once per feature. The test bodies are written
//! once and call every driver method through [`maybe_await!`], which expands
//! to `expr.await` in the async build and to plain `expr` in the sync build.

#![no_std]

pub mod board;
pub mod config;
pub mod pump;

#[cfg(all(feature = "async", feature = "sync"))]
compile_error!(
    "features `async` and `sync` are mutually exclusive (the driver enforces the same rule); \
     run the suite twice: once with the default features, once with \
     `--no-default-features --features sync`"
);
#[cfg(not(any(feature = "async", feature = "sync")))]
compile_error!(
    "enable exactly one of `async` (default) or `sync`; \
     `--no-default-features` alone leaves the driver with no transport"
);

/// Drive one driver call, in whichever build mode this suite was compiled for.
///
/// The `sync` and `async` builds of `neo-gps` expose the *same* method names
/// and signatures (that is the point of `maybe-async-cfg`), differing only in
/// whether the returned value is a future. Routing every call site through
/// this macro keeps a single copy of each test body.
///
/// ```ignore
/// let caps = maybe_await!(gps.probe())?;   // gps.probe().await? or gps.probe()?
/// ```
///
/// In the async build it also bounds the call by [`DRIVER_CALL_TIMEOUT_S`].
/// Every blocking driver entry point — `probe`, `send_cfg_acked` and the
/// `enable_*`/`set_*` wrappers built on it — waits on frames, not on a clock:
/// the driver's own docs say so, and recommend exactly this. Without the
/// bound, a module that says nothing turns every one of these tests into an
/// opaque "Test timed out after Ns" instead of naming what it was waiting for.
#[cfg(feature = "async")]
#[macro_export]
macro_rules! maybe_await {
    ($call:expr) => {
        match ::embassy_time::with_timeout(
            ::embassy_time::Duration::from_secs($crate::DRIVER_CALL_TIMEOUT_S),
            $call,
        )
        .await
        {
            Ok(value) => value,
            Err(_) => panic!(
                "`{}` blocked for {}s with nothing arriving from the module. \
                 Run `uart_receives_framed_bytes` first: it reads the UART \
                 directly and will say whether this is wiring, baud, or the driver.",
                stringify!($call),
                $crate::DRIVER_CALL_TIMEOUT_S
            ),
        }
    };
}

/// Blocking build: `embedded_io::Read` has no timeout and the driver owns the
/// UART, so there is nothing to bound the call with. A mute module surfaces as
/// the per-test `#[timeout(..)]` here.
#[cfg(feature = "sync")]
#[macro_export]
macro_rules! maybe_await {
    ($call:expr) => {
        $call
    };
}

/// How long a single driver call may block before the async suite calls it a
/// dead link. Comfortably above the driver's own reply budget — 64 frames,
/// about ten seconds of traffic at the 1 Hz default — and below the shortest
/// per-test `#[timeout]` that performs one.
pub const DRIVER_CALL_TIMEOUT_S: u64 = 15;

/// Name of the build mode, for log output.
pub const MODE: &str = if cfg!(feature = "async") {
    "async"
} else {
    "sync"
};

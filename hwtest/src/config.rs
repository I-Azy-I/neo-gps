//! Build-time configuration.
//!
//! There is no environment on the target, so the knobs the host-side suite
//! took from environment variables are baked into the binary instead. They are
//! read at *compile* time via [`option_env!`]; `build.rs` marks each one with
//! `cargo:rerun-if-env-changed`, so changing a value re-links the tests.
//!
//! ```sh
//! NEO_GEN=8 cargo test --test hardware
//! NEO_GEN=6 NEO_BAUD=38400 NEO_ALLOW_SAVE=1 cargo test --test hardware
//! ```

use neo_gps::Generation;

/// UART baud rate, `NEO_BAUD` (default 9600 — the u-blox factory setting).
pub const BAUD: u32 = parse_u32(option_env!("NEO_BAUD"), 9600);

/// Which module is attached, `NEO_GEN` (`6` | `7` | `8` | `9` | `10`).
///
/// `None` when unset: the probe test then only checks that classification
/// *succeeded*, instead of checking it against a known-good answer.
pub const EXPECTED_GEN: Option<Generation> = parse_gen(option_env!("NEO_GEN"));

/// Whether `save_config` may write the module's battery-backed RAM / flash,
/// `NEO_ALLOW_SAVE`. Off unless set, because it is a persistent side effect.
pub const ALLOW_SAVE: bool = option_env!("NEO_ALLOW_SAVE").is_some();

/// Whether the reset tests may run, `NEO_ALLOW_RESET`. Off unless set:
/// `factory_reset` erases the module's stored configuration for real,
/// including anything a previous `save_config` committed.
pub const ALLOW_RESET: bool = option_env!("NEO_ALLOW_RESET").is_some();

/// Whether the baud round-trip test may run, `NEO_ALLOW_BAUD`. Off unless
/// set: it moves the module off [`BAUD`] mid-run and puts it back, so an
/// interrupted run leaves the link mismatched until the module is power
/// cycled (the change is RAM-only, so a power cycle is enough to recover).
pub const ALLOW_BAUD: bool = option_env!("NEO_ALLOW_BAUD").is_some();

const fn parse_u32(s: Option<&str>, default: u32) -> u32 {
    match s {
        None => default,
        Some(s) => {
            let b = s.as_bytes();
            assert!(!b.is_empty(), "NEO_BAUD must not be empty");
            let mut acc = 0u32;
            let mut i = 0;
            while i < b.len() {
                assert!(
                    b[i] >= b'0' && b[i] <= b'9',
                    "NEO_BAUD must be a decimal number, e.g. NEO_BAUD=9600"
                );
                acc = acc * 10 + (b[i] - b'0') as u32;
                i += 1;
            }
            acc
        }
    }
}

const fn parse_gen(s: Option<&str>) -> Option<Generation> {
    match s {
        None => None,
        Some(s) => Some(match s.as_bytes() {
            b"6" => Generation::Series6,
            b"7" => Generation::Series7,
            b"8" => Generation::Series8,
            b"9" => Generation::Series9,
            b"10" => Generation::Series10,
            _ => panic!("NEO_GEN must be one of 6, 7, 8, 9, 10"),
        }),
    }
}

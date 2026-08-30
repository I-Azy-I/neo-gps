# neo-gps hardware-in-the-loop tests

The `neo-gps` suite in `../src` proves the driver against *scripted* bytes: a
mock UART replays frames captured from the u-blox protocol specifications. This
crate proves it against a **real module on a real wire** — the tests are built
for an ESP32-S3, flashed onto it, and executed on the MCU, with a NEO-xM
hanging off one of its UARTs.

That difference is the point. Only running here can show that the deframer
survives the module's actual interleaved NMEA/UBX stream, that `probe()`
classifies the silicon in front of it, that a `CFG-MSG` is not just ACKed but
*obeyed*, and that `CFG-RATE` really changes how often the module solves.

Everything runs twice: once against the driver's `async` build
(`embedded-io-async`) and once against its blocking `sync` build
(`embedded-io`), from a single copy of each test body.

## Wiring

| ESP32-S3 | NEO-xM | |
|---|---|---|
| `GPIO5` | `TX` | module talks, we listen |
| `GPIO6` | `RX` | we send UBX configuration frames |
| `GND` | `GND` | a common ground is required |
| `3V3` / `5V` | `VCC` | per your module's onboard regulator |

Any free GPIO works — the S3 routes UART signals through its GPIO matrix. To
move them, edit the `WIRING` block at the top of [`src/board.rs`](src/board.rs).

## One-time setup

```sh
cargo install espup probe-rs-tools
espup install                 # installs the "esp" Rust toolchain + Xtensa GCC
. ~/export-esp.sh             # puts xtensa-esp32s3-elf-gcc (the linker) on PATH
```

`. ~/export-esp.sh` is needed in **every shell** that builds this crate;
without it the build fails at link time with
`linker 'xtensa-esp32s3-elf-gcc' not found`.

`probe-rs` talks to the S3 over its built-in USB-Serial-JTAG, so no external
debug probe is needed — just the USB cable on the board's *USB* port (not the
UART-bridge port).

## Running

```sh
cd hwtest

# async driver build (the default), with a NEO-7 attached:
NEO_GEN=7 cargo test --test hardware

# blocking driver build, same module:
NEO_GEN=7 cargo test --test hardware --no-default-features --features sync

# a single test:
NEO_GEN=7 cargo test --test hardware -- probe_classifies_generation

# additionally decode the live stream through the external `nmea` and `ublox`
# crates, via the driver's `codec` adapters:
NEO_GEN=7 cargo test --test hardware --features codec-nmea,codec-ublox

# both at once — the codec features compose with either driver build:
NEO_GEN=7 cargo test --test hardware \
  --no-default-features --features sync,codec-nmea,codec-ublox
```

`codec-nmea` / `codec-ublox` only *add* tests; nothing is taken away. One thing
they document is a real limit of the external crates rather than of the driver:
`ublox` 0.4 declares NAV-PVT as 92 bytes, the protocol 15+ layout, so on a
protocol-14 module (NEO-7) it quietly reports the 84-byte NAV-PVT as
`PacketRef::Unknown`. The driver's own decoder handles both, which is why
`codec::ublox` is an option and not a replacement. The NAV-PVT test skips itself
below protocol 15 and NAV-POSLLH — 28 bytes in every protocol version — carries
the cross-check there.

`probe-rs run` is libtest-compatible, so the output looks like any other
`cargo test` run and IDEs can run individual tests from the gutter.

The **ESP32 is reset before each test case**, so every test starts with a fresh
driver at default settings. The **module is not** — it is separately powered
and keeps its RAM configuration for the whole run. A test that reconfigures the
module must therefore restore it, both so the next test isn't inheriting a
surprise and because `save_config_is_acked` would otherwise commit whatever it
found straight to the module's flash.

## Configuration

There is no environment on the target, so these are read at *build* time
(changing one relinks the suite):

| variable | default | meaning |
|---|---|---|
| `NEO_GEN` | unset | `6` \| `7` \| `8` \| `9` \| `10` — which module is attached. When set, `probe_classifies_generation` checks the driver's answer against it; when unset, it only requires that classification succeeded. |
| `NEO_BAUD` | `9600` | UART baud, i.e. the u-blox factory default unless you have changed it. |
| `NEO_ALLOW_SAVE` | unset | Set to anything to let `save_config_is_acked` write the module's battery-backed RAM / flash. Off by default because it is a persistent side effect on your hardware: it makes the module's *current* configuration its power-on default. The test writes a deliberate one — the standard NMEA sentence set at 1 Hz — rather than whatever an earlier test left in RAM. |

A bad value is a compile error, not a mysterious runtime failure:

```
error[E0080]: evaluation panicked: NEO_GEN must be one of 6, 7, 8, 9, 10
```

## What is covered

Every public entry point of the driver, exercised against the module:

| area | tests |
|---|---|
| capability policy | defaults are conservative, `set_capabilities` override, oversized `send_ubx` payload refused |
| probe | MON-VER classification vs. `NEO_GEN`, result stored, raw frame + payload accessors well-formed |
| NMEA stream | decodes GGA/RMC/GSA live with self-consistent fields, `last_nmea_line` checksum-verifies against its event, `skip_unknown_sentences` hides then surfaces `Other` frames |
| configuration | `Event::Ack` surfacing, `CFG-MSG` mutes *and restores* a sentence, `CFG-RATE` measurably changes the solution cadence, sub-minimum rate refused locally, unimplemented `CFG` id reported as `Error::Nak` |
| binary nav | `enable_binary_nav` picks by capability, NAV-PVT on 7-series+, NAV-PVT refused on legacy, NAV-POSLLH + NAV-SOL where the protocol has them, decoded fields consistent |
| fix-dependent | `next_coordinate` in range, NMEA and NAV-PVT positions agree |
| transport & lifecycle | fail-fast tolerance still streams on a healthy line, `free()` returns a working UART, `save_config` ACKed |
| external codecs (opt-in) | the `nmea` crate decodes a live GSV the built-in codec skips and agrees with it on a live GGA; the `ublox` crate decodes a live NAV-POSLLH — and, where the module emits the 92-byte layout it models, a NAV-PVT — from `last_ubx_frame()` and agrees with the built-in decoder |

Tests that need a satellite fix wait a bounded time and then **skip** the
fix-dependent assertions rather than fail — a driver test must not depend on
having sky view. Tests for a generation-specific feature skip themselves on
modules that do not have it, so the same suite is meaningful on a NEO-6M and on
an M10.

## Privacy

No test prints or asserts a literal position. Fix-dependent checks only
establish that a coordinate is inside the valid global range, that it is not
the null-island `0, 0` no-fix artifact, and that two decoders agree with each
other to within ~1 km. Your actual latitude and longitude never leave the
device.

## Layout

| file | |
|---|---|
| [`tests/hardware.rs`](tests/hardware.rs) | the suite itself |
| [`src/board.rs`](src/board.rs) | wiring, clocks, esp-rtos, UART bring-up |
| [`src/pump.rs`](src/pump.rs) | deadline-bounded event pumping and frame validation |
| [`src/config.rs`](src/config.rs) | the build-time knobs above |
| [`src/lib.rs`](src/lib.rs) | the `maybe_await!` macro that lets one test body serve both driver builds |

This is a **standalone workspace**: `cargo test` at the repo root never tries to
build it, and nothing that ships in the published crate depends on it.

## Porting to another board

The driver only ever asks for `embedded-io[-async]` `Read + Write`, so nothing
in `tests/hardware.rs` is ESP-specific. To move the suite, replace
`src/board.rs` with your HAL's UART bring-up, point `.cargo/config.toml` at
your target triple and `probe-rs --chip`, and swap the esp-hal/esp-rtos
dependencies for your HAL and an embedded-test executor.

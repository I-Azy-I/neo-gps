# neo-gps

[![crates.io](https://img.shields.io/crates/v/neo-gps.svg)](https://crates.io/crates/neo-gps)
[![docs.rs](https://docs.rs/neo-gps/badge.svg)](https://docs.rs/neo-gps)
[![CI](https://github.com/I-Azy-I/neo-gps/actions/workflows/ci.yml/badge.svg)](https://github.com/I-Azy-I/neo-gps/actions/workflows/ci.yml)
[![License](https://img.shields.io/badge/license-MIT%20OR%20Apache--2.0-blue.svg)](#license)

A `no_std`, zero-allocation Rust driver for the u-blox **NEO-xM** GPS family
(NEO-6M, NEO-7M, NEO-8M, NEO-M9N, NEO-M10) over UART. It works in both async
and blocking code.

## Quick start

```toml
[dependencies]
neo-gps = "0.1"
```

```rust
use neo_gps::NeoGps;

let uart = BufferedUart::new(/* ... */); // 9600 8N1, any embedded-io-async Read + Write
let mut gps = NeoGps::new(uart);

loop {
    let pos = gps.next_coordinate().await?; // only returns once there is a real fix
    defmt::info!("lat={} lon={}", pos.lat_1e7, pos.lon_1e7); // degrees × 1e7
}
```

That's the whole minimum. The module's default output is enough, and no
configuration is needed.

## A fuller example

```rust
use neo_gps::{NeoGps, Event, nmea::Sentence};

let mut gps = NeoGps::new(uart);

let caps = gps.probe().await?;                   // detect the module generation
gps.set_nav_rate_ms(caps.max_rate_ms()).await?;  // fastest rate it can sustain
gps.disable_nmea(0x03).await?;                   // mute GSV chatter
gps.enable_binary_nav().await?;                  // NAV-PVT, or POSLLH + SOL on a NEO-6M
gps.save_config().await?;                        // keep it across power cycles

loop {
    match gps.next_event().await? {
        Event::NavPvt(pvt) if pvt.gnss_fix_ok() => {
            defmt::info!("lat={} lon={} sats={}", pvt.lat_1e7, pvt.lon_1e7, pvt.num_sv);
        }
        Event::Nmea(Sentence::Rmc(rmc)) if rmc.valid => { /* NMEA path */ }
        _ => {}
    }
}
```

## Features

### Async or blocking

Pick one build mode. The API is the same in both, and the sync build simply
has no `.await`.

```toml
neo-gps = "0.1"                                                                # async: embedded-io-async
neo-gps = { version = "0.1", default-features = false, features = ["sync", "builtin-codec"] }  # blocking: embedded-io
```

The UART only has to implement `embedded-io[-async]` `Read + Write`, such as an
embassy `BufferedUart`, an RTIC async HAL, a blocking HAL UART, or a desktop
serial port through `embedded-io-adapters`.

### One driver for the whole family

The modules share one wire format (NMEA 0183 + UBX) but differ in what they
support. `gps.probe()` polls `UBX-MON-VER` and returns a `Capabilities` that
the driver uses to choose the right messages for you. If you don't probe, it
assumes a NEO-6M, which is safe on every module.

| | NEO-6M | NEO-7M | NEO-8M | M9 | M10 |
|---|---|---|---|---|---|
| NMEA talker | `GP` | `GP`/`GL` | `GN`/`GL`/`GA`/`GB` | same | same |
| Max nav rate (default GNSS) | 5 Hz | 10 Hz | 5 Hz | 25 Hz | 10 Hz |
| Max nav rate (single GNSS) | 5 Hz | 10 Hz | 10 Hz | 25 Hz | 25 Hz |
| `NAV-PVT` | ✗ | ✓ | ✓ | ✓ | ✓ |
| `NAV-POSLLH` / `NAV-SOL` | ✓ | ✓ | ✓ | ✗ | ✗ |
| `CFG-GNSS` | ✗ | ✓ | ✓ | ✓ | ✗ |
| legacy `CFG-*` | ✓ | ✓ | ✓ | ✓ | ✗ |

On an M10, wrappers built on the legacy `CFG-*` messages return
`Error::Unsupported` instead of sending a frame the module would NAK, because
the driver does not speak `CFG-VALSET` yet. `save_config`, `reset` and
`factory_reset` still work there. You can check this with
`Capabilities::has_legacy_cfg()`.

### Reading data

* `next_coordinate()` returns the next valid position from RMC, GGA or NAV-PVT.
* `next_event()` returns every frame as an `Event`: decoded NMEA (GGA, RMC,
  GSA), `NavPvt`, `NavPosllh`, `NavSol`, `NavVelned`, `Ack`, or
  `NmeaOther` / `UbxOther` for anything else.
* `satellites()` gives per-satellite info after `enable_satellite_info()`
  (NAV-SAT, or NAV-SVINFO on older modules).
* `last_nmea_line()`, `last_ubx_frame()` and `last_ubx_payload()` return the
  raw bytes of the last frame, so you can decode anything yourself.

All values are integers, so no float support is needed:

* lat/lon: degrees × 1e7
* altitude and accuracy: mm
* speed: mm/s
* course and heading: degrees × 1e5
* DOP: × 100

### Configuring the module

| Method | Message | |
|---|---|---|
| `set_nav_rate_ms` | `CFG-RATE` | solution rate, capped at what the module supports |
| `set_msg_rate`, `disable_nmea` | `CFG-MSG` | turn individual messages on/off |
| `enable_binary_nav`, `enable_nav_pvt`, `enable_nav_posllh`, `enable_nav_sol`, `enable_nav_velned`, `enable_satellite_info` | `CFG-MSG` | binary navigation output |
| `set_baud` | `CFG-PRT` | change the baud rate; reconfigure your UART afterwards |
| `set_dynamic_model` | `CFG-NAV5` | portable, automotive, airborne, ... |
| `set_constellation` | `CFG-GNSS` | enable/disable GPS, GLONASS, Galileo, BeiDou, ... |
| `set_power_save` | `CFG-RXM` | power-save mode |
| `save_config`, `factory_reset` | `CFG-CFG` | persist or wipe the configuration |
| `reset` | `CFG-RST` | hot, warm or cold restart |
| `send_ubx`, `send_cfg_acked` | any | send your own frame, with or without waiting for ACK/NAK |


### Bring your own decoder

| Feature | Default | Provides |
|---|---|---|
| `builtin-codec` | on | small built-in decoders: GGA/RMC/GSA, NAV-PVT/POSLLH/SOL/VELNED |
| `nmea` | off | `codec::nmea::decode(gps.last_nmea_line())` using the [`nmea`](https://crates.io/crates/nmea) crate |
| `ublox` | off | `codec::ublox::decode(&mut parser, gps.last_ubx_frame(), \|pkt\| ...)` using the [`ublox`](https://crates.io/crates/ublox) crate |

If you disable `builtin-codec`, every frame arrives as `NmeaOther` / `UbxOther`
and the external crates do all the decoding.

`ublox` is pinned to 0.4 to keep the MSRV at 1.75. On rustc ≥ 1.83 you can use
`ublox = { version = "0.9", default-features = false, features = ["ubx_proto23"] }`.

## ⚠️ Disclaimer: AI-generated code
I created this driver because I needed one for a small project I was working
on. Large parts of it were written with AI, and I give no guarantee that
everything works. I ran the hardware-in-the-loop tests with a Gen 7 module.

Most of this crate was written by AI from the u-blox datasheets, including the
**deframer**, which is the state machine that splits the UART byte stream into
NMEA and UBX frames. The host tests and the hardware suite in `hwtest/` cover
it, and the hardware suite has passed on a real **NEO-7M**. It has not been
tested on real 6, 8, M9 or M10 modules yet.

If you would rather rely on the parser from the established
[`ublox`](https://crates.io/crates/ublox) crate, you have two options:

* **Enable the `ublox` feature.** Each frame is then passed through
  `ublox::Parser`, which checks it again and does the decoding.
* **Skip this driver's deframer completely.** Configure the module with
  `neo-gps`, then call `gps.free()` to get the UART back and feed the raw
  bytes to `ublox::Parser` yourself.

## Limitations

* `send_cfg_acked` waits for a number of frames, not a timer. If the module is
  silent, a read can block forever, so wrap the call in your executor's
  timeout (e.g. `embassy_time::with_timeout`).
* After `set_baud`, you have to reconfigure your own UART. The usual pattern
  is `gps.free()`, change the baud rate, then `NeoGps::new(uart)`.
* GSV is checksum-checked but not decoded. Use the `nmea` feature or
  `last_nmea_line()` to read it.

## License

Licensed under either of [MIT](LICENSE-MIT) or [Apache-2.0](LICENSE-APACHE), at your option.

# neo-gps

Sync **or** async, `no_std`, zero-allocation Rust driver for the u-blox
**NEO-xM family** (NEO-6M, NEO-7M, NEO-8M, NEO-M9N/M10) over UART.

One codebase, two build modes via `maybe-async-cfg` (mutually exclusive
features):

```toml
neo-gps = "0.3"                                                    # async (default): embedded-io-async
neo-gps = { version = "0.3", default-features = false, features = ["sync"] }  # blocking: embedded-io
```

The API is identical in both modes — the sync build simply has no `.await`
points. Note the traits come from `embedded-io[-async]`, not
`embedded-hal[-async]`: the HAL crates deliberately contain no serial traits,
because a UART is a byte stream and byte streams are modeled by embedded-io.
Async works with embassy UARTs, RTIC async HALs, or desktop ports via
`embedded-io-adapters`; sync works with any blocking `embedded_io::Read +
Write` (superloop firmware, `std` targets via adapters).

```rust,ignore
// async (default feature)          // sync (feature = "sync")
let caps = gps.probe().await?;      let caps = gps.probe()?;
match gps.next_event().await? {     match gps.next_event()? {
```

## Why one driver for the whole family

The modules share transport (UART @ 9600 8N1), grammar (NMEA 0183 + UBX) and
differ only in *parameters*:

| | NEO-6M | NEO-7M | NEO-8M | M9 | M10 |
|---|---|---|---|---|---|
| NMEA talker | `GP` | `GP`/`GL` | `GN`/`GL`/`GA`/`GB` | same | same |
| NMEA version | 2.3 | 2.3 | 4.0 | 4.1x | 4.1x |
| Max nav rate, stock GNSS config | 5 Hz | 10 Hz | 5 Hz | 25 Hz | 10 Hz |
| Max nav rate, single GNSS | 5 Hz | 10 Hz | 10 Hz | 25 Hz | 25 Hz |
| `UBX-NAV-PVT` | ✗ | ✓ | ✓ | ✓ | ✓ |
| `UBX-NAV-POSLLH`/`SOL` | ✓ | ✓ | ✓ (deprecated) | ✗ (removed proto 24+) | ✗ |
| `UBX-CFG-GNSS` | ✗ | ✓ | ✓ | ✓ | ✗ |
| legacy `UBX-CFG-*` | ✓ | ✓ | ✓ | ✓ | ✗* |

So the parser is talker/version-tolerant, and the three version-dependent
features are gated on a `Capabilities` struct filled at runtime by polling
`UBX-MON-VER` (`gps.probe()`). Without probing, conservative NEO-6 defaults
apply and every command remains universally valid.

\* At protocol 34 the whole UBX-CFG class becomes `CFG-CFG`, `CFG-RST` and
`CFG-VALSET`/`VALGET`/`VALDEL`. This driver does not speak the key/value
interface yet, so on an M10 the wrappers built on `CFG-MSG`, `CFG-RATE`,
`CFG-PRT`, `CFG-NAV5` and `CFG-GNSS` return `Error::Unsupported` rather than
send a frame the module would NAK; `save_config`, `factory_reset` and `reset`
still work. Check `Capabilities::has_legacy_cfg()`, or reach for `send_ubx`.

## Usage (embassy)

```rust,ignore
use neo_gps::{NeoGps, Event, nmea::Sentence};

let uart = BufferedUart::new(p.USART2, /* ... */); // 9600 8N1
let mut gps = NeoGps::new(uart);

// Optional: detect module generation, then configure.
let caps = gps.probe().await?;
gps.set_nav_rate_ms(caps.max_rate_ms()).await?;   // fastest for the stock
                                                  // constellation setup
gps.disable_nmea(0x03).await?;                    // mute GSV chatter
gps.enable_binary_nav().await?;                   // NAV-PVT on 7/8/M9+,
                                                  // NAV-POSLLH + NAV-SOL on a NEO-6M
gps.save_config().await?;                         // survive power cycles (needs BBR/flash)

loop {
    match gps.next_event().await? {
        Event::NavPvt(pvt) if pvt.gnss_fix_ok() => {
            defmt::info!("lat={} lon={} (1e-7 deg), {} sats", pvt.lat_1e7, pvt.lon_1e7, pvt.num_sv);
        }
        Event::Nmea(Sentence::Rmc(rmc)) if rmc.valid => {
            // NMEA path — identical on 6M ($GPRMC) and 8M ($GNRMC)
        }
        _ => {}
    }
}
```

## Codec features: bring your own vocabulary

The crate's own scope is **transport**: deframing the interleaved NMEA/UBX
stream, checksums, the sync/async pump, ACK correlation, and family
capability policy. Message *vocabulary* is pluggable:

| feature | default | provides |
|---|---|---|
| `builtin-codec` | on | minimal zero-dep decoders: GGA/RMC/GSA, NAV-PVT/POSLLH/SOL |
| `nmea` | off | `codec::nmea::decode(gps.last_nmea_line())` → the [`nmea`](https://crates.io/crates/nmea) crate's `ParseResult` (dozens of sentence types) |
| `ublox` | off | `codec::ublox::decode(&mut parser, gps.last_ubx_frame(), \|pkt\| ...)` → the [`ublox`](https://crates.io/crates/ublox) crate's typed `PacketRef` (the full UBX catalogue) |

With `default-features = false` + `async`/`sync` + codec features, the
built-in decoders compile out entirely: every valid frame surfaces as
`Event::NmeaOther`/`UbxOther` and the external crates supply all typing.
The codec tests cross-validate both paths against the same framed bytes.

Note: `ublox` is pinned to 0.4 here for MSRV 1.75; on rustc >= 1.83 bump to
`ublox = { version = "0.9", default-features = false, features = ["ubx_proto23"] }`
(same adapter shape). No external crate covers the NEO-6M's protocol 12/13 —
its POSLLH/SOL happen to be wire-identical to protocol 14's, and the builtin
covers them natively.

## Units (integer-only, no float requirement)

* latitude/longitude: `i32`, degrees × 1e7 (UBX convention)
* altitude / accuracy: millimetres
* speed: mm/s   ·   course/heading: degrees × 1e5   ·   DOP: × 100

## Transport resilience

UART read errors (framing violations, line noise, the floating-line glitch
most modules produce at power-up) are treated as *data-level* corruption, not
link failure: `next_event` drops any partial frame, resyncs, and retries.
Only more than N **consecutive** failed reads (default 16, tunable via
`set_read_error_tolerance`, `0` = fail-fast) surface as `Error::Io` — that
pattern means a broken link, not a glitch. `Ok(0)` (closed port) is always
immediately fatal. `probe()` additionally re-sends its MON-VER poll once at
half budget, so an outbound poll eaten by line noise costs a delay instead of
a guaranteed `Timeout`.

## Notes & limitations

* `send_cfg_acked` waits for ACK/NAK bounded by a frame budget, not a timer —
  if the module is unpowered/mute, a pending `read()` can wait forever. Wrap
  calls in your executor's timeout (e.g. `embassy_time::with_timeout`) if that
  matters for your application.
* GSV (satellites-in-view) is framed and checksum-verified but not decoded; it
  surfaces as `Event::NmeaOther`, with the raw line available via
  `last_nmea_line()`. Unknown UBX frames surface as `Event::UbxOther` with the
  payload available via `last_ubx_payload()` (snapshots, stable until the next
  event of the same protocol), and `send_ubx` lets you transmit anything — so
  extending the driver, or layering the `ublox`/`nmea` crates on top, doesn't
  require forking it.
* Changing the module baud rate is intentionally not wrapped (it desynchronises
  the ACK); send `CFG-PRT` via `send_ubx` and reconfigure your UART manually.
* If you only need UBX message *definitions*, consider the `ublox` crate
  (ublox-rs) — it's a comprehensive sans-io codec you can layer under
  `send_ubx`/`UbxOther`.

## Tests

`cargo test` runs 70 host-side tests plus 16 doctests. Beyond the core suite
(deframing, resync, corrupt-frame handling, async probe/configure flows on a
scripted mock UART), a datasheet-derived module encodes facts read directly
from the official u-blox protocol specifications:

* **u-blox 6** (GPS.G6-SW-10018, FW 7.03): full default NMEA 2.3 sentence
  cycle with the GP talker, boot TXT sentences, absence of NAV-PVT and
  CFG-GNSS, absence of `PROTVER` in MON-VER classifying as Series6, 5 Hz cap.
* **u-blox 7** (GPS.G7-SW-12001, protocol 14): the 84-byte NAV-PVT variant
  (a real bug found by reading the spec: the 8-series message is 92 bytes),
  GL-talker output in GLONASS-only mode, capability boundary (PVT yes,
  CFG-GNSS no).
* **u-blox 8/M8** (UBX-13003221 R17): the firmware→protocol table (SPG 2.01
  →15.00 ... SPG 3.51→23.01, HPG/TIM variants), both MON-VER `PROTVER`
  spellings (space form ≤17, `=` form ≥18) amid realistic extension strings,
  GN main talker with per-constellation GSV talkers (GP/GL/GA/GB), NMEA 4.1
  GNS, ~110-char proprietary PUBX,00 sentences, gnssFixOK semantics.
* **M9 / M10**: PROTVER 32 and 34, including the M10 losing the legacy
  `UBX-CFG-*` class.
* **Wire level**: a published, externally captured CFG-NMEA frame with its
  real checksum validates the Fletcher implementation against actual u-blox
  hardware; byte-at-a-time delivery; oversized-payload truncation with intact
  framing; false sync bytes; missing-checksum rejection; the spec's own
  ddmm.mmmmm worked example and unit-scaling vectors (knots→mm/s, DOP×100).

### Hardware-in-the-loop

Everything above runs on the host against scripted bytes. [`hwtest/`](hwtest)
holds the other half: 22 tests that run **on an MCU** (an ESP32-S3, via
[`embedded-test`](https://crates.io/crates/embedded-test) and `probe-rs`) with a
real NEO module wired to its UART, covering every public entry point of the
driver against real silicon — probe classification, `CFG-MSG` that is not just
ACKed but obeyed, `CFG-RATE` that measurably changes the solution cadence,
NAK correlation, binary nav per generation, and the fix path. The whole suite
runs twice, once per driver build (`async` and `sync`), from one copy of each
test body.

```sh
cd hwtest && . ~/export-esp.sh
NEO_GEN=8 cargo test --test hardware                                      # async
NEO_GEN=8 cargo test --test hardware --no-default-features --features sync
```

It is a standalone workspace, so it never affects a root `cargo test` or the
published crate. See [`hwtest/README.md`](hwtest/README.md) for wiring, setup
and the coverage table.

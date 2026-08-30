# Changelog

All notable changes to this crate. Format loosely follows
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/); this project uses
[semantic versioning](https://semver.org/), with the pre-1.0 rule that a minor
bump may break.

## [0.7.0]

Corrects several capability gates against the u-blox interface descriptions,
and fixes a panic reachable from safe code. **Breaking**, mostly around
`Generation`.

### Fixed

* `last_ubx_payload()` panicked with "attempt to subtract with overflow" when
  called before any UBX frame had arrived, which is reachable from safe public
  API at any time.
* `has_cfg_gnss()` excluded the 7 series. `CFG-GNSS` is a full section of the
  u-blox 7 interface description, so `set_constellation` was refusing a
  message a NEO-7M supports. Now protocol 14 and later.
* The M10 dropped the whole legacy `UBX-CFG-*` class. Every wrapper built on
  `CFG-MSG`, `CFG-RATE`, `CFG-PRT`, `CFG-NAV5` and `CFG-GNSS` would have been
  NAKed. They now return `Error::Unsupported`; see `has_legacy_cfg()`.
* `max_rate_ms()` returned the single-constellation best case, so following
  the README's own advice asked a stock NEO-M8N for 10 Hz, which it cannot
  sustain on GPS + GLONASS. See "Changed" below.
* A stray `0xB5` swallowed the frame that followed it. The `UbxSync` state
  discarded a non-`0x62` byte instead of reconsidering it, losing the whole of
  `B5 B5 62 ...` and, worse, any sentence after a `B5`.
* `next_coordinate()` accepted a GGA with quality 6 (dead reckoning), 7
  (manual) or 8 (simulation), while rejecting the same epoch's RMC. Which
  position you got depended on which sentence arrived first.

### Added

* `enable_satellite_info()` and `satellites()`, decoding `NAV-SAT` (protocol
  15+) and `NAV-SVINFO` (before that) into a borrowing iterator of `SatInfo`.
  With `has_nav_sat()` and `has_nav_svinfo()`.
* `set_baud()` (`CFG-PRT`), the prerequisite for running above 1 Hz.
* `set_dynamic_model()` (`CFG-NAV5`), with the `DynamicModel` enum.
* `set_constellation()` (`CFG-GNSS`), polling and writing back so the other
  constellations keep their channel allocations. With `Constellation`.
* `reset()` (`CFG-RST`) with `ResetKind`, and `factory_reset()` (`CFG-CFG`).
  Neither waits for an acknowledgement: a resetting module does not send one,
  and a factory reset clears the port configuration, so its ACK races with the
  UART being reinitialised.
* `set_power_save()` (`CFG-RXM`).
* `enable_nav_velned()` and `Event::NavVelned`, which is the only binary
  velocity a 6-series module can produce.
* `Capabilities::has_legacy_cfg()` and `max_rate_ms_single_gnss()`.
* `FixQuality::is_gnss_fix()`, stricter than `has_fix()`.
* `[package.metadata.docs.rs]`, so `codec::nmea` and `codec::ublox` appear in
  the published documentation at all; they were absent before.
* `repository`, `documentation` and `readme` package metadata.
* `#![warn(missing_docs)]`, with every public item documented.

### Changed

* **Breaking.** `Generation::Series9Plus` is now `Series9` (protocol 24..=33)
  and `Series10` (34 and later). They behave differently, so one variant could
  not describe both.
* **Breaking.** `Generation`, `Event` and `Error` are `#[non_exhaustive]`, so
  future variants stop being breaking changes. Downstream `match`es need a
  wildcard arm.
* `max_rate_ms()` now reports the rate for the module's *default*
  constellation configuration; `max_rate_ms_single_gnss()` reports the
  hardware limit, and is what `set_nav_rate_ms()` enforces.
* `next_coordinate()` requires `FixQuality::is_gnss_fix()` for GGA, matching
  the strictness already applied to RMC and `NAV-PVT`.

## [0.6.0] and earlier

Not tracked here.

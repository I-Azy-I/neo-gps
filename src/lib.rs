//! # neo-gps
//!
//! A `no_std`, zero-allocation driver for the u-blox NEO-xM GPS family
//! (NEO-6M, NEO-7M, NEO-8M, NEO-M9N, NEO-M10) over UART. It works in both
//! async (`async` feature, the default) and blocking (`sync` feature) code.
//!
//! ## Example
//!
//! ```ignore
//! use neo_gps::NeoGps;
//!
//! let uart = BufferedUart::new(/* ... */); // 9600 8N1, any embedded-io-async Read + Write
//! let mut gps = NeoGps::new(uart);
//!
//! loop {
//!     let pos = gps.next_coordinate().await?; // only returns once there is a real fix
//!     defmt::info!("lat={} lon={}", pos.lat_1e7, pos.lon_1e7); // degrees × 1e7
//! }
//! ```
//!
//! See [`NeoGps`] for configuration, and the
//! [README](https://github.com/I-Azy-I/neo-gps) for the full feature list.

#![cfg_attr(not(test), no_std)]
#![deny(unsafe_code)]
// Kept on so a new public item cannot ship with an empty `///` again.
#![warn(missing_docs)]
// Set by docs.rs (see `[package.metadata.docs.rs]`) so feature-gated items
// carry an "Available on crate feature X" badge. Nightly-only, hence the cfg.
#![cfg_attr(docsrs, feature(doc_cfg))]

pub mod codec;
mod deframer;
pub mod nmea;
pub mod ubx;

#[cfg(all(feature = "sync", feature = "async"))]
compile_error!("features `sync` and `async` are mutually exclusive; enable exactly one");
#[cfg(not(any(feature = "sync", feature = "async")))]
compile_error!("enable one of the `sync` (embedded-io) or `async` (embedded-io-async) features");

#[cfg(all(feature = "sync", not(feature = "async")))]
pub(crate) use embedded_io as eio;
#[cfg(feature = "async")]
pub(crate) use embedded_io_async as eio;

use deframer::*;
use eio::{Read, Write};
use nmea::Sentence;
use ubx::{UbxFrame, CLASS_CFG, CLASS_MON, CLASS_NAV};
/// Errors produced by the driver.
///
/// `non_exhaustive`: new failure modes should not break your `match`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[non_exhaustive]
pub enum Error<E> {
    /// Underlying UART error
    Io(E),
    /// The UART read returned 0 bytes (EOF on adapted streams).
    Eof,
    /// The module answered a configuration frame with `ACK-NAK`.
    Nak {
        /// Message class that was refused.
        class: u8,
        /// Message id that was refused.
        id: u8,
    },
    /// No `ACK`/reply arrived within the frame budget.
    NoReply,
    /// The requested feature is not supported by the detected module
    /// (e.g. `CFG-GNSS` on a NEO-6M, or a nav rate above the module maximum).
    Unsupported,
    /// A frame was too large for the internal buffer.
    Overflow,
}

impl<E> From<E> for Error<E> {
    fn from(e: E) -> Self {
        Error::Io(e)
    }
}

/// Module generation, derived from the `PROTVER` reported by `UBX-MON-VER`.
///
/// `non_exhaustive`: u-blox keeps shipping generations, and adding one here
/// should not break your `match`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[non_exhaustive]
pub enum Generation {
    /// NEO-6 series (protocol < 14). GPS only.
    Series6,
    /// NEO-7 series (protocol 14). GPS + GLONASS (not concurrent).
    Series7,
    /// NEO-8 series (protocol 15..=23). Concurrent multi-GNSS.
    Series8,
    /// M9 series (protocol 24..=33). Still speaks the legacy `CFG` messages.
    Series9,
    /// M10 and later (protocol >= 34). Configured only through
    /// `CFG-VALSET`/`VALGET`, which this driver does not implement yet: see
    /// [`Capabilities::has_legacy_cfg`].
    Series10,
}

/// What the connected module can do. Filled by [`NeoGps::probe`],
/// or constructed manually if the module type is known at compile time.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Capabilities {
    /// Which family the module belongs to.
    pub generation: Generation,
    /// u-blox protocol version, e.g. 12 (NEO-6), 15..23 (NEO-8), 32 (M10).
    pub protocol_version: u8,
}

impl Capabilities {
    /// Capabilities to assume before probing: the oldest module in the family.
    ///
    /// # Example
    /// ```
    /// use neo_gps::{Capabilities, Generation};
    /// assert_eq!(Capabilities::conservative().generation, Generation::Series6);
    /// ```
    pub const fn conservative() -> Self {
        Capabilities {
            generation: Generation::Series6,
            protocol_version: 12,
        }
    }

    /// Fastest navigation update interval the module sustains **in its
    /// default constellation configuration**, in milliseconds.
    ///
    /// This is the number to hand to
    /// [`set_nav_rate_ms`](NeoGps::set_nav_rate_ms) when you have not changed
    /// which constellations are enabled. It is not always the datasheet
    /// headline: a stock NEO-M8N tracks GPS and GLONASS together and manages
    /// 5 Hz, though it reaches 10 Hz on one constellation. See
    /// [`max_rate_ms_single_gnss`](Self::max_rate_ms_single_gnss).
    ///
    /// # Example
    /// ```
    /// use neo_gps::Capabilities;
    /// assert_eq!(Capabilities::conservative().max_rate_ms(), 200); // NEO-6: 5 Hz
    /// ```
    pub fn max_rate_ms(&self) -> u16 {
        match self.generation {
            Generation::Series6 => 200,  // 5 Hz
            Generation::Series7 => 100,  // 10 Hz
            Generation::Series8 => 200,  // 5 Hz with the default GPS + GLONASS
            Generation::Series9 => 40,   // 25 Hz
            Generation::Series10 => 100, // 10 Hz with 3+ concurrent GNSS
        }
    }

    /// Fastest navigation update interval the hardware reaches at all, which
    /// generally means with a single constellation enabled.
    ///
    /// The bound [`set_nav_rate_ms`](NeoGps::set_nav_rate_ms) enforces, so a
    /// caller who has narrowed the constellations with
    /// [`set_constellation`](NeoGps::set_constellation) can still ask for the
    /// full rate. Equal to [`max_rate_ms`](Self::max_rate_ms) on the
    /// generations where the two do not differ.
    ///
    /// # Example
    /// ```
    /// use neo_gps::{Capabilities, Generation};
    /// let m8 = Capabilities { generation: Generation::Series8, protocol_version: 18 };
    /// assert_eq!(m8.max_rate_ms(), 200);             // stock: GPS + GLONASS
    /// assert_eq!(m8.max_rate_ms_single_gnss(), 100); // one constellation
    /// ```
    pub fn max_rate_ms_single_gnss(&self) -> u16 {
        match self.generation {
            Generation::Series6 => 200,
            Generation::Series7 => 100,
            Generation::Series8 => 100, // 10 Hz on one constellation
            Generation::Series9 => 40,
            Generation::Series10 => 40, // 25 Hz on one constellation
        }
    }

    /// Whether the module accepts the legacy `UBX-CFG-*` messages this driver
    /// is built on (`CFG-MSG`, `CFG-RATE`, `CFG-PRT`, `CFG-NAV5`, `CFG-GNSS`).
    ///
    /// False from protocol 34 (M10) on, where the whole `CFG` class is
    /// `CFG-CFG`, `CFG-RST` and the `CFG-VALSET`/`VALGET`/`VALDEL`
    /// configuration interface, which this driver does not speak yet. On such
    /// a module the affected methods return [`Error::Unsupported`] instead of
    /// sending a frame that would be NAKed;
    /// [`save_config`](NeoGps::save_config),
    /// [`factory_reset`](NeoGps::factory_reset) and [`reset`](NeoGps::reset)
    /// still work, since `CFG-CFG` and `CFG-RST` survived.
    ///
    /// # Example
    /// ```
    /// use neo_gps::{Capabilities, Generation};
    /// let m9 = Capabilities { generation: Generation::Series9, protocol_version: 32 };
    /// let m10 = Capabilities { generation: Generation::Series10, protocol_version: 34 };
    /// assert!(m9.has_legacy_cfg());
    /// assert!(!m10.has_legacy_cfg());
    /// ```
    pub fn has_legacy_cfg(&self) -> bool {
        self.protocol_version < 34
    }

    /// Whether the module has `UBX-NAV-PVT`, the all-in-one binary nav
    /// message (protocol 14 and later).
    ///
    /// # Example
    /// ```
    /// use neo_gps::{Capabilities, Generation};
    /// let m8 = Capabilities { generation: Generation::Series8, protocol_version: 18 };
    /// assert!(m8.has_nav_pvt());
    /// ```
    pub fn has_nav_pvt(&self) -> bool {
        self.protocol_version >= 14
    }

    /// Whether the module has `UBX-NAV-SOL` (u-blox 6/7/8). Gone from
    /// protocol 24 on, where `NAV-PVT` replaces it.
    ///
    /// # Example
    /// ```
    /// use neo_gps::Capabilities;
    /// assert!(Capabilities::conservative().has_nav_sol()); // NEO-6: yes
    /// ```
    pub fn has_nav_sol(&self) -> bool {
        self.protocol_version < 24
    }

    /// Whether the module has `UBX-NAV-SAT` for per-satellite data
    /// (protocol 15 and later). Older modules carry the same information in
    /// `UBX-NAV-SVINFO`; see [`has_nav_svinfo`](Self::has_nav_svinfo).
    ///
    /// # Example
    /// ```
    /// use neo_gps::{Capabilities, Generation};
    /// let m8 = Capabilities { generation: Generation::Series8, protocol_version: 18 };
    /// assert!(m8.has_nav_sat());
    /// assert!(!Capabilities::conservative().has_nav_sat()); // NEO-6: SVINFO
    /// ```
    pub fn has_nav_sat(&self) -> bool {
        self.protocol_version >= 15
    }

    /// Whether the module has `UBX-NAV-SVINFO`, the predecessor of
    /// `UBX-NAV-SAT`. Removed from protocol 24 on, alongside `NAV-SOL`.
    ///
    /// # Example
    /// ```
    /// use neo_gps::Capabilities;
    /// assert!(Capabilities::conservative().has_nav_svinfo());
    /// ```
    pub fn has_nav_svinfo(&self) -> bool {
        self.protocol_version < 24
    }

    /// Whether the module has `UBX-CFG-GNSS` for constellation selection.
    ///
    /// Introduced with the 7 series (protocol 14); absent from the 6 series,
    /// and gone again from protocol 34 (M10) with the rest of the legacy
    /// `CFG` class.
    ///
    /// # Example
    /// ```
    /// use neo_gps::{Capabilities, Generation};
    /// assert!(!Capabilities::conservative().has_cfg_gnss()); // not on NEO-6
    /// let neo7 = Capabilities { generation: Generation::Series7, protocol_version: 14 };
    /// assert!(neo7.has_cfg_gnss());
    /// ```
    pub fn has_cfg_gnss(&self) -> bool {
        self.protocol_version >= 14 && self.has_legacy_cfg()
    }
}

/// A parsed item from the module's output stream.
///
/// `non_exhaustive`: more message types get decoded over time, and adding a
/// variant should not break your `match`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[non_exhaustive]
pub enum Event {
    /// A supported, checksum-valid NMEA sentence.
    Nmea(Sentence),
    /// A checksum-valid NMEA sentence the driver does not decode
    /// (talker + type provided, e.g. `(b"GP", b"GSV")`).
    NmeaOther {
        /// Talker id, e.g. `b"GP"`.
        talker: [u8; 2],
        /// Sentence type, e.g. `b"GSV"`.
        mtype: [u8; 3],
    },
    /// `UBX-NAV-PVT` (only after [`NeoGps::enable_nav_pvt`], 7-series and later).
    NavPvt(ubx::NavPvt),
    /// `UBX-NAV-POSLLH` (after [`NeoGps::enable_nav_posllh`]; all generations,
    /// incl. NEO-6M).
    NavPosllh(ubx::NavPosllh),
    /// `UBX-NAV-SOL` (after [`NeoGps::enable_nav_sol`]; u-blox 6/7/8 only).
    NavSol(ubx::NavSol),
    /// `UBX-NAV-VELNED` (after [`NeoGps::enable_nav_velned`]; all generations).
    NavVelned(ubx::NavVelned),
    /// `UBX-ACK-ACK` / `UBX-ACK-NAK` for a `CFG` frame.
    Ack {
        /// Message class being acknowledged.
        class: u8,
        /// Message id being acknowledged.
        id: u8,
        /// `true` for `ACK-ACK`, `false` for `ACK-NAK`.
        ok: bool,
    },
    /// Any other checksum-valid UBX frame.
    UbxOther {
        /// Message class.
        class: u8,
        /// Message id.
        id: u8,
    },
}

/// A position from [`NeoGps::next_coordinate`], in degrees * 1e7. Never the
/// pre-fix `0, 0`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Coordinate {
    /// Latitude, degrees * 1e7, positive north.
    pub lat_1e7: i32,
    /// Longitude, degrees * 1e7, positive east.
    pub lon_1e7: i32,
}

/// Frames to read while waiting for an ACK or poll reply before giving up.
const REPLY_FRAME_BUDGET: usize = 64;

/// Driver for the NEO-xM family.
///
/// `S` is any `embedded-io[-async]` `Read + Write`, such as an embassy
/// `BufferedUart` or a serial port wrapped by `embedded-io-adapters`.
pub struct NeoGps<S> {
    uart: S,
    caps: Capabilities,
    deframer: Deframer,
    scratch: [u8; 64],
    /// Unconsumed tail of the last UART read.
    pending: usize,
    pending_pos: usize,
    /// Consecutive failed reads so far.
    read_err_streak: u8,
    /// How many consecutive failed reads to retry before surfacing `Io`.
    read_error_tolerance: u8,

    skip_unknown_sentences: bool,
}

/// Default for [`NeoGps::set_read_error_tolerance`].
pub const DEFAULT_READ_ERROR_TOLERANCE: u8 = 16;

#[maybe_async_cfg::maybe(sync(feature = "sync", keep_self), async(feature = "async", keep_self))]
impl<S: Read + Write> NeoGps<S> {
    /// Wrap a UART. Does no I/O. Capabilities start at
    /// [`Capabilities::conservative`] until you call [`probe`](Self::probe).
    ///
    /// # Example
    /// ```ignore
    /// let mut gps = NeoGps::new(uart); // uart: impl embedded_io_async::Read + Write
    /// ```
    pub fn new(uart: S) -> Self {
        NeoGps {
            uart,
            caps: Capabilities::conservative(),
            deframer: Deframer::new(),
            scratch: [0; 64],
            pending: 0,
            pending_pos: 0,
            read_err_streak: 0,
            read_error_tolerance: DEFAULT_READ_ERROR_TOLERANCE,
            skip_unknown_sentences: true,
        }
    }

    /// How many consecutive failed reads [`next_event`](Self::next_event)
    /// retries before returning [`Error::Io`]. `0` propagates the first one.
    ///
    /// # Example
    /// ```ignore
    /// gps.set_read_error_tolerance(0); // propagate every UART error
    /// ```
    pub fn set_read_error_tolerance(&mut self, n: u8) {
        self.read_error_tolerance = n;
    }

    /// Whether [`next_event`](Self::next_event) hides frames the driver does
    /// not decode ([`Event::NmeaOther`] and [`Event::UbxOther`]). `true` by
    /// default. Set it to `false` to decode them yourself from
    /// [`last_ubx_frame`](Self::last_ubx_frame) or
    /// [`last_nmea_line`](Self::last_nmea_line).
    ///
    /// # Example
    /// ```ignore
    /// gps.set_skip_unknown_sentences(false); // see GSV, PUBX, NAV-SAT, ...
    /// ```
    pub fn set_skip_unknown_sentences(&mut self, skip: bool) {
        self.skip_unknown_sentences = skip;
    }

    /// Release the UART.
    ///
    /// # Example
    /// ```ignore
    /// let uart = gps.free();
    /// ```
    pub fn free(self) -> S {
        self.uart
    }

    /// Currently assumed capabilities.
    ///
    /// # Example
    /// ```ignore
    /// let hz_max = 1000 / gps.capabilities().max_rate_ms() as u32;
    /// ```
    pub fn capabilities(&self) -> Capabilities {
        self.caps
    }

    /// Set the capabilities to assume, instead of calling
    /// [`probe`](Self::probe).
    ///
    /// # Example
    /// ```ignore
    /// gps.set_capabilities(Capabilities { generation: Generation::Series8, protocol_version: 18 });
    /// ```
    pub fn set_capabilities(&mut self, caps: Capabilities) {
        self.caps = caps;
    }

    /// Read and parse the next frame from the module.
    ///
    /// Frames the driver does not decode are skipped unless you call
    /// [`set_skip_unknown_sentences(false)`](Self::set_skip_unknown_sentences).
    ///
    /// # Example
    /// ```ignore
    /// while let Ok(ev) = gps.next_event().await {
    ///     if let Event::Nmea(Sentence::Rmc(rmc)) = ev { /* rmc.lat_1e7 ... */ }
    /// }
    /// ```
    pub async fn next_event(&mut self) -> Result<Event, Error<S::Error>> {
        loop {
            let ev = self.read_event().await?;
            if self.skip_unknown_sentences
                && matches!(ev, Event::NmeaOther { .. } | Event::UbxOther { .. })
            {
                continue;
            }
            return Ok(ev);
        }
    }

    /// Read frames until one carries a position with a confirmed fix, and
    /// return it. Everything else is discarded, so the result is never the
    /// pre-fix `0, 0`.
    ///
    /// Takes it from NMEA `RMC` or `GGA` (both on by default), or from
    /// `NAV-PVT` if you enabled it. Needs the `builtin-codec` feature, which
    /// is on by default.
    ///
    /// All three sources are judged equally strictly, so it does not matter
    /// which one arrives first: `RMC` must be status `A`, `NAV-PVT` must have
    /// `gnssFixOK`, and `GGA` must be
    /// [`is_gnss_fix`](crate::nmea::FixQuality::is_gnss_fix). A dead-reckoning
    /// or simulated position is not returned; use
    /// [`next_event`](Self::next_event) if you want those.
    ///
    /// # Example
    /// ```ignore
    /// loop {
    ///     let pos = gps.next_coordinate().await?;
    ///     info!("lat_1e7={} lon_1e7={}", pos.lat_1e7, pos.lon_1e7);
    /// }
    /// ```
    pub async fn next_coordinate(&mut self) -> Result<Coordinate, Error<S::Error>> {
        loop {
            let coord = match self.read_event().await? {
                Event::Nmea(Sentence::Rmc(rmc)) if rmc.valid => {
                    rmc.lat_1e7.zip(rmc.lon_1e7).map(|(lat, lon)| Coordinate {
                        lat_1e7: lat,
                        lon_1e7: lon,
                    })
                }
                Event::Nmea(Sentence::Gga(gga)) if gga.quality.is_gnss_fix() => {
                    gga.lat_1e7.zip(gga.lon_1e7).map(|(lat, lon)| Coordinate {
                        lat_1e7: lat,
                        lon_1e7: lon,
                    })
                }
                Event::NavPvt(pvt) if pvt.gnss_fix_ok() => Some(Coordinate {
                    lat_1e7: pvt.lat_1e7,
                    lon_1e7: pvt.lon_1e7,
                }),
                _ => None,
            };
            if let Some(c) = coord {
                return Ok(c);
            }
        }
    }

    /// The next frame, without the
    /// [`skip_unknown_sentences`](Self::set_skip_unknown_sentences) filter.
    async fn read_event(&mut self) -> Result<Event, Error<S::Error>> {
        loop {
            // Drain bytes already read from the UART first.
            while self.pending_pos < self.pending {
                let b = self.scratch[self.pending_pos];
                self.pending_pos += 1;
                if let Some(ev) = self.deframer.push(b) {
                    return Ok(ev);
                }
            }
            let n = match self.uart.read(&mut self.scratch).await {
                Ok(n) => {
                    self.read_err_streak = 0;
                    n
                }
                Err(e) => {
                    // Damaged bytes make the frame in progress suspect, so
                    // drop it rather than splice them into a length field.
                    self.deframer.reset();
                    self.read_err_streak = self.read_err_streak.saturating_add(1);
                    if self.read_err_streak > self.read_error_tolerance {
                        self.read_err_streak = 0;
                        return Err(Error::Io(e));
                    }
                    continue;
                }
            };
            if n == 0 {
                // Ok(0) is end-of-stream: the port is gone, retrying spins.
                return Err(Error::Eof);
            }
            self.pending = n;
            self.pending_pos = 0;
        }
    }

    /// Payload bytes of the most recent UBX frame from
    /// [`next_event`](Self::next_event), for decoding messages the driver
    /// does not wrap.
    ///
    /// Valid until the next UBX frame arrives. Payloads over 384 bytes are
    /// truncated.
    ///
    /// # Example
    /// ```ignore
    /// if let Event::UbxOther { class: 0x01, id: 0x35 } = ev { // NAV-SAT
    ///     let num_svs = gps.last_ubx_payload()[5];
    /// }
    /// ```
    pub fn last_ubx_payload(&self) -> &[u8] {
        self.deframer.last_ubx_payload()
    }

    /// The whole last UBX frame: sync bytes, header, payload, checksum. This
    /// is what external codecs such as the `ublox` crate take. Valid for as
    /// long as [`last_ubx_payload`](Self::last_ubx_payload).
    ///
    /// # Example
    /// ```ignore
    /// if let Event::UbxOther { .. } = ev {
    ///     codec::ublox::decode(&mut parser, gps.last_ubx_frame(), |pkt| { /* ... */ });
    /// }
    /// ```
    pub fn last_ubx_frame(&self) -> &[u8] {
        self.deframer.last_ubx_frame()
    }

    /// Bytes of the most recent NMEA sentence from
    /// [`next_event`](Self::next_event): the text between `$` and CR/LF,
    /// checksum trailer included. Use it to decode sentences the driver does
    /// not wrap (GSV, PUBX, ...) yourself or with the `nmea` crate.
    ///
    /// Valid until the next NMEA event.
    ///
    /// # Example
    /// ```ignore
    /// if let Event::NmeaOther { mtype, .. } = ev {
    ///     decode_gsv(gps.last_nmea_line()); // e.g. "GPGSV,3,1,...*hh"
    /// }
    /// ```
    pub fn last_nmea_line(&self) -> &[u8] {
        self.deframer.last_nmea_line()
    }

    /// Per-satellite data from the last `NAV-SAT` or `NAV-SVINFO` frame, or
    /// `None` if the last UBX frame was something else.
    ///
    /// Deliberately not an [`Event`] variant: these messages carry a few
    /// dozen satellites, which does not belong in a `Copy` enum. The iterator
    /// borrows the frame snapshot instead and decodes lazily, so it stays
    /// valid until the next UBX frame arrives.
    ///
    /// Enable the stream with
    /// [`enable_satellite_info`](Self::enable_satellite_info); both messages
    /// arrive as [`Event::UbxOther`], so
    /// [`set_skip_unknown_sentences(false)`](Self::set_skip_unknown_sentences)
    /// is needed to see them.
    ///
    /// # Example
    /// ```ignore
    /// if let Event::UbxOther { .. } = ev {
    ///     if let Some(sats) = gps.satellites() {
    ///         let used = sats.filter(|s| s.used_in_fix).count();
    ///     }
    /// }
    /// ```
    pub fn satellites(&self) -> Option<ubx::Satellites<'_>> {
        let frame = self.last_ubx_frame();
        if frame.len() < 8 || frame[2] != CLASS_NAV {
            return None;
        }
        match frame[3] {
            ubx::NAV_SAT => ubx::Satellites::from_nav_sat(self.last_ubx_payload()),
            ubx::NAV_SVINFO => ubx::Satellites::from_nav_svinfo(self.last_ubx_payload()),
            _ => None,
        }
    }

    /// Send a raw UBX frame, for messages the driver does not wrap.
    ///
    /// # Example
    /// ```ignore
    /// gps.send_ubx(0x06, 0x24, &cfg_nav5_payload).await?; // CFG-NAV5
    /// ```
    pub async fn send_ubx(
        &mut self,
        class: u8,
        id: u8,
        payload: &[u8],
    ) -> Result<(), Error<S::Error>> {
        let mut frame = UbxFrame::new(class, id);
        if frame.extend(payload).is_none() {
            return Err(Error::Overflow);
        }
        self.uart.write_all(frame.finish()).await?;
        Ok(())
    }

    /// Send a `CFG` frame and wait for its `ACK-ACK` or `ACK-NAK`.
    ///
    /// Frames arriving meanwhile are discarded, so use this while
    /// configuring, not while streaming fixes.
    ///
    /// # Example
    /// ```ignore
    /// gps.send_cfg_acked(0x24, &cfg_nav5_payload).await?; // Err(Nak{..}) if refused
    /// ```
    pub async fn send_cfg_acked(&mut self, id: u8, payload: &[u8]) -> Result<(), Error<S::Error>> {
        self.send_ubx(CLASS_CFG, id, payload).await?;
        for _ in 0..REPLY_FRAME_BUDGET {
            match self.read_event().await? {
                Event::Ack { class, id: aid, ok } if class == CLASS_CFG && aid == id => {
                    return if ok {
                        Ok(())
                    } else {
                        Err(Error::Nak { class, id: aid })
                    };
                }
                _ => {}
            }
        }
        Err(Error::NoReply)
    }

    /// Poll `UBX-MON-VER`, work out the module generation, and store it.
    ///
    /// Modules older than the `PROTVER` string (NEO-6) come back as
    /// [`Generation::Series6`].
    ///
    /// # Example
    /// ```ignore
    /// let caps = gps.probe().await?; // Series6/7/8/9Plus from MON-VER
    /// ```
    pub async fn probe(&mut self) -> Result<Capabilities, Error<S::Error>> {
        self.send_ubx(CLASS_MON, ubx::MON_VER, &[]).await?;
        for i in 0..REPLY_FRAME_BUDGET {
            // Re-poll once in case the first one was lost to line noise.
            if i == REPLY_FRAME_BUDGET / 2 {
                self.send_ubx(CLASS_MON, ubx::MON_VER, &[]).await?;
            }
            match self.read_event().await? {
                Event::UbxOther { class, id } if class == CLASS_MON && id == ubx::MON_VER => {
                    let protver = self.deframer.last_ubx_protver().unwrap_or(12);
                    let generation = match protver {
                        0..=13 => Generation::Series6,
                        14 => Generation::Series7,
                        15..=23 => Generation::Series8,
                        24..=33 => Generation::Series9,
                        _ => Generation::Series10,
                    };
                    self.caps = Capabilities {
                        generation,
                        protocol_version: protver,
                    };
                    return Ok(self.caps);
                }
                _ => {}
            }
        }
        Err(Error::NoReply)
    }

    /// Set the interval between fixes, in milliseconds (1000 = 1 Hz).
    /// Returns [`Error::Unsupported`] for rates the module cannot sustain.
    ///
    /// # Example
    /// ```ignore
    /// gps.set_nav_rate_ms(200).await?; // 5 Hz
    /// ```
    pub async fn set_nav_rate_ms(&mut self, ms: u16) -> Result<(), Error<S::Error>> {
        if !self.caps.has_legacy_cfg() || ms < self.caps.max_rate_ms_single_gnss() {
            return Err(Error::Unsupported);
        }
        let mut p = [0u8; 6];
        p[0..2].copy_from_slice(&ms.to_le_bytes());
        p[2..4].copy_from_slice(&1u16.to_le_bytes()); // navRate: 1 cycle
        p[4..6].copy_from_slice(&1u16.to_le_bytes()); // timeRef: GPS time
        self.send_cfg_acked(ubx::CFG_RATE, &p).await
    }

    /// Set how often a message is output on the current port (`CFG-MSG`).
    /// `rate = 0` disables it, `1` sends it with every solution.
    ///
    /// # Example
    /// ```ignore
    /// gps.set_msg_rate(0xF0, 0x03, 5).await?; // GSV every 5th fix
    /// ```
    pub async fn set_msg_rate(
        &mut self,
        class: u8,
        id: u8,
        rate: u8,
    ) -> Result<(), Error<S::Error>> {
        if !self.caps.has_legacy_cfg() {
            return Err(Error::Unsupported);
        }
        self.send_cfg_acked(ubx::CFG_MSG, &[class, id, rate]).await
    }

    /// Mute a standard NMEA sentence (class 0xF0).
    /// `id`: 0x00 GGA, 0x01 GLL, 0x02 GSA, 0x03 GSV, 0x04 RMC, 0x05 VTG.
    ///
    /// # Example
    /// ```ignore
    /// gps.disable_nmea(0x03).await?; // mute GSV
    /// ```
    pub async fn disable_nmea(&mut self, id: u8) -> Result<(), Error<S::Error>> {
        self.set_msg_rate(0xF0, id, 0).await
    }

    /// Start `UBX-NAV-PVT` output (7 series and later). Returns
    /// [`Error::Unsupported`] on older modules.
    ///
    /// # Example
    /// ```ignore
    /// if gps.capabilities().has_nav_pvt() { gps.enable_nav_pvt().await?; }
    /// ```
    pub async fn enable_nav_pvt(&mut self) -> Result<(), Error<S::Error>> {
        if !self.caps.has_nav_pvt() {
            return Err(Error::Unsupported);
        }
        self.set_msg_rate(CLASS_NAV, ubx::NAV_PVT, 1).await
    }

    /// Start `UBX-NAV-VELNED` output. Every generation has it, which makes it
    /// the way to get binary velocity out of a module with no `NAV-PVT`.
    ///
    /// # Example
    /// ```ignore
    /// gps.enable_nav_velned().await?; // binary velocity on a NEO-6M
    /// ```
    pub async fn enable_nav_velned(&mut self) -> Result<(), Error<S::Error>> {
        self.set_msg_rate(CLASS_NAV, ubx::NAV_VELNED, 1).await
    }

    /// Start whichever per-satellite message the module has: `NAV-SAT` on
    /// protocol 15 and later, `NAV-SVINFO` before that.
    ///
    /// Read the result with [`satellites`](Self::satellites).
    ///
    /// # Example
    /// ```ignore
    /// gps.probe().await?;
    /// gps.enable_satellite_info().await?;
    /// gps.set_skip_unknown_sentences(false); // both arrive as UbxOther
    /// ```
    pub async fn enable_satellite_info(&mut self) -> Result<(), Error<S::Error>> {
        if self.caps.has_nav_sat() {
            self.set_msg_rate(CLASS_NAV, ubx::NAV_SAT, 1).await
        } else if self.caps.has_nav_svinfo() {
            self.set_msg_rate(CLASS_NAV, ubx::NAV_SVINFO, 1).await
        } else {
            Err(Error::Unsupported)
        }
    }

    /// Start `UBX-NAV-POSLLH` output. Every generation has it.
    ///
    /// # Example
    /// ```ignore
    /// gps.enable_nav_posllh().await?; // binary position on a NEO-6M
    /// ```
    pub async fn enable_nav_posllh(&mut self) -> Result<(), Error<S::Error>> {
        self.set_msg_rate(CLASS_NAV, ubx::NAV_POSLLH, 1).await
    }

    /// Start `UBX-NAV-SOL` output (u-blox 6/7/8). Returns
    /// [`Error::Unsupported`] on M9 and later, which do not have it.
    ///
    /// # Example
    /// ```ignore
    /// gps.enable_nav_sol().await?; // fix status + DOP on a NEO-6M
    /// ```
    pub async fn enable_nav_sol(&mut self) -> Result<(), Error<S::Error>> {
        if !self.caps.has_nav_sol() {
            return Err(Error::Unsupported);
        }
        self.set_msg_rate(CLASS_NAV, ubx::NAV_SOL, 1).await
    }

    /// Start the best binary navigation output the module has: `NAV-PVT` on
    /// the 7 series and later, `NAV-POSLLH` plus `NAV-SOL` otherwise.
    ///
    /// Call it after [`probe`](Self::probe), then expect either
    /// [`Event::NavPvt`] or the [`Event::NavPosllh`] and [`Event::NavSol`]
    /// pair.
    ///
    /// # Example
    /// ```ignore
    /// gps.probe().await?;
    /// gps.enable_binary_nav().await?; // right messages for any NEO-xM
    /// ```
    pub async fn enable_binary_nav(&mut self) -> Result<(), Error<S::Error>> {
        if self.caps.has_nav_pvt() {
            self.enable_nav_pvt().await
        } else {
            self.enable_nav_posllh().await?;
            self.enable_nav_sol().await
        }
    }

    /// Tune the receiver's filters for how the antenna actually moves
    /// (`CFG-NAV5`). The factory default is
    /// [`Portable`](ubx::DynamicModel::Portable).
    ///
    /// # Example
    /// ```ignore
    /// use neo_gps::ubx::DynamicModel;
    /// gps.set_dynamic_model(DynamicModel::Airborne4G).await?;
    /// ```
    pub async fn set_dynamic_model(
        &mut self,
        model: ubx::DynamicModel,
    ) -> Result<(), Error<S::Error>> {
        if !self.caps.has_legacy_cfg() {
            return Err(Error::Unsupported);
        }
        let mut p = [0u8; 36];
        p[0..2].copy_from_slice(&0x0001u16.to_le_bytes()); // mask: dynModel only
        p[2] = model as u8;
        self.send_cfg_acked(ubx::CFG_NAV5, &p).await
    }

    /// Turn one GNSS constellation on or off (`CFG-GNSS`).
    ///
    /// Polls the module's current configuration, flips the enable bit for
    /// `which`, and writes it back, so the other constellations and their
    /// channel allocations are left exactly as they were.
    ///
    /// Only meaningful from the 8 series; check
    /// [`Capabilities::has_cfg_gnss`] first. Returns [`Error::Unsupported`]
    /// if the module does not report a block for `which`.
    ///
    /// # Example
    /// ```ignore
    /// use neo_gps::ubx::Constellation;
    /// if gps.capabilities().has_cfg_gnss() {
    ///     gps.set_constellation(Constellation::Glonass, true).await?;
    /// }
    /// ```
    pub async fn set_constellation(
        &mut self,
        which: ubx::Constellation,
        enable: bool,
    ) -> Result<(), Error<S::Error>> {
        if !self.caps.has_cfg_gnss() {
            return Err(Error::Unsupported);
        }
        self.send_ubx(CLASS_CFG, ubx::CFG_GNSS, &[]).await?;

        let mut cfg = [0u8; ubx::TX_MAX_PAYLOAD];
        let len = 'poll: {
            for _ in 0..REPLY_FRAME_BUDGET {
                if let Event::UbxOther { class, id } = self.read_event().await? {
                    if class == CLASS_CFG && id == ubx::CFG_GNSS {
                        let p = self.deframer.last_ubx_payload();
                        // 4-byte header then 8 bytes per configuration block.
                        if p.len() < 4 || p.len() > cfg.len() {
                            return Err(Error::Overflow);
                        }
                        cfg[..p.len()].copy_from_slice(p);
                        break 'poll p.len();
                    }
                }
            }
            return Err(Error::NoReply);
        };

        let blocks = cfg[3] as usize;
        for b in 0..blocks {
            let off = 4 + b * 8;
            if off + 8 > len {
                break;
            }
            if cfg[off] != which as u8 {
                continue;
            }
            if enable {
                cfg[off + 4] |= 0x01;
            } else {
                cfg[off + 4] &= !0x01;
            }
            return self.send_cfg_acked(ubx::CFG_GNSS, &cfg[..len]).await;
        }
        Err(Error::Unsupported)
    }

    /// Switch the receiver into power save mode, or back to continuous
    /// tracking (`CFG-RXM`).
    ///
    /// # Example
    /// ```ignore
    /// gps.set_power_save(true).await?; // battery-powered tracker
    /// ```
    pub async fn set_power_save(&mut self, on: bool) -> Result<(), Error<S::Error>> {
        if !self.caps.has_legacy_cfg() {
            return Err(Error::Unsupported);
        }
        // Byte 0 is reserved and the interface description says to send 8.
        self.send_cfg_acked(ubx::CFG_RXM, &[8, u8::from(on)]).await
    }

    /// Change the module's UART baud rate (`CFG-PRT`), leaving both UBX and
    /// NMEA enabled in and out at 8N1.
    ///
    /// **Your UART is not changed**, and cannot be: the driver only has a byte
    /// stream. The module switches as soon as the frame lands, so the two ends
    /// are mismatched from that moment until you reconfigure yours. There is
    /// also no ACK to wait for, since the reply would arrive at whichever rate
    /// the module has already moved to.
    ///
    /// # Example
    /// ```ignore
    /// gps.set_baud(115_200).await?;
    /// let uart = gps.free();                       // take the UART back
    /// let uart = reconfigure(uart, 115_200);       // your HAL's job
    /// let mut gps = NeoGps::new(uart);
    /// ```
    pub async fn set_baud(&mut self, baud: u32) -> Result<(), Error<S::Error>> {
        if !self.caps.has_legacy_cfg() {
            return Err(Error::Unsupported);
        }
        let mut p = [0u8; 20];
        p[0] = 1; // portID: UART1
        p[4..8].copy_from_slice(&0x0000_08C0u32.to_le_bytes()); // mode: 8N1
        p[8..12].copy_from_slice(&baud.to_le_bytes());
        p[12..14].copy_from_slice(&0x0007u16.to_le_bytes()); // in: UBX+NMEA+RTCM
        p[14..16].copy_from_slice(&0x0003u16.to_le_bytes()); // out: UBX+NMEA
        self.send_ubx(CLASS_CFG, ubx::CFG_PRT, &p).await
    }

    /// Restart the receiver (`CFG-RST`), keeping as much stored data as
    /// `kind` allows.
    ///
    /// Returns as soon as the frame is written. There is nothing to wait for:
    /// the module restarts instead of acknowledging, and goes quiet for a
    /// moment before its first sentence appears.
    ///
    /// # Example
    /// ```ignore
    /// use neo_gps::ubx::ResetKind;
    /// gps.reset(ResetKind::Cold).await?; // full restart, slowest fix
    /// ```
    pub async fn reset(&mut self, kind: ubx::ResetKind) -> Result<(), Error<S::Error>> {
        let mut p = [0u8; 4];
        p[0..2].copy_from_slice(&kind.bbr_mask().to_le_bytes());
        p[2] = 0x01; // controlled software reset
        self.send_ubx(CLASS_CFG, ubx::CFG_RST, &p).await
    }

    /// Wipe the saved configuration and load the factory defaults
    /// (`CFG-CFG`), undoing every setting including a saved
    /// [`save_config`](Self::save_config).
    ///
    /// The defaults land in RAM immediately. Follow with
    /// [`reset`](Self::reset) if you want the module to come up from scratch,
    /// and note this puts the baud rate back to 9600.
    ///
    /// Returns as soon as the frame is written, like [`reset`](Self::reset)
    /// and [`set_baud`](Self::set_baud). There is no ACK to rely on: the
    /// cleared configuration includes the port settings, so the module
    /// reinitialises the UART, and whether the acknowledgement escapes first
    /// is a race. Confirm by observing the defaults instead, for example a
    /// sentence you had muted coming back.
    ///
    /// # Example
    /// ```ignore
    /// gps.factory_reset().await?;
    /// // GSV is in the factory sentence set, so it returns on its own.
    /// ```
    pub async fn factory_reset(&mut self) -> Result<(), Error<S::Error>> {
        let mut p = [0u8; 12];
        p[0..4].copy_from_slice(&0x0000_FFFFu32.to_le_bytes()); // clearMask: all
        p[8..12].copy_from_slice(&0x0000_FFFFu32.to_le_bytes()); // loadMask: all
        self.send_ubx(CLASS_CFG, ubx::CFG_CFG, &p).await
    }

    /// Save the current configuration to battery-backed RAM and flash
    /// (`CFG-CFG`), so it survives a power cycle.
    ///
    /// # Example
    /// ```ignore
    /// gps.save_config().await?; // persists through power cycles (needs BBR/flash)
    /// ```
    pub async fn save_config(&mut self) -> Result<(), Error<S::Error>> {
        let mut p = [0u8; 12];
        p[4..8].copy_from_slice(&0x0000_FFFFu32.to_le_bytes()); // saveMask: all
        self.send_cfg_acked(ubx::CFG_CFG, &p).await
    }
}

#[cfg(all(test, feature = "builtin-codec"))]
mod tests;
#[cfg(all(test, any(feature = "nmea", feature = "ublox")))]
mod tests_codec;
#[cfg(all(test, feature = "builtin-codec"))]
mod tests_datasheet;
#[cfg(test)]
mod tests_transport;
#[cfg(test)]
pub(crate) mod testutil;

//! # neo-gps
//!
//! An async, `no_std`, zero-allocation driver for the u-blox NEO-xM GPS module
//! family (NEO-6M, NEO-7M, NEO-8M, NEO-M9N, ...) over UART.
//!
//! ## Example (embassy-style pseudocode)
//!
//! ```ignore
//! let mut gps = NeoGps::new(uart);
//! let caps = gps.probe().await?;           // optional; defaults are safe
//! gps.set_nav_rate_ms(caps.max_rate_ms()).await?;
//! loop {
//!     match gps.next_event().await? {
//!         Event::Nmea(Sentence::Rmc(rmc)) if rmc.valid => {
//!             let lat = rmc.lat_1e7; // degrees * 1e7
//!         }
//!         _ => {}
//!     }
//! }
//! ```

#![cfg_attr(not(test), no_std)]
#![deny(unsafe_code)]

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
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Error<E> {
    /// Underlying UART error
    Io(E),
    /// The UART read returned 0 bytes (EOF on adapted streams).
    Eof,
    /// The module answered a configuration frame with `ACK-NAK`.
    Nak { class: u8, id: u8 },
    /// No `ACK`/reply arrived within the frame budget.
    Timeout,
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
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Generation {
    /// NEO-6 series (protocol < 14). GPS only.
    Series6,
    /// NEO-7 series (protocol 14). GPS + GLONASS (not concurrent).
    Series7,
    /// NEO-8 series (protocol 15..=23). Concurrent multi-GNSS.
    Series8,
    /// M9/M10 and later (protocol >= 24).
    Series9Plus,
}

/// What the connected module can do. Filled by [`NeoGps::probe`],
/// or constructed manually if the module type is known at compile time.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Capabilities {
    pub generation: Generation,
    /// u-blox protocol version, e.g. 12 (NEO-6), 15..23 (NEO-8), 32 (M10).
    pub protocol_version: u8,
}

impl Capabilities {
    /// Conservative default used before/without probing: assume the oldest
    /// member of the family so every issued command is universally valid.
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

    /// Fastest supported navigation update interval, in milliseconds.
    ///
    /// # Example
    /// ```
    /// use neo_gps::Capabilities;
    /// assert_eq!(Capabilities::conservative().max_rate_ms(), 200); // NEO-6: 5 Hz
    /// ```
    pub fn max_rate_ms(&self) -> u16 {
        match self.generation {
            Generation::Series6 => 200,    // 5 Hz
            Generation::Series7 => 100,    // 10 Hz
            Generation::Series8 => 100,    // 10 Hz multi-GNSS capable
            Generation::Series9Plus => 40, // 25 Hz
        }
    }

    /// `UBX-NAV-PVT` (the all-in-one binary nav message) exists from protocol 14 on.
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

    /// `UBX-NAV-SOL` exists on protocols 12..=23 (u-blox 6/7/8); it was
    /// removed from protocol 24 (M9) on, where `NAV-PVT` fully replaces it.
    ///
    /// # Example
    /// ```
    /// use neo_gps::Capabilities;
    /// assert!(Capabilities::conservative().has_nav_sol()); // NEO-6: yes
    /// ```
    pub fn has_nav_sol(&self) -> bool {
        self.protocol_version < 24
    }

    /// `UBX-CFG-GNSS` (constellation selection) is meaningful from the 8 series.
    ///
    /// # Example
    /// ```
    /// use neo_gps::Capabilities;
    /// assert!(!Capabilities::conservative().has_cfg_gnss()); // not on NEO-6
    /// ```
    pub fn has_cfg_gnss(&self) -> bool {
        self.protocol_version >= 15
    }
}

/// A parsed item from the module's output stream.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Event {
    /// A supported, checksum-valid NMEA sentence.
    Nmea(Sentence),
    /// A checksum-valid NMEA sentence the driver does not decode
    /// (talker + type provided, e.g. `(b"GP", b"GSV")`).
    NmeaOther { talker: [u8; 2], mtype: [u8; 3] },
    /// `UBX-NAV-PVT` (only after [`NeoGps::enable_nav_pvt`], 7-series and later).
    NavPvt(ubx::NavPvt),
    /// `UBX-NAV-POSLLH` (after [`NeoGps::enable_nav_posllh`]; all generations,
    /// incl. NEO-6M).
    NavPosllh(ubx::NavPosllh),
    /// `UBX-NAV-SOL` (after [`NeoGps::enable_nav_sol`]; u-blox 6/7/8 only).
    NavSol(ubx::NavSol),
    /// `UBX-ACK-ACK` / `UBX-ACK-NAK` for a `CFG` frame.
    Ack { class: u8, id: u8, ok: bool },
    /// Any other checksum-valid UBX frame.
    UbxOther { class: u8, id: u8 },
}

/// How many parsed frames [`NeoGps`] will process while waiting for an ACK or
/// poll reply before giving up. At the default 1 Hz output this is a few
/// seconds' worth of traffic; it bounds the wait without needing a timer.
const REPLY_FRAME_BUDGET: usize = 64;

/// Async driver for the NEO-xM family.
///
/// `S` is any async byte stream: an embassy `BufferedUart`, an
/// `embedded-io-adapters` wrapped serial port, etc.
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
}

/// Default number of consecutive read errors [`NeoGps::next_event`] retries
/// before giving up: comfortably above real-world glitch bursts (1–3 at
/// power-up), far below "spins forever on a broken UART".
pub const DEFAULT_READ_ERROR_TOLERANCE: u8 = 16;

#[maybe_async_cfg::maybe(sync(feature = "sync", keep_self), async(feature = "async", keep_self))]
impl<S: Read + Write> NeoGps<S> {
    /// Wrap an async UART. No I/O is performed; capabilities default to
    /// [`Capabilities::conservative`] until [`probe`](Self::probe) is called.
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
        }
    }

    /// How many *consecutive* failed `read()`s to retry (with a deframer
    /// resync) before [`next_event`](Self::next_event) surfaces
    /// [`Error::Io`]. `0` restores fail-fast: the first read error propagates.
    ///
    /// # Example
    /// ```ignore
    /// gps.set_read_error_tolerance(0); // propagate every UART error
    /// ```
    pub fn set_read_error_tolerance(&mut self, n: u8) {
        self.read_error_tolerance = n;
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

    /// Override capabilities (e.g. known module type, skipping the probe).
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
    /// This is the main pump: call it in a loop. Unknown-but-valid frames are
    /// reported (not swallowed) so the application can layer its own decoding
    /// on top without forking the driver.
    ///
    /// # Example
    /// ```ignore
    /// while let Ok(ev) = gps.next_event().await {
    ///     if let Event::Nmea(Sentence::Rmc(rmc)) = ev { /* rmc.lat_1e7 ... */ }
    /// }
    /// ```
    pub async fn next_event(&mut self) -> Result<Event, Error<S::Error>> {
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
                    // A read error means byte(s) at this position were damaged
                    // (framing violation, noise, power-up glitch), so any frame
                    // in progress is suspect. Resetting bounds the damage to
                    // that one frame: continuing instead could splice a damaged
                    // byte into a UBX length field and swallow up to 64 KiB of
                    // good frames behind a bogus length. In the common case —
                    // the power-up glitch, which fires between frames — the
                    // parser is idle and the reset is a no-op.
                    self.deframer.reset();
                    self.read_err_streak = self.read_err_streak.saturating_add(1);
                    if self.read_err_streak > self.read_error_tolerance {
                        // This many in a row is a broken link, not a glitch.
                        self.read_err_streak = 0;
                        return Err(Error::Io(e));
                    }
                    continue;
                }
            };
            if n == 0 {
                // Ok(0) is end-of-stream on adapted transports: the port is
                // gone, retrying would spin forever. Always fatal.
                return Err(Error::Eof);
            }
            self.pending = n;
            self.pending_pos = 0;
        }
    }

    /// Raw payload bytes of the most recent UBX frame returned by
    /// [`next_event`](Self::next_event) (as [`Event::UbxOther`],
    /// [`Event::Ack`], or [`Event::NavPvt`]).
    ///
    /// Use this to decode messages the driver doesn't wrap — e.g. hand the
    /// bytes to the `ublox` crate's typed parsers after receiving
    /// `Event::UbxOther { class, id }`. Payloads longer than the internal
    /// buffer (384 bytes) are truncated; framing is unaffected.
    ///
    /// Valid until the next call to `next_event` completes another UBX frame.
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

    /// The complete last UBX frame (sync bytes, header, payload, checksum) —
    /// the form external codecs such as the `ublox` crate consume. Same
    /// snapshot stability guarantee as [`last_ubx_payload`](Self::last_ubx_payload).
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

    /// Bytes of the most recent checksum-valid NMEA sentence returned by
    /// [`next_event`](Self::next_event) (as [`Event::Nmea`] or
    /// [`Event::NmeaOther`]) — the text between `$` and CR/LF, checksum
    /// trailer included.
    ///
    /// Use this to decode sentences the driver doesn't wrap (GSV, PUBX, ...)
    /// with your own code or the `nmea` crate. Checksum-invalid lines are
    /// never retained here, so the bytes always correspond to the last
    /// NMEA event actually emitted.
    ///
    /// The bytes are a snapshot taken when the event was emitted: they remain
    /// stable until the *next NMEA event* is returned, even if corrupt or
    /// partial sentences are processed in between.
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

    /// Send a raw UBX frame (escape hatch for messages the driver doesn't wrap).
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

    /// Send a `CFG`-class frame and wait for its `ACK-ACK`/`ACK-NAK`.
    /// Non-matching frames received meanwhile are parsed and *discarded*;
    /// use this during configuration, not concurrently with fix streaming.
    ///
    /// # Example
    /// ```ignore
    /// gps.send_cfg_acked(0x24, &cfg_nav5_payload).await?; // Err(Nak{..}) if refused
    /// ```
    pub async fn send_cfg_acked(&mut self, id: u8, payload: &[u8]) -> Result<(), Error<S::Error>> {
        self.send_ubx(CLASS_CFG, id, payload).await?;
        for _ in 0..REPLY_FRAME_BUDGET {
            match self.next_event().await? {
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
        Err(Error::Timeout)
    }

    /// Poll `UBX-MON-VER`, detect the module generation, and store the result.
    ///
    /// Works across the whole family: modules that predate the `PROTVER`
    /// extension string (NEO-6) are classified as [`Generation::Series6`].
    ///
    /// # Example
    /// ```ignore
    /// let caps = gps.probe().await?; // Series6/7/8/9Plus from MON-VER
    /// ```
    pub async fn probe(&mut self) -> Result<Capabilities, Error<S::Error>> {
        self.send_ubx(CLASS_MON, ubx::MON_VER, &[]).await?;
        for i in 0..REPLY_FRAME_BUDGET {
            // If the outbound poll was eaten by the same line noise the RX
            // path tolerates, no reply will ever come; re-polling once at
            // half budget is free (MON-VER polls are side-effect-free) and
            // converts "lost poll" from a guaranteed Timeout into a delay.
            if i == REPLY_FRAME_BUDGET / 2 {
                self.send_ubx(CLASS_MON, ubx::MON_VER, &[]).await?;
            }
            match self.next_event().await? {
                Event::UbxOther { class, id } if class == CLASS_MON && id == ubx::MON_VER => {
                    let protver = self.deframer.last_ubx_protver().unwrap_or(12);
                    let generation = match protver {
                        0..=13 => Generation::Series6,
                        14 => Generation::Series7,
                        15..=23 => Generation::Series8,
                        _ => Generation::Series9Plus,
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
        Err(Error::Timeout)
    }

    /// Set the navigation solution rate. `ms` is the interval between fixes
    /// (1000 = 1 Hz). Rejects rates the detected module cannot sustain.
    ///
    /// # Example
    /// ```ignore
    /// gps.set_nav_rate_ms(200).await?; // 5 Hz
    /// ```
    pub async fn set_nav_rate_ms(&mut self, ms: u16) -> Result<(), Error<S::Error>> {
        if ms < self.caps.max_rate_ms() {
            return Err(Error::Unsupported);
        }
        let mut p = [0u8; 6];
        p[0..2].copy_from_slice(&ms.to_le_bytes());
        p[2..4].copy_from_slice(&1u16.to_le_bytes()); // navRate: 1 cycle
        p[4..6].copy_from_slice(&1u16.to_le_bytes()); // timeRef: GPS time
        self.send_cfg_acked(ubx::CFG_RATE, &p).await
    }

    /// Set the per-message output rate of an NMEA or UBX message on the
    /// current port (`CFG-MSG`). `rate = 0` disables, `1` = every solution.
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
        self.send_cfg_acked(ubx::CFG_MSG, &[class, id, rate]).await
    }

    /// Convenience: disable an NMEA standard sentence (class 0xF0).
    /// `id`: 0x00 GGA, 0x01 GLL, 0x02 GSA, 0x03 GSV, 0x04 RMC, 0x05 VTG.
    ///
    /// # Example
    /// ```ignore
    /// gps.disable_nmea(0x03).await?; // mute GSV
    /// ```
    pub async fn disable_nmea(&mut self, id: u8) -> Result<(), Error<S::Error>> {
        self.set_msg_rate(0xF0, id, 0).await
    }

    /// Enable periodic `UBX-NAV-PVT` output (7-series and later).
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

    /// Enable periodic `UBX-NAV-POSLLH` output (all generations, incl. NEO-6M).
    ///
    /// # Example
    /// ```ignore
    /// gps.enable_nav_posllh().await?; // binary position on a NEO-6M
    /// ```
    pub async fn enable_nav_posllh(&mut self) -> Result<(), Error<S::Error>> {
        self.set_msg_rate(CLASS_NAV, ubx::NAV_POSLLH, 1).await
    }

    /// Enable periodic `UBX-NAV-SOL` output (u-blox 6/7/8; refused on M9+
    /// where the message no longer exists).
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

    /// Enable the best binary navigation output the detected module offers:
    /// `NAV-PVT` where it exists (7-series+), otherwise `NAV-POSLLH` +
    /// `NAV-SOL` (NEO-6M). Call after [`probe`](Self::probe); afterwards
    /// expect [`Event::NavPvt`] *or* the [`Event::NavPosllh`]/[`Event::NavSol`]
    /// pair, depending on the module.
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

    /// Save the current configuration to battery-backed RAM + flash/EEPROM
    /// where present (`CFG-CFG`), so it survives power cycles.
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

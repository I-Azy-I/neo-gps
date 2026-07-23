//! Adapters from this driver's framed output to external vocabulary crates.
//!
//! The driver's own scope is transport: deframing the interleaved NMEA/UBX
//! stream, checksums, the async/sync pump, ACK correlation, and family
//! capability policy. Message *vocabulary* can be delegated:
//!
//! * feature `nmea` — the [`nmea`](https://crates.io/crates/nmea) crate
//!   decodes sentences from [`NeoGps::last_nmea_line`](crate::NeoGps::last_nmea_line).
//! * feature `ublox` — the [`ublox`](https://crates.io/crates/ublox) crate
//!   decodes packets from [`NeoGps::last_ubx_frame`](crate::NeoGps::last_ubx_frame).
//!
//! With `default-features = false` plus `async`/`sync` and these codec
//! features, the built-in decoders compile out entirely: every valid frame
//! surfaces as `Event::NmeaOther` / `Event::UbxOther`, and these adapters
//! supply the typed view.

/// External NMEA decoding via the `nmea` crate (feature `nmea`).
#[cfg(feature = "nmea")]
pub mod nmea {
    use nmea as nmea_crate;

    /// Why an adapter call produced no result.
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub enum DecodeError {
        /// Line exceeds the reassembly buffer (never happens for lines that
        /// came out of this driver's deframer).
        TooLong,
        /// The line is not valid UTF-8.
        Utf8,
        /// The `nmea` crate rejected the sentence.
        Parse,
    }

    /// Decode one framed sentence with the `nmea` crate.
    ///
    /// Input is exactly what [`last_nmea_line`](crate::NeoGps::last_nmea_line)
    /// returns: the bytes between `$` and CR/LF, checksum trailer included.
    /// The adapter re-adds the leading `$` the external crate expects.
    ///
    /// # Example
    /// ```ignore
    /// if let Event::NmeaOther { .. } = gps.next_event().await? {
    ///     if let Ok(nmea::ParseResult::GSV(gsv)) =
    ///         neo_gps::codec::nmea::decode(gps.last_nmea_line())
    ///     { /* per-satellite data, decoded externally */ }
    /// }
    /// ```
    pub fn decode(line: &[u8]) -> Result<nmea_crate::ParseResult, DecodeError> {
        let mut buf = [0u8; 1 + crate::NMEA_MAX];
        if line.len() > crate::NMEA_MAX {
            return Err(DecodeError::TooLong);
        }
        buf[0] = b'$';
        buf[1..1 + line.len()].copy_from_slice(line);
        let s = core::str::from_utf8(&buf[..1 + line.len()]).map_err(|_| DecodeError::Utf8)?;
        nmea_crate::parse_str(s).map_err(|_| DecodeError::Parse)
    }
}

/// External UBX decoding via the `ublox` crate (feature `ublox`).
#[cfg(feature = "ublox")]
pub mod ublox {
    use ublox as ublox_crate;

    /// Decode one complete framed UBX packet with the `ublox` crate,
    /// handing the typed [`PacketRef`](ublox_crate::PacketRef) to a closure.
    ///
    /// Input is exactly what [`last_ubx_frame`](crate::NeoGps::last_ubx_frame)
    /// returns: sync bytes through checksum. The closure style is imposed by
    /// the `ublox` parser's lifetimes (the packet borrows the parser's
    /// buffer); returns `None` if the crate does not recognise the packet.
    ///
    /// # Example
    /// ```ignore
    /// if let Event::UbxOther { .. } = gps.next_event().await? {
    ///     let mut parser = ublox::Parser::default();
    ///     let itow = neo_gps::codec::ublox::decode(&mut parser, gps.last_ubx_frame(), |pkt| {
    ///         if let ublox::PacketRef::NavSat(sat) = pkt { /* ... */ }
    ///     });
    /// }
    /// ```
    pub fn decode<B, R>(
        parser: &mut ublox_crate::Parser<B>,
        frame: &[u8],
        f: impl FnOnce(ublox_crate::PacketRef<'_>) -> R,
    ) -> Option<R>
    where
        B: ublox_crate::UnderlyingBuffer,
    {
        let mut it = parser.consume(frame);
        loop {
            match it.next() {
                Some(Ok(pkt)) => return Some(f(pkt)),
                Some(Err(_)) => continue,
                None => return None,
            }
        }
    }
}

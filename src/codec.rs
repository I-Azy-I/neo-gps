//! Hand the driver's frames to external decoding crates.
//!
//! * feature `nmea`: the [`nmea`](https://crates.io/crates/nmea) crate
//!   decodes sentences from [`NeoGps::last_nmea_line`](crate::NeoGps::last_nmea_line).
//! * feature `ublox`: the [`ublox`](https://crates.io/crates/ublox) crate
//!   decodes packets from [`NeoGps::last_ubx_frame`](crate::NeoGps::last_ubx_frame).
//!
//! Turn off `builtin-codec` to use these on their own. Every valid frame then
//! arrives as `Event::NmeaOther` or `Event::UbxOther` for you to decode here.

/// NMEA decoding with the `nmea` crate. Needs the `nmea` feature.
#[cfg(feature = "nmea")]
#[cfg_attr(docsrs, doc(cfg(feature = "nmea")))]
pub mod nmea {
    use nmea as nmea_crate;

    /// Why [`decode`] returned nothing.
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub enum DecodeError {
        /// The line is longer than an NMEA sentence can be.
        TooLong,
        /// The line is not valid UTF-8.
        Utf8,
        /// The `nmea` crate rejected the sentence.
        Parse,
    }

    /// Decode one sentence with the `nmea` crate.
    ///
    /// Pass it [`last_nmea_line`](crate::NeoGps::last_nmea_line) as-is. The
    /// leading `$` the external crate wants is added for you.
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

/// UBX decoding with the `ublox` crate. Needs the `ublox` feature.
#[cfg(feature = "ublox")]
#[cfg_attr(docsrs, doc(cfg(feature = "ublox")))]
pub mod ublox {
    use ublox as ublox_crate;

    /// Decode one UBX packet with the `ublox` crate and pass it to `f`.
    ///
    /// Pass it [`last_ubx_frame`](crate::NeoGps::last_ubx_frame) as-is: the
    /// whole frame, sync bytes through checksum. `f` takes the packet because
    /// it borrows the parser's buffer. `None` if the crate does not recognise
    /// the packet.
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

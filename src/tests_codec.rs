//! Codec-adapter tests. Where `builtin-codec` is also enabled, these
//! cross-validate: the external crate and the built-in decoder must agree
//! on the same framed bytes: two independent implementations checking
//! each other.

use crate::testutil::*;
use crate::ubx as ubx_int;
use crate::*;

#[cfg(feature = "nmea")]
mod nmea_ext {
    use super::*;
    use ::nmea::ParseResult;

    fn last_line_of(body: &str) -> Vec<u8> {
        let mut gps = NeoGps::new(MockUart::new(nmea_wire(body)));
        gps.set_skip_unknown_sentences(false); // undecoded sentences must surface
        block_on(gps.next_event()).unwrap();
        gps.last_nmea_line().to_vec()
    }

    /// The nmea crate accepts what our deframer emits, and agrees with the
    /// built-in decoder's coordinate conversion within f64 rounding.
    #[test]
    fn external_crate_decodes_deframed_gga() {
        let line =
            last_line_of("GPGGA,092725.00,4717.11399,N,00833.91590,E,1,08,1.01,499.6,M,48.0,M,,");
        let ParseResult::GGA(gga) = codec::nmea::decode(&line).unwrap() else {
            panic!("expected GGA");
        };
        let lat = gga.latitude.unwrap();
        let lon = gga.longitude.unwrap();
        // Built-in: 472852332 / 85652650 at 1e-7 deg.
        assert!((lat - 47.2852332).abs() < 1e-6, "{lat}");
        assert!((lon - 8.5652650).abs() < 1e-6, "{lon}");
        assert_eq!(gga.fix_satellites, Some(8));
    }

    /// GSV, a sentence the built-in deliberately does not decode, is the
    /// canonical use of the adapter: NmeaOther + external decode.
    #[test]
    fn external_crate_decodes_gsv_we_dont() {
        let line = last_line_of("GLGSV,3,1,09,65,04,037,,66,55,061,20,67,52,131,29,68,05,176,");
        match codec::nmea::decode(&line) {
            Ok(ParseResult::GSV(_)) => {}
            other => panic!("expected GSV, got {other:?}"),
        }
    }

    #[test]
    fn garbage_is_rejected_not_panicking() {
        assert!(codec::nmea::decode(b"GPXXX,not,a,real*00").is_err());
        assert!(codec::nmea::decode(&[0xFF; 40]).is_err());
    }
}

#[cfg(feature = "ublox")]
mod ublox_ext {
    use super::*;
    use ::ublox::{FixedLinearBuffer, PacketRef, Parser};

    /// The ublox crate parses the full frames our deframer snapshots, and
    /// agrees with the built-in NAV-PVT field extraction.
    #[test]
    fn external_crate_decodes_snapshotted_nav_pvt() {
        let mut p = [0u8; 92];
        p[4..6].copy_from_slice(&2026u16.to_le_bytes());
        p[20] = 3;
        p[21] = 0x01;
        p[23] = 14;
        p[24..28].copy_from_slice(&61432160i32.to_le_bytes());
        p[28..32].copy_from_slice(&462025986i32.to_le_bytes());

        let mut gps = NeoGps::new(MockUart::new(ubx_wire(
            ubx_int::CLASS_NAV,
            ubx_int::NAV_PVT,
            &p,
        )));
        // Without `builtin-codec` NAV-PVT arrives as UbxOther, which the
        // default filter hides.
        gps.set_skip_unknown_sentences(false);
        block_on(gps.next_event()).unwrap();
        let frame = gps.last_ubx_frame().to_vec();

        let mut buf = [0u8; 512];
        let mut parser = Parser::new(FixedLinearBuffer::new(&mut buf));
        let checked = codec::ublox::decode(&mut parser, &frame, |pkt| match pkt {
            PacketRef::NavPvt(pvt) => {
                // f64 round-trip through the external crate: tolerance, not
                // equality (the builtin's i32 1e-7 representation is exact).
                assert!((pvt.lat_degrees() - 46.2025986).abs() < 1e-9);
                assert!((pvt.lon_degrees() - 6.1432160).abs() < 1e-9);
                assert_eq!(pvt.num_satellites(), 14);
                true
            }
            _ => false,
        });
        assert_eq!(checked, Some(true), "ublox crate must recognise NAV-PVT");
    }

    /// MON-VER through the external crate: its extension iteration should
    /// see the same PROTVER our probe classification uses.
    #[test]
    fn external_crate_reads_mon_ver_extensions() {
        let payload = mon_ver_payload(
            "ROM CORE 3.01 (107888)",
            "00080000",
            &["FWVER=SPG 3.01", "PROTVER=18.00"],
        );
        let mut gps = NeoGps::new(MockUart::new(ubx_wire(
            ubx_int::CLASS_MON,
            ubx_int::MON_VER,
            &payload,
        )));
        gps.set_skip_unknown_sentences(false); // MON-VER surfaces as UbxOther
        block_on(gps.next_event()).unwrap();
        let frame = gps.last_ubx_frame().to_vec();

        let mut buf = [0u8; 512];
        let mut parser = Parser::new(FixedLinearBuffer::new(&mut buf));
        let found = codec::ublox::decode(&mut parser, &frame, |pkt| match pkt {
            PacketRef::MonVer(v) => v.extension().any(|e| e.contains("PROTVER=18.00")),
            _ => false,
        });
        assert_eq!(found, Some(true));
    }
}

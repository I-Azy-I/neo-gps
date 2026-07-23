//! Tests derived from the official u-blox protocol specifications:
//!
//! * u-blox 6 Receiver Description (GPS.G6-SW-10018-F, FW 7.03) — NMEA 2.3,
//!   GP talker, no NAV-PVT in the NAV class.
//! * u-blox 7 Receiver Description (GPS.G7-SW-12001, protocol 14) — NAV-PVT
//!   introduced with an 84-byte payload.
//! * u-blox 8 / M8 Receiver Description (UBX-13003221 R17, protocols
//!   15.00–23.01) — GN main talker, NMEA 4.0/4.1, 92-byte NAV-PVT,
//!   MON-VER `PROTVER 15.00` (proto <= 17) vs `PROTVER=18.00` (proto >= 18)
//!   extension formats, firmware/protocol version table.

use crate::testutil::{block_on, feed, mon_ver_payload, nmea_wire, ubx_wire, MockUart};
use crate::ubx::{self};
use crate::*;

/// Run `probe()` against a scripted MON-VER reply and return the result.
fn probe_with(exts: &[&str], sw: &str) -> Capabilities {
    let rx = ubx_wire(ubx::CLASS_MON, ubx::MON_VER, &mon_ver_payload(sw, "00080000", exts));
    let mut gps = NeoGps::new(MockUart::new(rx));
    block_on(gps.probe()).unwrap()
}

// ===========================================================================
// NEO-6M — u-blox 6, FW 7.03, protocol 12/13. GPS-only, NMEA 2.3, GP talker.
// ===========================================================================
mod neo6 {
    use super::*;

    /// FW 7.03 MON-VER carries no PROTVER extension at all; its absence is
    /// the 6-series signature and must classify as Series6.
    #[test]
    fn probe_classifies_missing_protver_as_series6() {
        let caps = probe_with(&[], "7.03 (45969)");
        assert_eq!(caps.generation, Generation::Series6);
        assert!(!caps.has_nav_pvt(), "u-blox 6 NAV class has no PVT");
        assert!(!caps.has_cfg_gnss());
        assert_eq!(caps.max_rate_ms(), 200, "u-blox 6 maxes out at 5 Hz");
    }

    /// The spec's NMEA chapter lists these standard sentences for u-blox 6.
    /// GGA/RMC/GSA decode; the rest must surface (not vanish) as NmeaOther
    /// with the GP talker preserved.
    #[test]
    fn full_default_sentence_cycle() {
        let mut rx = Vec::new();
        rx.extend(nmea_wire("GPGGA,092725.00,4717.11399,N,00833.91590,E,1,08,1.01,499.6,M,48.0,M,,"));
        rx.extend(nmea_wire("GPGLL,4717.11364,N,00833.91565,E,092321.00,A,A"));
        rx.extend(nmea_wire("GPGSA,A,3,29,26,31,21,25,16,05,,,,,,2.08,1.29,1.63"));
        rx.extend(nmea_wire("GPGSV,3,1,10,05,17,222,25,10,05,074,,16,35,296,29,21,29,067,32"));
        rx.extend(nmea_wire("GPRMC,083559.00,A,4717.11437,N,00833.91522,E,0.004,77.52,091202,,,A"));
        rx.extend(nmea_wire("GPVTG,77.52,T,,M,0.004,N,0.007,K,A"));
        let evs = feed(&rx);
        assert_eq!(evs.len(), 6);
        assert!(matches!(evs[0], Event::Nmea(nmea::Sentence::Gga(_))));
        assert_eq!(evs[1], Event::NmeaOther { talker: *b"GP", mtype: *b"GLL" });
        assert!(matches!(evs[2], Event::Nmea(nmea::Sentence::Gsa(_))));
        assert_eq!(evs[3], Event::NmeaOther { talker: *b"GP", mtype: *b"GSV" });
        assert!(matches!(evs[4], Event::Nmea(nmea::Sentence::Rmc(_))));
        assert_eq!(evs[5], Event::NmeaOther { talker: *b"GP", mtype: *b"VTG" });
    }

    /// NMEA 2.3 RMC ends with the mode indicator (no navStatus field).
    /// Spec: "$GPRMC,hhmmss.ss,A,....,ddmmyy,,,A" — 12 fields.
    #[test]
    fn rmc_nmea23_field_count() {
        let evs = feed(&nmea_wire(
            "GPRMC,162254.00,A,3723.02837,N,12159.39853,W,0.820,188.36,110706,,,A",
        ));
        let Event::Nmea(nmea::Sentence::Rmc(r)) = evs[0] else { panic!() };
        assert!(r.valid);
        assert_eq!(r.date, Some(nmea::Date { day: 11, month: 7, year: 6 }));
        assert!(r.lon_1e7.unwrap() < 0);
    }

    /// TXT sentences (boot screen over NMEA) are common on 6M power-up and
    /// must not confuse the parser.
    #[test]
    fn boot_txt_sentences_pass_through() {
        let evs = feed(&nmea_wire("GPTXT,01,01,02,u-blox ag - www.u-blox.com"));
        assert_eq!(evs[0], Event::NmeaOther { talker: *b"GP", mtype: *b"TXT" });
    }

    /// Feature gating: enable_nav_pvt must fail locally on a probed 6-series,
    /// without touching the wire.
    #[test]
    fn nav_pvt_refused_without_io() {
        let rx = ubx_wire(ubx::CLASS_MON, ubx::MON_VER, &mon_ver_payload("7.03 (45969)", "00040007", &[]));
        let mut gps = NeoGps::new(MockUart::new(rx));
        block_on(gps.probe()).unwrap();
        let tx_before = gps.free_len_tx();
        assert_eq!(block_on(gps.enable_nav_pvt()), Err(Error::Unsupported));
        assert_eq!(gps.free_len_tx(), tx_before, "no bytes may be sent");
    }


    /// NAV-POSLLH (28 bytes, GPS.G6-SW-10018 §35.6): the 6-series binary
    /// position message. Offsets: lon@4, lat@8, height@12, hMSL@16,
    /// hAcc@20, vAcc@24, all mm / 1e-7 deg.
    #[test]
    fn nav_posllh_parses() {
        let mut p = [0u8; 28];
        p[0..4].copy_from_slice(&123000u32.to_le_bytes());
        p[4..8].copy_from_slice(&85652650i32.to_le_bytes()); // 8.5652650 E
        p[8..12].copy_from_slice(&472852332i32.to_le_bytes()); // 47.2852332 N
        p[16..20].copy_from_slice(&499_600i32.to_le_bytes()); // hMSL 499.6 m
        p[20..24].copy_from_slice(&2500u32.to_le_bytes()); // hAcc 2.5 m
        let evs = feed(&ubx_wire(ubx::CLASS_NAV, ubx::NAV_POSLLH, &p));
        let Event::NavPosllh(pos) = evs[0] else { panic!("{:?}", evs[0]) };
        assert_eq!(pos.lat_1e7, 472852332);
        assert_eq!(pos.lon_1e7, 85652650);
        assert_eq!(pos.hmsl_mm, 499_600);
        assert_eq!(pos.h_acc_mm, 2500);
    }

    /// NAV-SOL (52 bytes, §35.11): gpsFix@10, flags@11 (bit0 GPSfixOK),
    /// pAcc@24 cm, pDOP@44 (x0.01), numSV@47.
    #[test]
    fn nav_sol_parses_with_fix_flags() {
        let mut p = [0u8; 52];
        p[8..10].copy_from_slice(&2374u16.to_le_bytes()); // GPS week
        p[10] = 3; // 3D fix
        p[11] = 0x0D; // GPSfixOK | WKNSET | TOWSET
        p[24..28].copy_from_slice(&310u32.to_le_bytes()); // pAcc 3.10 m
        p[44..46].copy_from_slice(&180u16.to_le_bytes()); // pDOP 1.80
        p[47] = 7;
        let Event::NavSol(sol) = feed(&ubx_wire(ubx::CLASS_NAV, ubx::NAV_SOL, &p))[0]
        else { panic!() };
        assert_eq!(sol.gps_fix, 3);
        assert!(sol.gps_fix_ok());
        assert_eq!(sol.week, 2374);
        assert_eq!(sol.p_acc_cm, 310);
        assert_eq!(sol.pdop_1e2, 180);
        assert_eq!(sol.num_sv, 7);
    }

    /// Truncated NAV-SOL degrades to UbxOther instead of mis-decoding.
    #[test]
    fn nav_sol_short_degrades() {
        let evs = feed(&ubx_wire(ubx::CLASS_NAV, ubx::NAV_SOL, &[0u8; 40]));
        assert_eq!(evs[0], Event::UbxOther { class: ubx::CLASS_NAV, id: ubx::NAV_SOL });
    }

    /// enable_binary_nav on a probed 6-series must fall back to
    /// POSLLH + SOL (two CFG-MSG frames), since NAV-PVT doesn't exist.
    #[test]
    fn binary_nav_routes_to_posllh_sol_on_series6() {
        let mut rx = ubx_wire(ubx::CLASS_MON, ubx::MON_VER,
            &mon_ver_payload("7.03 (45969)", "00040007", &[]));
        rx.extend(ubx_wire(ubx::CLASS_ACK, 0x01, &[ubx::CLASS_CFG, ubx::CFG_MSG]));
        rx.extend(ubx_wire(ubx::CLASS_ACK, 0x01, &[ubx::CLASS_CFG, ubx::CFG_MSG]));
        let mut gps = NeoGps::new(MockUart::new(rx));
        block_on(gps.probe()).unwrap();
        block_on(gps.enable_binary_nav()).unwrap();
        // Two CFG-MSG frames on the wire: rate 1 for NAV-POSLLH and NAV-SOL.
        let tx = &gps.uart_ref().tx;
        for id in [ubx::NAV_POSLLH, ubx::NAV_SOL] {
            let payload = [ubx::CLASS_NAV, id, 1];
            assert!(tx.windows(3).any(|w| w == payload), "CFG-MSG for 0x{id:02X} missing");
        }
        assert!(!tx.windows(3).any(|w| w == [ubx::CLASS_NAV, ubx::NAV_PVT, 1]),
                "must not try NAV-PVT on a 6-series");
    }

    /// 5 Hz (200 ms) is the fastest CFG-RATE the 6-series sustains; the
    /// driver must accept 200 and reject anything faster.
    #[test]
    fn rate_limits() {
        let mut gps = NeoGps::new(MockUart::new(
            ubx_wire(ubx::CLASS_ACK, 0x01, &[ubx::CLASS_CFG, ubx::CFG_RATE]),
        ));
        assert_eq!(block_on(gps.set_nav_rate_ms(100)), Err(Error::Unsupported));
        block_on(gps.set_nav_rate_ms(200)).unwrap();
    }
}

// ===========================================================================
// NEO-7M — u-blox 7, protocol 14. GPS+GLONASS (non-concurrent), NAV-PVT (84 B).
// ===========================================================================
mod neo7 {
    use super::*;

    #[test]
    fn probe_protver14_is_series7() {
        // Protocol <= 17 uses the space-separated form (UBX-13003221 §2.1.3).
        let caps = probe_with(&["PROTVER 14.00"], "1.00 (59842)");
        assert_eq!(caps.generation, Generation::Series7);
        assert_eq!(caps.protocol_version, 14);
        assert!(caps.has_nav_pvt(), "NAV-PVT introduced with protocol 14");
        assert!(!caps.has_cfg_gnss(), "constellation switching is 8-series+");
        assert_eq!(caps.max_rate_ms(), 100);
    }

    /// Regression for a real bug found while reading the specs: u-blox 7
    /// NAV-PVT is 84 bytes (GPS.G7-SW-12001), not 92. The parser must
    /// accept it — all decoded fields lie within the first 84 bytes.
    #[test]
    fn nav_pvt_84_bytes_parses() {
        let mut p = [0u8; 84];
        p[4..6].copy_from_slice(&2014u16.to_le_bytes());
        p[20] = 3; // 3D
        p[21] = 0x01; // gnssFixOK
        p[23] = 9;
        p[28..32].copy_from_slice(&471711399i32.to_le_bytes());
        let evs = feed(&ubx_wire(ubx::CLASS_NAV, ubx::NAV_PVT, &p));
        let Event::NavPvt(pvt) = evs[0] else {
            panic!("84-byte NAV-PVT must decode, got {:?}", evs[0]);
        };
        assert_eq!(pvt.year, 2014);
        assert!(pvt.gnss_fix_ok());
        assert_eq!(pvt.lat_1e7, 471711399);
    }

    /// Anything shorter than the u-blox 7 layout is malformed and must
    /// degrade to UbxOther rather than mis-decode.
    #[test]
    fn nav_pvt_shorter_than_84_degrades() {
        let evs = feed(&ubx_wire(ubx::CLASS_NAV, ubx::NAV_PVT, &[0u8; 76]));
        assert_eq!(evs[0], Event::UbxOther { class: ubx::CLASS_NAV, id: ubx::NAV_PVT });
    }

    /// In GLONASS-only mode the 7-series talks with the GL talker; sentences
    /// must decode identically.
    #[test]
    fn glonass_only_gl_talker() {
        let evs = feed(&nmea_wire(
            "GLRMC,094054.00,A,4717.11437,N,00833.91522,E,0.120,0.00,120423,,,A",
        ));
        assert!(matches!(evs[0], Event::Nmea(nmea::Sentence::Rmc(r)) if r.valid));
    }
}

// ===========================================================================
// NEO-8M / NEO-M8N — u-blox 8/M8, protocols 15.00–23.01. Concurrent GNSS,
// GN main talker, NMEA 4.0/4.1, 92-byte NAV-PVT.
// ===========================================================================
mod neo8 {
    use super::*;

    /// Firmware/protocol table from UBX-13003221 §2.2.1, both PROTVER
    /// extension spellings: space form for protocol <= 17, '=' from 18 on.
    #[test]
    fn firmware_protocol_version_table() {
        for (exts, sw, protver) in [
            (&["PROTVER 15.00"][..], "2.01 (75331)", 15), // SPG 2.01, ROM
            (&["ROM BASE 3.01 (107888)", "FWVER=SPG 3.01", "PROTVER=18.00", "MOD=NEO-M8N-0"][..], "EXT CORE 3.01 (107900)", 18), // SPG 3.01, Flash
            (&["FWVER=SPG 3.50", "PROTVER=23.00"][..], "EXT CORE 3.50 (190461)", 23),
            (&["FWVER=SPG 3.51", "PROTVER=23.01"][..], "ROM CORE 3.51 (19dc23)", 23),
            (&["FWVER=HPG 1.40", "PROTVER=20.30"][..], "EXT CORE 3.01 (db0c89)", 20), // NEO-M8P RTK
            (&["FWVER=TIM 1.10", "PROTVER=22.00"][..], "EXT CORE 3.01 (111141)", 22), // NEO-M8T timing
        ] {
            let caps = probe_with(exts, sw);
            assert_eq!(caps.protocol_version, protver, "exts {:?}", exts);
            assert_eq!(caps.generation, Generation::Series8, "exts {:?}", exts);
            assert!(caps.has_nav_pvt() && caps.has_cfg_gnss());
            assert_eq!(caps.max_rate_ms(), 100);
        }
    }

    /// Real M8 MON-VER extensions also carry GNSS lists etc.; PROTVER must
    /// be found regardless of its position among them.
    #[test]
    fn protver_found_among_other_extensions() {
        let caps = probe_with(
            &["ROM BASE 3.01 (107888)", "FWVER=SPG 3.01", "PROTVER=18.00",
              "GPS;GLO;GAL;BDS", "SBAS;IMES;QZSS", "GNSS OTP=GPS;GLO"],
            "ROM CORE 3.01 (107888)",
        );
        assert_eq!(caps.protocol_version, 18);
    }

    /// Multi-GNSS default output: GN main talker everywhere except GSV,
    /// which uses per-constellation talkers (UBX-13003221 §31.1.2).
    #[test]
    fn gn_main_talker_with_per_gnss_gsv() {
        let mut rx = Vec::new();
        rx.extend(nmea_wire("GNGGA,001043.00,4404.14036,N,12118.85961,W,1,12,0.98,1113.0,M,-21.3,M,,"));
        rx.extend(nmea_wire("GNGSA,A,3,80,71,73,79,69,,,,,,,,1.83,1.09,1.47"));
        rx.extend(nmea_wire("GPGSV,3,1,11,03,03,111,00,04,15,270,00,06,01,010,00,13,06,292,00"));
        rx.extend(nmea_wire("GLGSV,3,1,09,65,04,037,,66,55,061,20,67,52,131,29,68,05,176,"));
        rx.extend(nmea_wire("GAGSV,1,1,02,05,65,144,41,24,41,067,35"));
        rx.extend(nmea_wire("GBGSV,1,1,03,08,52,281,40,13,46,314,38,14,15,145,"));
        let evs = feed(&rx);
        assert!(matches!(evs[0], Event::Nmea(nmea::Sentence::Gga(_))));
        assert!(matches!(evs[1], Event::Nmea(nmea::Sentence::Gsa(_))));
        for (i, talker) in [(2usize, *b"GP"), (3, *b"GL"), (4, *b"GA"), (5, *b"GB")] {
            assert_eq!(evs[i], Event::NmeaOther { talker, mtype: *b"GSV" });
        }
    }

    /// NMEA 4.1 GNS (the recommended multi-GNSS fix sentence) is undecoded
    /// but must pass through.
    #[test]
    fn gns_passes_through() {
        let evs = feed(&nmea_wire(
            "GNGNS,103600.01,5114.51176,N,00012.29380,W,ANNN,07,1.18,111.5,45.6,,,V",
        ));
        assert_eq!(evs[0], Event::NmeaOther { talker: *b"GN", mtype: *b"GNS" });
    }

    /// u-blox proprietary NMEA uses the address "PUBX" (UBX-13003221 §31.3);
    /// it is a 4-char header, reported via the proprietary branch.
    #[test]
    fn pubx_proprietary_sentence() {
        let evs = feed(&nmea_wire(
            "PUBX,00,081350.00,4717.113210,N,00833.915187,E,546.589,G3,2.1,2.0,0.007,77.52,0.007,,0.92,1.19,0.77,9,0,0",
        ));
        let Event::NmeaOther { talker, mtype } = evs[0] else { panic!() };
        assert_eq!(&talker, b"PU", "proprietary header reported as-is");
        assert_eq!(&mtype, b"BX?");
    }

    /// 92-byte NAV-PVT with the 8-series tail present; gnssFixOK is the
    /// validity flag the spec tells users to check (§8.3).
    #[test]
    fn nav_pvt_92_bytes_fix_ok_flag() {
        let mut p = [0u8; 92];
        p[20] = 3;
        p[21] = 0x01;
        p[84..88].copy_from_slice(&123456i32.to_le_bytes()); // headVeh (ignored)
        let evs = feed(&ubx_wire(ubx::CLASS_NAV, ubx::NAV_PVT, &p));
        let Event::NavPvt(pvt) = evs[0] else { panic!() };
        assert!(pvt.gnss_fix_ok());

        // fixType 3 but gnssFixOK clear => fix exists but fails the output
        // filters; the flag must read false.
        let mut p2 = [0u8; 92];
        p2[20] = 3;
        let Event::NavPvt(pvt2) = feed(&ubx_wire(ubx::CLASS_NAV, ubx::NAV_PVT, &p2))[0]
        else { panic!() };
        assert!(!pvt2.gnss_fix_ok());
    }

    /// End-to-end on a probed M8: NAV-PVT can be enabled and NMEA muted.
    #[test]
    fn configure_binary_only_output() {
        let mut rx = ubx_wire(ubx::CLASS_MON, ubx::MON_VER,
            &mon_ver_payload("ROM CORE 3.01 (107888)", "00080000", &["PROTVER=18.00"]));
        rx.extend(ubx_wire(ubx::CLASS_ACK, 0x01, &[ubx::CLASS_CFG, ubx::CFG_MSG])); // ack enable PVT
        rx.extend(ubx_wire(ubx::CLASS_ACK, 0x01, &[ubx::CLASS_CFG, ubx::CFG_MSG])); // ack disable GGA
        let mut gps = NeoGps::new(MockUart::new(rx));
        block_on(gps.probe()).unwrap();
        block_on(gps.enable_nav_pvt()).unwrap();
        block_on(gps.disable_nmea(0x00)).unwrap(); // GGA
    }
}

// ===========================================================================
// M9/M10 forward compatibility.
// ===========================================================================
mod m9_m10 {
    use super::*;

    /// M10 SPG 5.10 reports PROTVER=34.10 (u-blox M10 interface description).
    #[test]
    fn m10_protver_34() {
        let caps = probe_with(&["FWVER=SPG 5.10", "PROTVER=34.10", "MOD=MAX-M10S"], "ROM SPG 5.10");
        assert_eq!(caps.generation, Generation::Series9Plus);
        assert_eq!(caps.protocol_version, 34);
        assert_eq!(caps.max_rate_ms(), 40);
    }


    /// NAV-SOL was removed from protocol 24 on: enable_nav_sol must refuse
    /// locally, and enable_binary_nav must route to NAV-PVT.
    #[test]
    fn nav_sol_refused_binary_nav_uses_pvt() {
        let mut rx = ubx_wire(ubx::CLASS_MON, ubx::MON_VER,
            &mon_ver_payload("ROM SPG 5.10", "00190000", &["PROTVER=34.10"]));
        rx.extend(ubx_wire(ubx::CLASS_ACK, 0x01, &[ubx::CLASS_CFG, ubx::CFG_MSG]));
        let mut gps = NeoGps::new(MockUart::new(rx));
        let caps = block_on(gps.probe()).unwrap();
        assert!(!caps.has_nav_sol());
        block_on(gps.enable_binary_nav()).unwrap();
        assert!(gps.uart_ref().tx.windows(3).any(|w| w == [ubx::CLASS_NAV, ubx::NAV_PVT, 1]));
        assert_eq!(block_on(gps.enable_nav_sol()), Err(Error::Unsupported));
    }

    #[test]
    fn m9_protver_32() {
        let caps = probe_with(&["FWVER=SPG 4.04", "PROTVER=32.01"], "ROM CORE 4.04");
        assert_eq!(caps.generation, Generation::Series9Plus);
    }
}

// ===========================================================================
// Wire-level robustness, incl. an externally published ground-truth frame.
// ===========================================================================
mod wire {
    use super::*;

    /// A real captured CFG-NMEA frame (published for the SAM-M8Q with its
    /// checksum 0x96 0xD9): external validation of the Fletcher checksum
    /// against u-blox hardware, not just against this crate's own encoder.
    const CFG_NMEA_CAPTURE: [u8; 28] = [
        0xb5, 0x62, 0x06, 0x17, 0x14, 0x00, 0x20, 0x40, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x96, 0xd9,
    ];

    #[test]
    fn published_cfg_nmea_capture_accepted() {
        let evs = feed(&CFG_NMEA_CAPTURE);
        assert_eq!(evs, vec![Event::UbxOther { class: 0x06, id: 0x17 }]);
    }

    #[test]
    fn encoder_reproduces_published_capture() {
        let rebuilt = ubx_wire(0x06, 0x17, &CFG_NMEA_CAPTURE[6..26]);
        assert_eq!(rebuilt, CFG_NMEA_CAPTURE);
    }

    /// One byte per read(): every state transition lands on a chunk boundary.
    #[test]
    fn byte_at_a_time_delivery() {
        let mut rx = nmea_wire("GNGGA,001043.00,4404.14036,N,12118.85961,W,1,12,0.98,1113.0,M,-21.3,M,,");
        rx.extend(ubx_wire(ubx::CLASS_ACK, 0x01, &[0x06, 0x08]));
        let mut uart = MockUart::new(rx);
        uart.chunk = 1;
        let mut gps = NeoGps::new(uart);
        assert!(matches!(block_on(gps.next_event()).unwrap(), Event::Nmea(_)));
        assert!(matches!(block_on(gps.next_event()).unwrap(), Event::Ack { ok: true, .. }));
    }

    /// Oversized UBX payloads (e.g. RXM-RAWX can exceed our 384-byte buffer)
    /// are truncated in storage but framing must stay byte-accurate: the
    /// following frame still parses.
    #[test]
    fn oversized_payload_keeps_framing() {
        let mut rx = ubx_wire(0x02, 0x15, &vec![0xAB; 600]); // RXM-RAWX-shaped
        rx.extend(ubx_wire(ubx::CLASS_ACK, 0x01, &[0x06, 0x01]));
        let evs = feed(&rx);
        assert_eq!(evs.len(), 2);
        assert_eq!(evs[0], Event::UbxOther { class: 0x02, id: 0x15 });
        assert_eq!(evs[1], Event::Ack { class: 0x06, id: 0x01, ok: true });
    }

    /// 0xB5 not followed by 0x62 is noise, not a frame start.
    #[test]
    fn false_sync_byte_resyncs() {
        let mut rx = vec![0xB5, 0x00, 0xB5, 0xB5, 0x42];
        rx.extend(ubx_wire(ubx::CLASS_ACK, 0x01, &[0x06, 0x08]));
        let evs = feed(&rx);
        assert_eq!(evs, vec![Event::Ack { class: 0x06, id: 0x08, ok: true }]);
    }

    /// u-blox receivers always transmit the NMEA checksum; a sentence
    /// without "*hh" is malformed and must be dropped.
    #[test]
    fn nmea_without_checksum_rejected() {
        assert!(feed(b"$GPGGA,092725.00,4717.11399,N,00833.91590,E,1,08,1.01,499.6,M,48.0,M,,\r\n").is_empty());
    }

    /// NMEA and UBX interleave freely on the wire (multi-protocol port);
    /// a UBX frame between two sentences must not disturb either.
    #[test]
    fn interleaved_protocols() {
        let mut rx = Vec::new();
        rx.extend(nmea_wire("GNGSA,A,3,80,71,,,,,,,,,,,1.83,1.09,1.47"));
        rx.extend(ubx_wire(ubx::CLASS_NAV, ubx::NAV_PVT, &[0u8; 92]));
        rx.extend(nmea_wire("GNRMC,204520.00,A,5109.0262,N,11401.8407,W,0.004,133.4,130522,0.0,E,D,V"));
        let evs = feed(&rx);
        assert!(matches!(evs[..], [
            Event::Nmea(nmea::Sentence::Gsa(_)),
            Event::NavPvt(_),
            Event::Nmea(nmea::Sentence::Rmc(_)),
        ]));
    }
}

// ===========================================================================
// Unit conversions per the spec's "Latitude and Longitude Format" section
// (ddmm.mmmmm) and NMEA field scalings.
// ===========================================================================
mod units {
    use super::*;

    fn gga_lat_lon(lat: &str, ns: &str, lon: &str, ew: &str) -> (Option<i32>, Option<i32>) {
        let body = format!("GPGGA,120000.00,{lat},{ns},{lon},{ew},1,08,1.0,10.0,M,0.0,M,,");
        let evs = feed(&nmea_wire(&body));
        let Event::Nmea(nmea::Sentence::Gga(g)) = evs[0] else { panic!() };
        (g.lat_1e7, g.lon_1e7)
    }

    /// The spec's own worked example: 4717.112671 N = 47° + 17.112671'/60.
    /// 17.112671 / 60 = 0.28521118333...° → 472852112 at 1e-7 (rounded).
    #[test]
    fn spec_coordinate_conversion_example() {
        let (lat, _) = gga_lat_lon("4717.112671", "N", "00833.914843", "E");
        assert_eq!(lat, Some(472852112));
    }

    #[test]
    fn hemispheres_and_extremes() {
        let (lat, lon) = gga_lat_lon("0000.00000", "S", "17959.99999", "W");
        assert_eq!(lat, Some(0));
        // 179° 59.99999' W = -179.9999998° → -1799999998
        assert_eq!(lon, Some(-1799999998));
        let (lat, _) = gga_lat_lon("8959.9999", "S", "00000.0000", "E");
        assert_eq!(lat, Some(-899999983)); // 89° 59.9999' = 89.9999983° S
    }

    #[test]
    fn invalid_hemisphere_yields_none() {
        let (lat, _) = gga_lat_lon("4717.11399", "X", "00833.91590", "E");
        assert_eq!(lat, None);
    }

    /// RMC speed is in knots; 1 knot = 0.514444 m/s exactly (1852 m / 3600 s).
    #[test]
    fn knots_to_mm_per_s() {
        for (knots, mm_s) in [("1.000", 514u32), ("0.004", 2), ("10.000", 5144), ("100.000", 51444)] {
            let body = format!("GPRMC,083559.00,A,4717.11437,N,00833.91522,E,{knots},77.52,091202,,,A");
            let Event::Nmea(nmea::Sentence::Rmc(r)) = feed(&nmea_wire(&body))[0] else { panic!() };
            assert_eq!(r.speed_mm_s, Some(mm_s), "{knots} knots");
        }
    }

    /// DOP fields scale by 100; the 6M's cold-start "99.99" must fit.
    #[test]
    fn dop_scaling_and_ceiling() {
        let Event::Nmea(nmea::Sentence::Gsa(g)) =
            feed(&nmea_wire("GPGSA,A,1,,,,,,,,,,,,,99.99,99.99,99.99"))[0]
        else { panic!() };
        assert_eq!(g.fix_type, 1);
        assert_eq!(g.pdop_1e2, Some(9999));
    }

    /// Sub-second time: u-blox emits hhmmss.ss (2 fractional digits);
    /// they scale to milliseconds.
    #[test]
    fn time_fraction_to_millis() {
        let Event::Nmea(nmea::Sentence::Gga(g)) =
            feed(&nmea_wire("GPGGA,092725.25,4717.11399,N,00833.91590,E,1,08,1.01,499.6,M,48.0,M,,"))[0]
        else { panic!() };
        assert_eq!(g.time.unwrap().millis, 250);
    }

    /// Negative geoid separation (common west of the prime meridian) and
    /// millimetre altitude scaling.
    #[test]
    fn altitude_millimetres_signed() {
        let Event::Nmea(nmea::Sentence::Gga(g)) =
            feed(&nmea_wire("GNGGA,001043.00,4404.14036,N,12118.85961,W,1,12,0.98,1113.0,M,-21.3,M,,"))[0]
        else { panic!() };
        assert_eq!(g.alt_msl_mm, Some(1_113_000));
        assert_eq!(g.geoid_sep_mm, Some(-21_300));
    }
}

// Byte count captured by the mock so far, via the shared testutil accessor.
impl NeoGps<MockUart> {
    fn free_len_tx(&self) -> usize {
        self.uart_ref().tx.len()
    }
}

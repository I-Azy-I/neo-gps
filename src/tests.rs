use crate::nmea::FixQuality;
use crate::testutil::*;
use crate::ubx::{self, UbxFrame};
use crate::*;

// -- NMEA: NEO-6M style (talker GP, NMEA 2.3) ------------------------------

#[test]
fn gga_neo6m() {
    // Canonical u-blox 6 GGA
    let evs = feed(&nmea_wire(
        "GPGGA,092725.00,4717.11399,N,00833.91590,E,1,08,1.01,499.6,M,48.0,M,,",
    ));
    assert_eq!(evs.len(), 1);
    let Event::Nmea(nmea::Sentence::Gga(g)) = evs[0] else {
        panic!("expected GGA, got {:?}", evs[0]);
    };
    assert_eq!(g.quality, FixQuality::Gps);
    assert_eq!(g.sats_in_use, 8);
    // 47° 17.11399' N = 47.28523316° → 472852332 (1e-7 deg, rounded)
    assert_eq!(g.lat_1e7, Some(472852332));
    // 8° 33.91590' E = 8.5652650° → 85652650
    assert_eq!(g.lon_1e7, Some(85652650));
    assert_eq!(g.alt_msl_mm, Some(499_600));
    assert_eq!(g.geoid_sep_mm, Some(48_000));
    assert_eq!(g.hdop_1e2, Some(101));
    let t = g.time.unwrap();
    assert_eq!((t.hour, t.minute, t.second), (9, 27, 25));
}

#[test]
fn rmc_neo6m() {
    let evs = feed(&nmea_wire(
        "GPRMC,083559.00,A,4717.11437,N,00833.91522,E,0.004,77.52,091202,,,A",
    ));
    let Event::Nmea(nmea::Sentence::Rmc(r)) = evs[0] else {
        panic!("expected RMC");
    };
    assert!(r.valid);
    assert_eq!(
        r.date,
        Some(nmea::Date {
            day: 9,
            month: 12,
            year: 2
        })
    );
    // 0.004 knots ≈ 2.06 mm/s → rounds to 2
    assert_eq!(r.speed_mm_s, Some(2));
    assert_eq!(r.course_1e5, Some(7_752_000));
    assert!(r.lat_1e7.unwrap() > 0);
}

// -- NMEA: NEO-8M style (talker GN, NMEA 4.x extra fields) -----------------

#[test]
fn gga_neo8m_gn_talker() {
    let evs = feed(&nmea_wire(
        "GNGGA,001043.00,4404.14036,N,12118.85961,W,1,12,0.98,1113.0,M,-21.3,M,,",
    ));
    let Event::Nmea(nmea::Sentence::Gga(g)) = evs[0] else {
        panic!("GN talker must parse identically to GP");
    };
    assert_eq!(g.sats_in_use, 12);
    assert!(g.lon_1e7.unwrap() < 0, "W hemisphere must be negative");
    assert_eq!(g.geoid_sep_mm, Some(-21_300));
}

#[test]
fn rmc_neo8m_nmea41_extra_field() {
    // NMEA 4.1 RMC has a trailing navStatus field ("V") the 2.3 parser
    // must tolerate.
    let evs = feed(&nmea_wire(
        "GNRMC,204520.00,A,5109.0262,N,11401.8407,W,0.004,133.4,130522,0.0,E,D,V",
    ));
    let Event::Nmea(nmea::Sentence::Rmc(r)) = evs[0] else {
        panic!("expected RMC");
    };
    assert!(r.valid);
    assert_eq!(
        r.date,
        Some(nmea::Date {
            day: 13,
            month: 5,
            year: 22
        })
    );
}

#[test]
fn gsa_and_unknown_sentences() {
    let mut bytes = nmea_wire("GNGSA,A,3,80,71,73,79,69,,,,,,,,1.83,1.09,1.47");
    bytes.extend(nmea_wire(
        "GLGSV,3,1,09,65,04,037,,66,55,061,20,67,52,131,29,68,05,176,",
    ));
    let evs = feed(&bytes);
    assert_eq!(evs.len(), 2);
    let Event::Nmea(nmea::Sentence::Gsa(g)) = evs[0] else {
        panic!("expected GSA");
    };
    assert_eq!(g.fix_type, 3);
    assert_eq!(g.pdop_1e2, Some(183));
    assert_eq!(g.hdop_1e2, Some(109));
    assert_eq!(g.vdop_1e2, Some(147));
    // GSV is valid but undecoded → NmeaOther with preserved talker
    assert_eq!(
        evs[1],
        Event::NmeaOther {
            talker: *b"GL",
            mtype: *b"GSV"
        }
    );
}

#[test]
fn empty_fields_and_no_fix() {
    // Cold start: empty position fields, quality 0
    let evs = feed(&nmea_wire("GPGGA,,,,,,0,00,99.99,,,,,,"));
    let Event::Nmea(nmea::Sentence::Gga(g)) = evs[0] else {
        panic!("expected GGA");
    };
    assert_eq!(g.quality, FixQuality::NoFix);
    assert!(!g.quality.has_fix());
    assert_eq!(g.lat_1e7, None);
    assert_eq!(g.time, None);
}

#[test]
fn bad_checksum_rejected() {
    let mut bytes =
        nmea_wire("GPGGA,092725.00,4717.11399,N,00833.91590,E,1,08,1.01,499.6,M,48.0,M,,");
    let star = bytes.iter().rposition(|&b| b == b'*').unwrap();
    bytes[star + 1] = b'0'; // corrupt checksum
    bytes[star + 2] = b'0';
    assert!(feed(&bytes).is_empty());
}

#[test]
fn resync_after_garbage() {
    let mut bytes = b"\x00\xFFgarbage$GPGG".to_vec(); // truncated sentence
    bytes.extend(nmea_wire("GPGSA,A,3,,,,,,,,,,,,,2.5,1.3,2.1")); // '$' restarts
    let evs = feed(&bytes);
    assert_eq!(evs.len(), 1);
    assert!(matches!(evs[0], Event::Nmea(nmea::Sentence::Gsa(_))));
}

// -- UBX -------------------------------------------------------------------

#[test]
fn ubx_frame_checksum_known_vector() {
    // CFG-RATE poll (empty payload): B5 62 06 08 00 00 0E 30
    let mut f = UbxFrame::new(0x06, 0x08);
    assert_eq!(
        f.finish(),
        &[0xB5, 0x62, 0x06, 0x08, 0x00, 0x00, 0x0E, 0x30]
    );
}

#[test]
fn ubx_ack_and_nak() {
    let mut bytes = ubx_wire(ubx::CLASS_ACK, 0x01, &[0x06, 0x08]); // ACK for CFG-RATE
    bytes.extend(ubx_wire(ubx::CLASS_ACK, 0x00, &[0x06, 0x3E])); // NAK for CFG-GNSS
    let evs = feed(&bytes);
    assert_eq!(
        evs,
        vec![
            Event::Ack {
                class: 0x06,
                id: 0x08,
                ok: true
            },
            Event::Ack {
                class: 0x06,
                id: 0x3E,
                ok: false
            },
        ]
    );
}

#[test]
fn ubx_corrupt_checksum_dropped_then_resync() {
    let mut bad = ubx_wire(ubx::CLASS_ACK, 0x01, &[0x06, 0x08]);
    let n = bad.len();
    bad[n - 1] ^= 0xFF;
    bad.extend(nmea_wire("GPGSA,A,3,,,,,,,,,,,,,2.5,1.3,2.1"));
    let evs = feed(&bad);
    assert_eq!(evs.len(), 1);
    assert!(matches!(evs[0], Event::Nmea(_)));
}

#[test]
fn nav_pvt_roundtrip() {
    // Hand-build a NAV-PVT payload with known values.
    let mut p = [0u8; 92];
    p[0..4].copy_from_slice(&123456u32.to_le_bytes()); // iTOW
    p[4..6].copy_from_slice(&2026u16.to_le_bytes());
    p[6] = 7; // month
    p[7] = 19; // day
    p[8] = 12;
    p[9] = 34;
    p[10] = 56;
    p[20] = 3; // 3D fix
    p[21] = 0x01; // gnssFixOK
    p[23] = 14; // numSV
    p[24..28].copy_from_slice(&61432160i32.to_le_bytes()); // lon 6.143216°
    p[28..32].copy_from_slice(&462025986i32.to_le_bytes()); // lat 46.2025986°
    p[36..40].copy_from_slice(&375_000i32.to_le_bytes()); // hMSL 375 m
    p[40..44].copy_from_slice(&1800u32.to_le_bytes()); // hAcc 1.8 m
    p[60..64].copy_from_slice(&1250i32.to_le_bytes()); // gSpeed 1.25 m/s
    p[76..78].copy_from_slice(&150u16.to_le_bytes()); // pDOP 1.50

    let evs = feed(&ubx_wire(ubx::CLASS_NAV, ubx::NAV_PVT, &p));
    let Event::NavPvt(pvt) = evs[0] else {
        panic!("expected NavPvt, got {:?}", evs[0]);
    };
    assert_eq!(pvt.year, 2026);
    assert!(pvt.gnss_fix_ok());
    assert_eq!(pvt.fix_type, 3);
    assert_eq!(pvt.num_sv, 14);
    assert_eq!(pvt.lat_1e7, 462025986);
    assert_eq!(pvt.hmsl_mm, 375_000);
    assert_eq!(pvt.gspeed_mm_s, 1250);
}

#[test]
fn mon_ver_protver_variants() {
    let payload = |ext: &[&str]| mon_ver_payload("", "", ext);
    // 8-series style
    assert_eq!(
        ubx::parse_mon_ver_protver(&payload(&["FWVER=SPG 3.01", "PROTVER=18.00"])),
        Some(18)
    );
    // space separator variant
    assert_eq!(
        ubx::parse_mon_ver_protver(&payload(&["PROTVER 15.00"])),
        Some(15)
    );
    // NEO-6: no PROTVER extension at all
    assert_eq!(
        ubx::parse_mon_ver_protver(&payload(&["7.03 (45969)"])),
        None
    );
}

// -- capabilities ----------------------------------------------------------

#[test]
fn capability_gating() {
    let six = Capabilities {
        generation: Generation::Series6,
        protocol_version: 12,
    };
    let eight = Capabilities {
        generation: Generation::Series8,
        protocol_version: 18,
    };
    assert!(!six.has_nav_pvt());
    assert!(!six.has_cfg_gnss());
    assert_eq!(six.max_rate_ms(), 200);
    assert!(eight.has_nav_pvt());
    assert!(eight.has_cfg_gnss());
    assert_eq!(eight.max_rate_ms(), 100);
}

// -- async end-to-end with a mock UART -------------------------------------
#[test]
fn probe_detects_series8_then_set_rate_acked() {
    // Script the module side: chatter, MON-VER reply, chatter, ACK for CFG-RATE.
    let mut mon_ver = vec![0u8; 40];
    let mut ext = [0u8; 30];
    ext[..13].copy_from_slice(b"PROTVER=18.00");
    mon_ver.extend_from_slice(&ext);

    let mut rx =
        nmea_wire("GNGGA,001043.00,4404.14036,N,12118.85961,W,1,12,0.98,1113.0,M,-21.3,M,,");
    rx.extend(ubx_wire(ubx::CLASS_MON, ubx::MON_VER, &mon_ver));
    rx.extend(nmea_wire("GNGSA,A,3,80,71,,,,,,,,,,,1.83,1.09,1.47"));
    rx.extend(ubx_wire(
        ubx::CLASS_ACK,
        0x01,
        &[ubx::CLASS_CFG, ubx::CFG_RATE],
    ));

    let mut gps = NeoGps::new(MockUart::new(rx));

    let caps = block_on(gps.probe()).unwrap();
    assert_eq!(caps.generation, Generation::Series8);
    assert_eq!(caps.protocol_version, 18);

    // 100 ms is allowed on series 8...
    block_on(gps.set_nav_rate_ms(100)).unwrap();
    // ...and the wire bytes are a correct CFG-RATE frame.
    let uart = gps.free();
    let sent = &uart.tx;
    let cfg_rate_pos = sent
        .windows(4)
        .position(|w| w == [0xB5, 0x62, 0x06, 0x08])
        .expect("CFG-RATE frame sent");
    assert_eq!(
        &sent[cfg_rate_pos + 6..cfg_rate_pos + 8],
        &100u16.to_le_bytes()
    );
}

#[test]
fn series6_rejects_fast_rate_without_io() {
    let mut gps = NeoGps::new(MockUart::new(Vec::new())); // conservative caps = series 6
    assert_eq!(block_on(gps.set_nav_rate_ms(100)), Err(Error::Unsupported));
    assert!(gps.free().tx.is_empty(), "must not touch the wire");
}

#[test]
fn nak_surfaces_as_error() {
    let rx = ubx_wire(ubx::CLASS_ACK, 0x00, &[ubx::CLASS_CFG, ubx::CFG_GNSS]);
    let mut gps = NeoGps::new(MockUart::new(rx));
    let r = block_on(gps.send_cfg_acked(ubx::CFG_GNSS, &[]));
    assert_eq!(
        r,
        Err(Error::Nak {
            class: 0x06,
            id: 0x3E
        })
    );
}

// -- raw-frame accessors ---------------------------------------------------

#[test]
fn raw_ubx_payload_accessible_after_ubx_other() {
    // NAV-SAT-shaped frame (class 0x01, id 0x35) the driver doesn't decode.
    let payload = [0x40u8, 0xE2, 0x01, 0x00, 0x01, 0x03, 0x00, 0x00, 0xAA, 0xBB];
    let rx = ubx_wire(0x01, 0x35, &payload);
    let mut gps = NeoGps::new(MockUart::new(rx));

    let ev = block_on(gps.next_event()).unwrap();
    assert_eq!(
        ev,
        Event::UbxOther {
            class: 0x01,
            id: 0x35
        }
    );
    assert_eq!(gps.last_ubx_payload(), &payload);
}

#[test]
fn raw_nmea_line_accessible_after_nmea_other() {
    // GSV is framed but undecoded → NmeaOther, raw line retrievable.
    let body = "GLGSV,3,1,09,65,04,037,,66,55,061,20,67,52,131,29,68,05,176,";
    let rx = nmea_wire(body);
    let mut gps = NeoGps::new(MockUart::new(rx));

    let ev = block_on(gps.next_event()).unwrap();
    assert_eq!(
        ev,
        Event::NmeaOther {
            talker: *b"GL",
            mtype: *b"GSV"
        }
    );
    // Accessor returns body + "*hh" checksum trailer, no $ or CR/LF.
    let line = gps.last_nmea_line();
    assert!(line.starts_with(body.as_bytes()));
    assert_eq!(line.len(), body.len() + 3);
}

#[test]
fn corrupt_line_does_not_clobber_retained_sentence() {
    let good = "GLGSV,3,1,09,65,04,037,,66,55,061,20,,,,";
    let mut rx = nmea_wire(good);
    let mut bad =
        nmea_wire("GPGGA,092725.00,4717.11399,N,00833.91590,E,1,08,1.01,499.6,M,48.0,M,,");
    let n = bad.len();
    bad[n - 4] = b'0'; // corrupt checksum
    bad[n - 3] = b'0';
    rx.extend(bad);
    let mut gps = NeoGps::new(MockUart::new(rx));

    // First event: the good GSV.
    block_on(gps.next_event()).unwrap();
    // Feed remaining bytes: the corrupt GGA produces no event (mock returns
    // Eof once drained), and must not have overwritten the retained line.
    let _ = block_on(gps.next_event());
    assert!(gps.last_nmea_line().starts_with(good.as_bytes()));
}

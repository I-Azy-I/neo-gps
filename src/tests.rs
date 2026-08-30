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
    // Stock M8 tracks GPS + GLONASS together, which caps it at 5 Hz; 10 Hz
    // needs a single constellation.
    assert_eq!(eight.max_rate_ms(), 200);
    assert_eq!(eight.max_rate_ms_single_gnss(), 100);
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

// -- skip-unknown filtering ------------------------------------------------

#[test]
fn unknown_frames_skipped_by_default() {
    // A GSV (undecoded NMEA) then a decoded RMC: by default next_event hides
    // the GSV and returns the RMC as the first event.
    let mut rx = nmea_wire("GLGSV,3,1,09,65,04,037,,66,55,061,20,,,,");
    rx.extend(nmea_wire(
        "GPRMC,092725.00,A,4717.11399,N,00833.91590,E,0.004,77.52,091202,,,A",
    ));
    let mut gps = NeoGps::new(MockUart::new(rx));

    assert!(matches!(
        block_on(gps.next_event()).unwrap(),
        Event::Nmea(nmea::Sentence::Rmc(_))
    ));
}

#[test]
fn unknown_frames_surfaced_when_opted_in() {
    let mut gps = NeoGps::new(MockUart::new(nmea_wire(
        "GLGSV,3,1,09,65,04,037,,66,55,061,20,,,,",
    )));
    gps.set_skip_unknown_sentences(false);
    assert!(matches!(
        block_on(gps.next_event()).unwrap(),
        Event::NmeaOther { .. }
    ));
}

// -- next_coordinate -------------------------------------------------------

#[test]
fn next_coordinate_yields_only_fixed_positions() {
    // GSA (no coords) and a void RMC (status 'V', no fix) must be skipped; the
    // valid RMC that follows is returned as a Coordinate.
    let mut rx = nmea_wire("GPGSA,A,3,01,02,,,,,,,,,,,2.5,1.3,2.1");
    rx.extend(nmea_wire("GPRMC,092725.00,V,,,,,,,091202,,,N")); // no fix
    rx.extend(nmea_wire(
        "GPRMC,092725.00,A,4717.11399,N,00833.91590,E,0.004,77.52,091202,,,A",
    ));
    let mut gps = NeoGps::new(MockUart::new(rx));

    let c = block_on(gps.next_coordinate()).unwrap();
    assert_eq!(c.lat_1e7, 472852332);
    assert_eq!(c.lon_1e7, 85652650);
}

#[test]
fn next_coordinate_reads_nav_pvt() {
    let mut p = [0u8; 92];
    p[20] = 3; // fix_type = 3D
    p[21] = 0x01; // gnssFixOK
    p[24..28].copy_from_slice(&73970762i32.to_le_bytes()); // lon
    p[28..32].copy_from_slice(&462308143i32.to_le_bytes()); // lat
    let mut gps = NeoGps::new(MockUart::new(ubx_wire(ubx::CLASS_NAV, ubx::NAV_PVT, &p)));

    let c = block_on(gps.next_coordinate()).unwrap();
    assert_eq!(c.lat_1e7, 462308143);
    assert_eq!(c.lon_1e7, 73970762);
}

// -- raw-frame accessors ---------------------------------------------------

#[test]
fn raw_ubx_payload_accessible_after_ubx_other() {
    // NAV-SAT-shaped frame (class 0x01, id 0x35) the driver doesn't decode.
    let payload = [0x40u8, 0xE2, 0x01, 0x00, 0x01, 0x03, 0x00, 0x00, 0xAA, 0xBB];
    let rx = ubx_wire(0x01, 0x35, &payload);
    let mut gps = NeoGps::new(MockUart::new(rx));
    gps.set_skip_unknown_sentences(false); // opt in to UbxOther

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
    gps.set_skip_unknown_sentences(false); // opt in to NmeaOther

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
    gps.set_skip_unknown_sentences(false); // the good GSV is an NmeaOther

    // First event: the good GSV.
    block_on(gps.next_event()).unwrap();
    // Feed remaining bytes: the corrupt GGA produces no event (mock returns
    // Eof once drained), and must not have overwritten the retained line.
    let _ = block_on(gps.next_event());
    assert!(gps.last_nmea_line().starts_with(good.as_bytes()));
}

// -- Configuration wrappers added for the wider NEO feature set ------------

/// Every CFG wrapper must put the exact payload the interface description
/// specifies on the wire. These check the bytes, not just that a call ACKs.
fn cfg_payload_written(tx: &[u8], id: u8) -> Vec<u8> {
    let hdr = [0xB5, 0x62, ubx::CLASS_CFG, id];
    // Last, not first: a poll-modify-write wrapper sends an empty poll frame
    // with the same class and id before the one we want to inspect.
    let at = tx
        .windows(4)
        .rposition(|w| w == hdr)
        .unwrap_or_else(|| panic!("no CFG frame with id {:#04X} was written", id));
    let len = u16::from_le_bytes([tx[at + 4], tx[at + 5]]) as usize;
    tx[at + 6..at + 6 + len].to_vec()
}

#[test]
fn set_dynamic_model_writes_cfg_nav5_with_only_the_dyn_bit() {
    let rx = ubx_wire(ubx::CLASS_ACK, 0x01, &[ubx::CLASS_CFG, ubx::CFG_NAV5]);
    let mut gps = NeoGps::new(MockUart::new(rx));
    block_on(gps.set_dynamic_model(ubx::DynamicModel::Airborne4G)).unwrap();

    let p = cfg_payload_written(&gps.free().tx, ubx::CFG_NAV5);
    assert_eq!(p.len(), 36, "CFG-NAV5 payload is 36 bytes");
    assert_eq!(&p[0..2], &[0x01, 0x00], "mask must apply dynModel only");
    assert_eq!(p[2], 8, "Airborne4G is dynModel 8");
    assert!(p[3..].iter().all(|&b| b == 0), "unmasked fields stay zero");
}

#[test]
fn set_baud_writes_cfg_prt_and_does_not_wait_for_an_ack() {
    // Empty rx: an ACK would never arrive at the old rate anyway, so the call
    // must complete on the strength of the write alone.
    let mut gps = NeoGps::new(MockUart::new(Vec::new()));
    block_on(gps.set_baud(115_200)).unwrap();

    let p = cfg_payload_written(&gps.free().tx, ubx::CFG_PRT);
    assert_eq!(p.len(), 20);
    assert_eq!(p[0], 1, "UART1");
    assert_eq!(&p[4..8], &0x0000_08C0u32.to_le_bytes(), "8N1");
    assert_eq!(&p[8..12], &115_200u32.to_le_bytes());
    assert_eq!(&p[12..14], &[0x07, 0x00], "in: UBX+NMEA+RTCM");
    assert_eq!(&p[14..16], &[0x03, 0x00], "out: UBX+NMEA");
}

#[test]
fn reset_writes_cfg_rst_and_does_not_wait_for_an_ack() {
    for (kind, mask) in [
        (ubx::ResetKind::Hot, 0x0000u16),
        (ubx::ResetKind::Warm, 0x0001),
        (ubx::ResetKind::Cold, 0xFFFF),
    ] {
        // A resetting module never ACKs, so an empty rx must still succeed.
        let mut gps = NeoGps::new(MockUart::new(Vec::new()));
        block_on(gps.reset(kind)).unwrap();
        let p = cfg_payload_written(&gps.free().tx, ubx::CFG_RST);
        assert_eq!(p.len(), 4);
        assert_eq!(&p[0..2], &mask.to_le_bytes(), "navBbrMask for {:?}", kind);
        assert_eq!(p[2], 0x01, "controlled software reset");
    }
}

#[test]
fn factory_reset_clears_and_reloads_but_saves_nothing() {
    // Empty rx: clearing the port configuration reinitialises the UART, so the
    // ACK may never escape. The call must complete on the write alone.
    let mut gps = NeoGps::new(MockUart::new(Vec::new()));
    block_on(gps.factory_reset()).unwrap();

    let p = cfg_payload_written(&gps.free().tx, ubx::CFG_CFG);
    assert_eq!(&p[0..4], &0x0000_FFFFu32.to_le_bytes(), "clearMask: all");
    assert_eq!(&p[4..8], &[0, 0, 0, 0], "saveMask must stay empty");
    assert_eq!(&p[8..12], &0x0000_FFFFu32.to_le_bytes(), "loadMask: all");
}

#[test]
fn save_config_sets_only_the_save_mask() {
    let rx = ubx_wire(ubx::CLASS_ACK, 0x01, &[ubx::CLASS_CFG, ubx::CFG_CFG]);
    let mut gps = NeoGps::new(MockUart::new(rx));
    block_on(gps.save_config()).unwrap();

    let p = cfg_payload_written(&gps.free().tx, ubx::CFG_CFG);
    assert_eq!(&p[0..4], &[0, 0, 0, 0], "must not clear anything");
    assert_eq!(&p[4..8], &0x0000_FFFFu32.to_le_bytes(), "saveMask: all");
}

#[test]
fn set_power_save_writes_cfg_rxm() {
    for (on, lp) in [(true, 1u8), (false, 0)] {
        let rx = ubx_wire(ubx::CLASS_ACK, 0x01, &[ubx::CLASS_CFG, ubx::CFG_RXM]);
        let mut gps = NeoGps::new(MockUart::new(rx));
        block_on(gps.set_power_save(on)).unwrap();
        assert_eq!(
            cfg_payload_written(&gps.free().tx, ubx::CFG_RXM),
            vec![8, lp]
        );
    }
}

/// A two-block CFG-GNSS poll reply: GPS enabled, GLONASS disabled.
fn cfg_gnss_reply() -> Vec<u8> {
    let mut p = vec![0x00, 32, 32, 0x02]; // version, trkChHw, trkChUse, 2 blocks
    p.extend_from_slice(&[0, 8, 16, 0, 0x01, 0x00, 0x01, 0x00]); // GPS, enabled
    p.extend_from_slice(&[6, 8, 14, 0, 0x00, 0x00, 0x01, 0x00]); // GLONASS, off
    p
}

#[test]
fn set_constellation_flips_one_block_and_leaves_the_rest_alone() {
    let mut rx = ubx_wire(ubx::CLASS_CFG, ubx::CFG_GNSS, &cfg_gnss_reply());
    rx.extend(ubx_wire(
        ubx::CLASS_ACK,
        0x01,
        &[ubx::CLASS_CFG, ubx::CFG_GNSS],
    ));

    let mut gps = NeoGps::new(MockUart::new(rx));
    gps.set_capabilities(Capabilities {
        generation: Generation::Series8,
        protocol_version: 18,
    });
    block_on(gps.set_constellation(ubx::Constellation::Glonass, true)).unwrap();

    let p = cfg_payload_written(&gps.free().tx, ubx::CFG_GNSS);
    // Block 1 spans bytes 12..20: gnssId, resTrkCh, maxTrkCh, reserved, then
    // the 4-byte flags at 16. Bit 0 of those flags is `enable`.
    let mut want = cfg_gnss_reply();
    want[16] |= 0x01;
    assert_eq!(p, want, "only the GLONASS enable bit may change");
}

#[test]
fn set_constellation_is_refused_on_modules_without_cfg_gnss() {
    // Conservative capabilities are NEO-6, which has no CFG-GNSS.
    let mut gps = NeoGps::new(MockUart::new(Vec::new()));
    assert_eq!(
        block_on(gps.set_constellation(ubx::Constellation::Galileo, true)),
        Err(Error::Unsupported)
    );
    assert!(gps.free().tx.is_empty(), "must not touch the wire");
}

#[test]
fn set_constellation_reports_a_constellation_the_module_does_not_list() {
    let mut rx = ubx_wire(ubx::CLASS_CFG, ubx::CFG_GNSS, &cfg_gnss_reply());
    rx.extend(ubx_wire(
        ubx::CLASS_ACK,
        0x01,
        &[ubx::CLASS_CFG, ubx::CFG_GNSS],
    ));
    let mut gps = NeoGps::new(MockUart::new(rx));
    gps.set_capabilities(Capabilities {
        generation: Generation::Series8,
        protocol_version: 18,
    });
    // The reply lists GPS and GLONASS only.
    assert_eq!(
        block_on(gps.set_constellation(ubx::Constellation::BeiDou, true)),
        Err(Error::Unsupported)
    );
}

#[test]
fn nav_velned_decodes_and_converts_to_mm_per_second() {
    let mut p = [0u8; 36];
    p[0..4].copy_from_slice(&123_456u32.to_le_bytes()); // iTOW
    p[4..8].copy_from_slice(&(-250i32).to_le_bytes()); // velN: -2.50 m/s
    p[8..12].copy_from_slice(&100i32.to_le_bytes()); // velE: 1.00 m/s
    p[12..16].copy_from_slice(&(-5i32).to_le_bytes()); // velD: climbing
    p[16..20].copy_from_slice(&270u32.to_le_bytes()); // 3D speed
    p[20..24].copy_from_slice(&269u32.to_le_bytes()); // ground speed
    p[24..28].copy_from_slice(&15_800_000i32.to_le_bytes()); // 158.0 degrees
    p[28..32].copy_from_slice(&12u32.to_le_bytes()); // speed accuracy
    p[32..36].copy_from_slice(&2_000_000u32.to_le_bytes()); // course accuracy

    let evs = feed(&ubx_wire(ubx::CLASS_NAV, ubx::NAV_VELNED, &p));
    let Event::NavVelned(v) = evs[0] else {
        panic!("expected NavVelned, got {:?}", evs[0]);
    };
    assert_eq!(v.itow_ms, 123_456);
    assert_eq!(v.vel_n_mm_s, -2_500, "cm/s converted to mm/s");
    assert_eq!(v.vel_e_mm_s, 1_000);
    assert_eq!(v.vel_d_mm_s, -50);
    assert_eq!(v.speed_mm_s, 2_700);
    assert_eq!(v.gspeed_mm_s, 2_690);
    assert_eq!(v.heading_1e5, 15_800_000, "heading is not rescaled");
    assert_eq!(v.s_acc_mm_s, 120);
    assert_eq!(v.c_acc_1e5, 2_000_000);
}

#[test]
fn short_nav_velned_falls_back_to_ubx_other() {
    let evs = feed(&ubx_wire(ubx::CLASS_NAV, ubx::NAV_VELNED, &[0u8; 20]));
    assert_eq!(
        evs[0],
        Event::UbxOther {
            class: ubx::CLASS_NAV,
            id: ubx::NAV_VELNED
        }
    );
}

#[test]
fn enable_nav_velned_asks_for_it_every_solution() {
    let rx = ubx_wire(ubx::CLASS_ACK, 0x01, &[ubx::CLASS_CFG, ubx::CFG_MSG]);
    let mut gps = NeoGps::new(MockUart::new(rx));
    block_on(gps.enable_nav_velned()).unwrap();
    assert_eq!(
        cfg_payload_written(&gps.free().tx, ubx::CFG_MSG),
        vec![ubx::CLASS_NAV, ubx::NAV_VELNED, 1]
    );
}

/// Regression: the raw-frame accessors are reachable before any frame has
/// arrived, and used to underflow `ubx_last_len - 2` on an empty snapshot.
#[test]
fn raw_accessors_are_empty_before_the_first_frame() {
    let gps = NeoGps::new(MockUart::new(Vec::new()));
    assert!(gps.last_ubx_payload().is_empty());
    assert!(gps.last_ubx_frame().is_empty());
    assert!(gps.last_nmea_line().is_empty());
}

/// Same, but after the driver has seen traffic that produces no UBX frame:
/// the NMEA snapshot fills in while the UBX one must stay empty.
#[test]
fn ubx_accessors_stay_empty_when_only_nmea_arrives() {
    let mut gps = NeoGps::new(MockUart::new(nmea_wire(
        "GPGGA,092725.00,4717.11399,N,00833.91590,E,1,08,1.01,499.6,M,48.0,M,,",
    )));
    block_on(gps.next_event()).unwrap();
    assert!(!gps.last_nmea_line().is_empty());
    assert!(gps.last_ubx_payload().is_empty());
    assert!(gps.last_ubx_frame().is_empty());
}

// -- Deframer resync after a stray sync byte -------------------------------

/// Regression: a lone `0xB5` used to swallow whatever followed it. The
/// `UbxSync` state dropped the non-`0x62` byte instead of reconsidering it,
/// so the byte that actually opened the next frame was thrown away.
#[test]
fn stray_sync_byte_does_not_swallow_the_next_ubx_frame() {
    let mut bytes = vec![0xB5];
    bytes.extend(ubx_wire(ubx::CLASS_ACK, 0x01, &[0x06, 0x08]));
    assert_eq!(
        feed(&bytes),
        vec![Event::Ack {
            class: 0x06,
            id: 0x08,
            ok: true
        }],
        "the 0xB5 opening the real frame was consumed as a non-0x62 byte"
    );
}

#[test]
fn stray_sync_byte_does_not_swallow_the_next_nmea_sentence() {
    let mut bytes = vec![0xB5];
    bytes.extend(nmea_wire("GPGSA,A,3,,,,,,,,,,,,,2.5,1.3,2.1"));
    let evs = feed(&bytes);
    assert_eq!(evs.len(), 1, "the `$` was consumed as a non-0x62 byte");
    assert!(matches!(evs[0], Event::Nmea(nmea::Sentence::Gsa(_))));
}

/// A run of sync bytes must still frame the message that follows: only the
/// last `0xB5` before the `0x62` is the real one.
#[test]
fn repeated_sync_bytes_still_frame() {
    for lead in 1..=4 {
        let mut bytes = vec![0xB5; lead];
        bytes.extend(ubx_wire(ubx::CLASS_ACK, 0x01, &[0x06, 0x08]));
        assert_eq!(
            feed(&bytes).len(),
            1,
            "{} leading sync bytes lost the frame",
            lead
        );
    }
}

// -- next_coordinate treats all three sources equally strictly -------------

/// GGA quality 6 is dead reckoning, which u-blox also reports when the user
/// limits are exceeded; the RMC for that same epoch says `V`. Returning a
/// position through one and refusing it through the other made the result
/// depend on which sentence happened to arrive first.
#[test]
fn next_coordinate_skips_a_dead_reckoning_gga() {
    let mut rx = nmea_wire(
        "GPGGA,092725.00,4717.11399,N,00833.91590,E,6,08,1.01,499.6,M,48.0,M,,", // DR
    );
    rx.extend(nmea_wire(
        "GPGGA,092726.00,4718.00000,N,00833.91590,E,1,08,1.01,499.6,M,48.0,M,,", // GPS
    ));
    let mut gps = NeoGps::new(MockUart::new(rx));
    let c = block_on(gps.next_coordinate()).unwrap();
    assert_eq!(
        c.lat_1e7, 473_000_000,
        "returned the dead-reckoning position instead of waiting for a real fix"
    );
}

/// A simulated position must never come back as a real one.
#[test]
fn next_coordinate_skips_a_simulated_gga() {
    let rx = nmea_wire("GPGGA,092725.00,4717.11399,N,00833.91590,E,8,08,1.01,499.6,M,48.0,M,,");
    let mut gps = NeoGps::new(MockUart::new(rx));
    // Nothing else on the wire, so the mock drains and reports end of stream
    // rather than yielding the simulated fix.
    assert_eq!(block_on(gps.next_coordinate()), Err(Error::Eof));
}

/// The GGA and RMC arms must agree: an invalid RMC and a non-GNSS GGA are
/// both refused, and a valid one of either is accepted.
#[test]
fn gga_and_rmc_arms_agree_on_what_counts_as_a_fix() {
    for (q, accepted) in [
        (b'1', true),  // GPS
        (b'2', true),  // DGPS
        (b'4', true),  // RTK fixed
        (b'5', true),  // RTK float
        (b'6', false), // dead reckoning
        (b'7', false), // manual
        (b'8', false), // simulation
        (b'0', false), // no fix
    ] {
        let body = format!(
            "GPGGA,092725.00,4717.11399,N,00833.91590,E,{},08,1.01,499.6,M,48.0,M,,",
            q as char
        );
        let mut gps = NeoGps::new(MockUart::new(nmea_wire(&body)));
        let got = block_on(gps.next_coordinate());
        assert_eq!(
            got.is_ok(),
            accepted,
            "GGA quality {} should {} be a coordinate",
            q as char,
            if accepted { "" } else { "not" }
        );
    }

    // An RMC with status V carries a position but must never be returned.
    let rx = nmea_wire("GPRMC,083559.00,V,4717.11437,N,00833.91522,E,0.004,77.52,091202,,,N");
    let mut gps = NeoGps::new(MockUart::new(rx));
    assert_eq!(block_on(gps.next_coordinate()), Err(Error::Eof));
}

// -- Per-satellite data: NAV-SAT and NAV-SVINFO ----------------------------

/// Build a NAV-SAT payload: 8-byte header then 12 bytes per satellite.
fn nav_sat_payload(sats: &[(u8, u8, u8, i8, i16, bool)]) -> Vec<u8> {
    let mut p = vec![0u8; 8];
    p[4] = 1; // version
    p[5] = sats.len() as u8;
    for &(gnss_id, sv_id, cno, elev, azim, used) in sats {
        let mut b = [0u8; 12];
        b[0] = gnss_id;
        b[1] = sv_id;
        b[2] = cno;
        b[3] = elev as u8;
        b[4..6].copy_from_slice(&azim.to_le_bytes());
        b[8] = if used { 0x08 } else { 0 }; // flags bit 3: svUsed
        p.extend_from_slice(&b);
    }
    p
}

/// Build a NAV-SVINFO payload: 8-byte header then 12 bytes per channel.
fn nav_svinfo_payload(sats: &[(u8, u8, i8, i16, bool)]) -> Vec<u8> {
    let mut p = vec![0u8; 8];
    p[4] = sats.len() as u8; // numCh
    for (chn, &(sv_id, cno, elev, azim, used)) in sats.iter().enumerate() {
        let mut b = [0u8; 12];
        b[0] = chn as u8;
        b[1] = sv_id;
        b[2] = if used { 0x01 } else { 0 }; // flags bit 0: svUsed
        b[4] = cno;
        b[5] = elev as u8;
        b[6..8].copy_from_slice(&azim.to_le_bytes());
        p.extend_from_slice(&b);
    }
    p
}

#[test]
fn satellites_decodes_nav_sat() {
    let payload = nav_sat_payload(&[
        (0, 12, 45, 60, 180, true),  // GPS 12, used
        (6, 78, 30, -5, 350, false), // GLONASS 78, below horizon, unused
        (2, 24, 38, 15, 90, true),   // Galileo 24, used
    ]);
    let mut gps = NeoGps::new(MockUart::new(ubx_wire(
        ubx::CLASS_NAV,
        ubx::NAV_SAT,
        &payload,
    )));
    gps.set_skip_unknown_sentences(false);
    block_on(gps.next_event()).unwrap();

    let sats: Vec<_> = gps
        .satellites()
        .expect("NAV-SAT must yield satellites")
        .collect();
    assert_eq!(sats.len(), 3);
    assert_eq!(sats[0].gnss_id, Some(0));
    assert_eq!(sats[0].sv_id, 12);
    assert_eq!(sats[0].cno_dbhz, 45);
    assert_eq!(sats[0].elev_deg, 60);
    assert_eq!(sats[0].azim_deg, 180);
    assert!(sats[0].used_in_fix);
    assert_eq!(sats[1].elev_deg, -5, "negative elevation must stay signed");
    assert!(!sats[1].used_in_fix);
    assert_eq!(sats.iter().filter(|s| s.used_in_fix).count(), 2);
}

#[test]
fn satellites_decodes_nav_svinfo() {
    let payload = nav_svinfo_payload(&[(12, 45, 60, 180, true), (25, 0, -10, 300, false)]);
    let mut gps = NeoGps::new(MockUart::new(ubx_wire(
        ubx::CLASS_NAV,
        ubx::NAV_SVINFO,
        &payload,
    )));
    gps.set_skip_unknown_sentences(false);
    block_on(gps.next_event()).unwrap();

    let sats: Vec<_> = gps
        .satellites()
        .expect("NAV-SVINFO must yield satellites")
        .collect();
    assert_eq!(sats.len(), 2);
    assert_eq!(sats[0].gnss_id, None, "NAV-SVINFO predates gnssId");
    assert_eq!(sats[0].sv_id, 12);
    assert_eq!(sats[0].cno_dbhz, 45);
    assert!(sats[0].used_in_fix);
    assert_eq!(sats[1].elev_deg, -10);
    assert!(!sats[1].used_in_fix);
}

#[test]
fn satellites_is_none_for_other_frames() {
    let mut gps = NeoGps::new(MockUart::new(ubx_wire(
        ubx::CLASS_ACK,
        0x01,
        &[ubx::CLASS_CFG, ubx::CFG_MSG],
    )));
    assert!(gps.satellites().is_none(), "no frame seen yet");
    block_on(gps.next_event()).unwrap();
    assert!(gps.satellites().is_none(), "an ACK carries no satellites");
}

/// A frame claiming more satellites than its payload can hold must yield the
/// ones that are actually there, not read past the end.
#[test]
fn satellites_clamps_a_lying_count() {
    let mut payload = nav_sat_payload(&[(0, 12, 45, 60, 180, true)]);
    payload[5] = 30; // numSvs says 30, payload holds 1
    let mut gps = NeoGps::new(MockUart::new(ubx_wire(
        ubx::CLASS_NAV,
        ubx::NAV_SAT,
        &payload,
    )));
    gps.set_skip_unknown_sentences(false);
    block_on(gps.next_event()).unwrap();
    assert_eq!(gps.satellites().unwrap().count(), 1);
}

#[test]
fn enable_satellite_info_picks_by_capability() {
    // Series8 has NAV-SAT.
    let rx = ubx_wire(ubx::CLASS_ACK, 0x01, &[ubx::CLASS_CFG, ubx::CFG_MSG]);
    let mut gps = NeoGps::new(MockUart::new(rx));
    gps.set_capabilities(Capabilities {
        generation: Generation::Series8,
        protocol_version: 18,
    });
    block_on(gps.enable_satellite_info()).unwrap();
    assert_eq!(
        cfg_payload_written(&gps.free().tx, ubx::CFG_MSG),
        vec![ubx::CLASS_NAV, ubx::NAV_SAT, 1]
    );

    // NEO-6 falls back to NAV-SVINFO.
    let rx = ubx_wire(ubx::CLASS_ACK, 0x01, &[ubx::CLASS_CFG, ubx::CFG_MSG]);
    let mut gps = NeoGps::new(MockUart::new(rx));
    block_on(gps.enable_satellite_info()).unwrap();
    assert_eq!(
        cfg_payload_written(&gps.free().tx, ubx::CFG_MSG),
        vec![ubx::CLASS_NAV, ubx::NAV_SVINFO, 1]
    );
}

//! Hardware-in-the-loop test suite for `neo-gps`, running **on the MCU** with
//! a u-blox NEO module wired to its UART.
//!
//! Unlike the crate's host-side unit tests (which feed the driver scripted
//! bytes through a mock UART), every test here talks to real silicon over a
//! real serial line: the deframer sees the module's actual interleaved
//! NMEA/UBX stream, and every configuration command is answered — or refused —
//! by the module itself.
//!
//! ## Wiring
//!
//! See the `WIRING` block in `src/board.rs`. Out of the box: module TX to
//! GPIO5, module RX to GPIO6, common ground.
//!
//! ## Running
//!
//! ```sh
//! cd hwtest
//!
//! # async driver build (the default), NEO-7 attached:
//! NEO_GEN=7 cargo test --test hardware
//!
//! # blocking driver build, same module:
//! NEO_GEN=7 cargo test --test hardware --no-default-features --features sync
//!
//! # one test only:
//! NEO_GEN=7 cargo test --test hardware -- probe_classifies_generation
//! ```
//!
//! Configuration knobs (`NEO_GEN`, `NEO_BAUD`, `NEO_ALLOW_SAVE`) are read at
//! *build* time — see `src/config.rs`.
//!
//! ## What resets between tests, and what does not
//!
//! `probe-rs` resets the **ESP32** before each test case, so every test gets a
//! fresh driver with default settings. The **module** is not reset: it is
//! separately powered and keeps its RAM configuration for the whole run. Any
//! test that changes the module's configuration must therefore put it back,
//! or the next test inherits it — and `save_config_is_acked` would commit it
//! to flash permanently.
//!
//! ## Fix-dependent tests
//!
//! Tests that need a real satellite fix wait a bounded time and then skip the
//! fix-dependent assertions rather than fail: a driver test must not depend on
//! having sky view. What they never do is reveal a position — the checks are
//! range/consistency checks only, and no coordinate is ever logged.

#![no_std]
#![no_main]

esp_bootloader_esp_idf::esp_app_desc!();

/// Bring up RTT logging before the harness starts, so embedded-test's own
/// progress lines ("Running test ...") are visible too.
///
/// `Info`, not the macro's default `Trace`: esp-hal logs several hundred
/// TRACE lines from `init` alone, and the default `NoBlockSkip` channel mode
/// *discards* messages once the buffer is full — so a failure message can be
/// swallowed by clock-setup chatter. `BlockIfFull` guarantees the assertion
/// that failed is the one you read.
#[cfg(test)]
#[embedded_test::setup]
fn setup_log() {
    rtt_target::rtt_init_log!(
        log::LevelFilter::Info,
        rtt_target::ChannelMode::BlockIfFull,
        1024
    );
}

#[cfg(test)]
#[embedded_test::tests(executor = esp_rtos::embassy::Executor::new())]
mod tests {
    use embassy_time::{Duration, Instant, Timer};
    #[cfg(feature = "builtin-codec")]
    use neo_gps::nmea::Sentence;
    #[cfg(feature = "builtin-codec")]
    use neo_gps::NeoGps;
    use neo_gps::{ubx, Capabilities, Error, Event, Generation};
    use neo_gps_hwtest::board::{self, Ctx, Gps};
    use neo_gps_hwtest::pump::*;
    use neo_gps_hwtest::{config, maybe_await};

    /// NMEA standard-message ids for `CFG-MSG` class `0xF0`.
    const NMEA_GGA: u8 = 0x00;
    #[cfg(feature = "builtin-codec")]
    const NMEA_GLL: u8 = 0x01;
    #[cfg(feature = "builtin-codec")]
    const NMEA_GSA: u8 = 0x02;
    const NMEA_GSV: u8 = 0x03;
    #[cfg(feature = "builtin-codec")]
    const NMEA_RMC: u8 = 0x04;
    #[cfg(feature = "builtin-codec")]
    const NMEA_VTG: u8 = 0x05;

    /// Turn a standard NMEA sentence on, so a test that needs it does not
    /// depend on the module's saved configuration.
    ///
    /// The u-blox factory default enables GGA/GLL/GSA/GSV/RMC/VTG, but that
    /// set lives in battery-backed RAM and flash: any module that has been
    /// configured before — including plenty sold second-hand or preloaded by
    /// the board vendor — can arrive emitting a subset. Asking for what the
    /// test needs is both more robust and a stronger exercise of `CFG-MSG`
    /// than assuming it was already there.
    async fn require_sentence(gps: &mut Gps, id: u8) {
        maybe_await!(gps.set_msg_rate(0xF0, id, 1))
            .expect("enabling a standard NMEA sentence should ACK");
    }

    #[init]
    fn init() -> Ctx {
        board::init()
    }

    // ----------------------------------------------------------------------
    // Level 0: is anything on the wire at all?
    // ----------------------------------------------------------------------

    /// Print the module's `MON-VER` strings verbatim, straight from the raw
    /// payload, without going through the driver's `PROTVER` parser.
    ///
    /// Run this whenever `probe_classifies_generation` disagrees with the
    /// `NEO_GEN` you set. The disagreement has two very different causes and
    /// only the module's own bytes can tell them apart: either the hardware is
    /// not the generation you thought (relabelled boards are common), or
    /// `parse_mon_ver_protver` is misreading a version string it should
    /// handle — a driver bug. Changing `NEO_GEN` to match the probe would hide
    /// the second case, so confirm which one you have before touching it.
    ///
    /// Layout per the u-blox interface description: `swVersion[30]`,
    /// `hwVersion[10]`, then zero or more 30-byte extension strings, each
    /// null-terminated ASCII.
    #[test]
    #[ignore = "diagnostic: prints module firmware strings, asserts nothing"]
    #[timeout(30)]
    async fn report_module_identity(ctx: Ctx) {
        let mut gps = ctx.gps;
        // Probe first so the driver's verdict is filled in, then poll again so
        // the raw payload can be printed next to it.
        let caps = probe(&mut gps).await;
        gps.set_skip_unknown_sentences(false);
        maybe_await!(gps.send_ubx(ubx::CLASS_MON, ubx::MON_VER, &[])).expect("send MON-VER poll");

        let found = pump_until(&mut gps, Duration::from_secs(10), |e| {
            matches!(e, Event::UbxOther { class, id } if *class == ubx::CLASS_MON && *id == ubx::MON_VER)
        })
        .await;
        assert!(found.is_some(), "no MON-VER reply within 10s");

        let payload = gps.last_ubx_payload();
        let show = |label: &str, field: &[u8]| {
            let end = field.iter().position(|&b| b == 0).unwrap_or(field.len());
            match core::str::from_utf8(&field[..end]) {
                Ok(s) => log::info!("MON-VER {}: {:?}", label, s),
                Err(_) => log::info!("MON-VER {}: <non-UTF8> {:02X?}", label, &field[..end]),
            }
        };

        log::info!("MON-VER payload is {} bytes", payload.len());
        show("swVersion", &payload[..30.min(payload.len())]);
        if payload.len() >= 40 {
            show("hwVersion", &payload[30..40]);
        }
        for ext in payload[40.min(payload.len())..].chunks(30) {
            show("extension", ext);
        }
        log::info!(
            "driver's verdict: {:?}, protver {}",
            caps.generation,
            caps.protocol_version
        );
    }

    /// Prove the ESP side of the link in isolation, with the GPS module out of
    /// the circuit entirely.
    ///
    /// Run this when `uart_receives_framed_bytes` reports nothing arriving and
    /// you need to know which end is at fault. Unplug the module, put a single
    /// jumper directly between **GPIO6 and GPIO5**, and run:
    ///
    /// ```sh
    /// cargo test --test hardware -- --ignored --exact tests::uart_loopback_selftest
    /// ```
    ///
    /// * **Passes** — the UART, both pins, the baud divisor and the pin matrix
    ///   routing are all good, so a silent line is the module, its power, or
    ///   its wiring.
    /// * **Fails** — the fault is on the ESP side: wrong pins for your board
    ///   (some S3 boards commit GPIO5/6 to an onboard peripheral), or a bad
    ///   `NEO_BAUD`.
    ///
    /// Ignored by default because it needs that jumper: with the module wired
    /// up normally it would be testing nothing.
    #[test]
    #[ignore = "needs a jumper between GPIO5 and GPIO6, with the module unplugged"]
    #[timeout(20)]
    async fn uart_loopback_selftest(ctx: Ctx) {
        let mut uart = ctx.gps.free();
        const PROBE: &[u8] = b"neo-gps loopback probe\r\n";

        uart.write(PROBE).expect("UART write failed");
        uart.flush().expect("UART flush failed");

        let mut back = [0u8; PROBE.len()];
        let mut n = 0;
        let deadline = Instant::now() + Duration::from_secs(2);
        while n < back.len() && Instant::now() < deadline {
            if uart.read_ready() {
                n += uart.read(&mut back[n..]).expect("UART read failed");
            }
        }

        assert!(
            n > 0,
            "wrote {} bytes to GPIO6 and read nothing back on GPIO5. With the \
             jumper in place that rules the module out: either GPIO5/GPIO6 are \
             not usable as UART pins on this board, or the UART is misconfigured.",
            PROBE.len()
        );
        assert_eq!(
            &back[..n],
            &PROBE[..n],
            "bytes came back corrupted, which is a baud/framing fault rather \
             than a wiring one (NEO_BAUD is {})",
            config::BAUD
        );
        log::info!("loopback OK: {} bytes made the round trip", n);
    }

    /// Read the UART **directly**, bypassing the driver entirely, and report
    /// what is physically arriving on the RX pin.
    ///
    /// Every other test in this file goes through `next_event()`, which blocks
    /// until a frame completes. That makes all three of "nothing is wired up",
    /// "the baud rate is wrong" and "the driver has a bug" look identical from
    /// the outside: a test that hangs until its `#[timeout]` fires. This one
    /// tells them apart before you go looking in the driver, and it is the
    /// first thing to run when a whole suite run goes red.
    #[test]
    #[timeout(20)]
    async fn uart_receives_framed_bytes(ctx: Ctx) {
        let mut uart = ctx.gps.free();
        let mut seen = [0u8; 256];
        let mut n = 0;
        let mut errors = 0u32;

        // Collect for 2s, or until the buffer is full.
        let deadline = Instant::now() + Duration::from_secs(2);
        while Instant::now() < deadline && n < seen.len() {
            // esp-hal's inherent `Uart` methods, not the driver's abstraction:
            // `read_ready` never blocks, so the deadline is always honoured.
            if uart.read_ready() {
                match uart.read(&mut seen[n..]) {
                    Ok(got) => n += got,
                    // A receive error is not a reason to stop — it is evidence
                    // the line is *live*. We start sampling in the middle of
                    // whatever the module was already transmitting, so the
                    // first byte is routinely a partial frame; the driver
                    // absorbs exactly this, and so must the probe that is
                    // supposed to diagnose it. Which error, and whether any
                    // good bytes follow, is what tells the two faults apart
                    // below.
                    Err(e) => {
                        errors += 1;
                        if errors == 1 {
                            log::info!("first read error (expected mid-stream): {:?}", e);
                        }
                    }
                }
            }
        }

        assert!(
            n > 0 || errors > 0,
            "not a single byte or error in 2s — the RX line is completely idle. \
             The module streams NMEA continuously from power-up, fix or no fix, \
             so this is wiring, not satellites: check the module's TX reaches \
             the RX pin (see the WIRING block in src/board.rs), that GND is \
             shared between the boards, and that the module is powered."
        );
        assert!(
            n > 0,
            "the line is active ({} receive errors in 2s) but not one byte \
             framed correctly. That is the baud-mismatch signature: the pins \
             are right, the bit rate is not. NEO_BAUD is currently {} — the \
             u-blox factory default is 9600.",
            errors,
            config::BAUD
        );

        let bytes = &seen[..n];
        log::info!(
            "{} bytes on RX ({} framing errors), first 16: {:02X?}",
            n,
            errors,
            &bytes[..n.min(16)]
        );

        // NMEA sentences open with '$'; UBX frames open with the sync pair
        // 0xB5 0x62. Bytes that frame correctly but never form either are
        // still not this module talking to us.
        let has_nmea = bytes.contains(&b'$');
        let has_ubx = bytes.windows(2).any(|w| w == [0xB5, 0x62]);
        assert!(
            has_nmea || has_ubx,
            "{} bytes arrived but none of them start an NMEA sentence ('$') or \
             a UBX frame (B5 62), so whatever is on this pin is not a u-blox \
             module at {} baud.",
            n,
            config::BAUD
        );
    }

    // ----------------------------------------------------------------------
    // Local capabilities policy — no I/O, but it must hold on the target too
    // ----------------------------------------------------------------------

    /// A fresh driver assumes the oldest member of the family, so every
    /// command it issues before `probe` is universally valid;
    /// `set_capabilities` is the escape hatch for known hardware.
    #[test]
    #[timeout(10)]
    async fn capabilities_default_to_conservative_and_can_be_overridden(ctx: Ctx) {
        let mut gps = ctx.gps;
        assert_eq!(gps.capabilities(), Capabilities::conservative());
        assert_eq!(gps.capabilities().generation, Generation::Series6);
        assert_eq!(gps.capabilities().max_rate_ms(), 200);

        let m9 = Capabilities {
            generation: Generation::Series9,
            protocol_version: 32,
        };
        gps.set_capabilities(m9);
        assert_eq!(gps.capabilities(), m9);
        assert_eq!(gps.capabilities().max_rate_ms(), 40);
        assert!(m9.has_nav_pvt() && !m9.has_nav_sol() && m9.has_cfg_gnss());
    }

    /// The transmit buffer is fixed-size: an oversized payload is refused
    /// locally instead of writing a truncated frame onto the wire.
    #[test]
    #[timeout(10)]
    async fn send_ubx_rejects_oversized_payload(ctx: Ctx) {
        let mut gps = ctx.gps;
        let too_big = [0u8; ubx::TX_MAX_PAYLOAD + 1];
        assert!(
            matches!(
                maybe_await!(gps.send_ubx(ubx::CLASS_CFG, ubx::CFG_MSG, &too_big)),
                Err(Error::Overflow)
            ),
            "a payload over TX_MAX_PAYLOAD must be rejected, not truncated"
        );
        // ... and the driver is still usable: the refused frame wrote nothing.
        maybe_await!(gps.send_ubx(ubx::CLASS_MON, ubx::MON_VER, &[])).expect("send after Overflow");
    }

    // ----------------------------------------------------------------------
    // Probe
    // ----------------------------------------------------------------------

    /// `probe()` polls MON-VER and classifies the module. With `NEO_GEN` set
    /// at build time the classification is checked against the module you said
    /// is attached; without it, we only require that it succeeded.
    #[test]
    #[timeout(30)]
    async fn probe_classifies_generation(ctx: Ctx) {
        let mut gps = ctx.gps;
        let caps = probe(&mut gps).await;

        assert!(
            caps.protocol_version >= 12,
            "no u-blox module reports a protocol version below 12"
        );
        match config::EXPECTED_GEN {
            Some(expected) => assert_eq!(
                caps.generation, expected,
                "probed generation != NEO_GEN (module reported protver {}). \
                 Two very different causes: the board is not the generation \
                 you think — boards labelled NEO-8M carrying 7-series silicon \
                 are common — or the driver misread MON-VER. Run \
                 `--ignored --exact tests::report_module_identity` and read \
                 hwVersion: 00070000 is a u-blox 7, 00080000 an 8. Only change \
                 NEO_GEN once that field agrees with you.",
                caps.protocol_version
            ),
            None => {
                log::warn!("NEO_GEN was not set at build time — classification not cross-checked")
            }
        }

        // The probe result is stored, not just returned.
        assert_eq!(gps.capabilities(), caps);
    }

    /// The MON-VER reply is also reachable as raw bytes, which is how an
    /// external decoder (the `ublox` crate, say) would consume it. Checks the
    /// escape hatch end to end: `send_ubx` out, `UbxOther` in, and both frame
    /// accessors describing the very same frame.
    #[test]
    #[timeout(30)]
    async fn raw_ubx_poll_exposes_a_well_formed_frame(ctx: Ctx) {
        let mut gps = ctx.gps;
        gps.set_skip_unknown_sentences(false);
        maybe_await!(gps.send_ubx(ubx::CLASS_MON, ubx::MON_VER, &[])).expect("send MON-VER poll");

        let ev = pump_until(&mut gps, Duration::from_secs(5), |e| {
            matches!(e, Event::UbxOther { class, id } if *class == ubx::CLASS_MON && *id == ubx::MON_VER)
        })
        .await;
        assert!(ev.is_some(), "no MON-VER reply to the raw poll");

        assert_well_formed_ubx(gps.last_ubx_frame(), ubx::CLASS_MON, ubx::MON_VER);

        // MON-VER's fixed part is swVersion[30] + hwVersion[10]; the optional
        // extension strings follow in 30-byte blocks.
        let payload = gps.last_ubx_payload();
        assert!(
            payload.len() >= 40,
            "MON-VER payload shorter than its fixed part: {} bytes",
            payload.len()
        );
        assert_eq!(
            gps.last_ubx_frame().len(),
            payload.len() + 8,
            "frame and payload accessors describe different frames"
        );
    }

    // ----------------------------------------------------------------------
    // The NMEA stream and the built-in codec
    // ----------------------------------------------------------------------

    /// Out of the box the module streams NMEA, and the deframer plus built-in
    /// codec turn it into typed events without any configuration at all.
    #[cfg(feature = "builtin-codec")]
    #[test]
    #[timeout(20)]
    async fn streams_decoded_nmea_without_configuration(ctx: Ctx) {
        let mut gps = ctx.gps;
        let ev = pump_until(&mut gps, Duration::from_secs(5), |e| {
            matches!(e, Event::Nmea(_))
        })
        .await;
        assert!(ev.is_some(), "no decoded NMEA sentence within 5s");
    }

    /// All three built-in sentence decoders fire against the live stream, and
    /// the values they produce are internally consistent — whatever the talker
    /// id, which varies by generation and constellation.
    #[cfg(feature = "builtin-codec")]
    #[test]
    #[timeout(40)]
    async fn decodes_gga_rmc_and_gsa_from_the_live_stream(ctx: Ctx) {
        let mut gps = ctx.gps;
        for id in [NMEA_GGA, NMEA_RMC, NMEA_GSA] {
            require_sentence(&mut gps, id).await;
        }
        let (mut gga, mut rmc, mut gsa) = (false, false, false);

        observe(&mut gps, Duration::from_secs(15), |ev| match ev {
            Event::Nmea(Sentence::Gga(g)) => {
                gga = true;
                assert!(g.sats_in_use <= 64, "implausible satellite count");
                if let Some(hdop) = g.hdop_1e2 {
                    assert!(hdop <= 10_000, "HDOP over 100 is not a real value");
                }
                if g.quality.has_fix() {
                    if let (Some(lat), Some(lon)) = (g.lat_1e7, g.lon_1e7) {
                        assert_plausible(lat, lon);
                    }
                }
            }
            Event::Nmea(Sentence::Rmc(r)) => {
                rmc = true;
                if let Some(t) = r.time {
                    assert!(t.hour < 24 && t.minute < 60 && t.second < 61);
                }
                if let Some(d) = r.date {
                    assert!((1..=31).contains(&d.day) && (1..=12).contains(&d.month));
                }
                if r.valid {
                    if let (Some(lat), Some(lon)) = (r.lat_1e7, r.lon_1e7) {
                        assert_plausible(lat, lon);
                    }
                }
            }
            Event::Nmea(Sentence::Gsa(g)) => {
                gsa = true;
                assert!(
                    (1..=3).contains(&g.fix_type),
                    "GSA fix type {} outside 1..=3",
                    g.fix_type
                );
            }
            _ => {}
        })
        .await;

        assert!(gga, "no GGA decoded in 15s");
        assert!(rmc, "no RMC decoded in 15s");
        assert!(gsa, "no GSA decoded in 15s");
    }

    /// `last_nmea_line` must hand out exactly the bytes the event was decoded
    /// from — that is the contract external decoders rely on. Verified by
    /// re-checking the sentence's own checksum over those bytes.
    #[test]
    #[timeout(20)]
    async fn last_nmea_line_matches_the_event_it_belongs_to(ctx: Ctx) {
        let mut gps = ctx.gps;
        // GSV is framed but not decoded by the built-in codec, so it is the
        // sentence that reaches us as `NmeaOther`.
        require_sentence(&mut gps, NMEA_GSV).await;
        gps.set_skip_unknown_sentences(false);

        let ev = pump_until(&mut gps, Duration::from_secs(10), |e| {
            matches!(e, Event::NmeaOther { .. })
        })
        .await
        .expect("no undecoded NMEA sentence within 10s, even after enabling GSV");

        let Event::NmeaOther { talker, mtype } = ev else {
            unreachable!()
        };
        assert_well_formed_nmea(gps.last_nmea_line(), talker, mtype);
    }

    /// `skip_unknown_sentences` (default `true`) hides frames the driver does
    /// not decode; turning it off surfaces them. GSV is the undecoded sentence
    /// used here — enabled first, so the "hides them" half is proving that a
    /// sentence which *is* on the wire stays hidden, not that the wire is idle.
    #[test]
    #[timeout(30)]
    async fn skip_unknown_hides_then_surfaces_other_frames(ctx: Ctx) {
        let mut gps = ctx.gps;
        require_sentence(&mut gps, NMEA_GSV).await;

        gps.set_skip_unknown_sentences(true);
        let leaked = pump_until(&mut gps, Duration::from_secs(5), |e| {
            matches!(e, Event::NmeaOther { .. } | Event::UbxOther { .. })
        })
        .await;
        assert!(
            leaked.is_none(),
            "an Other event surfaced despite skip=true"
        );

        gps.set_skip_unknown_sentences(false);
        let other = pump_until(&mut gps, Duration::from_secs(5), |e| {
            matches!(e, Event::NmeaOther { .. })
        })
        .await;
        assert!(
            other.is_some(),
            "expected an undecoded sentence (GLL/GSV/VTG) with skip=false"
        );
    }

    // ----------------------------------------------------------------------
    // Configuration: CFG-MSG, CFG-RATE, ACK/NAK correlation
    // ----------------------------------------------------------------------

    /// A raw `CFG` frame is answered with `ACK-ACK`, and that acknowledgement
    /// reaches the caller as an `Event::Ack` through the ordinary event pump —
    /// the path `send_cfg_acked` is built on.
    #[test]
    #[timeout(30)]
    async fn ack_surfaces_as_an_event(ctx: Ctx) {
        let mut gps = ctx.gps;
        // Set GGA to its default rate: a no-op change that still gets an ACK.
        maybe_await!(gps.send_ubx(ubx::CLASS_CFG, ubx::CFG_MSG, &[0xF0, NMEA_GGA, 1]))
            .expect("send CFG-MSG");

        let ev = pump_until(&mut gps, Duration::from_secs(5), |e| {
            matches!(e, Event::Ack { class, id, .. } if *class == ubx::CLASS_CFG && *id == ubx::CFG_MSG)
        })
        .await
        .expect("no ACK for CFG-MSG within 5s");

        assert!(
            matches!(ev, Event::Ack { ok: true, .. }),
            "module refused a default-rate CFG-MSG: {:?}",
            ev
        );
    }

    /// `disable_nmea` / `set_msg_rate` (CFG-MSG) must be ACKed *and take
    /// effect*: the muted sentence stops arriving, and comes back when
    /// restored. Only RAM is touched, so a power cycle undoes it.
    #[test]
    #[timeout(60)]
    async fn cfg_msg_mutes_and_restores_a_sentence(ctx: Ctx) {
        let mut gps = ctx.gps;
        gps.set_skip_unknown_sentences(false); // GSV is not decoded by default

        // Establish the baseline rather than assume it: turn GSV on and
        // confirm it arrives, so the mute below is provably a change.
        require_sentence(&mut gps, NMEA_GSV).await;
        assert!(
            pump_until(&mut gps, Duration::from_secs(10), is_gsv)
                .await
                .is_some(),
            "GSV did not start arriving after CFG-MSG enabled it, so muting it \
             below would prove nothing"
        );

        maybe_await!(gps.disable_nmea(NMEA_GSV)).expect("mute GSV should ACK");
        // Let sentences already in flight drain before judging.
        observe(&mut gps, Duration::from_secs(2), |_| {}).await;
        assert!(
            pump_until(&mut gps, Duration::from_secs(5), is_gsv)
                .await
                .is_none(),
            "GSV still arriving 2s after CFG-MSG muted it"
        );

        maybe_await!(gps.set_msg_rate(0xF0, NMEA_GSV, 1)).expect("restore GSV should ACK");
        assert!(
            pump_until(&mut gps, Duration::from_secs(10), is_gsv)
                .await
                .is_some(),
            "GSV did not come back after being restored"
        );
    }

    /// `set_nav_rate_ms` (CFG-RATE) is ACKed and actually changes how often the
    /// module solves: doubling the rate must visibly shorten the interval
    /// between RMC sentences.
    ///
    /// The chatty default sentence set is muted first — at 9600 baud, GSV plus
    /// GLL plus VTG at 2 Hz would saturate the link and the measurement would
    /// be timing the UART, not the navigation rate.
    #[cfg(feature = "builtin-codec")]
    #[test]
    #[timeout(90)]
    async fn nav_rate_is_acked_and_changes_the_solution_cadence(ctx: Ctx) {
        let mut gps = ctx.gps;
        let caps = probe(&mut gps).await;
        assert!(
            caps.max_rate_ms() <= 500,
            "every supported generation can do at least 2 Hz"
        );

        for id in [NMEA_GSV, NMEA_GLL, NMEA_VTG, NMEA_GSA] {
            maybe_await!(gps.disable_nmea(id)).expect("muting a default sentence should ACK");
        }

        maybe_await!(gps.set_nav_rate_ms(1000)).expect("CFG-RATE at 1 Hz should ACK");
        let at_1hz = mean_rmc_interval_ms(&mut gps, 4).await;
        assert!(
            (600..=1600).contains(&at_1hz),
            "asked for 1 Hz, RMC arrives every {}ms",
            at_1hz
        );

        maybe_await!(gps.set_nav_rate_ms(500)).expect("CFG-RATE at 2 Hz should ACK");
        let at_2hz = mean_rmc_interval_ms(&mut gps, 4).await;
        assert!(
            at_2hz < at_1hz * 3 / 4,
            "2 Hz ({}ms) is not measurably faster than 1 Hz ({}ms)",
            at_2hz,
            at_1hz
        );

        // Put back everything this test muted. Only RAM is touched, so a power
        // cycle would undo it anyway — but the module keeps its RAM across the
        // whole suite run, and `save_config_is_acked` could otherwise commit
        // this stripped-down sentence set to flash permanently.
        maybe_await!(gps.set_nav_rate_ms(1000)).expect("restoring 1 Hz should ACK");
        for id in [NMEA_GSV, NMEA_GLL, NMEA_VTG, NMEA_GSA] {
            require_sentence(&mut gps, id).await;
        }
        settle(&mut gps).await;
    }

    /// A rate faster than the probed module can sustain is refused locally, by
    /// the capability guard, without ever reaching the wire.
    #[test]
    #[timeout(30)]
    async fn nav_rate_below_module_minimum_is_refused_locally(ctx: Ctx) {
        let mut gps = ctx.gps;
        let caps = probe(&mut gps).await;
        let too_fast = caps.max_rate_ms() - 1; // one ms below the shortest interval
        assert!(
            matches!(
                maybe_await!(gps.set_nav_rate_ms(too_fast)),
                Err(Error::Unsupported)
            ),
            "{}ms is below the {:?} minimum of {}ms and must be refused",
            too_fast,
            caps.generation,
            caps.max_rate_ms()
        );
    }

    /// The other half of ACK correlation: a `CFG` message the module does not
    /// implement comes back as `ACK-NAK`, and `send_cfg_acked` reports it as
    /// `Error::Nak` rather than as success or a timeout.
    #[test]
    #[timeout(30)]
    async fn unimplemented_cfg_message_is_reported_as_nak(ctx: Ctx) {
        let mut gps = ctx.gps;
        // 0x99 is not a CFG message on any generation of the family.
        const NO_SUCH_CFG: u8 = 0x99;
        let result = maybe_await!(gps.send_cfg_acked(NO_SUCH_CFG, &[]));
        assert_eq!(
            result,
            Err(Error::Nak {
                class: ubx::CLASS_CFG,
                id: NO_SUCH_CFG
            }),
            "an unimplemented CFG id should be NAKed, got {:?}",
            result
        );
    }

    // ----------------------------------------------------------------------
    // Binary navigation output
    // ----------------------------------------------------------------------

    /// `enable_binary_nav` picks the right message for whatever is attached —
    /// NAV-PVT on 7-series and later, NAV-POSLLH + NAV-SOL on a NEO-6M — and a
    /// binary nav frame follows on either path.
    #[cfg(feature = "builtin-codec")]
    #[test]
    #[timeout(40)]
    async fn enable_binary_nav_yields_a_binary_frame(ctx: Ctx) {
        let mut gps = ctx.gps;
        let caps = probe(&mut gps).await;
        maybe_await!(gps.enable_binary_nav()).expect("enable_binary_nav should ACK");

        let ev = pump_until(&mut gps, Duration::from_secs(10), |e| {
            matches!(e, Event::NavPvt(_) | Event::NavPosllh(_) | Event::NavSol(_))
        })
        .await
        .expect("no binary nav frame after enable_binary_nav");

        // It must have chosen by capability, not at random.
        if caps.has_nav_pvt() {
            assert!(
                matches!(ev, Event::NavPvt(_)),
                "module has NAV-PVT but got {:?}",
                ev
            );
        } else {
            assert!(
                matches!(ev, Event::NavPosllh(_) | Event::NavSol(_)),
                "pre-protocol-14 module got {:?}",
                ev
            );
        }
    }

    /// 7-series and later: `enable_nav_pvt` is ACKed, NAV-PVT frames follow,
    /// and the decoded fields are self-consistent. Skips itself on a NEO-6M.
    #[cfg(feature = "builtin-codec")]
    #[test]
    #[timeout(60)]
    async fn nav_pvt_on_modern_modules(ctx: Ctx) {
        let mut gps = ctx.gps;
        let caps = probe(&mut gps).await;
        if !caps.has_nav_pvt() {
            log::info!(
                "skip: module has no NAV-PVT (protver {})",
                caps.protocol_version
            );
            return;
        }

        maybe_await!(gps.enable_nav_pvt()).expect("enable NAV-PVT should ACK");
        let ev = pump_until(&mut gps, Duration::from_secs(10), |e| {
            matches!(e, Event::NavPvt(_))
        })
        .await
        .expect("no NAV-PVT frame after enabling");

        let Event::NavPvt(pvt) = ev else {
            unreachable!()
        };
        assert!(
            pvt.itow_ms < 7 * 24 * 3600 * 1000,
            "iTOW outside a GPS week"
        );
        assert!(
            pvt.fix_type <= 5,
            "fix type {} is not defined",
            pvt.fix_type
        );
        assert!(pvt.num_sv <= 64, "implausible satellite count");
        assert!(pvt.month <= 12 && pvt.day <= 31 && pvt.hour < 24);
        if pvt.gnss_fix_ok() {
            assert!(pvt.fix_type >= 2, "gnssFixOK set but fix type is no-fix/DR");
            assert_plausible(pvt.lat_1e7, pvt.lon_1e7);
        }

        maybe_await!(gps.set_msg_rate(ubx::CLASS_NAV, ubx::NAV_PVT, 0)).ok(); // stop the stream
    }

    /// NEO-6 (protocol < 14) has no NAV-PVT, so the driver refuses it locally
    /// instead of sending a frame the module would NAK. Skips itself on
    /// anything newer.
    #[test]
    #[timeout(30)]
    async fn nav_pvt_is_refused_on_legacy_modules(ctx: Ctx) {
        let mut gps = ctx.gps;
        let caps = probe(&mut gps).await;
        if caps.has_nav_pvt() {
            log::info!("skip: module supports NAV-PVT");
            return;
        }
        assert!(
            matches!(maybe_await!(gps.enable_nav_pvt()), Err(Error::Unsupported)),
            "NAV-PVT must be refused on a pre-protocol-14 module"
        );
    }

    /// Legacy binary nav: NAV-POSLLH on every generation, plus NAV-SOL on the
    /// protocols that still have it (removed at protver 24). Both are ACKed
    /// and their frames decode into consistent values.
    #[cfg(feature = "builtin-codec")]
    #[test]
    #[timeout(60)]
    async fn nav_posllh_and_nav_sol_on_legacy_protocols(ctx: Ctx) {
        let mut gps = ctx.gps;
        let caps = probe(&mut gps).await;

        maybe_await!(gps.enable_nav_posllh()).expect("enable NAV-POSLLH should ACK");
        let ev = pump_until(&mut gps, Duration::from_secs(10), |e| {
            matches!(e, Event::NavPosllh(_))
        })
        .await
        .expect("no NAV-POSLLH frame after enabling");
        let Event::NavPosllh(pos) = ev else {
            unreachable!()
        };
        assert!(
            pos.itow_ms < 7 * 24 * 3600 * 1000,
            "iTOW outside a GPS week"
        );
        maybe_await!(gps.set_msg_rate(ubx::CLASS_NAV, ubx::NAV_POSLLH, 0)).ok();

        if !caps.has_nav_sol() {
            log::info!(
                "skip NAV-SOL: removed at protver {} (M9+)",
                caps.protocol_version
            );
            return;
        }

        maybe_await!(gps.enable_nav_sol()).expect("enable NAV-SOL should ACK");
        let ev = pump_until(&mut gps, Duration::from_secs(10), |e| {
            matches!(e, Event::NavSol(_))
        })
        .await
        .expect("no NAV-SOL frame after enabling");
        let Event::NavSol(sol) = ev else {
            unreachable!()
        };
        assert!(sol.gps_fix <= 5, "fix type {} is not defined", sol.gps_fix);
        assert!(sol.num_sv <= 64, "implausible satellite count");
        assert!(
            !sol.gps_fix_ok() || sol.gps_fix >= 2,
            "gpsFixOK set but fix type is no-fix/DR"
        );
        maybe_await!(gps.set_msg_rate(ubx::CLASS_NAV, ubx::NAV_SOL, 0)).ok();
    }

    // ----------------------------------------------------------------------
    // Fix-dependent (skipped without sky view; never reveals a position)
    // ----------------------------------------------------------------------

    /// The high-level convenience pump: once a fix exists, `next_coordinate`
    /// yields an in-range position and never the pre-fix `0, 0`.
    #[cfg(feature = "builtin-codec")]
    #[test]
    #[timeout(120)]
    async fn next_coordinate_is_plausible_when_fixed(ctx: Ctx) {
        let mut gps = ctx.gps;
        if !wait_for_fix(&mut gps, Duration::from_secs(60)).await {
            log::warn!("no GPS fix within 60s — skipping the coordinate check");
            return;
        }
        // Fixes are flowing, so this returns promptly.
        let c = maybe_await!(gps.next_coordinate()).expect("next_coordinate after a confirmed fix");
        assert_plausible(c.lat_1e7, c.lon_1e7);
        log::info!("got an in-range fix (coordinates withheld)");
    }

    /// With a fix, the NMEA and binary paths must describe the *same* place:
    /// a disagreement of more than ~1 km between `next_coordinate` (RMC/GGA)
    /// and NAV-PVT means one of the two decoders is wrong. Still reveals
    /// nothing — only the difference is examined.
    #[cfg(feature = "builtin-codec")]
    #[test]
    #[timeout(150)]
    async fn nmea_and_binary_positions_agree(ctx: Ctx) {
        let mut gps = ctx.gps;
        let caps = probe(&mut gps).await;
        if !caps.has_nav_pvt() {
            log::info!("skip: no NAV-PVT to cross-check the NMEA position against");
            return;
        }
        if !wait_for_fix(&mut gps, Duration::from_secs(60)).await {
            log::warn!("no GPS fix within 60s — skipping the cross-check");
            return;
        }

        let from_nmea = maybe_await!(gps.next_coordinate()).expect("NMEA coordinate");
        maybe_await!(gps.enable_nav_pvt()).expect("enable NAV-PVT should ACK");
        let ev = pump_until(
            &mut gps,
            Duration::from_secs(20),
            |e| matches!(e, Event::NavPvt(p) if p.gnss_fix_ok()),
        )
        .await;
        let Some(Event::NavPvt(pvt)) = ev else {
            log::warn!("fix was lost before a NAV-PVT arrived — skipping the cross-check");
            return;
        };

        // 1e-2 degrees is roughly 1 km of latitude: far beyond any real
        // difference between two solutions seconds apart, far below anything
        // that would identify a location.
        const TOLERANCE_1E7: i32 = 100_000;
        assert!(
            (from_nmea.lat_1e7 - pvt.lat_1e7).abs() < TOLERANCE_1E7
                && (from_nmea.lon_1e7 - pvt.lon_1e7).abs() < TOLERANCE_1E7,
            "NMEA and NAV-PVT positions differ by more than ~1 km"
        );
    }

    // ----------------------------------------------------------------------
    // External codec crates
    //
    // Everything above proves the driver's *transport*: the deframer hands out
    // whole, checksum-valid frames. These prove the other half of the crate's
    // contract — that those frames are exactly what the external vocabulary
    // crates expect, decoded through `neo_gps::codec` on live bytes rather
    // than on captured ones.
    //
    //     NEO_GEN=7 cargo test --test hardware --features codec-nmea,codec-ublox
    //
    // Without those features the suite is unchanged; they only add tests.
    // ----------------------------------------------------------------------

    /// The `nmea` crate decodes a live sentence the built-in codec does not.
    ///
    /// GSV is the whole reason `codec::nmea` exists: the built-in decoder
    /// covers GGA/RMC/GSA and surfaces everything else as `NmeaOther`, so
    /// per-satellite data is reachable *only* through the external crate. The
    /// input is the driver's `last_nmea_line()` — no leading `$`, checksum
    /// trailer included — so this also pins down the adapter's re-framing
    /// against what `nmea::parse_str` actually accepts.
    #[cfg(feature = "codec-nmea")]
    #[test]
    #[timeout(30)]
    async fn external_nmea_crate_decodes_a_sentence_the_builtin_codec_skips(ctx: Ctx) {
        use nmea::ParseResult;

        let mut gps = ctx.gps;
        require_sentence(&mut gps, NMEA_GSV).await;
        gps.set_skip_unknown_sentences(false);

        let ev = pump_until(&mut gps, Duration::from_secs(10), is_gsv)
            .await
            .expect("no GSV within 10s, even after enabling it");
        let Event::NmeaOther { talker, mtype } = ev else {
            unreachable!()
        };
        assert_well_formed_nmea(gps.last_nmea_line(), talker, mtype);

        match neo_gps::codec::nmea::decode(gps.last_nmea_line()) {
            Ok(ParseResult::GSV(gsv)) => {
                assert!(
                    gsv.sentence_num >= 1 && gsv.sentence_num <= gsv.number_of_sentences,
                    "GSV {} of {} is not a valid position in the group",
                    gsv.sentence_num,
                    gsv.number_of_sentences
                );
                assert!(
                    gsv.sats_in_view <= 64,
                    "implausible satellites-in-view count"
                );
                log::info!(
                    "external `nmea` crate: GSV {}/{}, {} satellites in view",
                    gsv.sentence_num,
                    gsv.number_of_sentences,
                    gsv.sats_in_view
                );
            }
            // Deliberately not printing the decoded value: a mis-decode could
            // put a position in it, and no test here reveals one.
            Ok(_) => panic!("the `nmea` crate read a GSV line as a different sentence type"),
            Err(e) => panic!(
                "the `nmea` crate rejected a line our deframer checksum-verified: {:?}",
                e
            ),
        }
    }

    /// The built-in GGA decoder and the `nmea` crate agree on the *same live
    /// bytes*.
    ///
    /// Two independent parsers over one sentence: a disagreement means one of
    /// them is reading the wrong field, and the driver is the one under test.
    /// Only the delta is ever logged — the position itself never leaves the
    /// device.
    #[cfg(feature = "codec-nmea")]
    #[cfg(feature = "builtin-codec")]
    #[test]
    #[timeout(120)]
    async fn builtin_and_external_nmea_agree_on_a_live_gga(ctx: Ctx) {
        use nmea::ParseResult;

        let mut gps = ctx.gps;
        require_sentence(&mut gps, NMEA_GGA).await;

        let ev = pump_until(&mut gps, Duration::from_secs(60), |e| {
            matches!(e, Event::Nmea(Sentence::Gga(g)) if g.lat_1e7.is_some() && g.lon_1e7.is_some())
        })
        .await;
        let Some(Event::Nmea(Sentence::Gga(builtin))) = ev else {
            log::warn!("no GGA carrying a position within 60s (no sky view?) — skipping");
            return;
        };

        let Ok(ParseResult::GGA(external)) = neo_gps::codec::nmea::decode(gps.last_nmea_line())
        else {
            panic!("the `nmea` crate did not return a GGA for a line our codec decoded as one");
        };

        let (blat, blon) = (builtin.lat_1e7.unwrap(), builtin.lon_1e7.unwrap());
        assert_plausible(blat, blon);
        let elat = (external.latitude.expect("external GGA has no latitude") * 1e7) as i32;
        let elon = (external.longitude.expect("external GGA has no longitude") * 1e7) as i32;

        // 1e-4 degrees, ~11 m: comfortably above the rounding difference
        // between the driver's integer arithmetic and the crate's `f64`, and
        // far below what a genuine field mix-up would produce.
        const TOLERANCE_1E7: i32 = 1_000;
        let (dlat, dlon) = ((blat - elat).abs(), (blon - elon).abs());
        assert!(
            dlat < TOLERANCE_1E7 && dlon < TOLERANCE_1E7,
            "built-in and `nmea`-crate positions differ by {} / {} (units of 1e-7 deg)",
            dlat,
            dlon
        );
        assert_eq!(
            u32::from(builtin.sats_in_use),
            external
                .fix_satellites
                .expect("external GGA has no satellite count"),
            "the two decoders disagree on the satellite count in one sentence"
        );
        log::info!(
            "built-in and external GGA agree (delta {} / {} in 1e-7 deg)",
            dlat,
            dlon
        );
    }

    /// The `ublox` crate decodes a live NAV-POSLLH frame from
    /// `last_ubx_frame()`, and its typed view matches the built-in decoder's.
    ///
    /// NAV-POSLLH is the UBX message to cross-check on: 28 bytes in every
    /// protocol version u-blox has shipped, so the external crate's fixed
    /// layout is right for any module the driver supports. (NAV-PVT is not —
    /// see the next test.)
    ///
    /// This also pins down the adapter's input contract: `last_ubx_frame` must
    /// hand out the *whole* frame — sync bytes through checksum — because the
    /// `ublox` parser is a stream parser and would simply find nothing in a
    /// bare payload rather than complain.
    ///
    /// `Parser::default()` needs `alloc`, so the parser is built here over a
    /// `FixedLinearBuffer`, which is how a `no_std` user has to do it.
    #[cfg(feature = "codec-ublox")]
    #[test]
    #[timeout(60)]
    async fn external_ublox_crate_decodes_a_live_nav_posllh(ctx: Ctx) {
        let mut gps = ctx.gps;
        // Without `builtin-codec` the frame arrives as UbxOther, which the
        // default filter hides.
        gps.set_skip_unknown_sentences(false);

        maybe_await!(gps.enable_nav_posllh()).expect("enable NAV-POSLLH should ACK");
        let ev = pump_until(&mut gps, Duration::from_secs(15), |e| {
            matches!(e, Event::NavPosllh(_))
                || matches!(e, Event::UbxOther { class, id }
                    if *class == ubx::CLASS_NAV && *id == ubx::NAV_POSLLH)
        })
        .await
        .expect("no NAV-POSLLH frame after enabling");
        assert_well_formed_ubx(gps.last_ubx_frame(), ubx::CLASS_NAV, ubx::NAV_POSLLH);

        let (itow, lat_deg, lon_deg) =
            decode_with_ublox_crate(gps.last_ubx_frame(), "NAV-POSLLH", |pkt| match pkt {
                ublox::PacketRef::NavPosLlh(pos) => {
                    Some((pos.itow(), pos.lat_degrees(), pos.lon_degrees()))
                }
                _ => None,
            });
        let (elat, elon) = ((lat_deg * 1e7) as i32, (lon_deg * 1e7) as i32);

        // With the built-in decoder present, cross-check the two against each
        // other. Without it the external crate is the only decoder there is,
        // which is exactly the configuration this build exercises, so fall
        // back to range checks.
        if let Event::NavPosllh(builtin) = ev {
            assert_eq!(itow, builtin.itow_ms, "the two decoders disagree on iTOW");
            // Both read the same scaled `i32`, so anything beyond one unit of
            // `f64` truncation means they are not reading the same field.
            let (dlat, dlon) = (
                (builtin.lat_1e7 - elat).abs(),
                (builtin.lon_1e7 - elon).abs(),
            );
            assert!(
                dlat <= 1 && dlon <= 1,
                "built-in and `ublox`-crate NAV-POSLLH positions differ by {} / {} (1e-7 deg)",
                dlat,
                dlon
            );
            log::info!("built-in and external NAV-POSLLH agree on iTOW and position");
        } else {
            assert!(itow < 7 * 24 * 3600 * 1000, "iTOW outside a GPS week");
            assert!(
                (-900_000_000..=900_000_000).contains(&elat)
                    && (-1_800_000_000..=1_800_000_000).contains(&elon),
                "external decoder produced an out-of-range position"
            );
            log::info!("external `ublox` crate is the only NAV-POSLLH decoder in this build");
        }

        maybe_await!(gps.set_msg_rate(ubx::CLASS_NAV, ubx::NAV_POSLLH, 0)).ok();
        // stop the stream
    }

    /// The `ublox` crate decodes a live NAV-PVT frame, on the modules whose
    /// NAV-PVT it actually models.
    ///
    /// `ublox` 0.4 declares NAV-PVT as `fixed_payload_len = 92`, the protocol
    /// 15+ layout. Protocol 14 (NEO-7) emits the original **84**-byte one,
    /// without `headingOfVehicle` or the magnetic-declination fields, and the
    /// crate's generated match arm is guarded on the length: it does not error,
    /// it quietly falls through to `PacketRef::Unknown`. This driver's own
    /// decoder handles both, which is exactly why `codec::ublox` is optional
    /// rather than a replacement.
    ///
    /// So the gate here is the *protocol version*, not `has_nav_pvt()`: a
    /// NEO-7 has NAV-PVT, just not the one the external crate can read.
    #[cfg(feature = "codec-ublox")]
    #[cfg(feature = "builtin-codec")]
    #[test]
    #[timeout(60)]
    async fn external_ublox_crate_decodes_a_live_nav_pvt(ctx: Ctx) {
        /// First protocol version whose NAV-PVT is the 92-byte layout the
        /// `ublox` crate hard-codes.
        const NAV_PVT_92_BYTE_PROTVER: u8 = 15;

        let mut gps = ctx.gps;
        let caps = probe(&mut gps).await;
        if !caps.has_nav_pvt() {
            log::info!(
                "skip: module has no NAV-PVT (protver {})",
                caps.protocol_version
            );
            return;
        }
        if caps.protocol_version < NAV_PVT_92_BYTE_PROTVER {
            log::info!(
                "skip: protver {} emits the 84-byte NAV-PVT; the `ublox` crate only models the \
                 92-byte protver {}+ layout (the built-in decoder handles both). \
                 `external_ublox_crate_decodes_a_live_nav_posllh` covers the adapter here.",
                caps.protocol_version,
                NAV_PVT_92_BYTE_PROTVER
            );
            return;
        }

        maybe_await!(gps.enable_nav_pvt()).expect("enable NAV-PVT should ACK");
        let ev = pump_until(&mut gps, Duration::from_secs(15), |e| {
            matches!(e, Event::NavPvt(_))
        })
        .await
        .expect("no NAV-PVT frame after enabling");
        let Event::NavPvt(builtin) = ev else {
            unreachable!()
        };
        assert_well_formed_ubx(gps.last_ubx_frame(), ubx::CLASS_NAV, ubx::NAV_PVT);

        let (itow, num_sv, lat_deg, lon_deg) =
            decode_with_ublox_crate(gps.last_ubx_frame(), "NAV-PVT", |pkt| match pkt {
                ublox::PacketRef::NavPvt(pvt) => Some((
                    pvt.itow(),
                    pvt.num_satellites(),
                    pvt.lat_degrees(),
                    pvt.lon_degrees(),
                )),
                _ => None,
            });

        assert_eq!(itow, builtin.itow_ms, "the two decoders disagree on iTOW");
        assert_eq!(
            num_sv, builtin.num_sv,
            "the two decoders disagree on the satellite count"
        );

        if builtin.gnss_fix_ok() {
            let (elat, elon) = ((lat_deg * 1e7) as i32, (lon_deg * 1e7) as i32);
            assert_plausible(elat, elon);
            let (dlat, dlon) = (
                (builtin.lat_1e7 - elat).abs(),
                (builtin.lon_1e7 - elon).abs(),
            );
            assert!(
                dlat <= 1 && dlon <= 1,
                "built-in and `ublox`-crate NAV-PVT positions differ by {} / {} (units of 1e-7 deg)",
                dlat,
                dlon
            );
            log::info!("built-in and external NAV-PVT agree on iTOW, satellites and position");
        } else {
            log::info!(
                "external `ublox` crate decoded NAV-PVT; no fix yet, so position not cross-checked"
            );
        }

        maybe_await!(gps.set_msg_rate(ubx::CLASS_NAV, ubx::NAV_PVT, 0)).ok(); // stop the stream
    }

    // ----------------------------------------------------------------------
    // Transport policy and lifecycle
    // ----------------------------------------------------------------------

    /// Fail-fast mode (`tolerance = 0`) is a policy for broken links, not a
    /// handicap: on a healthy line the stream must keep flowing unchanged.
    #[test]
    #[timeout(30)]
    async fn fail_fast_tolerance_still_streams_on_a_healthy_line(ctx: Ctx) {
        let mut gps = ctx.gps;
        gps.set_read_error_tolerance(0);
        // Codec-agnostic: without `builtin-codec` nothing is ever decoded into
        // `Event::Nmea`, so counting only that variant would report a healthy
        // line as silent.
        gps.set_skip_unknown_sentences(false);
        let mut decoded = 0;
        observe(&mut gps, Duration::from_secs(5), |ev| {
            if matches!(ev, Event::Nmea(_) | Event::NmeaOther { .. }) {
                decoded += 1;
            }
        })
        .await;
        assert!(
            decoded >= 3,
            "only {} sentences in 5s with tolerance=0 on a healthy line",
            decoded
        );
    }

    /// `free` hands the UART back intact: a second driver built on the
    /// returned stream picks up where the first left off.
    #[cfg(feature = "builtin-codec")]
    #[test]
    #[timeout(30)]
    async fn free_returns_a_working_uart(ctx: Ctx) {
        let mut gps = ctx.gps;
        assert!(
            pump_until(&mut gps, Duration::from_secs(5), |e| matches!(
                e,
                Event::Nmea(_)
            ))
            .await
            .is_some(),
            "no NMEA before free()"
        );

        let mut gps = NeoGps::new(gps.free());
        assert!(
            pump_until(&mut gps, Duration::from_secs(5), |e| matches!(
                e,
                Event::Nmea(_)
            ))
            .await
            .is_some(),
            "the UART returned by free() no longer streams"
        );
    }

    /// `save_config` (CFG-CFG) persists settings to battery-backed RAM and
    /// flash — a real, lasting side effect on the module, so it only runs when
    /// the build explicitly opted in with `NEO_ALLOW_SAVE`.
    #[cfg(feature = "builtin-codec")]
    #[test]
    #[timeout(30)]
    async fn save_config_is_acked(ctx: Ctx) {
        if !config::ALLOW_SAVE {
            log::warn!("skip: rebuild with NEO_ALLOW_SAVE=1 to exercise save_config");
            return;
        }
        let mut gps = ctx.gps;

        // CFG-CFG freezes whatever is in the module's RAM *right now* and makes
        // it the power-on default. embedded-test resets the ESP32 between test
        // cases but the module keeps its RAM across the whole run, so without
        // this the test would persist whichever half-configured state an
        // earlier test happened to leave behind. Write a deliberate one.
        for id in [NMEA_GGA, NMEA_GLL, NMEA_GSA, NMEA_GSV, NMEA_RMC, NMEA_VTG] {
            require_sentence(&mut gps, id).await;
        }
        maybe_await!(gps.set_nav_rate_ms(1000)).expect("1 Hz should ACK");

        // Confirm that configuration is live before committing it: saving a
        // module that has stopped talking would be worse than not saving.
        assert!(
            pump_until(&mut gps, Duration::from_secs(10), |e| matches!(
                e,
                Event::Nmea(_)
            ))
            .await
            .is_some(),
            "module went quiet after the restore, refusing to persist that"
        );

        maybe_await!(gps.save_config()).expect("CFG-CFG save should ACK");
        log::info!("persisted: default NMEA sentence set at 1 Hz");
    }

    // ----------------------------------------------------------------------
    // Wider CFG coverage: platform model, constellations, power, reset, baud
    //
    // The destructive ones are behind their own build-time gates, because
    // the module keeps its configuration for the whole run and two of these
    // can outlive it: `NEO_ALLOW_RESET` for the reset pair, `NEO_ALLOW_BAUD`
    // for the baud round trip.
    // ----------------------------------------------------------------------

    /// `CFG-NAV5` is ACKed and the module keeps solving afterwards.
    ///
    /// The second half matters more than the ACK: a dynamic model the module
    /// dislikes can leave it ACKing happily while its filters stop producing
    /// fixes, so the test insists the stream is still alive before putting the
    /// factory default back.
    #[test]
    #[timeout(60)]
    async fn dynamic_model_is_acked_and_the_stream_survives(ctx: Ctx) {
        let mut gps = ctx.gps;

        maybe_await!(gps.set_dynamic_model(ubx::DynamicModel::Airborne4G))
            .expect("CFG-NAV5 with dynModel=Airborne4G should ACK");
        assert!(
            streams_within(&mut gps, 10).await,
            "module stopped emitting NMEA after the dynamic model changed"
        );

        // Back to the factory default, so nothing downstream inherits an
        // airborne filter (and `save_config_is_acked` cannot persist one).
        maybe_await!(gps.set_dynamic_model(ubx::DynamicModel::Portable))
            .expect("restoring dynModel=Portable should ACK");
    }

    /// `NAV-VELNED` streams, and its ground speed matches `NAV-PVT`'s.
    ///
    /// This is the only binary velocity a 6-series module can produce, so it
    /// is worth proving on the wire rather than only against captured bytes.
    #[cfg(feature = "builtin-codec")]
    #[test]
    #[timeout(90)]
    async fn nav_velned_streams_and_agrees_with_nav_pvt(ctx: Ctx) {
        let mut gps = ctx.gps;
        let caps = probe(&mut gps).await;

        maybe_await!(gps.enable_nav_velned()).expect("enable NAV-VELNED should ACK");
        let ev = pump_until(&mut gps, Duration::from_secs(15), |e| {
            matches!(e, Event::NavVelned(_))
        })
        .await
        .expect("no NAV-VELNED frame after enabling");
        let Event::NavVelned(vel) = ev else {
            unreachable!()
        };
        assert_well_formed_ubx(gps.last_ubx_frame(), ubx::CLASS_NAV, ubx::NAV_VELNED);

        assert!(
            vel.itow_ms < 7 * 24 * 3600 * 1000,
            "iTOW outside a GPS week"
        );
        // 300 m/s is past any NEO use case and well short of i32 saturation,
        // so this catches a byte-offset mistake without being flaky.
        assert!(
            vel.gspeed_mm_s < 300_000,
            "implausible ground speed {} mm/s",
            vel.gspeed_mm_s
        );
        assert!(
            (0..=36_000_000).contains(&vel.heading_1e5),
            "heading {} outside 0..360 degrees",
            vel.heading_1e5
        );

        if caps.has_nav_pvt() {
            maybe_await!(gps.enable_nav_pvt()).expect("enable NAV-PVT should ACK");
            if let Some(Event::NavPvt(pvt)) = pump_until(
                &mut gps,
                Duration::from_secs(15),
                |e| matches!(e, Event::NavPvt(p) if p.gnss_fix_ok()),
            )
            .await
            {
                // Same solution seconds apart, so they agree closely; 1 m/s of
                // slack covers the gap between the two epochs while still
                // catching a decoder reading the wrong field.
                let delta = (pvt.gspeed_mm_s - vel.gspeed_mm_s as i32).abs();
                assert!(
                    delta < 1_000,
                    "NAV-PVT and NAV-VELNED ground speeds differ by {} mm/s",
                    delta
                );
                log::info!("NAV-VELNED and NAV-PVT agree on ground speed");
            } else {
                log::warn!("no fixed NAV-PVT to cross-check velocity against");
            }
            maybe_await!(gps.set_msg_rate(ubx::CLASS_NAV, ubx::NAV_PVT, 0)).ok();
        }
        maybe_await!(gps.set_msg_rate(ubx::CLASS_NAV, ubx::NAV_VELNED, 0)).ok();
    }

    /// `set_constellation` does the right thing for the module in front of it.
    ///
    /// Below protocol 15 there is no `CFG-GNSS`, and the driver must say so
    /// locally rather than send a frame the module would NAK. At 15 and above
    /// it polls, flips one enable bit and writes back, which this checks by
    /// toggling GLONASS off and on again.
    #[test]
    #[timeout(60)]
    async fn set_constellation_follows_module_capability(ctx: Ctx) {
        let mut gps = ctx.gps;
        let caps = probe(&mut gps).await;

        if !caps.has_cfg_gnss() {
            assert!(
                matches!(
                    maybe_await!(gps.set_constellation(ubx::Constellation::Glonass, true)),
                    Err(Error::Unsupported)
                ),
                "protver {} has no CFG-GNSS, so this must be refused locally",
                caps.protocol_version
            );
            log::info!(
                "skip the live half: protver {} predates CFG-GNSS",
                caps.protocol_version
            );
            return;
        }

        // Off then on, so the module ends the test as it started it.
        maybe_await!(gps.set_constellation(ubx::Constellation::Glonass, false))
            .expect("disabling GLONASS should ACK");
        assert!(
            streams_within(&mut gps, 10).await,
            "module went quiet after a constellation change"
        );
        maybe_await!(gps.set_constellation(ubx::Constellation::Glonass, true))
            .expect("re-enabling GLONASS should ACK");
        // Toggling a constellation restarts the module's GNSS engine, which
        // leaves it briefly deaf to configuration.
        settle(&mut gps).await;
    }

    /// `CFG-RXM` is answered, and continuous tracking is restored.
    ///
    /// Either answer proves the driver: power save mode has preconditions
    /// (1 Hz, single constellation on some firmware) and a module entitled to
    /// refuse it sends `ACK-NAK`, which is a correlated reply just the same.
    /// What would be a failure is silence.
    #[test]
    #[timeout(60)]
    async fn power_save_is_answered_and_continuous_is_restored(ctx: Ctx) {
        let mut gps = ctx.gps;

        match maybe_await!(gps.set_power_save(true)) {
            Ok(()) => {
                log::info!("module accepted power save mode");
                maybe_await!(gps.set_power_save(false))
                    .expect("returning to continuous tracking should ACK");
            }
            Err(Error::Nak { .. }) => {
                log::info!("module refused power save mode with NAK, which is a valid answer")
            }
            Err(e) => panic!("CFG-RXM got no usable reply: {:?}", e),
        }

        assert!(
            streams_within(&mut gps, 10).await,
            "module is not streaming after the power mode round trip"
        );
        settle(&mut gps).await;
    }

    /// `factory_reset` really does reload the defaults.
    ///
    /// Checked by observing the configuration change rather than the ACK:
    /// GSV is muted first, and it has to come back on its own, because the
    /// u-blox factory sentence set includes it.
    #[test]
    #[timeout(90)]
    async fn factory_reset_restores_the_default_sentence_set(ctx: Ctx) {
        if !config::ALLOW_RESET {
            log::warn!("skip: rebuild with NEO_ALLOW_RESET=1 to exercise factory_reset");
            return;
        }
        let mut gps = ctx.gps;
        gps.set_skip_unknown_sentences(false); // GSV arrives as NmeaOther

        require_sentence(&mut gps, NMEA_GSV).await;
        assert!(
            pump_until(&mut gps, Duration::from_secs(10), is_gsv)
                .await
                .is_some(),
            "GSV never appeared, so muting it would prove nothing"
        );
        maybe_await!(gps.disable_nmea(NMEA_GSV)).expect("muting GSV should ACK");
        assert!(
            pump_until(&mut gps, Duration::from_secs(5), is_gsv)
                .await
                .is_none(),
            "GSV still arriving after being muted"
        );

        // No ACK to wait for: clearing the port configuration reinitialises
        // the UART. The proof is behavioural, on the next assertion.
        maybe_await!(gps.factory_reset()).expect("CFG-CFG should be written");

        assert!(
            pump_until(&mut gps, Duration::from_secs(20), is_gsv)
                .await
                .is_some(),
            "GSV did not come back, so the factory defaults were not loaded"
        );
        log::info!("factory defaults reloaded: the muted sentence returned");
        settle(&mut gps).await;
    }

    /// A hot reset is accepted and the module comes back talking.
    ///
    /// `CFG-RST` is never acknowledged, so the only evidence available is
    /// behavioural: the stream has to return. `Hot` keeps the ephemeris and
    /// almanac, so this costs a second or two rather than a fresh sky search.
    #[test]
    #[timeout(90)]
    async fn hot_reset_is_accepted_and_the_stream_returns(ctx: Ctx) {
        if !config::ALLOW_RESET {
            log::warn!("skip: rebuild with NEO_ALLOW_RESET=1 to exercise reset");
            return;
        }
        let mut gps = ctx.gps;

        assert!(
            streams_within(&mut gps, 10).await,
            "module was not streaming before the reset"
        );

        maybe_await!(gps.reset(ubx::ResetKind::Hot)).expect("CFG-RST should be written");

        // A restarting module emits partial frames as it comes up, so the
        // deframer will see junk before it sees a sentence. That is exactly
        // the resync path, and it must not need a driver restart.
        assert!(
            streams_within(&mut gps, 30).await,
            "no NMEA within 30s of a hot reset"
        );
        log::info!("module restarted and resumed streaming");
        settle(&mut gps).await;
    }

    /// The baud rate changes on both ends and comes back.
    ///
    /// The only test here that reconfigures the ESP32's own UART, because
    /// `set_baud` moves the module and leaves this side behind by design. It
    /// proves the round trip: stream at the configured rate, move both ends
    /// up, stream again, move both back.
    ///
    /// If this test dies in the middle, the module is left at the faster rate
    /// and everything afterwards fails. `CFG-PRT` is RAM-only unless saved, so
    /// power cycling the module puts it back at 9600.
    #[test]
    #[timeout(120)]
    async fn baud_change_round_trip(ctx: Ctx) {
        if !config::ALLOW_BAUD {
            log::warn!("skip: rebuild with NEO_ALLOW_BAUD=1 to exercise set_baud");
            return;
        }
        /// Comfortably above the 9600 default and slow enough to stay reliable
        /// on breadboard jumpers.
        const FAST: u32 = 38_400;

        let mut gps = ctx.gps;
        assert!(
            streams_within(&mut gps, 10).await,
            "module was not streaming at {} baud to begin with",
            config::BAUD
        );

        maybe_await!(gps.set_baud(FAST)).expect("CFG-PRT should be written");
        // Let the frame finish leaving the FIFO and the module retune before
        // this side moves; there is no ACK to synchronise on.
        Timer::after(Duration::from_millis(200)).await;
        let mut gps = board::rebuild_at_baud(gps, FAST);

        let fast_ok = streams_within(&mut gps, 15).await;

        // Put both ends back before asserting, so a failure at the faster rate
        // still leaves the module where the rest of the suite expects it.
        maybe_await!(gps.set_baud(config::BAUD)).expect("CFG-PRT back to the default baud");
        Timer::after(Duration::from_millis(200)).await;
        let mut gps = board::rebuild_at_baud(gps, config::BAUD);

        assert!(fast_ok, "nothing decoded at {} baud after the change", FAST);
        assert!(
            streams_within(&mut gps, 15).await,
            "module did not come back at {} baud",
            config::BAUD
        );
        log::info!(
            "baud round trip {} -> {} -> {} ok",
            config::BAUD,
            FAST,
            config::BAUD
        );
        settle(&mut gps).await;
    }

    /// Per-satellite data streams and decodes, on whichever of `NAV-SAT` and
    /// `NAV-SVINFO` the module has.
    ///
    /// Neither message is decoded by the built-in codec, so both arrive as
    /// `UbxOther` and this test is meaningful in either build.
    #[test]
    #[timeout(60)]
    async fn satellite_info_streams_and_decodes(ctx: Ctx) {
        let mut gps = ctx.gps;
        let caps = probe(&mut gps).await;
        gps.set_skip_unknown_sentences(false);

        maybe_await!(gps.enable_satellite_info()).expect("per-satellite output should ACK");
        let want_id = if caps.has_nav_sat() {
            ubx::NAV_SAT
        } else {
            ubx::NAV_SVINFO
        };
        let found = pump_until(&mut gps, Duration::from_secs(15), |e| {
            matches!(e, Event::UbxOther { class, id }
                if *class == ubx::CLASS_NAV && *id == want_id)
        })
        .await;
        assert!(found.is_some(), "no per-satellite frame after enabling");
        assert_well_formed_ubx(gps.last_ubx_frame(), ubx::CLASS_NAV, want_id);

        let sats = gps
            .satellites()
            .expect("satellites() must decode the frame it just handed out");
        let (mut total, mut used, mut best_cno) = (0u32, 0u32, 0u8);
        for s in sats {
            total += 1;
            if s.used_in_fix {
                used += 1;
            }
            best_cno = best_cno.max(s.cno_dbhz);
            assert!(s.cno_dbhz <= 64, "implausible C/N0 {} dB-Hz", s.cno_dbhz);
            // Only the tracked entries carry meaningful geometry. `NAV-SVINFO`
            // reports one block per receiver *channel*, and an idle channel's
            // elevation and azimuth are whatever was last left there: this
            // module emits -91 degrees for them.
            if s.cno_dbhz > 0 {
                assert!(
                    (-90..=90).contains(&s.elev_deg),
                    "tracked satellite elevation {} outside -90..90",
                    s.elev_deg
                );
                assert!(
                    (0..=360).contains(&s.azim_deg),
                    "tracked satellite azimuth {} outside 0..360",
                    s.azim_deg
                );
            }
        }
        assert!(used <= total, "more satellites used than reported");
        if total == 0 {
            log::warn!("module reported zero satellites, so only the framing was checked");
        } else {
            log::info!(
                "{} satellites reported, {} used in the fix, best C/N0 {} dB-Hz",
                total,
                used,
                best_cno
            );
        }

        maybe_await!(gps.set_msg_rate(ubx::CLASS_NAV, want_id, 0)).ok();
    }

    /// With `tolerance = 0`, a genuinely broken link surfaces as
    /// [`Error::Io`] rather than being retried forever.
    ///
    /// The other tolerance test proves the healthy-line case; this one proves
    /// the half that actually needs a broken line. The break is made by
    /// retuning *this* side of the UART while the module keeps transmitting at
    /// its own rate, so the bytes arrive mid-bit and the receiver reports
    /// framing violations. Both ends are put back before the test ends.
    #[test]
    #[timeout(90)]
    async fn read_errors_surface_when_tolerance_is_zero(ctx: Ctx) {
        // Three times the module's rate: every real bit is sampled about three
        // times, so frames cannot align and the UART raises framing errors
        // instead of quietly delivering plausible bytes.
        let mut gps = board::rebuild_at_baud(ctx.gps, config::BAUD * 3);
        gps.set_read_error_tolerance(0);
        gps.set_skip_unknown_sentences(false);

        let deadline = Instant::now() + Duration::from_secs(20);
        let mut saw_io = false;
        while let Some(result) = next_event_before(&mut gps, deadline).await {
            if let Err(Error::Io(e)) = result {
                log::info!("fail-fast surfaced the broken link: {:?}", e);
                saw_io = true;
                break;
            }
        }

        // Put the link back before asserting, so a failure here does not leave
        // the rest of the suite talking at the wrong rate.
        let mut gps = board::rebuild_at_baud(gps, config::BAUD);

        assert!(
            saw_io,
            "a mistuned UART produced no Error::Io in 20s with tolerance=0"
        );
        assert!(
            streams_within(&mut gps, 15).await,
            "the link did not recover once the baud rate was put back"
        );
        settle(&mut gps).await;
    }

    // ----------------------------------------------------------------------
    // Local helpers
    // ----------------------------------------------------------------------

    /// Hand one complete UBX frame to the `ublox` crate through the driver's
    /// adapter and pull `want`'s fields out of the typed packet.
    ///
    /// The parser lives on the stack: `Parser::default()` needs `alloc`, so a
    /// `FixedLinearBuffer` is what a `no_std` user has to reach for. 256 bytes
    /// covers every NAV message the driver enables.
    ///
    /// Both failure modes are `log::error!`d *before* the panic, not only
    /// inside its message: the panic banner does not always survive the RTT
    /// channel, and the class/id plus payload length is the whole diagnosis.
    /// Nothing else about the packet is logged — a mis-decode could otherwise
    /// put a position in the message.
    #[cfg(feature = "codec-ublox")]
    fn decode_with_ublox_crate<R>(
        frame: &[u8],
        name: &str,
        want: impl FnOnce(ublox::PacketRef<'_>) -> Option<R>,
    ) -> R {
        let payload_len = frame.len() - 8;
        let mut storage = [0u8; 256];
        let mut parser = ublox::Parser::new(ublox::FixedLinearBuffer::new(&mut storage));

        let decoded = neo_gps::codec::ublox::decode(&mut parser, frame, |pkt| {
            let class_and_id = pkt.class_and_msg_id();
            want(pkt).ok_or(class_and_id)
        });

        match decoded {
            Some(Ok(fields)) => {
                log::info!(
                    "external `ublox` crate decoded a {}-byte {} payload",
                    payload_len,
                    name
                );
                fields
            }
            Some(Err((class, id))) => {
                log::error!(
                    "external `ublox` crate read our {}-byte {} payload as class 0x{:02X} \
                     id 0x{:02X} — its declared layout does not match this module's",
                    payload_len,
                    name,
                    class,
                    id
                );
                panic!(
                    "the `ublox` crate did not return the expected {} packet",
                    name
                );
            }
            None => {
                log::error!(
                    "external `ublox` crate found no packet at all in a {}-byte {} payload \
                     our deframer checksum-verified",
                    payload_len,
                    name
                );
                panic!(
                    "the `ublox` crate found no packet in a valid {} frame",
                    name
                );
            }
        }
    }

    /// Wait until the module answers a configuration command again.
    ///
    /// A test that restarts, retunes or power-cycles the module must not
    /// finish on "bytes are flowing" alone. The next test's first act is
    /// usually a `CFG` frame, and a module still settling will drop it,
    /// failing an unrelated test with `NoReply` several tests later. Proving
    /// responsiveness here keeps the blame on the test that caused it.
    ///
    /// Re-enabling GGA is the harmless probe: it is on in the factory
    /// configuration anyway, so this restores rather than disturbs.
    async fn settle(gps: &mut Gps) {
        for attempt in 1..=5 {
            if maybe_await!(gps.set_msg_rate(0xF0, NMEA_GGA, 1)).is_ok() {
                return;
            }
            log::warn!("module has not answered yet (attempt {}), waiting", attempt);
            Timer::after(Duration::from_secs(2)).await;
        }
        panic!("module never answered a CFG frame again after this test disturbed it");
    }

    /// Whether any NMEA sentence arrives within `secs`. The cheapest possible
    /// "is the link still alive" check, used by the tests that reconfigure the
    /// module and need to know they have not silenced it.
    ///
    /// Deliberately codec-agnostic: without `builtin-codec` every sentence
    /// arrives as [`Event::NmeaOther`], so both are accepted and the skip
    /// filter is turned off to let the second kind through. That keeps the
    /// eight tests using this helper meaningful in either configuration
    /// instead of compiling them out.
    async fn streams_within(gps: &mut Gps, secs: u64) -> bool {
        gps.set_skip_unknown_sentences(false);
        pump_until(gps, Duration::from_secs(secs), |e| {
            matches!(e, Event::Nmea(_) | Event::NmeaOther { .. })
        })
        .await
        .is_some()
    }

    fn is_gsv(ev: &Event) -> bool {
        matches!(ev, Event::NmeaOther { mtype, .. } if mtype == b"GSV")
    }

    #[cfg(feature = "builtin-codec")]
    fn is_rmc(ev: &Event) -> bool {
        matches!(ev, Event::Nmea(Sentence::Rmc(_)))
    }

    /// Mean milliseconds between consecutive RMC sentences, over `samples`
    /// intervals. The first sentence is discarded: the driver may have joined
    /// the stream part-way through it, which would bias the first interval.
    #[cfg(feature = "builtin-codec")]
    async fn mean_rmc_interval_ms(gps: &mut Gps, samples: u64) -> u64 {
        pump_until(gps, Duration::from_secs(10), is_rmc)
            .await
            .expect("no RMC to time against");
        let start = Instant::now();
        for _ in 0..samples {
            pump_until(gps, Duration::from_secs(10), is_rmc)
                .await
                .expect("RMC stream stopped mid-measurement");
        }
        (Instant::now() - start).as_millis() / samples
    }
}

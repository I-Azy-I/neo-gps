//! Transport-resilience tests: UART read errors (framing violations, noise,
//! the power-up floating-line glitch) must be absorbed like garbage bytes,
//! not propagated as link failure. Compiled for every feature combination —
//! these exercise the ungated transport layer only.

use crate::testutil::*;
use crate::ubx;
use crate::*;

/// The field regression: the line glitches during power-up (real HALs report
/// FrameFormatViolated/GlitchOccurred on nearly every cold boot), then the
/// module starts talking. probe() must ride through it.
#[test]
fn probe_survives_startup_glitch_burst() {
    let rx = ubx_wire(
        ubx::CLASS_MON,
        ubx::MON_VER,
        &mon_ver_payload("ROM CORE 3.01 (107888)", "00080000", &["PROTVER=18.00"]),
    );
    let mut uart = MockUart::new(rx);
    uart.fail_next_reads = 3; // typical cold-boot burst
    let mut gps = NeoGps::new(uart);
    let caps = block_on(gps.probe()).expect("glitches at startup must not abort probe");
    assert_eq!(caps.protocol_version, 18);
}

/// A glitch arriving mid-frame must drop the partial frame (its bytes are
/// suspect) and resync: the interrupted frame never surfaces, the next
/// intact frame parses normally, and nothing hybrid is emitted.
#[test]
fn glitch_mid_frame_resyncs_without_corruption() {
    let good = ubx_wire(ubx::CLASS_ACK, 0x01, &[0x06, 0x08]);
    let mut rx = good[..5].to_vec(); // read #0: half a frame
    rx.extend(&good); //                reads #2..: an intact frame
    let mut uart = MockUart::new(rx);
    uart.chunk = 5;
    uart.fail_on_reads = vec![1]; // read #1: the glitch, mid-frame
    let mut gps = NeoGps::new(uart);
    let ev = block_on(gps.next_event()).unwrap();
    assert_eq!(
        ev,
        Event::Ack {
            class: 0x06,
            id: 0x08,
            ok: true
        }
    );
    // Exactly one frame came out: the partial one was discarded, not merged.
    assert_eq!(block_on(gps.next_event()), Err(Error::Eof));
}

/// More consecutive errors than the tolerance is a broken link: Io surfaces.
#[test]
fn broken_uart_surfaces_io_after_tolerance() {
    let mut uart = MockUart::new(Vec::new());
    uart.fail_next_reads = usize::MAX; // fails forever
    let mut gps = NeoGps::new(uart);
    assert_eq!(block_on(gps.next_event()), Err(Error::Io(Never)));
}

/// The streak is *consecutive*: successful reads reset it, so a line that
/// glitches now and then never trips the threshold.
#[test]
fn intermittent_glitches_never_trip_threshold() {
    let mut rx = Vec::new();
    for _ in 0..4 {
        rx.extend(ubx_wire(ubx::CLASS_ACK, 0x01, &[0x06, 0x01]));
    }
    let mut uart = MockUart::new(rx);
    uart.chunk = 10; // one whole frame per read: glitches land between frames
    let mut gps = NeoGps::new(uart);
    // Before each frame, a burst at the tolerance: successful reads in
    // between reset the streak, so this must never surface Io. (Bursts are
    // aligned to frame boundaries — the realistic power-up pattern, where
    // the parser is idle and the glitch damages no frame in progress.)
    for _ in 0..4 {
        gps.uart_mut().fail_next_reads = DEFAULT_READ_ERROR_TOLERANCE as usize;
        let ev = block_on(gps.next_event()).unwrap();
        assert_eq!(
            ev,
            Event::Ack {
                class: 0x06,
                id: 0x01,
                ok: true
            }
        );
    }
}

/// Why the reset matters: a glitch that damages a byte inside a UBX header
/// could otherwise fabricate a huge length field, making the parser swallow
/// tens of seconds of good frames. Resetting on the error bounds the damage
/// to the one frame the glitch actually hit.
#[test]
fn reset_prevents_bogus_length_trap() {
    let good = ubx_wire(ubx::CLASS_ACK, 0x01, &[0x06, 0x08]);
    // Read #0 delivers a frame fragment ending inside the header (sync +
    // class + id + len_lo) — the glitch then hits where len_hi would be.
    let mut rx = good[..5].to_vec();
    rx.extend(&good);
    let mut uart = MockUart::new(rx);
    uart.chunk = 5;
    uart.fail_on_reads = vec![1];
    let mut gps = NeoGps::new(uart);
    // With the reset, the intact frame that follows parses; without it, the
    // spliced header would read a bogus length and swallow it.
    assert_eq!(
        block_on(gps.next_event()),
        Ok(Event::Ack {
            class: 0x06,
            id: 0x08,
            ok: true
        })
    );
}

/// Tolerance 0 restores the old fail-fast contract.
#[test]
fn tolerance_zero_is_fail_fast() {
    let mut uart = MockUart::new(ubx_wire(ubx::CLASS_ACK, 0x01, &[0x06, 0x08]));
    uart.fail_next_reads = 1;
    let mut gps = NeoGps::new(uart);
    gps.set_read_error_tolerance(0);
    assert_eq!(block_on(gps.next_event()), Err(Error::Io(Never)));
    // The error was surfaced, but the stream is not poisoned: retrying works.
    assert!(matches!(block_on(gps.next_event()), Ok(Event::Ack { .. })));
}

/// Ok(0) (closed port on adapted transports) must stay immediately fatal —
/// retrying an EOF spins forever.
#[test]
fn eof_is_not_retried() {
    let mut gps = NeoGps::new(MockUart::new(Vec::new()));
    assert_eq!(block_on(gps.next_event()), Err(Error::Eof));
}

/// probe() re-polls at half budget: if the first outbound poll is lost
/// (module never replies to it), the retry still gets an answer within the
/// same budget instead of a guaranteed Timeout.
#[test]
fn probe_repolls_when_first_poll_lost() {
    // Script the module as if it never saw poll #1: it emits chatter frames
    // only, then answers — but only after enough frames have passed that the
    // driver must have re-polled (budget/2 chatter frames first).
    let mut rx = Vec::new();
    for _ in 0..(64 / 2) {
        rx.extend(nmea_wire("GNGSA,A,3,80,71,,,,,,,,,,,1.83,1.09,1.47"));
    }
    rx.extend(ubx_wire(
        ubx::CLASS_MON,
        ubx::MON_VER,
        &mon_ver_payload("", "", &["PROTVER=18.00"]),
    ));
    let mut gps = NeoGps::new(MockUart::new(rx));
    let caps = block_on(gps.probe()).unwrap();
    assert_eq!(caps.protocol_version, 18);
    // Two MON-VER polls must be on the wire.
    let polls = gps
        .uart_ref()
        .tx
        .windows(4)
        .filter(|w| w == &[0xB5, 0x62, 0x0A, 0x04])
        .count();
    assert_eq!(polls, 2, "expected the half-budget re-poll");
}

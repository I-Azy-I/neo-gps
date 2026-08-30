//! Event-pump helpers shared by the test bodies.
//!
//! All of them are bounded by an `embassy_time` deadline rather than a frame
//! count, so "did X appear within 3 seconds" means the same thing on a 1 Hz
//! NEO-6M and a 10 Hz NEO-M9N.

use embassy_time::{Duration, Instant};
use neo_gps::nmea::Sentence;
use neo_gps::{Capabilities, Error, Event};

use crate::board::{Gps, GpsError};
use crate::maybe_await;

/// One event, or `None` if `deadline` passed first.
///
/// `next_event` blocks until a frame *completes*, so on a silent module it
/// never returns and a plain `while Instant::now() < deadline` loop would
/// never get to re-check its own deadline — the test would hang until the
/// harness `#[timeout(..)]` killed it, reporting "timed out" instead of the
/// assertion that actually failed. Bounding each individual wait is what makes
/// those messages legible.
///
/// Cancelling `next_event` mid-await drops at most one in-flight UART read.
/// That is only reachable on the path where the test is about to fail anyway,
/// so the lost bytes cost nothing.
#[cfg(feature = "async")]
pub async fn next_event_before(
    gps: &mut Gps,
    deadline: Instant,
) -> Option<Result<Event, GpsError>> {
    let remaining = deadline.checked_duration_since(Instant::now())?;
    embassy_time::with_timeout(remaining, gps.next_event())
        .await
        .ok()
}

/// Blocking counterpart. `embedded_io::Read` has no timeout and the driver owns
/// the UART, so there is nothing to bound the wait with here: a silent module
/// still surfaces as the per-test `#[timeout(..)]`. Run
/// `uart_receives_framed_bytes` to tell that apart from a driver fault.
#[cfg(feature = "sync")]
pub async fn next_event_before(
    gps: &mut Gps,
    deadline: Instant,
) -> Option<Result<Event, GpsError>> {
    if Instant::now() >= deadline {
        return None;
    }
    Some(gps.next_event())
}

/// Pump events until `pred` matches or `timeout` elapses.
///
/// `Error::Io` is logged and shrugged off: the driver only surfaces it after
/// more consecutive failed reads than its tolerance, and a breadboarded module
/// on a long jumper does occasionally produce a burst of framing errors. Any
/// other error is a driver-level fault and fails the test immediately.
pub async fn pump_until(
    gps: &mut Gps,
    timeout: Duration,
    pred: impl Fn(&Event) -> bool,
) -> Option<Event> {
    let deadline = Instant::now() + timeout;
    while let Some(result) = next_event_before(gps, deadline).await {
        match result {
            Ok(ev) if pred(&ev) => return Some(ev),
            Ok(_) => {}
            Err(Error::Io(e)) => log::warn!("tolerated UART error while pumping: {:?}", e),
            Err(e) => panic!("unexpected driver error while pumping: {:?}", e),
        }
    }
    None
}

/// Pump every event for the whole `window`, handing each to `f`.
///
/// Used for the "which sentences does the module actually emit" style of
/// check, where a single matching event is not enough.
pub async fn observe(gps: &mut Gps, window: Duration, mut f: impl FnMut(&Event)) {
    let deadline = Instant::now() + window;
    while let Some(result) = next_event_before(gps, deadline).await {
        match result {
            Ok(ev) => f(&ev),
            Err(Error::Io(e)) => log::warn!("tolerated UART error while observing: {:?}", e),
            Err(e) => panic!("unexpected driver error while observing: {:?}", e),
        }
    }
}

/// Probe the module, failing the test with a useful message if it stays quiet.
pub async fn probe(gps: &mut Gps) -> Capabilities {
    let caps = maybe_await!(gps.probe())
        .expect("probe: no MON-VER reply (is TX wired to the module's RX, and the baud right?)");
    log::info!(
        "probed {:?}, protocol version {}",
        caps.generation,
        caps.protocol_version
    );
    caps
}

/// Bounded wait for evidence of a real fix: a valid RMC, a GGA with a fix
/// quality, or a NAV-PVT with `gnssFixOK`.
///
/// Returns `false` on timeout rather than failing — indoors there may simply
/// be no sky view, and a driver test must not depend on the weather.
pub async fn wait_for_fix(gps: &mut Gps, timeout: Duration) -> bool {
    pump_until(gps, timeout, |ev| match ev {
        Event::Nmea(Sentence::Rmc(r)) => r.valid && r.lat_1e7.is_some() && r.lon_1e7.is_some(),
        Event::Nmea(Sentence::Gga(g)) => {
            g.quality.has_fix() && g.lat_1e7.is_some() && g.lon_1e7.is_some()
        }
        Event::NavPvt(p) => p.gnss_fix_ok(),
        _ => false,
    })
    .await
    .is_some()
}

/// Range-check a coordinate **without ever revealing it**.
///
/// No test in this suite prints or asserts a literal position: the checks only
/// establish that the value is inside the valid global range and is not the
/// null-island `0, 0` artifact modules emit before a fix.
pub fn assert_plausible(lat_1e7: i32, lon_1e7: i32) {
    assert!(
        (-900_000_000..=900_000_000).contains(&lat_1e7),
        "latitude outside [-90, 90] degrees"
    );
    assert!(
        (-1_800_000_000..=1_800_000_000).contains(&lon_1e7),
        "longitude outside [-180, 180] degrees"
    );
    assert!(
        !(lat_1e7 == 0 && lon_1e7 == 0),
        "0,0 is the null-island no-fix artifact, not a real fix"
    );
}

/// Verify a complete UBX frame the way an external decoder would: sync bytes,
/// a length field that matches the frame, and a correct Fletcher-8 checksum.
pub fn assert_well_formed_ubx(frame: &[u8], class: u8, id: u8) {
    assert!(frame.len() >= 8, "UBX frame shorter than its own header");
    assert_eq!(&frame[..2], &[0xB5, 0x62], "missing UBX sync bytes");
    assert_eq!(frame[2], class, "unexpected UBX class");
    assert_eq!(frame[3], id, "unexpected UBX id");

    let len = u16::from_le_bytes([frame[4], frame[5]]) as usize;
    assert_eq!(
        frame.len(),
        8 + len,
        "UBX length field disagrees with the frame it came in"
    );

    let (mut ck_a, mut ck_b) = (0u8, 0u8);
    for &b in &frame[2..6 + len] {
        ck_a = ck_a.wrapping_add(b);
        ck_b = ck_b.wrapping_add(ck_a);
    }
    assert_eq!(
        (ck_a, ck_b),
        (frame[6 + len], frame[7 + len]),
        "UBX checksum does not cover the frame the accessor handed out"
    );
}

/// Verify an NMEA line the way an external decoder would: a `$`-less body that
/// starts with the talker + type the event reported, ending in `*hh`.
pub fn assert_well_formed_nmea(line: &[u8], talker: [u8; 2], mtype: [u8; 3]) {
    assert!(line.len() > 8, "NMEA line too short to be a sentence");
    assert_eq!(&line[..2], &talker[..], "line's talker != the event's");
    assert_eq!(&line[2..5], &mtype[..], "line's type != the event's");

    let star = line
        .iter()
        .rposition(|&b| b == b'*')
        .expect("NMEA line has no checksum delimiter");
    assert_eq!(
        line.len() - star,
        3,
        "checksum is not exactly two hex digits"
    );

    let mut ck = 0u8;
    for &b in &line[..star] {
        ck ^= b;
    }
    let hex = |n: u8| b"0123456789ABCDEF"[n as usize];
    assert_eq!(
        [line[star + 1], line[star + 2]].map(|b| b.to_ascii_uppercase()),
        [hex(ck >> 4), hex(ck & 0x0F)],
        "NMEA checksum does not cover the line the accessor handed out"
    );
}

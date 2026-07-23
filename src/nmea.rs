//! Talker-agnostic NMEA 0183 parsing (versions 2.3 and 4.x).
//!
//! The talker ID (`GP` on a GPS-only NEO-6M, `GN`/`GL`/`GA`/`GB` on
//! multi-GNSS 8-series and later) is captured but never used for dispatch:
//! sentences are identified by their 3-letter type only. Field counts are not
//! assumed, so NMEA 2.3 (NEO-6 default) and 4.0+ (NEO-8 default, which adds
//! trailing fields to some sentences) both parse.
//!
//! All values are integers:
//! * latitude/longitude: 1e-7 degrees (`i32`, the UBX convention)
//! * altitude / geoid separation: millimetres
//! * speed over ground: mm/s
//! * course over ground: 1e-5 degrees
//! * HDOP/PDOP/VDOP: 1e-2 (i.e. `123` = 1.23)

use crate::Event;

/// GNSS fix quality from GGA field 6.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FixQuality {
    NoFix,
    Gps,
    Dgps,
    Pps,
    RtkFixed,
    RtkFloat,
    Estimated,
    Manual,
    Simulation,
    Other(u8),
}

impl FixQuality {
    fn from_u8(v: u8) -> Self {
        match v {
            0 => FixQuality::NoFix,
            1 => FixQuality::Gps,
            2 => FixQuality::Dgps,
            3 => FixQuality::Pps,
            4 => FixQuality::RtkFixed,
            5 => FixQuality::RtkFloat,
            6 => FixQuality::Estimated,
            7 => FixQuality::Manual,
            8 => FixQuality::Simulation,
            n => FixQuality::Other(n),
        }
    }

    ///
    /// # Example
    /// ```
    /// use neo_gps::nmea::FixQuality;
    /// assert!(FixQuality::Dgps.has_fix());
    /// assert!(!FixQuality::NoFix.has_fix());
    /// ```
    pub fn has_fix(&self) -> bool {
        !matches!(self, FixQuality::NoFix)
    }
}

/// UTC time of day.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct Time {
    pub hour: u8,
    pub minute: u8,
    pub second: u8,
    pub millis: u16,
}

/// UTC date.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct Date {
    pub day: u8,
    pub month: u8,
    /// Two-digit year as transmitted (add 2000 for this hardware's lifetime).
    pub year: u8,
}

/// `xxGGA` — fix data.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Gga {
    pub time: Option<Time>,
    /// Degrees * 1e7, positive north.
    pub lat_1e7: Option<i32>,
    /// Degrees * 1e7, positive east.
    pub lon_1e7: Option<i32>,
    pub quality: FixQuality,
    pub sats_in_use: u8,
    /// Horizontal dilution of precision * 100.
    pub hdop_1e2: Option<u16>,
    /// Altitude above mean sea level, millimetres.
    pub alt_msl_mm: Option<i32>,
    /// Geoid separation, millimetres.
    pub geoid_sep_mm: Option<i32>,
}

/// `xxRMC` — recommended minimum.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Rmc {
    pub time: Option<Time>,
    /// Status field 'A' = valid.
    pub valid: bool,
    pub lat_1e7: Option<i32>,
    pub lon_1e7: Option<i32>,
    /// Speed over ground, mm/s (converted from knots).
    pub speed_mm_s: Option<u32>,
    /// Course over ground, degrees * 1e5.
    pub course_1e5: Option<u32>,
    pub date: Option<Date>,
}

/// `xxGSA` — DOP and active satellites.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Gsa {
    /// 1 = no fix, 2 = 2D, 3 = 3D.
    pub fix_type: u8,
    pub pdop_1e2: Option<u16>,
    pub hdop_1e2: Option<u16>,
    pub vdop_1e2: Option<u16>,
}

/// Decoded NMEA sentences. `talker` is informational only.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Sentence {
    Gga(Gga),
    Rmc(Rmc),
    Gsa(Gsa),
}

/// Parse one NMEA line (the bytes between `$` and CR/LF, checksum included).
/// Returns `None` for empty input, bad checksums, or malformed headers.
pub(crate) fn parse_line(line: &[u8]) -> Option<Event> {
    if line.is_empty() {
        return None;
    }

    // Split off and verify "*hh" checksum. u-blox always sends it; we require it.
    let star = line.iter().rposition(|&b| b == b'*')?;
    let (body, ck) = (&line[..star], &line[star + 1..]);
    if ck.len() < 2 {
        return None;
    }
    let want = (hex_val(ck[0])? << 4) | hex_val(ck[1])?;
    let got = body.iter().fold(0u8, |a, &b| a ^ b);
    if want != got {
        return None;
    }

    // Header: 2-char talker + 3-char type (proprietary "P..." sentences are
    // reported as NmeaOther with a "P?" pseudo-talker).
    let comma = body.iter().position(|&b| b == b',').unwrap_or(body.len());
    let head = &body[..comma];
    if head.len() != 5 {
        // e.g. $PUBX,...
        let mut talker = [b'P', b'?'];
        let mut mtype = [b'?'; 3];
        for (d, s) in talker.iter_mut().zip(head.iter()) {
            *d = *s;
        }
        for (d, s) in mtype.iter_mut().zip(head.iter().skip(2)) {
            *d = *s;
        }
        return Some(Event::NmeaOther { talker, mtype });
    }
    let talker = [head[0], head[1]];
    let mtype = [head[2], head[3], head[4]];

    #[cfg(feature = "builtin-codec")]
    {
        let fields = Fields::new(&body[comma..]);
        let sentence = match &mtype {
            b"GGA" => parse_gga(fields).map(Sentence::Gga),
            b"RMC" => parse_rmc(fields).map(Sentence::Rmc),
            b"GSA" => parse_gsa(fields).map(Sentence::Gsa),
            _ => None,
        };
        if let Some(s) = sentence {
            return Some(Event::Nmea(s));
        }
    }
    Some(Event::NmeaOther { talker, mtype })
}

// ---------------------------------------------------------------------------
// Field iteration & scalar parsing (integer-only, tolerant of empty fields)
// ---------------------------------------------------------------------------

struct Fields<'a> {
    rest: &'a [u8],
}

impl<'a> Fields<'a> {
    /// `rest` starts at the comma *before* the first field.
    fn new(rest: &'a [u8]) -> Self {
        Fields { rest }
    }

    fn next(&mut self) -> Option<&'a [u8]> {
        if self.rest.is_empty() {
            return None;
        }
        debug_assert_eq!(self.rest[0], b',');
        let body = &self.rest[1..];
        match body.iter().position(|&b| b == b',') {
            Some(i) => {
                self.rest = &body[i..];
                Some(&body[..i])
            }
            None => {
                self.rest = &[];
                Some(body)
            }
        }
    }
}

fn hex_val(b: u8) -> Option<u8> {
    match b {
        b'0'..=b'9' => Some(b - b'0'),
        b'A'..=b'F' => Some(b - b'A' + 10),
        b'a'..=b'f' => Some(b - b'a' + 10),
        _ => None,
    }
}

/// Parse an unsigned decimal integer.
fn parse_uint(s: &[u8]) -> Option<u64> {
    if s.is_empty() {
        return None;
    }
    let mut v: u64 = 0;
    for &b in s {
        if !b.is_ascii_digit() {
            return None;
        }
        v = v.checked_mul(10)?.checked_add((b - b'0') as u64)?;
    }
    Some(v)
}

/// Parse `ddd.ddd` into an integer scaled by 10^`scale`, with sign support.
/// Extra fractional digits are truncated; missing ones are zero-padded.
fn parse_fixed(s: &[u8], scale: u32) -> Option<i64> {
    if s.is_empty() {
        return None;
    }
    let (neg, s) = match s[0] {
        b'-' => (true, &s[1..]),
        b'+' => (false, &s[1..]),
        _ => (false, s),
    };
    let dot = s.iter().position(|&b| b == b'.');
    let (int_part, frac_part) = match dot {
        Some(i) => (&s[..i], &s[i + 1..]),
        None => (s, &[][..]),
    };
    let mut v = if int_part.is_empty() {
        0
    } else {
        parse_uint(int_part)? as i64
    };
    for i in 0..scale {
        let digit = frac_part
            .get(i as usize)
            .map(|&b| {
                if b.is_ascii_digit() {
                    Some((b - b'0') as i64)
                } else {
                    None
                }
            })
            .unwrap_or(Some(0))?;
        v = v.checked_mul(10)?.checked_add(digit)?;
    }
    Some(if neg { -v } else { v })
}

/// `hhmmss.sss` → Time.
fn parse_time(s: &[u8]) -> Option<Time> {
    if s.len() < 6 {
        return None;
    }
    let h = parse_uint(&s[0..2])? as u8;
    let m = parse_uint(&s[2..4])? as u8;
    let sec = parse_uint(&s[4..6])? as u8;
    let millis = if s.len() > 7 && s[6] == b'.' {
        // pad/truncate fraction to 3 digits
        let f = &s[7..];
        let mut v: u16 = 0;
        for i in 0..3 {
            let d = f.get(i).copied().unwrap_or(b'0');
            if !d.is_ascii_digit() {
                return None;
            }
            v = v * 10 + (d - b'0') as u16;
        }
        v
    } else {
        0
    };
    Some(Time {
        hour: h,
        minute: m,
        second: sec,
        millis,
    })
}

/// `ddmmyy` → Date.
fn parse_date(s: &[u8]) -> Option<Date> {
    if s.len() != 6 {
        return None;
    }
    Some(Date {
        day: parse_uint(&s[0..2])? as u8,
        month: parse_uint(&s[2..4])? as u8,
        year: parse_uint(&s[4..6])? as u8,
    })
}

/// NMEA `(d)ddmm.mmmm(m)` + hemisphere → degrees * 1e7.
///
/// Integer math throughout: minutes are scaled to 1e7 then divided by 60
/// with rounding, using i64 intermediates (no overflow: < 2^43).
fn parse_coord(field: &[u8], hemi: &[u8], deg_digits: usize) -> Option<i32> {
    if field.len() < deg_digits + 2 {
        return None;
    }
    let deg = parse_uint(&field[..deg_digits])? as i64;
    let min_1e7 = parse_fixed(&field[deg_digits..], 7)?; // minutes * 1e7
    let frac_deg_1e7 = (min_1e7 + 30) / 60; // → degrees * 1e7, rounded
    let mut v = deg * 10_000_000 + frac_deg_1e7;
    match hemi.first() {
        Some(b'N') | Some(b'E') => {}
        Some(b'S') | Some(b'W') => v = -v,
        _ => return None,
    }
    i32::try_from(v).ok()
}

// ---------------------------------------------------------------------------
// Sentence bodies
// ---------------------------------------------------------------------------

fn parse_gga(mut f: Fields) -> Option<Gga> {
    let time = f.next().and_then(parse_time);
    let lat_f = f.next()?;
    let lat_h = f.next()?;
    let lon_f = f.next()?;
    let lon_h = f.next()?;
    let quality = FixQuality::from_u8(f.next().and_then(parse_uint).unwrap_or(0) as u8);
    let sats_in_use = f.next().and_then(parse_uint).unwrap_or(0) as u8;
    let hdop_1e2 = f
        .next()
        .and_then(|s| parse_fixed(s, 2))
        .and_then(|v| u16::try_from(v).ok());
    let alt_msl_mm = f.next().and_then(|s| parse_fixed(s, 3)).map(|v| v as i32);
    let _alt_unit = f.next();
    let geoid_sep_mm = f.next().and_then(|s| parse_fixed(s, 3)).map(|v| v as i32);
    // remaining fields (units, DGPS age/station) ignored — count varies by version

    Some(Gga {
        time,
        lat_1e7: parse_coord(lat_f, lat_h, 2),
        lon_1e7: parse_coord(lon_f, lon_h, 3),
        quality,
        sats_in_use,
        hdop_1e2,
        alt_msl_mm,
        geoid_sep_mm,
    })
}

fn parse_rmc(mut f: Fields) -> Option<Rmc> {
    let time = f.next().and_then(parse_time);
    let valid = matches!(f.next(), Some(b"A"));
    let lat_f = f.next()?;
    let lat_h = f.next()?;
    let lon_f = f.next()?;
    let lon_h = f.next()?;
    // knots * 1e4 → mm/s: 1 knot = 514.444 mm/s = 0.514444 m/s
    // v[mm/s] = knots_1e4 * 514444 / 1e7  (i64 headroom is ample)
    let speed_mm_s = f
        .next()
        .and_then(|s| parse_fixed(s, 4))
        .map(|k_1e4| ((k_1e4 * 514_444 + 5_000_000) / 10_000_000) as u32);
    let course_1e5 = f
        .next()
        .and_then(|s| parse_fixed(s, 5))
        .and_then(|v| u32::try_from(v).ok());
    let date = f.next().and_then(parse_date);
    // magnetic variation + (NMEA 2.3+) mode + (4.1+) nav status: ignored

    Some(Rmc {
        time,
        valid,
        lat_1e7: parse_coord(lat_f, lat_h, 2),
        lon_1e7: parse_coord(lon_f, lon_h, 3),
        speed_mm_s,
        course_1e5,
        date,
    })
}

fn parse_gsa(mut f: Fields) -> Option<Gsa> {
    let _mode = f.next(); // A/M
    let fix_type = f.next().and_then(parse_uint).unwrap_or(1) as u8;
    // 12 satellite-ID slots
    for _ in 0..12 {
        f.next();
    }
    let pdop_1e2 = f
        .next()
        .and_then(|s| parse_fixed(s, 2))
        .and_then(|v| u16::try_from(v).ok());
    let hdop_1e2 = f
        .next()
        .and_then(|s| parse_fixed(s, 2))
        .and_then(|v| u16::try_from(v).ok());
    let vdop_1e2 = f
        .next()
        .and_then(|s| parse_fixed(s, 2))
        .and_then(|v| u16::try_from(v).ok());
    // NMEA 4.1+ appends systemId — ignored

    Some(Gsa {
        fix_type,
        pdop_1e2,
        hdop_1e2,
        vdop_1e2,
    })
}

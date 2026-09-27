//! UBX binary protocol: build frames, name the message IDs the driver uses,
//! and decode the `NAV` messages it understands.

/// `UBX-NAV`: navigation results.
pub const CLASS_NAV: u8 = 0x01;
/// `UBX-ACK`: acknowledgements for `CFG` frames.
pub const CLASS_ACK: u8 = 0x05;
/// `UBX-CFG`: configuration.
pub const CLASS_CFG: u8 = 0x06;
/// `UBX-MON`: receiver monitoring.
pub const CLASS_MON: u8 = 0x0A;

/// Geodetic position.
pub const NAV_POSLLH: u8 = 0x02;
/// Fix status and ECEF solution. Gone from protocol 24.
pub const NAV_SOL: u8 = 0x06;
/// Position, velocity and time in one message. Protocol 14 and later.
pub const NAV_PVT: u8 = 0x07;
/// Velocity in north/east/down.
pub const NAV_VELNED: u8 = 0x12;
/// Per-satellite data, before protocol 15. Gone from protocol 24.
pub const NAV_SVINFO: u8 = 0x30;
/// Per-satellite data, protocol 15 and later.
pub const NAV_SAT: u8 = 0x35;
/// Port configuration, including the UART baud rate.
pub const CFG_PRT: u8 = 0x00;
/// Per-message output rate.
pub const CFG_MSG: u8 = 0x01;
/// Receiver restart. Never acknowledged.
pub const CFG_RST: u8 = 0x04;
/// Navigation solution rate.
pub const CFG_RATE: u8 = 0x08;
/// Clear, save and load the stored configuration.
pub const CFG_CFG: u8 = 0x09;
/// Power mode: continuous or power save.
pub const CFG_RXM: u8 = 0x11;
/// Navigation engine settings, including the dynamic platform model.
pub const CFG_NAV5: u8 = 0x24;
/// Constellation selection. Protocol 14 up to 33.
pub const CFG_GNSS: u8 = 0x3E;
/// Firmware and hardware version strings.
pub const MON_VER: u8 = 0x04;

/// Motion profile the receiver tunes its filters for
/// ([`NeoGps::set_dynamic_model`](crate::NeoGps::set_dynamic_model)).
///
/// Picking the wrong one costs accuracy: `Portable` throws away a fast
/// vehicle's velocity, and `Automotive` fights a drone's vertical motion.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum DynamicModel {
    /// The factory default. Fine for handheld and low-speed use.
    Portable = 0,
    /// Receiver does not move. Best accuracy when that is true.
    Stationary = 2,
    /// Walking pace.
    Pedestrian = 3,
    /// Road vehicle.
    Automotive = 4,
    /// Sea level, no vertical motion.
    Sea = 5,
    /// Aircraft, acceleration up to 1 g.
    Airborne1G = 6,
    /// Aircraft, acceleration up to 2 g.
    Airborne2G = 7,
    /// Aircraft, acceleration up to 4 g. What rockets and racing drones want.
    Airborne4G = 8,
}

/// A GNSS constellation, as `gnssId` in `CFG-GNSS`
/// ([`NeoGps::set_constellation`](crate::NeoGps::set_constellation)).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum Constellation {
    /// GPS (United States).
    Gps = 0,
    /// Satellite-based augmentation (WAAS, EGNOS, MSAS).
    Sbas = 1,
    /// Galileo (European Union).
    Galileo = 2,
    /// BeiDou (China).
    BeiDou = 3,
    /// IMES indoor positioning (Japan).
    Imes = 4,
    /// QZSS regional augmentation (Japan).
    Qzss = 5,
    /// GLONASS (Russia).
    Glonass = 6,
}

/// How much of the receiver's stored data to throw away on a restart
/// ([`NeoGps::reset`](crate::NeoGps::reset)).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ResetKind {
    /// Keep everything. Fastest time to first fix.
    Hot,
    /// Drop the ephemeris, keep the almanac, time and last position.
    Warm,
    /// Drop everything. Slowest time to first fix, and the one to use when
    /// the receiver is confused about where or when it is.
    Cold,
}

impl ResetKind {
    /// The `navBbrMask` value for this restart.
    pub fn bbr_mask(self) -> u16 {
        match self {
            ResetKind::Hot => 0x0000,
            ResetKind::Warm => 0x0001,
            ResetKind::Cold => 0xFFFF,
        }
    }
}

/// Largest payload the frame builder accepts (CFG frames are all small).
pub const TX_MAX_PAYLOAD: usize = 64;

/// Builds a UBX frame on the stack. Call [`new`](UbxFrame::new),
/// [`extend`](UbxFrame::extend), then [`finish`](UbxFrame::finish).
///
/// ```
/// # use neo_gps::ubx::UbxFrame;
/// let mut f = UbxFrame::new(0x06, 0x08);          // CFG-RATE
/// f.extend(&[0xE8, 0x03, 0x01, 0x00, 0x01, 0x00]); // 1000 ms, 1 cycle, GPS
/// let bytes = f.finish();
/// assert_eq!(&bytes[..4], &[0xB5, 0x62, 0x06, 0x08]);
/// ```
pub struct UbxFrame {
    buf: [u8; 8 + TX_MAX_PAYLOAD],
    len: usize,
}

impl UbxFrame {
    /// Start a frame for the given message class and id.
    ///
    /// # Example
    /// ```
    /// use neo_gps::ubx::UbxFrame;
    /// let mut poll = UbxFrame::new(0x0A, 0x04); // MON-VER poll
    /// assert_eq!(&poll.finish()[2..4], &[0x0A, 0x04]);
    /// ```
    pub fn new(class: u8, id: u8) -> Self {
        let mut buf = [0u8; 8 + TX_MAX_PAYLOAD];
        buf[0] = 0xB5;
        buf[1] = 0x62;
        buf[2] = class;
        buf[3] = id;
        // buf[4..6] = length, patched in finish()
        UbxFrame { buf, len: 6 }
    }

    /// Append payload bytes. Returns `None` if the frame would overflow.
    ///
    /// # Example
    /// ```
    /// use neo_gps::ubx::UbxFrame;
    /// let mut f = UbxFrame::new(0x06, 0x01);
    /// assert!(f.extend(&[0xF0, 0x03, 0x00]).is_some()); // CFG-MSG: GSV off
    /// ```
    #[must_use]
    pub fn extend(&mut self, payload: &[u8]) -> Option<()> {
        if self.len + payload.len() > 6 + TX_MAX_PAYLOAD {
            return None;
        }
        self.buf[self.len..self.len + payload.len()].copy_from_slice(payload);
        self.len += payload.len();
        Some(())
    }

    /// Finish the frame and return the bytes to write to the UART.
    ///
    /// # Example
    /// ```
    /// use neo_gps::ubx::UbxFrame;
    /// // Known vector: empty CFG-RATE poll checksums to 0x0E 0x30.
    /// assert_eq!(UbxFrame::new(0x06, 0x08).finish(), &[0xB5, 0x62, 0x06, 0x08, 0, 0, 0x0E, 0x30]);
    /// ```
    pub fn finish(&mut self) -> &[u8] {
        let plen = (self.len - 6) as u16;
        self.buf[4..6].copy_from_slice(&plen.to_le_bytes());
        let (mut a, mut b) = (0u8, 0u8);
        for &byte in &self.buf[2..self.len] {
            a = a.wrapping_add(byte);
            b = b.wrapping_add(a);
        }
        self.buf[self.len] = a;
        self.buf[self.len + 1] = b;
        &self.buf[..self.len + 2]
    }
}

/// Decoded `UBX-NAV-PVT`: position, velocity and time in one message
/// (protocol 14 and later).
///
/// Both payload lengths parse: 84 bytes on u-blox 7, 92 on u-blox 8 and
/// later. Field names and units follow the u-blox interface description.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct NavPvt {
    /// GPS time of week, ms.
    pub itow_ms: u32,
    /// UTC year, four digits.
    pub year: u16,
    /// UTC month, 1 to 12.
    pub month: u8,
    /// UTC day of month.
    pub day: u8,
    /// UTC hour.
    pub hour: u8,
    /// UTC minute.
    pub minute: u8,
    /// UTC second.
    pub second: u8,
    /// validDate | validTime | fullyResolved flags (bit 0..2).
    pub valid_flags: u8,
    /// 0 no fix, 2 = 2D, 3 = 3D, 4 = GNSS+DR, 5 = time-only.
    pub fix_type: u8,
    /// gnssFixOK is bit 0.
    pub flags: u8,
    /// Satellites used in the solution.
    pub num_sv: u8,
    /// Degrees * 1e7.
    pub lon_1e7: i32,
    /// Degrees * 1e7.
    pub lat_1e7: i32,
    /// Height above ellipsoid, mm.
    pub height_mm: i32,
    /// Height above mean sea level, mm.
    pub hmsl_mm: i32,
    /// Horizontal accuracy estimate, mm.
    pub h_acc_mm: u32,
    /// Vertical accuracy estimate, mm.
    pub v_acc_mm: u32,
    /// Ground speed, mm/s.
    pub gspeed_mm_s: i32,
    /// Heading of motion, degrees * 1e5.
    pub head_mot_1e5: i32,
    /// Position DOP * 100.
    pub pdop_1e2: u16,
}

impl NavPvt {
    /// Whether the fix is usable (`gnssFixOK`). Check this before trusting
    /// the position.
    ///
    /// # Example
    /// ```
    /// use neo_gps::ubx::NavPvt;
    /// let pvt = NavPvt::parse(&[0u8; 84]).unwrap(); // flags byte clear
    /// assert!(!pvt.gnss_fix_ok());
    /// ```
    pub fn gnss_fix_ok(&self) -> bool {
        self.flags & 0x01 != 0
    }

    /// Decode a `NAV-PVT` payload. `None` if it is too short.
    ///
    /// # Example
    /// ```
    /// use neo_gps::ubx::NavPvt;
    /// assert!(NavPvt::parse(&[0u8; 84]).is_some()); // u-blox 7 length
    /// assert!(NavPvt::parse(&[0u8; 92]).is_some()); // u-blox 8+ length
    /// assert!(NavPvt::parse(&[0u8; 40]).is_none());
    /// ```
    pub fn parse(p: &[u8]) -> Option<Self> {
        // 84 bytes: u-blox 7 (protocol 14). 92 bytes: u-blox 8+ (protocol 15+).
        if p.len() < 84 {
            return None;
        }
        let u16le = |i: usize| u16::from_le_bytes([p[i], p[i + 1]]);
        let u32le = |i: usize| u32::from_le_bytes([p[i], p[i + 1], p[i + 2], p[i + 3]]);
        let i32le = |i: usize| u32le(i) as i32;
        Some(NavPvt {
            itow_ms: u32le(0),
            year: u16le(4),
            month: p[6],
            day: p[7],
            hour: p[8],
            minute: p[9],
            second: p[10],
            valid_flags: p[11],
            fix_type: p[20],
            flags: p[21],
            num_sv: p[23],
            lon_1e7: i32le(24),
            lat_1e7: i32le(28),
            height_mm: i32le(32),
            hmsl_mm: i32le(36),
            h_acc_mm: u32le(40),
            v_acc_mm: u32le(44),
            gspeed_mm_s: i32le(60),
            head_mot_1e5: i32le(64),
            pdop_1e2: u16le(76),
        })
    }
}

/// Decoded `UBX-NAV-POSLLH`: geodetic position, on every protocol version.
///
/// It carries no fix flag, so pair it with [`NavSol`] on modules without
/// `NAV-PVT`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct NavPosllh {
    /// GPS time of week, ms.
    pub itow_ms: u32,
    /// Degrees * 1e7.
    pub lon_1e7: i32,
    /// Degrees * 1e7.
    pub lat_1e7: i32,
    /// Height above ellipsoid, mm.
    pub height_mm: i32,
    /// Height above mean sea level, mm.
    pub hmsl_mm: i32,
    /// Horizontal accuracy estimate, mm.
    pub h_acc_mm: u32,
    /// Vertical accuracy estimate, mm.
    pub v_acc_mm: u32,
}

impl NavPosllh {
    /// Decode a `NAV-POSLLH` payload. `None` if it is too short.
    ///
    /// # Example
    /// ```
    /// use neo_gps::ubx::NavPosllh;
    /// assert!(NavPosllh::parse(&[0u8; 28]).is_some());
    /// assert!(NavPosllh::parse(&[0u8; 20]).is_none());
    /// ```
    pub fn parse(p: &[u8]) -> Option<Self> {
        if p.len() < 28 {
            return None;
        }
        let u32le = |i: usize| u32::from_le_bytes([p[i], p[i + 1], p[i + 2], p[i + 3]]);
        let i32le = |i: usize| u32le(i) as i32;
        Some(NavPosllh {
            itow_ms: u32le(0),
            lon_1e7: i32le(4),
            lat_1e7: i32le(8),
            height_mm: i32le(12),
            hmsl_mm: i32le(16),
            h_acc_mm: u32le(20),
            v_acc_mm: u32le(24),
        })
    }
}

/// Decoded `UBX-NAV-VELNED`: velocity in north/east/down, on every protocol
/// version.
///
/// The binary velocity source on modules without `NAV-PVT`, which carries the
/// same numbers on 7-series and later.
///
/// Speeds are millimetres per second here, matching the rest of this crate;
/// the wire format is centimetres per second.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct NavVelned {
    /// GPS time of week, ms.
    pub itow_ms: u32,
    /// North velocity, mm/s.
    pub vel_n_mm_s: i32,
    /// East velocity, mm/s.
    pub vel_e_mm_s: i32,
    /// Down velocity, mm/s.
    pub vel_d_mm_s: i32,
    /// 3D speed, mm/s.
    pub speed_mm_s: u32,
    /// Ground speed (2D), mm/s.
    pub gspeed_mm_s: u32,
    /// Heading of motion, degrees * 1e5.
    pub heading_1e5: i32,
    /// Speed accuracy estimate, mm/s.
    pub s_acc_mm_s: u32,
    /// Course accuracy estimate, degrees * 1e5.
    pub c_acc_1e5: u32,
}

impl NavVelned {
    /// Decode a `NAV-VELNED` payload. `None` if it is too short.
    ///
    /// # Example
    /// ```
    /// use neo_gps::ubx::NavVelned;
    /// assert!(NavVelned::parse(&[0u8; 36]).is_some());
    /// assert!(NavVelned::parse(&[0u8; 20]).is_none());
    /// ```
    pub fn parse(p: &[u8]) -> Option<Self> {
        if p.len() < 36 {
            return None;
        }
        let u32le = |i: usize| u32::from_le_bytes([p[i], p[i + 1], p[i + 2], p[i + 3]]);
        let i32le = |i: usize| u32le(i) as i32;
        // cm/s to mm/s. Saturating because a corrupt frame must not panic in
        // a release build and must not wrap in a debug one.
        let cm_to_mm_i = |v: i32| v.saturating_mul(10);
        let cm_to_mm_u = |v: u32| v.saturating_mul(10);
        Some(NavVelned {
            itow_ms: u32le(0),
            vel_n_mm_s: cm_to_mm_i(i32le(4)),
            vel_e_mm_s: cm_to_mm_i(i32le(8)),
            vel_d_mm_s: cm_to_mm_i(i32le(12)),
            speed_mm_s: cm_to_mm_u(u32le(16)),
            gspeed_mm_s: cm_to_mm_u(u32le(20)),
            heading_1e5: i32le(24),
            s_acc_mm_s: cm_to_mm_u(u32le(28)),
            c_acc_1e5: u32le(32),
        })
    }
}

/// One satellite, from `UBX-NAV-SAT` or the older `UBX-NAV-SVINFO`.
///
/// Yielded by [`Satellites`], which
/// [`NeoGps::satellites`](crate::NeoGps::satellites) hands you.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SatInfo {
    /// Which constellation, as u-blox `gnssId`: 0 GPS, 1 SBAS, 2 Galileo,
    /// 3 BeiDou, 4 IMES, 5 QZSS, 6 GLONASS.
    ///
    /// `None` from `NAV-SVINFO`, which predates the field.
    pub gnss_id: Option<u8>,
    /// Satellite id within its constellation.
    pub sv_id: u8,
    /// Carrier-to-noise density, dB-Hz. 0 when the satellite is not tracked.
    pub cno_dbhz: u8,
    /// Elevation above the horizon, degrees.
    ///
    /// Only meaningful while the satellite is tracked. `NAV-SVINFO` reports
    /// one entry per receiver *channel*, and an idle channel carries whatever
    /// was last left in this field, which can be outside -90..90. Check
    /// [`cno_dbhz`](Self::cno_dbhz) first.
    pub elev_deg: i8,
    /// Azimuth, degrees. Meaningful under the same condition as
    /// [`elev_deg`](Self::elev_deg).
    pub azim_deg: i16,
    /// Whether the navigation solution uses this satellite.
    pub used_in_fix: bool,
}

/// Which of the two per-satellite messages a [`Satellites`] iterator is
/// walking. They carry the same information in different layouts.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum SatFormat {
    /// `NAV-SAT`, protocol 15 and later.
    NavSat,
    /// `NAV-SVINFO`, protocol 14 and earlier, deprecated on the 8 series.
    NavSvinfo,
}

/// Walks the satellites in a `NAV-SAT` or `NAV-SVINFO` payload.
///
/// Borrowed rather than collected: these messages carry up to a few dozen
/// satellites, which is more than belongs in a `Copy` event. Decode is lazy,
/// so nothing is parsed for satellites you do not look at.
///
/// # Example
/// ```
/// # use neo_gps::ubx::Satellites;
/// # let payload = [0u8; 8];
/// // An empty NAV-SAT payload: header only, no satellites.
/// let sats = Satellites::from_nav_sat(&payload).unwrap();
/// assert_eq!(sats.count(), 0);
/// ```
#[derive(Debug, Clone)]
pub struct Satellites<'a> {
    blocks: &'a [u8],
    format: SatFormat,
}

impl<'a> Satellites<'a> {
    /// Walk a `UBX-NAV-SAT` payload. `None` if it is too short for its
    /// own header.
    pub fn from_nav_sat(payload: &'a [u8]) -> Option<Self> {
        Self::new(payload, SatFormat::NavSat, payload.first().copied())
    }

    /// Walk a `UBX-NAV-SVINFO` payload. `None` if it is too short for its
    /// own header.
    pub fn from_nav_svinfo(payload: &'a [u8]) -> Option<Self> {
        Self::new(payload, SatFormat::NavSvinfo, None)
    }

    /// Both messages open with an 8-byte header and then repeat a 12-byte
    /// block per satellite. The count byte sits at offset 5 in `NAV-SAT`
    /// (`numSvs`) and offset 4 in `NAV-SVINFO` (`numCh`); it is clamped to
    /// what the payload actually holds, so a truncated frame yields fewer
    /// satellites instead of reading past the end.
    fn new(payload: &'a [u8], format: SatFormat, _version: Option<u8>) -> Option<Self> {
        if payload.len() < 8 {
            return None;
        }
        let claimed = match format {
            SatFormat::NavSat => payload[5],
            SatFormat::NavSvinfo => payload[4],
        } as usize;
        let available = (payload.len() - 8) / 12;
        let n = claimed.min(available);
        Some(Satellites {
            blocks: &payload[8..8 + n * 12],
            format,
        })
    }
}

impl Iterator for Satellites<'_> {
    type Item = SatInfo;

    fn next(&mut self) -> Option<SatInfo> {
        if self.blocks.len() < 12 {
            return None;
        }
        let b = &self.blocks[..12];
        self.blocks = &self.blocks[12..];
        let i16le = |i: usize| i16::from_le_bytes([b[i], b[i + 1]]);
        Some(match self.format {
            // gnssId, svId, cno, elev, azim(2), prRes(2), flags(4)
            SatFormat::NavSat => SatInfo {
                gnss_id: Some(b[0]),
                sv_id: b[1],
                cno_dbhz: b[2],
                elev_deg: b[3] as i8,
                azim_deg: i16le(4),
                used_in_fix: b[8] & 0x08 != 0, // flags bit 3: svUsed
            },
            // chn, svid, flags, quality, cno, elev, azim(2), prRes(4)
            SatFormat::NavSvinfo => SatInfo {
                gnss_id: None,
                sv_id: b[1],
                cno_dbhz: b[4],
                elev_deg: b[5] as i8,
                azim_deg: i16le(6),
                used_in_fix: b[2] & 0x01 != 0, // flags bit 0: svUsed
            },
        })
    }
}

/// Decoded `UBX-NAV-SOL`: fix status, DOP and ECEF solution.
///
/// The fix-state message on modules without `NAV-PVT`. Gone from protocol 24
/// on, so check
/// [`Capabilities::has_nav_sol`](crate::Capabilities::has_nav_sol) first.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct NavSol {
    /// GPS time of week, ms.
    pub itow_ms: u32,
    /// GPS week number.
    pub week: i16,
    /// 0 no fix, 1 DR, 2 = 2D, 3 = 3D, 4 = GPS+DR, 5 = time-only.
    pub gps_fix: u8,
    /// Bit 0 gpsFixOK, 1 diffSoln, 2 wknSet, 3 towSet.
    pub flags: u8,
    /// 3D position accuracy estimate, cm.
    pub p_acc_cm: u32,
    /// Speed accuracy estimate, cm/s.
    pub s_acc_cm_s: u32,
    /// Position DOP * 100.
    pub pdop_1e2: u16,
    /// Satellites used in the solution.
    pub num_sv: u8,
}

impl NavSol {
    /// Whether the fix is usable (`gpsFixOK`). Check this alongside
    /// `gps_fix`, the same way as [`NavPvt::gnss_fix_ok`].
    ///
    /// # Example
    /// ```
    /// use neo_gps::ubx::NavSol;
    /// let sol = NavSol::parse(&[0u8; 52]).unwrap();
    /// assert!(!sol.gps_fix_ok()); // flags byte clear
    /// ```
    pub fn gps_fix_ok(&self) -> bool {
        self.flags & 0x01 != 0
    }

    /// Decode a `NAV-SOL` payload. `None` if it is too short.
    ///
    /// # Example
    /// ```
    /// use neo_gps::ubx::NavSol;
    /// assert!(NavSol::parse(&[0u8; 52]).is_some());
    /// assert!(NavSol::parse(&[0u8; 40]).is_none());
    /// ```
    pub fn parse(p: &[u8]) -> Option<Self> {
        if p.len() < 52 {
            return None;
        }
        let u16le = |i: usize| u16::from_le_bytes([p[i], p[i + 1]]);
        let u32le = |i: usize| u32::from_le_bytes([p[i], p[i + 1], p[i + 2], p[i + 3]]);
        Some(NavSol {
            itow_ms: u32le(0),
            week: u16le(8) as i16,
            gps_fix: p[10],
            flags: p[11],
            p_acc_cm: u32le(24),
            s_acc_cm_s: u32le(40),
            pdop_1e2: u16le(44),
            num_sv: p[47],
        })
    }
}

/// Find `PROTVER` in a `MON-VER` payload and return its major version.
///
/// `None` when the payload has no `PROTVER` extension, which is the case on
/// 6-series firmware. Treat that as an old module.
///
/// # Example
/// ```
/// use neo_gps::ubx::parse_mon_ver_protver;
/// let mut p = vec![0u8; 40];                 // swVersion + hwVersion
/// let mut ext = [0u8; 30];
/// ext[..13].copy_from_slice(b"PROTVER=18.00"); // NEO-M8N, SPG 3.01
/// p.extend_from_slice(&ext);
/// assert_eq!(parse_mon_ver_protver(&p), Some(18));
/// assert_eq!(parse_mon_ver_protver(&[0u8; 40]), None); // NEO-6: no PROTVER
/// ```
pub fn parse_mon_ver_protver(payload: &[u8]) -> Option<u8> {
    if payload.len() < 40 {
        return None;
    }
    let mut off = 40;
    while off + 30 <= payload.len() {
        let ext = &payload[off..off + 30];
        if let Some(rest) = ext.strip_prefix(b"PROTVER") {
            // Skip '=' / ' ' separators, then read leading digits.
            let digits = rest
                .iter()
                .skip_while(|&&b| b == b'=' || b == b' ')
                .take_while(|&&b| b.is_ascii_digit());
            let mut v: u16 = 0;
            let mut any = false;
            for &b in digits {
                any = true;
                v = v * 10 + (b - b'0') as u16;
                if v > 255 {
                    return None;
                }
            }
            if any {
                return Some(v as u8);
            }
        }
        off += 30;
    }
    None
}

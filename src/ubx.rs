//! Minimal UBX binary protocol support: frame construction with Fletcher
//! checksum, the message IDs the driver uses, and parsers for `NAV-PVT`
//! and the `PROTVER` extension of `MON-VER`.

pub const CLASS_NAV: u8 = 0x01;
pub const CLASS_ACK: u8 = 0x05;
pub const CLASS_CFG: u8 = 0x06;
pub const CLASS_MON: u8 = 0x0A;

pub const NAV_POSLLH: u8 = 0x02;
pub const NAV_SOL: u8 = 0x06;
pub const NAV_PVT: u8 = 0x07;
pub const CFG_MSG: u8 = 0x01;
pub const CFG_RATE: u8 = 0x08;
pub const CFG_CFG: u8 = 0x09;
pub const CFG_GNSS: u8 = 0x3E;
pub const MON_VER: u8 = 0x04;

/// Largest payload the frame builder accepts (CFG frames are all small).
pub const TX_MAX_PAYLOAD: usize = 64;

/// Stack-allocated UBX frame builder.
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

    /// Patch in the length, compute the checksum, return the wire bytes.
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

/// Decoded `UBX-NAV-PVT` (protocol >= 14; the "one message to rule them all").
///
/// The payload is 84 bytes on u-blox 7 (protocol 14) and 92 bytes on u-blox 8
/// and later (which append `headVeh`, `magDec`, `magAcc`); all fields decoded
/// here lie within the common first 84 bytes, so both variants parse.
/// Field names and units follow the u-blox interface description.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct NavPvt {
    /// GPS time of week, ms.
    pub itow_ms: u32,
    pub year: u16,
    pub month: u8,
    pub day: u8,
    pub hour: u8,
    pub minute: u8,
    pub second: u8,
    /// validDate | validTime | fullyResolved flags (bit 0..2).
    pub valid_flags: u8,
    /// 0 no fix, 2 = 2D, 3 = 3D, 4 = GNSS+DR, 5 = time-only.
    pub fix_type: u8,
    /// gnssFixOK is bit 0.
    pub flags: u8,
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

/// Decoded `UBX-NAV-POSLLH` (28 bytes, all protocol versions): geodetic
/// position. On a NEO-6M (no NAV-PVT) this is the binary position message;
/// pair it with [`NavSol`] for fix status.
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

/// Decoded `UBX-NAV-SOL` (52 bytes): fix status, DOP and ECEF solution.
/// The NEO-6M's fix-state message. Deprecated on the 8 series (prefer
/// NAV-PVT) and removed from protocol 24 (M9) on — see
/// [`Capabilities::has_nav_sol`](crate::Capabilities::has_nav_sol).
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
    /// `gpsFixOK`: the flag the u-blox 6 spec says to check alongside
    /// `gpsFix`, mirroring [`NavPvt::gnss_fix_ok`].
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
/// Payload layout: `swVersion[30]`, `hwVersion[10]`, then N × `extension[30]`
/// NUL-padded strings. 8-series and later report e.g. `PROTVER=18.00` or
/// `PROTVER 15.00`; 6-series firmware has no such extension → `None`
/// (the caller treats that as "old", which is correct).
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

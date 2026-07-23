use ubx::{CLASS_ACK, CLASS_NAV};

use crate::{nmea, ubx, Event};
// NMEA 0183 caps sentences at 82 chars, but u-blox proprietary PUBX,00 runs
// ~110 and high-precision mode lengthens standard sentences; 128 covers all.
pub(crate) const NMEA_MAX: usize = 128;
const UBX_MAX_PAYLOAD: usize = 384; // NAV-PVT = 92, MON-VER ≈ 40 + 30·n

#[derive(Clone, Copy)]
enum DState {
    Idle,
    /// Collecting an NMEA sentence body (after `$`, before CR/LF).
    Nmea,
    /// Saw 0xB5, expecting 0x62.
    UbxSync,
    /// Collecting UBX header (class, id, len_lo, len_hi).
    UbxHeader,
    /// Collecting `remaining` payload bytes (payload may be truncated in
    /// the buffer if oversized, but framing stays byte-accurate).
    UbxPayload {
        remaining: usize,
    },
    /// Collecting the two checksum bytes.
    UbxCksum {
        got: u8,
    },
}

pub(crate) struct Deframer {
    state: DState,
    nmea_buf: [u8; NMEA_MAX],
    nmea_len: usize,
    /// Snapshot of the last sentence that produced an event (accessor-stable:
    /// later garbage or partial frames can't clobber it).
    nmea_last: [u8; NMEA_MAX],
    nmea_last_len: usize,
    ubx_hdr: [u8; 4],
    ubx_hdr_len: usize,
    ubx_buf: [u8; UBX_MAX_PAYLOAD],
    ubx_len: usize,
    /// Snapshot of the last complete UBX frame (sync bytes through checksum)
    /// that produced an event; payload accessors slice into it.
    ubx_last: [u8; 8 + UBX_MAX_PAYLOAD],
    ubx_last_len: usize,
    /// Running Fletcher checksum over class..payload.
    ck: (u8, u8),
    rx_ck: [u8; 2],
}

impl Deframer {
    pub(crate) fn new() -> Self {
        Deframer {
            state: DState::Idle,
            nmea_buf: [0; NMEA_MAX],
            nmea_len: 0,
            nmea_last: [0; NMEA_MAX],
            nmea_last_len: 0,
            ubx_hdr: [0; 4],
            ubx_hdr_len: 0,
            ubx_buf: [0; UBX_MAX_PAYLOAD],
            ubx_len: 0,
            ubx_last: [0; 8 + UBX_MAX_PAYLOAD],
            ubx_last_len: 0,
            ck: (0, 0),
            rx_ck: [0; 2],
        }
    }

    /// Drop any partially assembled frame and return to hunting for sync.
    /// Snapshots of already-delivered events are untouched.
    pub(crate) fn reset(&mut self) {
        self.state = DState::Idle;
    }

    /// Payload of the last UBX frame that produced an event.
    pub(crate) fn last_ubx_payload(&self) -> &[u8] {
        &self.ubx_last[6..self.ubx_last_len - 2]
    }

    /// The complete last UBX frame: sync, header, payload, checksum.
    pub(crate) fn last_ubx_frame(&self) -> &[u8] {
        &self.ubx_last[..self.ubx_last_len]
    }

    /// Bytes of the last NMEA sentence that produced an event
    /// ($ and CR/LF excluded, checksum trailer included).
    pub(crate) fn last_nmea_line(&self) -> &[u8] {
        &self.nmea_last[..self.nmea_last_len]
    }

    /// Extract `PROTVER` from a just-received MON-VER payload, if present.
    pub(crate) fn last_ubx_protver(&self) -> Option<u8> {
        ubx::parse_mon_ver_protver(self.last_ubx_payload())
    }

    pub(crate) fn push(&mut self, b: u8) -> Option<Event> {
        match self.state {
            DState::Idle => {
                match b {
                    b'$' => {
                        self.nmea_len = 0;
                        self.state = DState::Nmea;
                    }
                    0xB5 => self.state = DState::UbxSync,
                    _ => {} // noise between frames
                }
                None
            }

            DState::Nmea => {
                match b {
                    b'\r' | b'\n' => {
                        self.state = DState::Idle;
                        let ev = nmea::parse_line(&self.nmea_buf[..self.nmea_len]);
                        if ev.is_some() {
                            self.nmea_last[..self.nmea_len]
                                .copy_from_slice(&self.nmea_buf[..self.nmea_len]);
                            self.nmea_last_len = self.nmea_len;
                        }
                        return ev;
                    }
                    b'$' => {
                        // Lost sync mid-sentence; restart.
                        self.nmea_len = 0;
                    }
                    _ => {
                        if self.nmea_len < NMEA_MAX {
                            self.nmea_buf[self.nmea_len] = b;
                            self.nmea_len += 1;
                        } else {
                            // Oversized garbage: drop the sentence.
                            self.state = DState::Idle;
                        }
                    }
                }
                None
            }

            DState::UbxSync => {
                if b == 0x62 {
                    self.ubx_hdr_len = 0;
                    self.ck = (0, 0);
                    self.state = DState::UbxHeader;
                } else {
                    self.state = DState::Idle;
                }
                None
            }

            DState::UbxHeader => {
                self.ubx_hdr[self.ubx_hdr_len] = b;
                self.ubx_hdr_len += 1;
                self.ck_add(b);
                if self.ubx_hdr_len == 4 {
                    let len = u16::from_le_bytes([self.ubx_hdr[2], self.ubx_hdr[3]]) as usize;
                    self.ubx_len = 0;
                    if len == 0 {
                        self.state = DState::UbxCksum { got: 0 };
                    } else {
                        self.state = DState::UbxPayload { remaining: len };
                    }
                }
                None
            }

            DState::UbxPayload { remaining } => {
                self.ck_add(b);
                if self.ubx_len < UBX_MAX_PAYLOAD {
                    self.ubx_buf[self.ubx_len] = b;
                    self.ubx_len += 1;
                }
                let remaining = remaining - 1;
                self.state = if remaining == 0 {
                    DState::UbxCksum { got: 0 }
                } else {
                    DState::UbxPayload { remaining }
                };
                None
            }

            DState::UbxCksum { got } => {
                self.rx_ck[got as usize] = b;
                if got == 0 {
                    self.state = DState::UbxCksum { got: 1 };
                    return None;
                }
                self.state = DState::Idle;
                if self.rx_ck != [self.ck.0, self.ck.1] {
                    return None; // bad checksum: drop silently
                }
                // Checksum valid → this frame will produce an event; snapshot
                // the complete frame so accessors (and external codecs, which
                // want sync+checksum) survive later corrupt frames.
                self.ubx_last[0] = 0xB5;
                self.ubx_last[1] = 0x62;
                self.ubx_last[2..6].copy_from_slice(&self.ubx_hdr);
                self.ubx_last[6..6 + self.ubx_len].copy_from_slice(&self.ubx_buf[..self.ubx_len]);
                self.ubx_last[6 + self.ubx_len..8 + self.ubx_len].copy_from_slice(&self.rx_ck);
                self.ubx_last_len = 8 + self.ubx_len;
                let (class, id) = (self.ubx_hdr[0], self.ubx_hdr[1]);
                let payload = &self.ubx_last[6..self.ubx_last_len - 2];
                match (class, id) {
                    // ACK correlation is transport machinery, never gated.
                    (CLASS_ACK, aid @ (0x00 | 0x01)) if payload.len() >= 2 => Some(Event::Ack {
                        class: payload[0],
                        id: payload[1],
                        ok: aid == 0x01,
                    }),
                    #[cfg(feature = "builtin-codec")]
                    (CLASS_NAV, ubx::NAV_PVT) => Some(
                        ubx::NavPvt::parse(payload)
                            .map(Event::NavPvt)
                            .unwrap_or(Event::UbxOther { class, id }),
                    ),
                    #[cfg(feature = "builtin-codec")]
                    (CLASS_NAV, ubx::NAV_POSLLH) => Some(
                        ubx::NavPosllh::parse(payload)
                            .map(Event::NavPosllh)
                            .unwrap_or(Event::UbxOther { class, id }),
                    ),
                    #[cfg(feature = "builtin-codec")]
                    (CLASS_NAV, ubx::NAV_SOL) => Some(
                        ubx::NavSol::parse(payload)
                            .map(Event::NavSol)
                            .unwrap_or(Event::UbxOther { class, id }),
                    ),
                    _ => Some(Event::UbxOther { class, id }),
                }
            }
        }
    }

    fn ck_add(&mut self, b: u8) {
        self.ck.0 = self.ck.0.wrapping_add(b);
        self.ck.1 = self.ck.1.wrapping_add(self.ck.0);
    }
}

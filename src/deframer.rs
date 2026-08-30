use ubx::CLASS_ACK;
#[cfg(feature = "builtin-codec")]
use ubx::CLASS_NAV;

use crate::{nmea, ubx, Event};
// 128 covers the longest sentence u-blox sends (PUBX,00 at ~110 chars).
pub(crate) const NMEA_MAX: usize = 128;
const UBX_MAX_PAYLOAD: usize = 384; // NAV-PVT = 92, MON-VER ≈ 40 + 30·n

#[derive(Clone, Copy)]
enum DState {
    Idle,
    Nmea,
    UbxSync,
    UbxHeader,
    UbxPayload { remaining: usize },
    UbxCksum { got: u8 },
}

pub(crate) struct Deframer {
    state: DState,
    nmea_buf: [u8; NMEA_MAX],
    nmea_len: usize,
    nmea_last: [u8; NMEA_MAX],
    nmea_last_len: usize,
    ubx_hdr: [u8; 4],
    ubx_hdr_len: usize,
    ubx_buf: [u8; UBX_MAX_PAYLOAD],
    ubx_len: usize,
    ubx_last: [u8; 8 + UBX_MAX_PAYLOAD],
    ubx_last_len: usize,
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

    /// Drop the frame in progress and hunt for sync again. Snapshots stay.
    pub(crate) fn reset(&mut self) {
        self.state = DState::Idle;
    }

    /// Payload of the last UBX frame that produced an event.
    pub(crate) fn last_ubx_payload(&self) -> &[u8] {
        // A snapshot is sync(2) + header(4) + payload + checksum(2), so fewer
        // than 8 bytes means no frame has completed and there is nothing to
        // slice. Reachable from safe public API: a caller may look before the
        // first UBX frame arrives.
        if self.ubx_last_len < 8 {
            return &[];
        }
        &self.ubx_last[6..self.ubx_last_len - 2]
    }

    /// The complete last UBX frame: sync, header, payload, checksum.
    pub(crate) fn last_ubx_frame(&self) -> &[u8] {
        &self.ubx_last[..self.ubx_last_len]
    }

    /// Bytes of the last NMEA sentence that produced an event: no `$` or
    /// CR/LF, checksum trailer included.
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
                        // Lost sync mid-sentence.
                        self.nmea_len = 0;
                    }
                    _ => {
                        if self.nmea_len < NMEA_MAX {
                            self.nmea_buf[self.nmea_len] = b;
                            self.nmea_len += 1;
                        } else {
                            // Too long to be a sentence.
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
                    return None;
                }
                // Not a UBX frame after all, and this byte may open one
                // itself: a second 0xB5, or the `$` of a sentence. Hand it
                // back to `Idle` instead of dropping it, or a single stray
                // 0xB5 swallows the whole frame behind it. Re-entry stops
                // here, since `Idle` never dispatches again.
                self.state = DState::Idle;
                self.push(b)
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
                // Snapshot the whole frame so the accessors survive later
                // corrupt frames. External codecs want sync+checksum.
                self.ubx_last[0] = 0xB5;
                self.ubx_last[1] = 0x62;
                self.ubx_last[2..6].copy_from_slice(&self.ubx_hdr);
                self.ubx_last[6..6 + self.ubx_len].copy_from_slice(&self.ubx_buf[..self.ubx_len]);
                self.ubx_last[6 + self.ubx_len..8 + self.ubx_len].copy_from_slice(&self.rx_ck);
                self.ubx_last_len = 8 + self.ubx_len;
                let (class, id) = (self.ubx_hdr[0], self.ubx_hdr[1]);
                let payload = &self.ubx_last[6..self.ubx_last_len - 2];
                match (class, id) {
                    // ACK correlation is transport, so never feature-gated.
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
                    #[cfg(feature = "builtin-codec")]
                    (CLASS_NAV, ubx::NAV_VELNED) => Some(
                        ubx::NavVelned::parse(payload)
                            .map(Event::NavVelned)
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

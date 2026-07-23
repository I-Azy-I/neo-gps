//! Shared test infrastructure: wire builders, deframer feeder, mock UART,
//! and a mode-agnostic block_on. Compiled for all feature combinations.

use crate::*;

/// Build a wire-format NMEA sentence with a computed checksum.
pub(crate) fn nmea_wire(body: &str) -> Vec<u8> {
    let ck = body.bytes().fold(0u8, |a, b| a ^ b);
    format!("${}*{:02X}\r\n", body, ck).into_bytes()
}

/// Build a wire-format UBX frame of any size (module→host frames can exceed
/// the driver's small outgoing-frame buffer, so this doesn't use `UbxFrame`).
pub(crate) fn ubx_wire(class: u8, id: u8, payload: &[u8]) -> Vec<u8> {
    let mut v = vec![0xB5, 0x62, class, id];
    v.extend_from_slice(&(payload.len() as u16).to_le_bytes());
    v.extend_from_slice(payload);
    let (mut a, mut b) = (0u8, 0u8);
    for &byte in &v[2..] {
        a = a.wrapping_add(byte);
        b = b.wrapping_add(a);
    }
    v.push(a);
    v.push(b);
    v
}

pub(crate) fn feed(bytes: &[u8]) -> Vec<Event> {
    let mut d = Deframer::new();
    bytes.iter().filter_map(|&b| d.push(b)).collect()
}

/// Build a MON-VER payload: 30-byte swVersion, 10-byte hwVersion,
/// N x 30-byte NUL-padded extension strings (per the interface description).
pub(crate) fn mon_ver_payload(sw: &str, hw: &str, exts: &[&str]) -> Vec<u8> {
    let mut p = vec![0u8; 40];
    p[..sw.len()].copy_from_slice(sw.as_bytes());
    p[30..30 + hw.len()].copy_from_slice(hw.as_bytes());
    for e in exts {
        let mut block = [0u8; 30];
        block[..e.len()].copy_from_slice(e.as_bytes());
        p.extend_from_slice(&block);
    }
    p
}

/// Scripted duplex mock: serves `rx` in small chunks, records writes.
pub(crate) struct MockUart {
    pub(crate) rx: Vec<u8>,
    pub(crate) pos: usize,
    pub(crate) tx: Vec<u8>,
    /// Max bytes delivered per read() — exercises chunk-boundary handling.
    pub(crate) chunk: usize,
    /// The next N calls to read() return Err (scripted UART glitches).
    pub(crate) fail_next_reads: usize,
    /// Specific read-call indices (0-based) that return Err.
    pub(crate) fail_on_reads: Vec<usize>,
    /// Read calls made so far.
    pub(crate) reads: usize,
}

impl MockUart {
    pub(crate) fn new(rx: Vec<u8>) -> Self {
        MockUart {
            rx,
            pos: 0,
            tx: Vec::new(),
            chunk: 7,
            fail_next_reads: 0,
            fail_on_reads: Vec::new(),
            reads: 0,
        }
    }
}

use crate::eio;

#[derive(Debug, PartialEq, Eq)]
pub(crate) struct Never;
impl eio::Error for Never {
    fn kind(&self) -> eio::ErrorKind {
        eio::ErrorKind::Other
    }
}
impl eio::ErrorType for MockUart {
    type Error = Never;
}

#[maybe_async_cfg::maybe(sync(feature = "sync", keep_self), async(feature = "async", keep_self))]
impl eio::Read for MockUart {
    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, Never> {
        let i = self.reads;
        self.reads += 1;
        if self.fail_next_reads > 0 {
            self.fail_next_reads -= 1;
            return Err(Never);
        }
        if self.fail_on_reads.contains(&i) {
            return Err(Never);
        }
        // Deliver at most `chunk` bytes per read to exercise boundary handling.
        let n = (self.rx.len() - self.pos).min(buf.len()).min(self.chunk);
        buf[..n].copy_from_slice(&self.rx[self.pos..self.pos + n]);
        self.pos += n;
        Ok(n)
    }
}

#[maybe_async_cfg::maybe(sync(feature = "sync", keep_self), async(feature = "async", keep_self))]
impl eio::Write for MockUart {
    async fn write(&mut self, buf: &[u8]) -> Result<usize, Never> {
        self.tx.extend_from_slice(buf);
        Ok(buf.len())
    }

    // Required by blocking embedded_io::Write; defaulted in the async trait.
    async fn flush(&mut self) -> Result<(), Never> {
        Ok(())
    }
}

/// In the sync build there is nothing to drive: values are already values.
#[cfg(all(feature = "sync", not(feature = "async")))]
pub(crate) fn block_on<T>(v: T) -> T {
    v
}

#[cfg(feature = "async")]
#[allow(unsafe_code)] // test-only micro-executor; the library itself forbids unsafe
pub(crate) fn block_on<F: core::future::Future>(mut fut: F) -> F::Output {
    use core::task::{Context, Poll, RawWaker, RawWakerVTable, Waker};
    fn raw() -> RawWaker {
        RawWaker::new(core::ptr::null(), &VT)
    }
    static VT: RawWakerVTable = RawWakerVTable::new(|_| raw(), |_| {}, |_| {}, |_| {});
    let waker = unsafe { Waker::from_raw(raw()) };
    let mut cx = Context::from_waker(&waker);
    let mut fut = unsafe { core::pin::Pin::new_unchecked(&mut fut) };
    loop {
        if let Poll::Ready(v) = fut.as_mut().poll(&mut cx) {
            return v;
        }
    }
}

impl<S> NeoGps<S> {
    /// Test-only shared access to the wrapped stream.
    pub(crate) fn uart_ref(&self) -> &S {
        &self.uart
    }

    /// Test-only mutable access (e.g. to script glitches mid-run).
    pub(crate) fn uart_mut(&mut self) -> &mut S {
        &mut self.uart
    }
}

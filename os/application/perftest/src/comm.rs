//! TCP-based control-path communication between the server and client processes.
//!
//! The protocol is intentionally simple and platform-neutral: every field is
//! encoded in network byte order (big-endian) and written/read as a fixed-size
//! byte buffer so that no external serialisation library is required.

use network::{NetworkError, TcpStream};

/// All information that the two sides exchange over TCP before starting the
/// RDMA test.
///
/// Layout on the wire (all big-endian):
/// ```text
/// qp_num : u32   (4 bytes)
/// lid    : u16   (2 bytes)
/// gid    : [u8; 16] – all-zeros when GID routing is not used
/// addr   : u64   (8 bytes) – virtual address of the registered MR
/// rkey   : u32   (4 bytes) – remote key for the MR
/// ─────────────────────────
///          34 bytes total
/// ```
///
/// The client's `addr` / `rkey` fields are zero because the server is the
/// RDMA-READ target; only the server's values are meaningful.
pub struct PeerInfo {
    pub qp_num: u32,
    pub lid: u16,
    /// 16-byte GID. All-zeros means InfiniBand / LID-only routing.
    pub gid: [u8; 16],
    /// Virtual address of the peer's registered memory region.
    pub addr: u64,
    /// Remote key for the peer's memory region.
    pub rkey: u32,
}

impl PeerInfo {
    const LEN: usize = 4 + 2 + 16 + 8 + 4;

    pub fn write_to(&self, stream: &mut TcpStream) -> Result<usize, NetworkError> {
        let mut buf = [0u8; Self::LEN];
        let mut pos = 0;
        buf[pos..pos + 4].copy_from_slice(&self.qp_num.to_be_bytes());
        pos += 4;
        buf[pos..pos + 2].copy_from_slice(&self.lid.to_be_bytes());
        pos += 2;
        buf[pos..pos + 16].copy_from_slice(&self.gid);
        pos += 16;
        buf[pos..pos + 8].copy_from_slice(&self.addr.to_be_bytes());
        pos += 8;
        buf[pos..pos + 4].copy_from_slice(&self.rkey.to_be_bytes());
        stream.write(&buf)
    }

    pub fn read_from(stream: &mut TcpStream) -> Result<Self, NetworkError> {
        let mut buf = [0u8; Self::LEN];
        stream.read(&mut buf)?;
        let mut pos = 0;

        let qp_num = u32::from_be_bytes(buf[pos..pos + 4].try_into().unwrap());
        pos += 4;
        let lid = u16::from_be_bytes(buf[pos..pos + 2].try_into().unwrap());
        pos += 2;
        let gid: [u8; 16] = buf[pos..pos + 16].try_into().unwrap();
        pos += 16;
        let addr = u64::from_be_bytes(buf[pos..pos + 8].try_into().unwrap());
        pos += 8;
        let rkey = u32::from_be_bytes(buf[pos..pos + 4].try_into().unwrap());

        Ok(Self { qp_num, lid, gid, addr, rkey })
    }
}

/// One-byte synchronisation barrier: both sides write a byte and then read
/// a byte so that neither proceeds until the other has also reached this point.
pub fn sync(stream: &mut TcpStream) -> Result<(), NetworkError> {
    stream.write(&[0u8])?;
    let mut buf = [0u8; 1];
    stream.read(&mut buf)?;
    Ok(())
}

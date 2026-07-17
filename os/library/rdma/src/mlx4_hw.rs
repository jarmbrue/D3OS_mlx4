//! Hardware-layout structs shared between the kernel mlx4 driver
//! (`os/kernel/src/device/mlx4`) and the userspace kernel-bypass fast path
//! (`os/library/ibverbs`).
//!
//! These are bit-exact ConnectX-3 WQE / CQE / doorbell layouts. Keeping a
//! single, shared definition (instead of one copy per side of the syscall
//! boundary) avoids silent WQE/CQE corruption from two independently
//! maintained copies drifting apart - the HCA does *something* with a
//! mismatched layout, just not what either side intended, and that failure
//! mode is expensive to debug.
//!
//! Scope: RC/UC (send + RDMA read/write) only. UD (datagram) QP support is
//! not part of the kernel-bypass fast path, so `WqeDatagramSegment` and
//! friends remain private to the kernel driver.

#![allow(non_camel_case_types)]

use core::sync::atomic::{compiler_fence, Ordering};

use bitflags::bitflags;
use byteorder::BigEndian;
use modular_bitfield_msb::{bitfield, prelude::{B4, B5, B7, B12, B24}};
use strum_macros::FromRepr;
use tock_registers::{register_bitfields, register_structs, registers::WriteOnly};
use zerocopy::{FromBytes, U32, U64};

use crate::ib_core::{ibv_send_flags, ibv_send_wr_wr, ibv_sge};

/// Control segment, present at the start of every WQE.
#[derive(FromBytes)]
#[repr(C)]
pub struct WqeControlSegment {
    pub owner_opcode: U32<BigEndian>,
    pub vlan_cv_f_ds: U32<BigEndian>,
    pub flags: U32<BigEndian>,
    pub flags2: U32<BigEndian>,
}

impl WqeControlSegment {
    pub fn size(&self) -> u32 {
        (self.vlan_cv_f_ds.get() & 0x3f) << 4
    }
}

bitflags! {
    pub struct WqeControlSegmentFlags: u32 {
        const NEC = 1 << 29;
        const IIP = 1 << 28;
        const ILP = 1 << 27;
        const FENCE = 1 << 6;
        const CQ_UPDATE = 3 << 2;
        const SOLICITED = 1 << 1;
        const IP_CSUM = 1 << 4;
        const TCP_UDP_CSUM = 1 << 5;
        const INS_CVLAN = 1 << 6;
        const INS_SVLAN = 1 << 7;
        const STRONG_ORDER = 1 << 7;
        const FORCE_LOOPBACK = 1 << 0;
    }
}

impl From<ibv_send_flags> for WqeControlSegmentFlags {
    fn from(flags: ibv_send_flags) -> Self {
        let mut out = WqeControlSegmentFlags::empty();

        if flags.contains(ibv_send_flags::FENCE) {
            out |= WqeControlSegmentFlags::FENCE;
        }
        if flags.contains(ibv_send_flags::SOLICITED) {
            out |= WqeControlSegmentFlags::SOLICITED;
        }
        // CQ update for signaled WRs
        if flags.contains(ibv_send_flags::SIGNALED) {
            out |= WqeControlSegmentFlags::CQ_UPDATE;
        }
        out
    }
}

/// Scatter/gather data segment, used both for send-queue SGEs and
/// receive-queue elements.
#[derive(FromBytes)]
#[repr(C)]
pub struct WqeDataSegment {
    byte_count: U32<BigEndian>,
    lkey: U32<BigEndian>,
    addr: U64<BigEndian>,
}

impl WqeDataSegment {
    /// Fill this data segment from an sge and its already-resolved physical
    /// address. Address translation (virtual -> physical) is caller-specific
    /// (a kernel page-table walk on the syscall path, an `ibv_mr`-relative
    /// computation on the userspace fast path), so it happens before this
    /// call - this method only writes the segment's fields, in the order
    /// hardware requires.
    pub fn set(&mut self, sge: &ibv_sge, phys_addr: u64) {
        self.lkey.set(sge.lkey);
        self.addr.set(phys_addr);
        // sending needs a barrier here before writing the byte_count
        // field to make sure that all the data is visible before the
        // byte_count field is set. Otherwise, if the segment begins a new
        // cacheline, the HCA prefetcher could grab the 64-byte chunk and
        // get a valid (!= * 0xffffffff) byte count but stale data, and end
        // up sending the wrong data.
        compiler_fence(Ordering::SeqCst);
        self.byte_count.set(sge.length);
    }

    /// Create a dummy element to be the last in the queue.
    pub fn last() -> WqeDataSegment {
        const INVALID_LKEY: u32 = 0x100;
        Self {
            byte_count: 0.into(),
            lkey: INVALID_LKEY.into(),
            addr: 0.into(),
        }
    }
}

/// Remote address segment, used by RC/UC RDMA read/write WQEs.
#[derive(FromBytes)]
#[repr(C)]
pub struct WqeRemoteAddressSegment {
    va: U64<BigEndian>,
    key: U32<BigEndian>,
    rsvd: u32,
}

impl WqeRemoteAddressSegment {
    /// Create a remote address segment from a wr wr.
    pub fn from_wr(wr: &ibv_send_wr_wr) -> Result<Self, &'static str> {
        if let ibv_send_wr_wr::rdma { remote_addr, rkey } = wr {
            Ok(Self {
                va: (*remote_addr).into(),
                key: (*rkey).into(),
                rsvd: 0,
            })
        } else {
            Err("invalid wr field")
        }
    }
}

/// Send-queue opcodes, as they appear in a WQE's control segment / a send
/// CQE.
#[repr(u32)]
#[derive(FromRepr)]
pub enum QueuePairOpcode {
    Nop = 0x00,
    SendInval = 0x01,
    RdmaWrite = 0x08,
    RdmaWriteImm = 0x09,
    Send = 0x0a,
    SendImm = 0x0b,
    Lso = 0x0e,
    RdmaRead = 0x10,
    AtomicCs = 0x11,
    AtomicFa = 0x12,
    MaskedAtomicCs = 0x14,
    MaskedAtomicFa = 0x15,
    BindMw = 0x18,
    Fmr = 0x19,
    LocalInval = 0x1b,
    ConfigCmd = 0x1f,
}

/// Receive-queue opcodes, as they appear in a receive CQE.
#[repr(u32)]
#[derive(FromRepr)]
pub enum ReceiveOpcode {
    RdmaWriteImm = 0x0,
    Send = 0x1,
    SendImm = 0x2,
    SendInval = 0x3,
}

/// Error syndrome, as it appears in an error CQE's `checksum` field (high
/// byte).
#[repr(u8)]
#[derive(Debug, FromRepr)]
pub enum Syndrome {
    LocalLengthError = 0x01,
    LocalQpOperationError = 0x02,
    LocalProtError = 0x04,
    WrFlushError = 0x05,
    MwBindError = 0x06,
    BadResponseError = 0x10,
    LocalAccessError = 0x11,
    RemoteInvalidRequestError = 0x12,
    RemoteAccessError = 0x13,
    RemoteOperationError = 0x14,
    TransportRetryExceededError = 0x15,
    RnrRetryExceededError = 0x16,
    RemoteAbortedErr = 0x22,
}

/// A single completion queue entry. CQE size is 32 bytes on ConnectX-3
/// (64 B CQEs are also supported by the hardware but not used here).
#[bitfield(bytes = 32)]
#[derive(Debug)]
pub struct CompletionQueueEntry {
    #[skip]
    __: u8,
    pub qp_number: B24,
    pub immed_rss_invalid: u32,
    pub g: bool,
    pub mlpath: B7,
    pub rqpn: B24,
    pub sl: B4,
    #[skip]
    vid: B12,
    pub slid: u16,
    #[skip]
    __: u32,
    pub byte_cnt: u32,
    pub wqe_index: u16,
    /// vendor_err_syndrome (u8) and syndrome (u8) on error
    pub checksum: u16,
    #[skip]
    __: B24,
    pub owner: bool,
    pub is_send: bool,
    #[skip]
    __: bool,
    pub opcode: B5,
}

/// Write-only 32-bit register that ignores endianness conversion at the
/// call site (callers pass an already big-endian-encoded value, matching
/// hardware's on-the-wire format) and always writes with volatile
/// semantics. A plain non-volatile store would be a correctness bug here:
/// nothing in this translation unit ever reads these fields back, so an
/// optimizer could otherwise treat the store as dead and elide it, even
/// though the HCA/host-memory side does observe it.
#[repr(transparent)]
#[derive(Debug)]
pub struct WriteOnlyBe32(u32);

impl WriteOnlyBe32 {
    pub fn set(&mut self, value: u32) {
        unsafe { core::ptr::write_volatile(&mut self.0, value) }
    }
}

/// Per-QP doorbell record (DMA host memory, not MMIO).
#[repr(transparent)]
#[derive(Debug)]
pub struct QueuePairDoorbell {
    pub receive_wqe_index: WriteOnlyBe32,
}

/// Per-CQ doorbell record (DMA host memory, not MMIO).
#[repr(C)]
#[derive(Debug)]
pub struct CompletionQueueDoorbell {
    pub update_consumer_index: WriteOnlyBe32,
    pub arm_consumer_index: WriteOnlyBe32,
}

// The UAR (User Access Region) doorbell MMIO page. Unlike the two doorbell
// records above, this one is genuine device MMIO (BAR2-backed), so it keeps
// using `tock_registers` for its richer read/write/bitfield semantics.

register_bitfields![u32,
    pub SendQueueNumber [
        NUM OFFSET(8) NUMBITS(24)
    ],
    pub CpSnCmdNum [
        CPN OFFSET(0)  NUMBITS(24),
        CMD OFFSET(24) NUMBITS(3),
        SN  OFFSET(28) NUMBITS(2)
    ],
    pub CpConsumerIndex [
        CP_CI OFFSET(0) NUMBITS(24),
    ],
    pub DoorbellEqField [
        CI OFFSET(0)  NUMBITS(24),
        A  OFFSET(31) NUMBITS(1)
    ]
];

pub struct DoorbellEq {
    pub val: WriteOnly<u32, DoorbellEqField::Register>,
    _reserved1: u32,
}

register_structs! {
    pub DoorbellPage {
    (0x000 => _reserved1),
    (0x014 => pub send_queue_number: WriteOnly<u32, SendQueueNumber::Register>),
    (0x018 => _reserved2),

    // CQ
    /// contains the sequence number, the command and the cq number
    (0x020 => pub cq_sn_cmd_num: WriteOnly<u32, CpSnCmdNum::Register>),
    (0x024 => pub cq_consumer_index: WriteOnly<u32, CpConsumerIndex::Register>),

    // skip 502 u32
    (0x028 => _padding4),

    // EQ
    // for the EQ number n the relevant doorbell is in
    // DoorbellPage (n / 4) and eq (n % 4)
    (0x800 => pub eqs: [DoorbellEq; 4]),

    // skip 503 u32
    (0x820 => _padding9),
    (0x1000 => @END),
    }
}

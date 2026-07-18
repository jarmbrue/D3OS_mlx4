#![allow(non_camel_case_types)]

use num_enum::TryFromPrimitive;

use super::ib_core::*;

#[repr(u8)]
#[derive(Debug, Copy, Clone, TryFromPrimitive)]
pub enum UverbsInnerCmd {
    // Completion queue operations
    CreateCq       = 1,
    DestroyCq      = 2,
    PollCq         = 3,

    // Queue pair operations
    CreateQp       = 4,
    ModifyQp       = 5,
    QueryQp        = 6,
    DestroyQp      = 7,
    OpPostSend     = 8,
    OpPostRecv     = 9,

    // Memory region operations
    RegMr          = 10,
    DeregMr        = 11,
    SetMrSize      = 12,

    // Query
    QueryDevice    = 13,
    QueryPort      = 14,
    QueryDevices   = 15,

    // Kernel-bypass fast path setup: map a QP's/CQ's ring buffer, doorbell
    // record(s) and UAR doorbell/BlueFlame page(s) into the calling
    // process, once, right after CreateQp/CreateCq. post_send/post_recv/
    // poll_cq then operate directly on that mapped memory, without a
    // syscall.
    MmapQp         = 16,
    MmapCq         = 17,
}

type MagicHeader  = u16;
type CommandSize  = u16;
type MinorPresent = u8;
type CommandNum   = u8;

pub enum UverbsCmd {
    Call(UverbsInnerCmd, CommandNum, CommandSize, MagicHeader, MinorPresent)
}

// Define bit widths for each field
pub const UVERBS_CMD_BITS: u64 = 8;
pub const UVERBS_NR_BITS: u64 = 8;
pub const UVERBS_SIZE_BITS: u64 = 16;
pub const UVERBS_MAGIC_BITS: u64 = 16;
pub const UVERBS_MINOR_PRESENT_BITS: u64 = 1;

// Masks
pub const UVERBS_CMD_MASK: u64 = (1 << UVERBS_CMD_BITS) - 1;
pub const UVERBS_NR_MASK: u64 = (1 << UVERBS_NR_BITS) - 1;
pub const UVERBS_SIZE_MASK: u64 = (1 << UVERBS_SIZE_BITS) - 1;
pub const UVERBS_MAGIC_MASK: u64 = (1 << UVERBS_MAGIC_BITS) - 1;
pub const UVERBS_MINOR_PRESENT_MASK: u64 = (1 << UVERBS_MINOR_PRESENT_BITS) - 1;

pub const UVERBS_SIZE_SHIFT_IN_PLACE: u64 = UVERBS_CMD_BITS + UVERBS_NR_BITS;
pub const UVERBS_SIZE_MASK_IN_PLACE: u64 = 0xFFFF << UVERBS_SIZE_SHIFT_IN_PLACE;

pub const UVERBS_MAGIC: u16 = 0xABCD;
pub const UVERBS_MINOR_NOT_PRESENT: u8 = 0;
pub const UVERBS_MINOR_PRESENT: u8 = 1;

pub const UVERBS_MAX_USER_TRUST_SIZE: usize = 0x06400000; // allow user space to allocate up to 100MB
pub const UVERBS_MAX_USER_WC_REQ: usize = 16000;
pub const UVERBS_MAX_QUERY_DEVICES_REQ: usize = 10;

/// Maximum number of scatter/gather elements a single POD wire-format work
/// request ([`ibv_send_wr_uapi`]/[`ibv_recv_wr_uapi`]) can carry. Every
/// consumer in this repo (`os/application/rdma/mlx4`,
/// `os/application/perftest`) requests `max_send_sge`/`max_recv_sge == 1`;
/// this generous fixed bound costs nothing in practice while comfortably
/// covering the driver's own negotiated caps
/// (`WorkQueue::new_send_queue`/`new_receive_queue`,
/// `os/kernel/src/device/mlx4/queue_pair.rs`), which are bounded further by
/// the HCA's max WQE size (`hca_caps.max_desc_sz_sq()`) well below this.
pub const UVERBS_MAX_SGE: usize = 32;

const CHAR_BUF: &[u8] = &[0u8; 64];

pub const UVERBS_CMD_QUERY_DEVICES: usize = UverbsCmd::Call(UverbsInnerCmd::QueryDevices, 1, 0, UVERBS_MAGIC, UVERBS_MINOR_NOT_PRESENT).encode();
pub const UVERBS_CMD_QUERY_DEVICE: usize = UverbsCmd::Call(UverbsInnerCmd::QueryDevice, 2, size_of::<ibv_device_attr_container>() as u16, UVERBS_MAGIC, UVERBS_MINOR_PRESENT).encode();
pub const UVERBS_CMD_QUERY_PORT: usize = UverbsCmd::Call(UverbsInnerCmd::QueryPort, 3, size_of::<ibv_port_attr_container>() as u16, UVERBS_MAGIC, UVERBS_MINOR_PRESENT).encode();
pub const UVERBS_CMD_REGISTER_MR: usize = UverbsCmd::Call(UverbsInnerCmd::RegMr, 4, size_of::<ibv_mr_container>() as u16, UVERBS_MAGIC, UVERBS_MINOR_PRESENT).encode();
pub const UVERBS_CMD_SET_MR_SIZE: usize = UverbsCmd::Call(UverbsInnerCmd::SetMrSize, 5, size_of::<usize>() as u16, UVERBS_MAGIC, UVERBS_MINOR_PRESENT).encode();
pub const UVERBS_CMD_CREATE_CQ: usize = UverbsCmd::Call(UverbsInnerCmd::CreateCq, 6, size_of::<ibv_cq_container>() as u16, UVERBS_MAGIC, UVERBS_MINOR_PRESENT).encode();
pub const UVERBS_CMD_CREATE_QP: usize = UverbsCmd::Call(UverbsInnerCmd::CreateQp, 7, size_of::<ibv_qp_container>() as u16, UVERBS_MAGIC, UVERBS_MINOR_PRESENT).encode();
pub const UVERBS_CMD_MODIFY_QP: usize = UverbsCmd::Call(UverbsInnerCmd::ModifyQp, 8, size_of::<ibv_qp_modify_container>() as u16, UVERBS_MAGIC, UVERBS_MINOR_PRESENT).encode();
pub const UVERBS_CMD_POLL_CQ: usize = UverbsCmd::Call(UverbsInnerCmd::PollCq, 9, size_of::<ibv_cq_poll_container>() as u16, UVERBS_MAGIC, UVERBS_MINOR_PRESENT).encode();
pub const UVERBS_CMD_POST_SEND: usize = UverbsCmd::Call(UverbsInnerCmd::OpPostSend, 10, size_of::<ibv_qp_post_send_container>() as u16, UVERBS_MAGIC, UVERBS_MINOR_PRESENT).encode();
pub const UVERBS_CMD_POST_RECV: usize = UverbsCmd::Call(UverbsInnerCmd::OpPostRecv, 11, size_of::<ibv_qp_post_recv_container>() as u16, UVERBS_MAGIC, UVERBS_MINOR_PRESENT).encode();
pub const UVERBS_CMD_DESTROY_CQ: usize = UverbsCmd::Call(UverbsInnerCmd::DestroyCq, 12, 0, UVERBS_MAGIC, UVERBS_MINOR_PRESENT).encode();
pub const UVERBS_CMD_DESTROY_QP: usize = UverbsCmd::Call(UverbsInnerCmd::DestroyQp, 13, 0, UVERBS_MAGIC, UVERBS_MINOR_PRESENT).encode();
pub const UVERBS_CMD_DEREGISTER_MR: usize = UverbsCmd::Call(UverbsInnerCmd::DeregMr, 14, 0, UVERBS_MAGIC, UVERBS_MINOR_PRESENT).encode();
pub const UVERBS_CMD_MMAP_QP: usize = UverbsCmd::Call(UverbsInnerCmd::MmapQp, 15, size_of::<ibv_qp_mmap_container>() as u16, UVERBS_MAGIC, UVERBS_MINOR_PRESENT).encode();
pub const UVERBS_CMD_MMAP_CQ: usize = UverbsCmd::Call(UverbsInnerCmd::MmapCq, 16, size_of::<ibv_cq_mmap_container>() as u16, UVERBS_MAGIC, UVERBS_MINOR_PRESENT).encode();

#[macro_export]
macro_rules! UVERBS_CMD_SIZE {
    ($cmd:expr) => {
        (($cmd & $crate::uverbs_uapi::UVERBS_SIZE_MASK_IN_PLACE) >> $crate::uverbs_uapi::UVERBS_SIZE_SHIFT_IN_PLACE) as usize
    };
}

type UverbsCmdEnc = usize;
type UverbsCmdSupportedSize = usize;

pub trait TypeSize {
    const S: usize;
}

impl TypeSize for ibv_device_attr {
    const S: usize = CHAR_BUF.len();
}

impl TypeSize for ibv_mr_container {
    const S: usize = UVERBS_MAX_USER_TRUST_SIZE;
}

impl TypeSize for ibv_cq_poll_container {
    const S: usize = size_of::<ibv_wc>() * UVERBS_MAX_USER_WC_REQ;
}

#[repr(C)]
pub struct ibv_device_attr_container {
    pub fw_ver: [u8; ibv_device_attr::S],
    pub phys_port_cnt: u8
}

#[repr(C)]
#[derive(Default, Clone, Copy)]
pub struct ibv_port_attr_container {
    pub ibv_port_attr: ibv_port_attr,
    pub port_num: u8
}

#[repr(C)]
#[derive(Default, Clone, Copy)]
pub struct ibv_mr_container {
    pub ibv_access_flags: ibv_access_flags,
    pub data_ptr: *mut u8,
    pub len: usize,
    pub ibv_mr_res: ibv_mr_res
}

#[repr(C)]
#[derive(Default, Clone, Copy)]
pub struct ibv_mr_res {
    pub index: u32,
    pub addr: usize,
    pub lkey: u32,
    pub rkey: u32
}

#[repr(C)]
#[derive(Default, Clone, Copy)]
pub struct ibv_cq_container {
    pub cq_entries: i32,
    pub cq_num: u32
}

#[repr(C)]
#[derive(Default, Clone, Copy)]
pub struct ibv_cq_poll_container {
    pub wc: *mut ibv_wc,
    pub wc_len: usize,
    pub cq_num: u32,
    /// Extension 1 (`docs/thesis-plan-1-3.md`): if true and no completion
    /// is immediately available, `uverbs_ctl` genuinely blocks the calling
    /// thread off the CPU until the mlx4 interrupt handler observes a
    /// completion for this CQ (or forever, if none ever arrives - the
    /// caller decides how long to keep calling, same contract as real
    /// `ibv_poll_cq`).
    ///
    /// This is an implicit blocking-mode flag on the existing
    /// `UVERBS_CMD_POLL_CQ` command rather than a new syscall/
    /// `UverbsInnerCmd` variant: real ibverbs signals "block for a
    /// completion" via a separate completion channel + `ibv_get_cq_event`,
    /// which doesn't exist in `os/library/ibverbs` yet and would be a much
    /// larger addition than this extension needs. Growing this
    /// already-existing, single-purpose container by one `bool` field
    /// keeps the `SystemCall`/`UverbsInnerCmd` enum-variant-order contract
    /// (`docs/new-syscall.howto.md`) untouched - both sides of the syscall
    /// boundary already share this exact struct via the `rdma` crate, so
    /// `size_of::<ibv_cq_poll_container>()` (and therefore
    /// `UVERBS_CMD_POLL_CQ`'s encoded size) stays in sync automatically on
    /// a rebuild, with nothing to update by hand.
    pub blocking: bool,
}

/// Fully POD wire-format representation of a single send work request, used
/// only for the `UVERBS_CMD_POST_SEND` syscall boundary - distinct from the
/// heap-allocated, linked-list `ibv_send_wr` (`Vec<ibv_sge>` + `next: *mut
/// Self`) used by the higher-level `os/library/ibverbs` API and the
/// kernel-bypass fast path, which never crosses the syscall boundary and so
/// doesn't need to be POD. `sg_list` is a fixed-capacity array instead of a
/// `Vec` (bounded by [`UVERBS_MAX_SGE`]), and there is no `next` pointer -
/// WQE chains are walked in userspace instead
/// (`os/library/ibverbs/src/ibverbs_sys.rs`'s `ibv_post_send`), issuing one
/// `Uverb` syscall per work request rather than one per chain.
#[repr(C)]
#[derive(Clone, Copy)]
pub struct ibv_send_wr_uapi {
    pub wr_id: u64,
    pub sg_list: [ibv_sge; UVERBS_MAX_SGE],
    pub num_sge: u32,
    pub opcode: ibv_wr_opcode,
    pub send_flags: ibv_send_flags,
    pub wr: ibv_send_wr_wr,
}

impl Default for ibv_send_wr_uapi {
    fn default() -> Self {
        Self {
            wr_id: 0,
            sg_list: [ibv_sge::default(); UVERBS_MAX_SGE],
            num_sge: 0,
            opcode: ibv_wr_opcode::IBV_WR_SEND,
            send_flags: ibv_send_flags::empty(),
            wr: Default::default(),
        }
    }
}

/// Fully POD wire-format representation of a single receive work request.
/// See [`ibv_send_wr_uapi`] for the general rationale.
#[repr(C)]
#[derive(Clone, Copy)]
pub struct ibv_recv_wr_uapi {
    pub wr_id: u64,
    pub sg_list: [ibv_sge; UVERBS_MAX_SGE],
    pub num_sge: u32,
}

impl Default for ibv_recv_wr_uapi {
    fn default() -> Self {
        Self { wr_id: 0, sg_list: [ibv_sge::default(); UVERBS_MAX_SGE], num_sge: 0 }
    }
}

#[repr(C)]
#[derive(Clone, Copy)]
pub struct ibv_qp_container {
    pub qp_type: ibv_qp_type::Type,
    pub send_cq_num: u32,
    pub recv_cq_num: u32,
    pub ib_caps: ibv_qp_cap,
    pub qp_num: u32
}

impl Default for ibv_qp_container {
    fn default() -> Self {
        Self { 
            qp_type: ibv_qp_type::IBV_QPT_RC, // just place holder
            send_cq_num: Default::default(), 
            recv_cq_num: Default::default(), 
            ib_caps: Default::default(), 
            qp_num: Default::default() 
        }
    }
}

#[repr(C)]
#[derive(Clone, Copy)]
pub struct ibv_qp_modify_container {
    pub qp_num: u32,
    pub attr: ibv_qp_attr,
    pub attr_mask: ibv_qp_attr_mask
}

impl Default for ibv_qp_modify_container {
    fn default() -> Self {
        Self { 
            qp_num: Default::default(), 
            attr: Default::default(), 
            attr_mask: ibv_qp_attr_mask::IBV_QP_PORT // just place holder
        }
    }
}

impl Default for ibv_send_wr {
    fn default() -> Self {
        Self { 
            wr_id: Default::default(), 
            next: Default::default(), 
            sg_list: Default::default(), 
            num_sge: Default::default(), 
            opcode: ibv_wr_opcode::IBV_WR_SEND, 
            send_flags: ibv_send_flags::SIGNALED, 
            __bindgen_anon_1: Default::default(), 
            wr: Default::default(), 
            qp_type: Default::default(), 
            __bindgen_anon_2: Default::default() }
    }
}

impl Default for ibv_recv_wr {
    fn default() -> Self {
        Self { 
            wr_id: Default::default(), 
            next: Default::default(), 
            sg_list: Default::default(), 
            num_sge: Default::default() 
        }
    }
}

#[repr(C)]
#[derive(Default, Clone, Copy)]
pub struct ibv_qp_post_send_container {
    pub wr: ibv_send_wr_uapi,
    pub qp_num: u32
}

#[repr(C)]
#[derive(Default, Clone, Copy)]
pub struct ibv_qp_post_recv_container {
    pub wr: ibv_recv_wr_uapi,
    pub qp_num: u32
}

/// Kernel-bypass fast path setup for a queue pair: on input, `qp_num`
/// identifies the QP; on output, the remaining fields describe the memory
/// regions the kernel has just mapped into the calling process (all
/// addresses/lengths are in the *calling process's* user address space) and
/// the ring geometry needed to interpret them, mirroring what
/// `WorkQueue::new_send_queue`/`new_receive_queue` compute kernel-side from
/// HCA capabilities userspace cannot otherwise observe.
#[repr(C)]
#[derive(Default, Clone, Copy)]
pub struct ibv_qp_mmap_container {
    pub qp_num: u32,
    /// Combined SQ+RQ ring buffer.
    pub ring_buf_addr: usize,
    pub ring_buf_len: usize,
    pub sq_offset: u32,
    pub sq_wqe_cnt: u32,
    pub sq_wqe_shift: u32,
    pub sq_spare_wqes: u32,
    pub sq_max_gs: u32,
    pub sq_max_post: u32,
    pub rq_offset: u32,
    pub rq_wqe_cnt: u32,
    pub rq_wqe_shift: u32,
    pub rq_max_gs: u32,
    pub rq_max_post: u32,
    /// Per-QP doorbell record (DMA host memory, cacheable).
    pub qp_doorbell_addr: usize,
    /// UAR doorbell MMIO page for this QP (uncacheable).
    pub uar_doorbell_addr: usize,
    /// BlueFlame MMIO page for this QP, if the HCA supports it; `bf_len`
    /// is 0 otherwise.
    pub bf_addr: usize,
    pub bf_len: usize,
    pub bf_reg_size: u32,
}

/// Kernel-bypass fast path setup for a completion queue. Same shape as
/// [`ibv_qp_mmap_container`], see its docs for the general pattern.
#[repr(C)]
#[derive(Default, Clone, Copy)]
pub struct ibv_cq_mmap_container {
    pub cq_num: u32,
    pub cqe_ring_addr: usize,
    pub cqe_ring_len: usize,
    pub num_entries: u32,
    /// Per-CQ doorbell record (DMA host memory, cacheable).
    pub cq_doorbell_addr: usize,
}

impl From<(u32, usize, u32, u32)> for ibv_mr_res {
    fn from(value: (u32, usize, u32, u32)) -> Self {
        ibv_mr_res { index: value.0, addr: value.1, lkey: value.2, rkey: value.3 }
    }
}

impl UverbsCmd {
    /// Encode into a single u64
    pub const fn encode(&self) -> usize {
        match self {
            UverbsCmd::Call(cmd, seq, size, magic, minor) => {
                let inner_cmd = *cmd as u64 & UVERBS_CMD_MASK;
                let cmd_num = *seq as u64 & UVERBS_NR_MASK;
                let size_u64 = *size as u64 & UVERBS_SIZE_MASK;
                let magic_u64 = *magic as u64 & UVERBS_MAGIC_MASK;
                let minor_u64 = *minor as u64 & UVERBS_MINOR_PRESENT_MASK;

                ((minor_u64 << (UVERBS_SIZE_BITS + UVERBS_NR_BITS + UVERBS_CMD_BITS + UVERBS_MAGIC_BITS)) |
                (magic_u64 << (UVERBS_SIZE_BITS + UVERBS_NR_BITS + UVERBS_CMD_BITS)) |
                (size_u64 << (UVERBS_NR_BITS + UVERBS_CMD_BITS)) |
                (cmd_num << UVERBS_CMD_BITS) |
                inner_cmd) as usize
            }
        }
    }

    /// Decode from u64
    pub fn decode(encoded: u64) -> Self {
        let cmd_num = (encoded & UVERBS_CMD_MASK) as u8;
        let seq = ((encoded >> UVERBS_CMD_BITS) & UVERBS_NR_MASK) as u8;
        let size = ((encoded >> (UVERBS_CMD_BITS + UVERBS_NR_BITS)) & UVERBS_SIZE_MASK) as u16;
        let magic = ((encoded >> (UVERBS_CMD_BITS + UVERBS_NR_BITS + UVERBS_SIZE_BITS)) & UVERBS_MAGIC_MASK) as u16;
        let minor = ((encoded >> (UVERBS_CMD_BITS + UVERBS_NR_BITS + UVERBS_SIZE_BITS + UVERBS_MAGIC_BITS)) & UVERBS_MINOR_PRESENT_MASK) as u8;

        UverbsCmd::Call(UverbsInnerCmd::try_from(cmd_num).unwrap(), seq, size, magic, minor)
    }

    /// Accessors
    pub fn cmd(&self) -> UverbsInnerCmd {
        match self {
            UverbsCmd::Call(cmd, _, _, _, _) => *cmd,
        }
    }

    pub fn seq(&self) -> u8 {
        match self {
            UverbsCmd::Call(_, seq, _, _, _) => *seq,
        }
    }

    pub fn size(&self) -> u16 {
        match self {
            UverbsCmd::Call(_, _, size, _, _) => *size,
        }
    }

    pub fn magic(&self) -> u16 {
        match self {
            UverbsCmd::Call(_, _, _, magic, _) => *magic,
        }
    }

    pub fn decompose(&self) -> (UverbsInnerCmd, u8, u16, u16, u8) {
        match self {
            UverbsCmd::Call(cmd, seq, size, magic, minor) => (*cmd, *seq, *size, *magic, *minor),
        }
    }
}

/// Accessors work from the encoded value
pub fn cmd(encoded: u64) -> UverbsInnerCmd {
    let cmd_num = (encoded & UVERBS_CMD_MASK) as u8;
    UverbsInnerCmd::try_from(cmd_num).unwrap()
}

pub fn seq(encoded: u64) -> u8 {
    ((encoded >> UVERBS_CMD_BITS) & UVERBS_NR_MASK) as u8
}

pub fn size(encoded: u64) -> u32 {
    ((encoded >> (UVERBS_NR_BITS + UVERBS_CMD_BITS)) & UVERBS_SIZE_MASK) as u32
}

pub fn magic(encoded: u64) -> u16 {
    ((encoded >> (UVERBS_SIZE_BITS + UVERBS_NR_BITS + UVERBS_CMD_BITS)) & UVERBS_MAGIC_MASK) as u16
}

pub fn minor_present(encoded: u64) -> u8 {
    ((encoded >> (UVERBS_SIZE_BITS + UVERBS_NR_BITS + UVERBS_CMD_BITS + UVERBS_MINOR_PRESENT_BITS)) & UVERBS_MINOR_PRESENT_MASK) as u8
}
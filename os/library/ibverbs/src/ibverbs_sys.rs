//! This crate is a replacement for rdma-core on Linux.
//!
//! The struct definitions are partly taken from the rust-bindgen output.
#![allow(non_camel_case_types)]

extern crate alloc;

use alloc::{boxed::Box, collections::BTreeMap, rc::Rc, string::{String, ToString}, vec, vec::Vec};
use core::cell::RefCell;
use core::mem::size_of;
use core::sync::atomic::{compiler_fence, Ordering};
use core3::io::{Error, ErrorKind, Result as Result};
pub use rdma::{
    __be64, ibv_access_flags, ibv_ah_attr, ibv_device_attr, ibv_gid, ibv_mtu,
    ibv_port_attr, ibv_port_state,
    ibv_qp_attr, ibv_qp_attr_mask, ibv_qp_cap, ibv_qp_state, ibv_qp_type,
    ibv_recv_wr, ibv_send_wr, ibv_send_wr_wr, ibv_send_flags, ibv_sge,
    ibv_wr_opcode, ibv_wc, ibv_wc_flags, ibv_wc_opcode, ibv_wc_status,
};
pub(crate) use rdma::ibv_device;
use rdma::mlx4_hw::{
    CompletionQueueDoorbell, CompletionQueueEntry, DoorbellPage, QueuePairDoorbell, QueuePairOpcode, ReceiveOpcode, Syndrome, WqeControlSegment,
    WqeControlSegmentFlags, WqeDataSegment, WqeRemoteAddressSegment,
};
use syscall::{syscall, SystemCall::Uverb};
use tock_registers::interfaces::Writeable;
use rdma::uverbs_uapi::{UVERBS_CMD_CREATE_CQ, UVERBS_CMD_CREATE_QP, UVERBS_CMD_DEREGISTER_MR, UVERBS_CMD_DESTROY_CQ, UVERBS_CMD_DESTROY_QP, UVERBS_CMD_MMAP_CQ, UVERBS_CMD_MMAP_QP, UVERBS_CMD_MODIFY_QP, UVERBS_CMD_POLL_CQ, UVERBS_CMD_POST_RECV, UVERBS_CMD_POST_SEND, UVERBS_CMD_QUERY_DEVICE, UVERBS_CMD_QUERY_DEVICES, UVERBS_CMD_QUERY_PORT, UVERBS_CMD_REGISTER_MR, ibv_cq_container, ibv_cq_mmap_container, ibv_cq_poll_container, ibv_mr_container, ibv_mr_res, ibv_port_attr_container, ibv_qp_container, ibv_qp_mmap_container, ibv_qp_modify_container, ibv_qp_post_recv_container, ibv_qp_post_send_container, ibv_recv_wr_uapi, ibv_send_wr_uapi, UVERBS_MAX_QUERY_DEVICES_REQ, UVERBS_MAX_SGE};

pub struct ibv_context_ops {
    pub poll_cq: Option<fn(
        &ibv_cq<'_>, &mut [ibv_wc],
    ) -> Result<i32>>,
    /// This is unsafe because the sges contain raw addresses.
    // TODO: figure out a way to return the bad wr
    pub post_send: Option<unsafe fn(
        &mut ibv_qp<'_, '_>, &mut ibv_send_wr,
    ) -> Result<()>>,
    /// This is unsafe because the sges contain raw addresses.
    // TODO: figure out a way to return the bad wr
    pub post_recv: Option<unsafe fn(
        &mut ibv_qp<'_, '_>, &mut ibv_recv_wr,
    ) -> Result<()>>,
}

// Default build: zero-syscall data plane (post_send/post_recv/poll_cq
// operate on memory mapped once at create_qp/create_cq time - see the
// "Kernel-bypass fast path" section below). Build with
// `--no-default-features` to fall back to the original syscall-based
// implementations (`ibv_poll_cq`/`ibv_post_send`/`ibv_post_recv`), e.g. as
// a known-good reference during real-hardware bring-up.
#[cfg(feature = "fastpath-verbs")]
const IBV_CONTEXT_OPS: ibv_context_ops = ibv_context_ops {
    poll_cq: Some(fastpath_poll_cq),
    post_send: Some(fastpath_post_send),
    post_recv: Some(fastpath_post_recv),
};

#[cfg(not(feature = "fastpath-verbs"))]
const IBV_CONTEXT_OPS: ibv_context_ops = ibv_context_ops {
    poll_cq: Some(ibv_poll_cq),
    post_send: Some(ibv_post_send),
    post_recv: Some(ibv_post_recv),
};

pub struct ibv_context {
    pub ops: ibv_context_ops,
    nic: usize,
    /// Fast-path QP ring state, keyed by qp_num, shared with each `ibv_qp`'s
    /// own `ring_state` handle. `fastpath_poll_cq` only receives `&ibv_cq`,
    /// not the posting `ibv_qp`, so it reaches the right QP's state through
    /// here. Assumes a given QP is only ever posted/polled from one thread
    /// within this process - see the fast-path notes on [`QpRingState`].
    qp_registry: RefCell<BTreeMap<u32, Rc<RefCell<QpRingState>>>>,
    /// Registered memory regions' (physical base, virtual base), keyed by
    /// lkey, so the fast path can translate an `ibv_sge`'s virtual address
    /// to a physical one without a syscall. See [`fastpath_translate_sge`].
    mr_registry: RefCell<BTreeMap<u32, (u64, u64)>>,
}

impl ibv_context {
    /// Get access to the underlying device fd.
    fn lock(&self) -> usize {
        self.nic
    }
}

pub struct ibv_cq<'ctx> {
    context: &'ctx ibv_context,
    number: u32,
    /// Consumer-supplied context returned for completion events
    _cq_context: isize,
    /// Fast-path CQE ring state, mapped once at `ibv_create_cq` time.
    cq_state: RefCell<CqRingState>,
}

impl Drop for ibv_cq<'_> {
    fn drop(&mut self) {
        let dev_fd = self.context.lock();

        syscall(Uverb, &[
            dev_fd,
            UVERBS_CMD_DESTROY_CQ,
            self.number as usize
        ]).expect("failed to destroy completion queue");
    }
}

pub struct ibv_mr<'pd> {
    pd: &'pd ibv_pd<'pd>,
    index: u32,
    /// physical address
    pub addr: usize,
    pub length: usize,
    pub lkey: u32,
    pub rkey: u32,
}

impl Drop for ibv_mr<'_> {
    fn drop(&mut self) {
        let dev_fd = self.pd.context.lock();

        self.pd.context.mr_registry.borrow_mut().remove(&self.lkey);

        syscall(Uverb, &[
            dev_fd,
            UVERBS_CMD_DEREGISTER_MR,
            self.index as usize
        ]).expect("failed to destroy memory region");
    }
}

pub struct ibv_pd<'ctx> {
    context: &'ctx ibv_context,
}

// pub struct ibv_srq {}

pub struct ibv_qp<'ctx, 'cq> {
    pub ops: &'ctx ibv_context_ops,
    pub qp_num: u32,
    send_cq: &'cq ibv_cq<'ctx>,
    recv_cq: &'cq ibv_cq<'ctx>,
    /// Fast-path ring state, mapped once at `ibv_create_qp` time. Also
    /// registered in `send_cq.context.qp_registry` under the same `Rc`, so
    /// `fastpath_poll_cq` can reach it too.
    ring_state: Rc<RefCell<QpRingState>>,
}

impl Drop for ibv_qp<'_, '_> {
    fn drop(&mut self) {
        let dev_fd = self.send_cq.context.lock();

        self.send_cq.context.qp_registry.borrow_mut().remove(&self.qp_num);

        syscall(Uverb, &[
            dev_fd,
            UVERBS_CMD_DESTROY_QP,
            self.qp_num as usize
        ]).expect("failed to destroy queue pair");
    }
}

#[allow(dead_code)]
pub struct ibv_qp_init_attr<'cq, 'ctx> {
    pub qp_context: isize,
    pub send_cq: &'cq ibv_cq<'ctx>,
    pub recv_cq: &'cq ibv_cq<'ctx>,
    pub srq: Option<()>,
    pub cap: ibv_qp_cap,
    pub qp_type: ibv_qp_type::Type,
    pub sq_sig_all: i32,
}

/// Get list of IB devices currently available
///
/// Return a array of IB devices.
pub fn ibv_get_device_list() -> Result<Vec<ibv_device>> {
    let mut devices : Vec<ibv_device> = Vec::with_capacity(UVERBS_MAX_QUERY_DEVICES_REQ);
    syscall(Uverb, &[0, UVERBS_CMD_QUERY_DEVICES, devices.as_mut_ptr() as usize])
        .map(|count| {
            unsafe { devices.set_len(count) };
            devices
        })
        .map_err(|_| todo!())
}

/// Return kernel device name
pub fn ibv_get_device_name(_device: &ibv_device) -> Option<String> {
    // TODO: don't hardcode this
    Some("mlx4_todo".to_string())
}

/// Return kernel device index
///
/// Available for the kernel with support of IB device query
/// over netlink interface. For the unsupported kernels, the
/// relevant error will be returned.
pub fn ibv_get_device_index(device: &ibv_device) -> Result<i32> {
    device.nic.try_into()
        .map_err(|x| Error::from(ErrorKind::InvalidData))
}

/// Return device's node GUID
pub fn ibv_get_device_guid(_device: &ibv_device) -> Result<__be64> {
    todo!()
}


/// Initialize device for use
pub fn ibv_open_device(device: &ibv_device) -> Result<ibv_context> {
    Ok(ibv_context {
        nic: device.nic,
        ops: IBV_CONTEXT_OPS,
        qp_registry: RefCell::new(BTreeMap::new()),
        mr_registry: RefCell::new(BTreeMap::new()),
    })
}

/// Get device properties
pub fn ibv_query_device(context: &ibv_context) -> Result<ibv_device_attr> {
    let dev_fd = context.lock();

    let device_attr = ibv_device_attr::default();

    match syscall(Uverb, &[
        dev_fd,
        UVERBS_CMD_QUERY_DEVICE,
        &device_attr as *const _ as usize,
        ]) {
        Ok(_) => Ok(device_attr),
        Err(_) => Err(Error::from(ErrorKind::Other))
    }
}

/// Get port properties
pub fn ibv_query_port(
    context: &ibv_context, port_num: u8,
) -> Result<ibv_port_attr> {
    let dev_fd = context.lock();

    let ibv_port_container = ibv_port_attr_container {
        ibv_port_attr: Default::default(),
        port_num
    };

    match syscall(Uverb, &[
            dev_fd,
            UVERBS_CMD_QUERY_PORT,
            ((&ibv_port_container) as *const ibv_port_attr_container).addr()
        ]) {
        Ok(_) => Ok(ibv_port_container.ibv_port_attr),
        Err(_) => Err(Error::from(ErrorKind::Other))
    }
}

/// Get a GID table entry
pub fn ibv_query_gid(
    _context: &ibv_context, _port_num: u8, _index: i32,
) -> Result<ibv_gid> {
    // TODO: figure out how to actually do this as the Nautilus driver can't
    Ok(ibv_gid { raw: [0; 16] })
}

/// Allocate a protection domain
///
/// This is currently just a stub.
pub fn ibv_alloc_pd(context: &ibv_context) -> Result<ibv_pd<'_>> {
    // TODO: figure out how to actually do this as the Nautilus driver has no
    // concept of protection domains
    Ok(ibv_pd { context })
}

/// Register a memory region
pub fn ibv_reg_mr<'pd, T>(
    pd: &'pd ibv_pd<'_>, data: &mut [T], access: ibv_access_flags,
) -> Result<ibv_mr<'pd>> {
    let data_u8 = data.as_mut_ptr().cast::<u8>();
    let virt_addr = data_u8 as usize as u64;

    let dev_fd = pd.context.lock();

    let ibv_mr_container = ibv_mr_container {
        ibv_access_flags: access,
        data_ptr: data_u8,
        len: data.len(),
        ibv_mr_res: ibv_mr_res {
            index: Default::default(),
            addr:  Default::default(),
            lkey:  Default::default(),
            rkey:  Default::default()
        }
    };

    match syscall(Uverb, &[
        dev_fd,
        UVERBS_CMD_REGISTER_MR,
        (&ibv_mr_container as *const ibv_mr_container).addr()
    ]) {
        Ok(_) => {
            let ibv_mr_res { index, addr, lkey, rkey } = ibv_mr_container.ibv_mr_res;
            let length = data.len();

            // Record the (physical base, virtual base) pair so the fast
            // path can translate an ibv_sge's virtual address to a
            // physical one without a syscall - see
            // `fastpath_translate_sge`.
            pd.context.mr_registry.borrow_mut().insert(lkey, (addr as u64, virt_addr));

            Ok(ibv_mr { pd, index, addr, length, lkey, rkey })
        },
        Err(_) => Err(Error::from(ErrorKind::Other))
    }
}

/// Create a completion queue
///
/// @context - Context CQ will be attached to
/// @cqe - Minimum number of entries required for CQ
/// @cq_context - Consumer-supplied context returned for completion events
/// @channel - Completion channel where completion events will be queued.
///     May be NULL if completion events will not be used.
/// @comp_vector - Completion vector used to signal completion events.
///     Must be >= 0 and < context->num_comp_vectors.
pub fn ibv_create_cq(
    context: &ibv_context, cqe: i32, cq_context: isize,
    channel: Option<()>, comp_vector: i32,
) -> Result<ibv_cq<'_>> {
    assert!(channel.is_none());
    assert_eq!(comp_vector, 0);

    let dev_fd = context.lock();

    let ibv_cq_container = ibv_cq_container {
        cq_entries: cqe,
        cq_num: Default::default()
    };

    match syscall(Uverb, &[
        dev_fd,
        UVERBS_CMD_CREATE_CQ,
        (&ibv_cq_container as *const ibv_cq_container).addr()
    ]) {
        Ok(_) => {
            // Map this CQ's CQE ring buffer and doorbell record into our
            // address space, once, so fastpath_poll_cq never needs a
            // syscall afterwards. Done unconditionally (even if the
            // fast-path vtable isn't selected) so switching paths never
            // needs a second setup pass.
            let mmap_container = ibv_cq_mmap_container { cq_num: ibv_cq_container.cq_num, ..Default::default() };
            syscall(Uverb, &[
                dev_fd,
                UVERBS_CMD_MMAP_CQ,
                (&mmap_container as *const ibv_cq_mmap_container).addr()
            ]).map_err(|_| Error::from(ErrorKind::Other))?;

            let cq_state = RefCell::new(CqRingState {
                cqe_ring_addr: mmap_container.cqe_ring_addr,
                num_entries: mmap_container.num_entries,
                cq_doorbell_addr: mmap_container.cq_doorbell_addr,
                consumer_index: 0,
            });

            Ok( ibv_cq { context, number: ibv_cq_container.cq_num, _cq_context: cq_context, cq_state } )
        },
        Err(_) => Err(Error::from(ErrorKind::Other))
    }
}

/// Create a queue pair.
pub fn ibv_create_qp<'ctx, 'cq>(
    pd: &'ctx ibv_pd<'_>, qp_init_attr: &mut ibv_qp_init_attr<'cq, 'ctx>,
) -> Result<ibv_qp<'ctx, 'cq>> {
    let send_cq = qp_init_attr.send_cq;
    let recv_cq = qp_init_attr.recv_cq;
    assert!(core::ptr::eq(send_cq.context, recv_cq.context));

    let dev_fd = pd.context.lock();

    let ibv_qp_container = ibv_qp_container {
        qp_type: qp_init_attr.qp_type,
        send_cq_num: send_cq.number,
        recv_cq_num: recv_cq.number,
        ib_caps: qp_init_attr.cap,
        qp_num: Default::default()
    };

    match syscall(Uverb, &[
        dev_fd,
        UVERBS_CMD_CREATE_QP,
        (&ibv_qp_container as *const ibv_qp_container).addr()
    ]) {
        Ok(_) => {
            let qp_num = ibv_qp_container.qp_num;

            // Map this QP's ring buffer, doorbell record and UAR
            // doorbell/BlueFlame page(s) into our address space, once, so
            // fastpath_post_send/fastpath_post_recv never need a syscall
            // afterwards. Done unconditionally (even if the fast-path
            // vtable isn't selected) so switching paths never needs a
            // second setup pass.
            let mmap_container = ibv_qp_mmap_container { qp_num, ..Default::default() };
            syscall(Uverb, &[
                dev_fd,
                UVERBS_CMD_MMAP_QP,
                (&mmap_container as *const ibv_qp_mmap_container).addr()
            ]).map_err(|_| Error::from(ErrorKind::Other))?;

            let sq_wqe_cnt = mmap_container.sq_wqe_cnt as usize;
            let rq_wqe_cnt = mmap_container.rq_wqe_cnt as usize;

            let ring_state = Rc::new(RefCell::new(QpRingState {
                qp_number: qp_num,
                qp_type: qp_init_attr.qp_type,
                ring_buf_addr: mmap_container.ring_buf_addr,
                sq_offset: mmap_container.sq_offset,
                sq_wqe_cnt: mmap_container.sq_wqe_cnt,
                sq_wqe_shift: mmap_container.sq_wqe_shift,
                sq_spare_wqes: mmap_container.sq_spare_wqes,
                sq_max_gs: mmap_container.sq_max_gs,
                sq_max_post: mmap_container.sq_max_post,
                rq_offset: mmap_container.rq_offset,
                rq_wqe_cnt: mmap_container.rq_wqe_cnt,
                rq_wqe_shift: mmap_container.rq_wqe_shift,
                rq_max_gs: mmap_container.rq_max_gs,
                rq_max_post: mmap_container.rq_max_post,
                qp_doorbell_addr: mmap_container.qp_doorbell_addr,
                uar_doorbell_addr: mmap_container.uar_doorbell_addr,
                bf_addr: mmap_container.bf_addr,
                bf_len: mmap_container.bf_len,
                bf_reg_size: mmap_container.bf_reg_size as usize,
                sq_head: 0,
                sq_tail: 0,
                rq_head: 0,
                rq_tail: 0,
                sq_meta: vec![(0u64, 0u32); sq_wqe_cnt],
                rq_meta: vec![(0u64, 0u32); rq_wqe_cnt],
            }));

            pd.context.qp_registry.borrow_mut().insert(qp_num, Rc::clone(&ring_state));

            Ok(ibv_qp {
                ops: &IBV_CONTEXT_OPS,
                qp_num,
                send_cq,
                recv_cq,
                ring_state,
            })
        },
        Err(_) => Err(Error::from(ErrorKind::Other))
    }

}

/// Modify a queue pair.
pub fn ibv_modify_qp(
    qp: &mut ibv_qp<'_, '_>, attr: &ibv_qp_attr, attr_mask: ibv_qp_attr_mask,
) -> Result<()> {
    let dev_fd = qp.recv_cq.context.lock();

    let ibv_qp_modify_container = ibv_qp_modify_container {
        qp_num: qp.qp_num,
        attr: *attr,
        attr_mask
    };

    match syscall(Uverb, &[
        dev_fd,
        UVERBS_CMD_MODIFY_QP,
        (&ibv_qp_modify_container as *const ibv_qp_modify_container).addr()
    ]) {
        Ok(_) => Ok(()),
        Err(_) => Err(Error::from(ErrorKind::Other))
    }
}

/// poll a completion queue (CQ)
fn ibv_poll_cq(
    cq: &ibv_cq<'_>, wc: &mut [ibv_wc],
) -> Result<i32> {
    let dev_fd = cq.context.lock();

    let ibv_cq_poll_container = ibv_cq_poll_container {
        wc: wc.as_mut_ptr(),
        wc_len: wc.len(),
        cq_num: cq.number,
    };

    match syscall(Uverb, &[
        dev_fd,
        UVERBS_CMD_POLL_CQ,
        (&ibv_cq_poll_container as *const ibv_cq_poll_container).addr()
    ]) {
        Ok(wc_count) => Ok(wc_count.try_into().unwrap()),
        Err(_) => Err(Error::from(ErrorKind::Other))
    }
}

/// Post a (possibly chained, via `wr.next`) list of work requests to a send
/// queue.
///
/// `ibv_send_wr` (the heap-allocated, `Vec<ibv_sge>` + `next: *mut Self`
/// linked-list type built by `os/library/ibverbs/src/ibverbs.rs`'s
/// `QueuePair::post_send`/`rdma_write`/`rdma_read`) is not POD, so it can't
/// be handed to the kernel as a single blob - the kernel previously
/// raw-byte-copied the `Vec`'s internal `(ptr, len, cap)` triple out of user
/// memory and dereferenced `wr.next` (a user address) directly in kernel
/// context, which was unsound independent of the missing validation. The
/// fix: walk the chain here, in userspace (safe - this is just following
/// heap pointers within our own process, the same thing `fastpath_post_send`
/// below already does), translating each `ibv_send_wr` into the fully POD
/// `ibv_send_wr_uapi` wire format and issuing one `Uverb`/`UVERBS_CMD_POST_SEND`
/// syscall per WR instead of one per chain.
///
/// This does change the chain's atomicity: previously the whole chain was
/// validated/written into the ring buffer by a single kernel call before any
/// doorbell was rung (so a failure partway through left nothing posted to
/// hardware); now each WR is fully posted - including its doorbell ring -
/// before the next one is attempted, so a failure partway through a chain
/// can leave a strict prefix of the chain already visible to the HCA. See
/// `docs/thesis-plan-1-3.md` section 3, option (a) - this is the tradeoff it
/// explicitly calls out for moving chain-walking out of the kernel.
unsafe fn ibv_post_send(
    qp: &mut ibv_qp<'_, '_>, wr: &mut ibv_send_wr,
) -> Result<()> {
    let dev_fd = qp.send_cq.context.lock();

    let mut current: Option<&mut ibv_send_wr> = Some(wr);
    while let Some(curr) = current.take() {
        if curr.sg_list.len() > UVERBS_MAX_SGE {
            return Err(Error::from(ErrorKind::Other));
        }

        let mut wire = ibv_send_wr_uapi {
            wr_id: curr.wr_id,
            num_sge: curr.sg_list.len() as u32,
            opcode: curr.opcode,
            send_flags: curr.send_flags,
            wr: curr.wr,
            ..Default::default()
        };
        for (dst, sge) in wire.sg_list.iter_mut().zip(curr.sg_list.iter()) {
            *dst = *sge;
        }

        let container = ibv_qp_post_send_container { wr: wire, qp_num: qp.qp_num };

        syscall(Uverb, &[
            dev_fd,
            UVERBS_CMD_POST_SEND,
            (&container as *const ibv_qp_post_send_container).addr()
        ]).map_err(|_| Error::from(ErrorKind::Other))?;

        current = unsafe {
            if !curr.next.is_null() {
                Some(&mut *curr.next)
            } else {
                None
            }
        };
    }
    Ok(())
}

/// Post a (possibly chained, via `wr.next`) list of work requests to a
/// receive queue. See `ibv_post_send` above for the full rationale - same
/// per-WR-syscall design, same atomicity caveat.
unsafe fn ibv_post_recv(
    qp: &mut ibv_qp<'_, '_>, wr: &mut ibv_recv_wr,
) -> Result<()> {
    let dev_fd = qp.recv_cq.context.lock();

    let mut current: Option<&mut ibv_recv_wr> = Some(wr);
    while let Some(curr) = current.take() {
        if curr.sg_list.len() > UVERBS_MAX_SGE {
            return Err(Error::from(ErrorKind::Other));
        }

        let mut wire = ibv_recv_wr_uapi {
            wr_id: curr.wr_id,
            num_sge: curr.sg_list.len() as u32,
            ..Default::default()
        };
        for (dst, sge) in wire.sg_list.iter_mut().zip(curr.sg_list.iter()) {
            *dst = *sge;
        }

        let container = ibv_qp_post_recv_container { wr: wire, qp_num: qp.qp_num };

        syscall(Uverb, &[
            dev_fd,
            UVERBS_CMD_POST_RECV,
            (&container as *const ibv_qp_post_recv_container).addr()
        ]).map_err(|_| Error::from(ErrorKind::Other))?;

        current = unsafe {
            if !curr.next.is_null() {
                Some(&mut *curr.next)
            } else {
                None
            }
        };
    }
    Ok(())
}

// ============================================================================
// Kernel-bypass fast path.
//
// post_send/post_recv/poll_cq below operate directly on memory mapped by
// UVERBS_CMD_MMAP_QP/UVERBS_CMD_MMAP_CQ (see ibv_create_qp/ibv_create_cq
// above) instead of issuing the Uverb syscall on every call. They mirror
// the kernel driver's QueuePair::post_send/post_receive and
// CompletionQueue::poll/poll_one/get_next_cqe_sw
// (os/kernel/src/device/mlx4/{queue_pair,completion_queue}.rs) field-for-
// field, using the same shared `rdma::mlx4_hw` hardware layout structs the
// kernel driver uses, so there is exactly one definition of the WQE/CQE/
// doorbell byte layouts on each side of what used to be the syscall
// boundary.
//
// Scope: RC/UC (send + RDMA read/write) only, matching every consumer in
// os/application/rdma/mlx4. UD (datagram) QPs are not supported by this
// fast path.
//
// Threading: a given QP/CQ pair is assumed to be posted/polled from a
// single thread within the owning process - matches how the current
// consumer apps use it, and lets this state use plain Rc<RefCell<...>>
// instead of Arc<spin::Mutex<...>>, avoiding synchronization overhead that
// would partially defeat the point of removing the syscall. Concurrent use
// from multiple threads is undefined behavior (a RefCell double-borrow
// panic at best).
// ============================================================================

/// Per-QP kernel-bypass fast-path state: the memory regions mapped by
/// `UVERBS_CMD_MMAP_QP`, plus locally-tracked ring indices and per-WQE
/// metadata (wr_id/chain_size) that mirror the kernel driver's
/// `WorkQueue`. The kernel keeps this per-QP bookkeeping today only
/// because one kernel driver instance serves every process; once posting
/// and polling both happen in the same process that owns the QP, it can
/// live here instead, with no kernel involvement at all.
struct QpRingState {
    qp_number: u32,
    qp_type: ibv_qp_type::Type,

    /// Base address of the combined SQ+RQ ring buffer, mapped by
    /// UVERBS_CMD_MMAP_QP.
    ring_buf_addr: usize,

    sq_offset: u32,
    sq_wqe_cnt: u32,
    sq_wqe_shift: u32,
    sq_spare_wqes: u32,
    sq_max_gs: u32,
    sq_max_post: u32,

    rq_offset: u32,
    rq_wqe_cnt: u32,
    rq_wqe_shift: u32,
    rq_max_gs: u32,
    rq_max_post: u32,

    /// Per-QP doorbell record (DMA host memory).
    qp_doorbell_addr: usize,
    /// UAR doorbell MMIO page for this QP.
    uar_doorbell_addr: usize,
    /// BlueFlame MMIO page for this QP; only usable if `bf_len > 0`.
    bf_addr: usize,
    bf_len: usize,
    bf_reg_size: usize,

    sq_head: u32,
    sq_tail: u32,
    rq_head: u32,
    rq_tail: u32,

    /// (wr_id, chain_size) per SQ WQE index - mirrors kernel's
    /// `WorkQueue.meta` for the send queue.
    sq_meta: Vec<(u64, u32)>,
    /// (wr_id, chain_size) per RQ WQE index.
    rq_meta: Vec<(u64, u32)>,
}

/// Per-CQ kernel-bypass fast-path state: the memory regions mapped by
/// `UVERBS_CMD_MMAP_CQ`, plus the locally-tracked consumer index that
/// mirrors the kernel driver's `CompletionQueue::consumer_index`.
struct CqRingState {
    /// Base address of the CQE ring buffer, mapped by UVERBS_CMD_MMAP_CQ.
    cqe_ring_addr: usize,
    num_entries: u32,
    /// Per-CQ doorbell record (DMA host memory).
    cq_doorbell_addr: usize,
    consumer_index: u32,
}

/// Byte address of the send-queue WQE at (unwrapped) `index`, mirroring
/// `WorkQueue::get_element` applied to the send queue.
fn sq_wqe_addr(state: &QpRingState, index: u32) -> usize {
    let wrapped = index & (state.sq_wqe_cnt - 1);
    state.ring_buf_addr + (state.sq_offset + (wrapped << state.sq_wqe_shift)) as usize
}

/// Byte address of the receive-queue WQE at (unwrapped) `index`, mirroring
/// `WorkQueue::get_element` applied to the receive queue.
fn rq_wqe_addr(state: &QpRingState, index: u32) -> usize {
    let wrapped = index & (state.rq_wqe_cnt - 1);
    state.ring_buf_addr + (state.rq_offset + (wrapped << state.rq_wqe_shift)) as usize
}

/// Stamp a send-queue WQE so hardware treats it as invalid if prefetched:
/// mark the first four bytes of every 64-byte chunk after the first with
/// 0xff. Mirrors `WorkQueue::stamp_wqe`. Uses volatile writes for the same
/// reason `WriteOnlyBe32` does - nothing else in this translation unit
/// reads these bytes back, so a plain write could be optimized away.
///
/// # Safety
/// `state.ring_buf_addr` must point at a live mapping of this QP's ring
/// buffer, at least `size` bytes past `sq_wqe_addr(state, index)`.
unsafe fn sq_stamp_wqe(state: &QpRingState, index: u32) {
    let ctrl_addr = sq_wqe_addr(state, index);
    let size = unsafe { (*(ctrl_addr as *const WqeControlSegment)).size() } as usize;
    let mut i = 64;
    while i < size {
        unsafe {
            let p = (ctrl_addr + i) as *mut u8;
            core::ptr::write_volatile(p, 0xff);
            core::ptr::write_volatile(p.add(1), 0xff);
            core::ptr::write_volatile(p.add(2), 0xff);
            core::ptr::write_volatile(p.add(3), 0xff);
        }
        i += 64;
    }
}

/// Translate an SGE's virtual address to a physical one, without a kernel
/// round-trip: the MR's physical base (recorded at `ibv_reg_mr` time, which
/// already did the one-time translation kernel-side) plus the SGE's offset
/// from the MR's virtual base. This relies on the same assumption
/// `ibv_reg_mr`'s single (physical) `addr` already makes about the
/// registered region - that it is physically contiguous - which is the
/// same assumption the kernel-side `WqeDataSegment::set` call site
/// implicitly makes today for any single-page SGE.
fn fastpath_translate_sge(mr_registry: &RefCell<BTreeMap<u32, (u64, u64)>>, sge: &ibv_sge) -> Result<u64> {
    let registry = mr_registry.borrow();
    let (phys_base, virt_base) = registry.get(&sge.lkey).ok_or_else(|| Error::from(ErrorKind::Other))?;
    Ok(phys_base + (sge.addr - virt_base))
}

/// Kernel-bypass send: builds the WQE directly in the mapped ring buffer
/// and rings the doorbell (BlueFlame MMIO copy or normal doorbell MMIO
/// register), without a syscall. Mirrors `QueuePair::post_send`
/// (`os/kernel/src/device/mlx4/queue_pair.rs`) field-for-field. RC/UC only.
unsafe fn fastpath_post_send(qp: &mut ibv_qp<'_, '_>, wr: &mut ibv_send_wr) -> Result<()> {
    let mr_registry = &qp.send_cq.context.mr_registry;
    let mut state = qp.ring_state.borrow_mut();

    if state.qp_type != ibv_qp_type::IBV_QPT_RC && state.qp_type != ibv_qp_type::IBV_QPT_UC {
        // UD is not supported by the fast path.
        return Err(Error::from(ErrorKind::Other));
    }

    let mut index = state.sq_head;
    let mut current: Option<&mut ibv_send_wr> = Some(wr);
    let mut num_req: u32 = 0;
    let mut chain_size: u32 = 1;

    while let Some(curr) = current.take() {
        if state.sq_head.wrapping_sub(state.sq_tail) + num_req >= state.sq_max_post {
            return Err(Error::from(ErrorKind::Other));
        }
        if u32::try_from(curr.num_sge).unwrap() > state.sq_max_gs {
            return Err(Error::from(ErrorKind::Other));
        }

        let ctrl_addr = sq_wqe_addr(&state, index);
        unsafe {
            let ctrl = &mut *(ctrl_addr as *mut WqeControlSegment);
            ctrl.vlan_cv_f_ds = 0.into();
            let wqe_flags: WqeControlSegmentFlags = curr.send_flags.into();
            ctrl.flags = wqe_flags.bits().into();
            ctrl.flags2 = 0.into();
        }

        let mut wqe_offset = ctrl_addr + size_of::<WqeControlSegment>();
        let mut wqe_size = size_of::<WqeControlSegment>();

        if curr.opcode == ibv_wr_opcode::IBV_WR_RDMA_READ || curr.opcode == ibv_wr_opcode::IBV_WR_RDMA_WRITE {
            let seg = WqeRemoteAddressSegment::from_wr(&curr.wr).map_err(|_| Error::from(ErrorKind::Other))?;
            unsafe { (wqe_offset as *mut WqeRemoteAddressSegment).write(seg) };
            wqe_offset += size_of::<WqeRemoteAddressSegment>();
            wqe_size += size_of::<WqeRemoteAddressSegment>();
        }

        // Write data segments in reverse order, so as to overwrite the
        // cacheline stamp last within each cacheline - same rationale as
        // the kernel driver (avoids issues with WQE prefetching).
        wqe_offset += (usize::try_from(curr.num_sge).unwrap() - 1) * size_of::<WqeDataSegment>();
        for sge in curr.sg_list.iter().rev() {
            let phys_addr = fastpath_translate_sge(mr_registry, sge)?;
            unsafe {
                let seg = &mut *(wqe_offset as *mut WqeDataSegment);
                seg.set(sge, phys_addr);
            }
            wqe_offset -= size_of::<WqeDataSegment>();
            wqe_size += size_of::<WqeDataSegment>();
        }

        // Make sure descriptor is fully written before setting ownership
        // bit (because HW can start executing as soon as we do).
        compiler_fence(Ordering::SeqCst);
        unsafe {
            let ctrl = &mut *(ctrl_addr as *mut WqeControlSegment);
            ctrl.vlan_cv_f_ds = u32::try_from(wqe_size / 16).unwrap().into();
        }
        compiler_fence(Ordering::SeqCst);

        let opcode = match curr.opcode {
            ibv_wr_opcode::IBV_WR_RDMA_WRITE => QueuePairOpcode::RdmaWrite,
            ibv_wr_opcode::IBV_WR_SEND => QueuePairOpcode::Send,
            ibv_wr_opcode::IBV_WR_RDMA_READ => QueuePairOpcode::RdmaRead,
        } as u32;
        let owner = if index & state.sq_wqe_cnt == 0 { 0 } else { 1u32 << 31 };
        unsafe {
            let ctrl = &mut *(ctrl_addr as *mut WqeControlSegment);
            ctrl.owner_opcode = (owner | opcode).into();
        }

        // Improve latency by not stamping the last send queue WQE until
        // after ringing the doorbell, so only stamp here if there are
        // still more WQEs to post.
        if !curr.next.is_null() {
            unsafe { sq_stamp_wqe(&state, index.wrapping_add(state.sq_spare_wqes)) };
        }

        if curr.send_flags.contains(ibv_send_flags::SIGNALED) {
            // write wr id, so that poll_cq can recover it
            let idx = (index & (state.sq_wqe_cnt - 1)) as usize;
            state.sq_meta[idx] = (curr.wr_id, chain_size);
            chain_size = 1;
        } else {
            chain_size += 1;
        }

        num_req += 1;
        index = index.wrapping_add(1);
        current = unsafe {
            if !curr.next.is_null() {
                Some(&mut *curr.next)
            } else {
                None
            }
        };
    }

    if num_req == 0 {
        return Ok(());
    }

    if state.bf_len > 0 && num_req == 1 {
        index = index.wrapping_sub(1);
        let ctrl_addr = sq_wqe_addr(&state, index);
        let (size, bf_offset) = unsafe {
            let ctrl = &mut *(ctrl_addr as *mut WqeControlSegment);
            ctrl.owner_opcode.set(ctrl.owner_opcode.get() | ((state.sq_head & 0xffff) << 8));
            ctrl.vlan_cv_f_ds.set(ctrl.vlan_cv_f_ds.get() | (state.qp_number << 8));
            // the UAR determines which BlueFlame page we can use; use the
            // first register (0..bf_reg_size), alternating between its two
            // halves (bf_reg_size/2 each), matching the kernel driver.
            (ctrl.size() as usize, (index as usize % 2) * (state.bf_reg_size / 2))
        };
        // Make sure that descriptor is written to memory before writing to
        // the BlueFlame page.
        compiler_fence(Ordering::SeqCst);
        unsafe {
            core::ptr::copy_nonoverlapping(ctrl_addr as *const u8, (state.bf_addr + bf_offset) as *mut u8, size);
        }
    } else {
        // Make sure that descriptors are written before doorbell.
        compiler_fence(Ordering::SeqCst);
        unsafe {
            let doorbell = &mut *(state.uar_doorbell_addr as *mut DoorbellPage);
            doorbell.send_queue_number.set((state.qp_number << 8).to_be());
        }
    }

    unsafe { sq_stamp_wqe(&state, index.wrapping_add(state.sq_spare_wqes).wrapping_sub(1)) };
    state.sq_head = state.sq_head.wrapping_add(num_req);

    Ok(())
}

/// Kernel-bypass receive: writes SGE descriptors directly into the mapped
/// ring buffer and updates the per-QP doorbell record, without a syscall.
/// Mirrors `QueuePair::post_receive`
/// (`os/kernel/src/device/mlx4/queue_pair.rs`) field-for-field.
unsafe fn fastpath_post_recv(qp: &mut ibv_qp<'_, '_>, wr: &mut ibv_recv_wr) -> Result<()> {
    let mr_registry = &qp.recv_cq.context.mr_registry;
    let mut state = qp.ring_state.borrow_mut();

    let mut index = state.rq_head;
    let mut current: Option<&mut ibv_recv_wr> = Some(wr);
    let mut num_req: u32 = 0;

    while let Some(curr) = current.take() {
        if state.rq_head.wrapping_sub(state.rq_tail) + num_req >= state.rq_max_post {
            return Err(Error::from(ErrorKind::Other));
        }
        if u32::try_from(curr.num_sge).unwrap() > state.rq_max_gs {
            return Err(Error::from(ErrorKind::Other));
        }

        let mut sge_index: u32 = 0;
        for sge in &curr.sg_list {
            let phys_addr = fastpath_translate_sge(mr_registry, sge)?;
            let elem_addr = rq_wqe_addr(&state, index.wrapping_add(sge_index));
            unsafe {
                let seg = &mut *(elem_addr as *mut WqeDataSegment);
                seg.set(sge, phys_addr);
            }
            sge_index += 1;
        }

        // write wr id, so that poll_cq can recover it. Every receive WR
        // retires exactly one queue slot, so its chain_size is always 1
        // (unlike send WRs, which can batch several unsignaled sends
        // behind one signaled completion).
        let idx = (index & (state.rq_wqe_cnt - 1)) as usize;
        state.rq_meta[idx] = (curr.wr_id, 1);

        // fill the last one
        let last_addr = rq_wqe_addr(&state, index.wrapping_add(sge_index));
        unsafe {
            *(last_addr as *mut WqeDataSegment) = WqeDataSegment::last();
        }

        num_req += 1;
        index = index.wrapping_add(1);
        current = unsafe {
            if !curr.next.is_null() {
                Some(&mut *curr.next)
            } else {
                None
            }
        };
    }

    if num_req == 0 {
        return Ok(());
    }

    state.rq_head = state.rq_head.wrapping_add(num_req);
    // make sure that the descriptors are written before the doorbell
    compiler_fence(Ordering::SeqCst);
    unsafe {
        let doorbell = &mut *(state.qp_doorbell_addr as *mut QueuePairDoorbell);
        doorbell.receive_wqe_index.set(
            (state.rq_head as u16 as u32).to_be(), // wrap around at u16::MAX
        );
    }

    Ok(())
}

/// Kernel-bypass poll: reads completions directly from the mapped CQE ring
/// and updates the CQ's doorbell record, without a syscall. Mirrors
/// `CompletionQueue::poll` (`os/kernel/src/device/mlx4/completion_queue.rs`).
fn fastpath_poll_cq(cq: &ibv_cq<'_>, wc: &mut [ibv_wc]) -> Result<i32> {
    let mut cq_state = cq.cq_state.borrow_mut();
    let mut completions = 0usize;

    while completions < wc.len() {
        if fastpath_poll_one(cq.context, &mut cq_state, &mut wc[completions])? {
            completions += 1;
        } else {
            break;
        }
    }

    unsafe {
        let doorbell = &mut *(cq_state.cq_doorbell_addr as *mut CompletionQueueDoorbell);
        doorbell.update_consumer_index.set((cq_state.consumer_index & 0xffffff).to_be());
    }

    Ok(completions as i32)
}

/// Read the next CQE if hardware has produced one, mirroring
/// `CompletionQueue::get_next_cqe_sw` - checks the ownership bit, which
/// flips every ring wraparound.
fn fastpath_get_next_cqe_sw(cq_state: &CqRingState) -> Option<CompletionQueueEntry> {
    let index = cq_state.consumer_index;
    let byte_offset = (index & (cq_state.num_entries - 1)) as usize * size_of::<CompletionQueueEntry>();
    let cqe_bytes = unsafe { core::slice::from_raw_parts((cq_state.cqe_ring_addr + byte_offset) as *const u8, size_of::<CompletionQueueEntry>()) };
    let cqe = CompletionQueueEntry::from_bytes(cqe_bytes.try_into().unwrap());
    if cqe.owner() ^ ((index & cq_state.num_entries) != 0) {
        None
    } else {
        Some(cqe)
    }
}

/// Decode one completion, mirroring `CompletionQueue::poll_one` field-for-
/// field. Returns `Ok(true)` if a completion was consumed.
fn fastpath_poll_one(context: &ibv_context, cq_state: &mut CqRingState, wc: &mut ibv_wc) -> Result<bool> {
    const CQE_OPCODE_ERROR: u8 = 0x1e;

    *wc = ibv_wc::default();
    let Some(cqe) = fastpath_get_next_cqe_sw(cq_state) else {
        return Ok(false);
    };
    cq_state.consumer_index = cq_state.consumer_index.wrapping_add(1);
    // Make sure we read CQ entry contents after we've checked the
    // ownership bit.
    compiler_fence(Ordering::SeqCst);

    wc.qp_num = cqe.qp_number();
    if let Some(ring_state) = context.qp_registry.borrow().get(&cqe.qp_number()) {
        let mut qp_state = ring_state.borrow_mut();
        let wqe_idx = cqe.wqe_index() as usize;
        let (wr_id, chain_size) = if cqe.is_send() {
            let idx = wqe_idx & (qp_state.sq_wqe_cnt as usize - 1);
            qp_state.sq_meta[idx]
        } else {
            let idx = wqe_idx & (qp_state.rq_wqe_cnt as usize - 1);
            qp_state.rq_meta[idx]
        };
        if cqe.is_send() {
            qp_state.sq_tail = qp_state.sq_tail.wrapping_add(chain_size);
        } else {
            qp_state.rq_tail = qp_state.rq_tail.wrapping_add(chain_size);
        }
        wc.wr_id = wr_id;
    }

    if cqe.opcode() == CQE_OPCODE_ERROR {
        let checksum_bytes = cqe.checksum().to_be_bytes();
        let vendor_err_syndrome = checksum_bytes[0];
        let syndrome = Syndrome::from_repr(checksum_bytes[1]).ok_or_else(|| Error::from(ErrorKind::Other))?;
        wc.status = match syndrome {
            Syndrome::LocalLengthError => ibv_wc_status::IBV_WC_LOC_LEN_ERR,
            Syndrome::LocalQpOperationError => ibv_wc_status::IBV_WC_LOC_QP_OP_ERR,
            Syndrome::LocalProtError => ibv_wc_status::IBV_WC_LOC_PROT_ERR,
            Syndrome::WrFlushError => ibv_wc_status::IBV_WC_WR_FLUSH_ERR,
            Syndrome::MwBindError => ibv_wc_status::IBV_WC_MW_BIND_ERR,
            Syndrome::BadResponseError => ibv_wc_status::IBV_WC_BAD_RESP_ERR,
            Syndrome::LocalAccessError => ibv_wc_status::IBV_WC_LOC_ACCESS_ERR,
            Syndrome::RemoteInvalidRequestError => ibv_wc_status::IBV_WC_REM_INV_REQ_ERR,
            Syndrome::RemoteAccessError => ibv_wc_status::IBV_WC_REM_ACCESS_ERR,
            Syndrome::RemoteOperationError => ibv_wc_status::IBV_WC_REM_OP_ERR,
            Syndrome::TransportRetryExceededError => ibv_wc_status::IBV_WC_RETRY_EXC_ERR,
            Syndrome::RnrRetryExceededError => ibv_wc_status::Type::IBV_WC_RNR_RETRY_EXC_ERR,
            Syndrome::RemoteAbortedErr => ibv_wc_status::IBV_WC_REM_ABORT_ERR,
            #[allow(unreachable_patterns)]
            _ => ibv_wc_status::Type::IBV_WC_GENERAL_ERR,
        };
        wc.vendor_err = vendor_err_syndrome.into();
        return Ok(true);
    }

    wc.status = ibv_wc_status::IBV_WC_SUCCESS;
    wc.wc_flags = ibv_wc_flags::empty();
    if cqe.is_send() {
        let opcode = QueuePairOpcode::from_repr(cqe.opcode().into()).ok_or_else(|| Error::from(ErrorKind::Other))?;
        match opcode {
            QueuePairOpcode::RdmaWrite => {
                wc.opcode = ibv_wc_opcode::IBV_WC_RDMA_WRITE;
            }
            QueuePairOpcode::RdmaWriteImm => {
                wc.opcode = ibv_wc_opcode::IBV_WC_RDMA_WRITE;
                wc.wc_flags.insert(ibv_wc_flags::IBV_WC_WITH_IMM);
            }
            QueuePairOpcode::Send => {
                wc.opcode = ibv_wc_opcode::IBV_WC_SEND;
            }
            QueuePairOpcode::SendImm => {
                wc.opcode = ibv_wc_opcode::IBV_WC_SEND;
                wc.wc_flags.insert(ibv_wc_flags::IBV_WC_WITH_IMM);
            }
            QueuePairOpcode::SendInval => {
                wc.opcode = ibv_wc_opcode::IBV_WC_SEND;
            }
            QueuePairOpcode::RdmaRead => {
                wc.opcode = ibv_wc_opcode::IBV_WC_RDMA_READ;
                wc.byte_len = cqe.byte_cnt();
            }
            QueuePairOpcode::AtomicCs | QueuePairOpcode::MaskedAtomicCs => {
                wc.opcode = ibv_wc_opcode::IBV_WC_COMP_SWAP;
                wc.byte_len = 8;
            }
            QueuePairOpcode::AtomicFa | QueuePairOpcode::MaskedAtomicFa => {
                wc.opcode = ibv_wc_opcode::IBV_WC_FETCH_ADD;
                wc.byte_len = 8;
            }
            QueuePairOpcode::LocalInval => {
                wc.opcode = ibv_wc_opcode::IBV_WC_LOCAL_INV;
            }
            _ => {}
        }
    } else {
        let opcode = ReceiveOpcode::from_repr(cqe.opcode().into()).ok_or_else(|| Error::from(ErrorKind::Other))?;
        wc.byte_len = cqe.byte_cnt();
        match opcode {
            ReceiveOpcode::RdmaWriteImm => {
                wc.opcode = ibv_wc_opcode::IBV_WC_RECV_RDMA_WITH_IMM;
                wc.wc_flags.insert(ibv_wc_flags::IBV_WC_WITH_IMM);
                wc.imm_data = cqe.immed_rss_invalid();
            }
            ReceiveOpcode::SendInval => {
                wc.opcode = ibv_wc_opcode::IBV_WC_RECV;
                wc.wc_flags.insert(ibv_wc_flags::IBV_WC_WITH_INV);
                todo!("set invalidate_rkey");
            }
            ReceiveOpcode::Send => {
                wc.opcode = ibv_wc_opcode::IBV_WC_RECV;
            }
            ReceiveOpcode::SendImm => {
                wc.opcode = ibv_wc_opcode::IBV_WC_RECV;
                wc.wc_flags.insert(ibv_wc_flags::IBV_WC_WITH_IMM);
                wc.imm_data = cqe.immed_rss_invalid();
            }
        }
        wc.src_qp = cqe.rqpn();
        wc.dlid_path_bits = cqe.mlpath();
        if cqe.g() {
            wc.wc_flags.insert(ibv_wc_flags::IBV_WC_GRH);
        }
        wc.pkey_index = (cqe.immed_rss_invalid() & 0x7f).try_into().unwrap();
        wc.slid = cqe.slid();
        wc.sl = cqe.sl();
    }

    Ok(true)
}

pub fn ibv_send_wr_builder(wr_id: u64, opcode: ibv_wr_opcode, send_flags: ibv_send_flags,
    wr: ibv_send_wr_wr, next: *mut ibv_send_wr, sg_list: Vec<ibv_sge>) -> Box<ibv_send_wr> {
    let num_sge = sg_list.len() as i32;
    Box::new(ibv_send_wr {
                wr_id,
                next,
                sg_list,
                num_sge,
                opcode,
                send_flags,
                wr,
                qp_type: Default::default(),
                __bindgen_anon_1: Default::default(),
                __bindgen_anon_2: Default::default(),
    })
}

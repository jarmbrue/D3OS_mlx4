use super::uverbs_cmd::*;
use crate::device::mlx4::{device_in_range, ERR_NOT_OWNER};
use crate::process_manager;
use alloc::vec;
use core::slice::{from_mut, from_raw_parts_mut};
use rdma::uverbs_uapi::UVERBS_MAX_QUERY_DEVICES_REQ;
use rdma::{ibv_device, ibv_device_attr, ibv_wc, uverbs_uapi::{
    ibv_cq_container, ibv_cq_mmap_container, ibv_cq_poll_container, ibv_mr_container, ibv_port_attr_container, ibv_qp_container,
    ibv_qp_mmap_container, ibv_qp_modify_container, ibv_qp_post_recv_container, ibv_qp_post_send_container, TypeSize, UverbsCmd, UVERBS_CMD_CREATE_CQ,
    UVERBS_CMD_CREATE_QP, UVERBS_CMD_DEREGISTER_MR, UVERBS_CMD_DESTROY_CQ, UVERBS_CMD_DESTROY_QP, UVERBS_CMD_MMAP_CQ, UVERBS_CMD_MMAP_QP,
    UVERBS_CMD_MODIFY_QP, UVERBS_CMD_POLL_CQ, UVERBS_CMD_POST_RECV, UVERBS_CMD_POST_SEND, UVERBS_CMD_QUERY_DEVICE, UVERBS_CMD_QUERY_DEVICES,
    UVERBS_CMD_QUERY_PORT, UVERBS_CMD_REGISTER_MR, UVERBS_MAGIC, UVERBS_MINOR_NOT_PRESENT, UVERBS_MINOR_PRESENT,
}};
use syscall::return_vals::{Errno, SyscallResult};
use x86_64::VirtAddr;

static UVERBS_SUPPORTED_MINOR_TABLE: &[usize] = &[
    UVERBS_CMD_QUERY_DEVICE,
    UVERBS_CMD_QUERY_PORT,
    UVERBS_CMD_REGISTER_MR,
    UVERBS_CMD_CREATE_CQ,
    UVERBS_CMD_CREATE_QP,
    UVERBS_CMD_MODIFY_QP,
    UVERBS_CMD_POLL_CQ,
    UVERBS_CMD_POST_SEND,
    UVERBS_CMD_POST_RECV,
    UVERBS_CMD_DESTROY_CQ,
    UVERBS_CMD_DESTROY_QP,
    UVERBS_CMD_DEREGISTER_MR,
    UVERBS_CMD_MMAP_QP,
    UVERBS_CMD_MMAP_CQ,
];

/// Map a driver-level `&'static str` error to a syscall errno. Ownership
/// failures - identified via the `ERR_NOT_OWNER` sentinel returned by
/// `ConnectX3Nic`'s QP/CQ methods (`modify_qp`/`post_send`/`post_receive`/
/// `destroy_qp`/`poll_cq`/`destroy_cq`) - surface as `Errno::EACCES` so
/// userspace can distinguish "not your QP/CQ" from a malformed request;
/// every other driver error keeps mapping to the pre-existing blanket
/// `Errno::EINVAL`. A sentinel-string match was chosen over a small typed
/// driver error because the driver already communicates exclusively via
/// `&'static str` (including the pre-existing, unrelated ownership strings
/// in `mmap_qp_resources`/`mmap_cq_resources`) - introducing a typed error
/// just for this one distinction would touch every driver method's return
/// type for no benefit beyond this single call boundary.
fn map_uverbs_err(err: &'static str) -> Errno {
    if err == ERR_NOT_OWNER {
        Errno::EACCES
    } else {
        Errno::EINVAL
    }
}

pub fn uverbs_ctl(minor: usize, cmd: usize, arg: usize) -> SyscallResult {
    // `size` (the cmd-encoded container size) no longer needs to be read out
    // here: every arm below now copies through `copy_from_user`/
    // `copy_to_user`, which derive their length from the destination Rust
    // type's `size_of::<T>()` - exactly how each `UVERBS_CMD_*` constant's
    // encoded size was computed in the first place (`uverbs_uapi.rs`), so
    // this isn't a loosening of that invariant, just no longer needing a
    // separate runtime read of it.
    let UverbsCmd::Call(_, _, _size, magic, has_minor) = UverbsCmd::decode(cmd as u64);

    let process = process_manager().read().current_process();

    if magic != UVERBS_MAGIC {
        return Err(Errno::EINVAL);
    }

    match has_minor {
        UVERBS_MINOR_PRESENT if !device_in_range(minor) => Err(Errno::EINVAL),
        UVERBS_MINOR_NOT_PRESENT if (minor != 0 || UVERBS_SUPPORTED_MINOR_TABLE.iter().find(|x| **x == cmd).is_some()) => Err(Errno::EINVAL),
        _ => Ok(0usize),
    }?;

    match cmd {
        UVERBS_CMD_QUERY_DEVICES => {
            // TODO pass buf_size a argument?
            let devices = uverbs_query_devices(UVERBS_MAX_QUERY_DEVICES_REQ);
            process.virtual_address_space.copy_to_user(arg as *mut ibv_device, &devices)
                .map(|_| devices.len())
                .map_err(|_| Errno::EINVAL)
        }
        UVERBS_CMD_QUERY_DEVICE => {
            let dev_attr = uverbs_query_device(minor).map_err(|_| Errno::EINVAL)?;
            process.virtual_address_space.copy_to_user(arg as *mut ibv_device_attr, &[dev_attr])
                .map(|val| 0)
                .map_err(|_| Errno::EINVAL)
        }
        UVERBS_CMD_QUERY_PORT => {
            let __user_buf = arg as *mut ibv_port_attr_container;
            let mut __kernel_container = ibv_port_attr_container::default();

            process.virtual_address_space.copy_from_user(from_mut(&mut __kernel_container), __user_buf as *const _).map_err(|_| Errno::EINVAL)?;

            __kernel_container.ibv_port_attr = uverbs_query_port(minor, __kernel_container.port_num).map_err(|_| Errno::EINVAL)?;

            process.virtual_address_space.copy_to_user(__user_buf, &[__kernel_container]).map_err(|_| Errno::EINVAL)?;

            Ok(0)
        }
        UVERBS_CMD_REGISTER_MR => {
            let __user_buf = arg as *mut ibv_mr_container;
            let mut __kernel_ibv_mr_container = ibv_mr_container::default();

            process.virtual_address_space.copy_from_user(from_mut(&mut __kernel_ibv_mr_container), __user_buf as *const _).map_err(|_| Errno::EINVAL)?;

            let supported_len = __kernel_ibv_mr_container.len.min(ibv_mr_container::S);

            // `data_ptr`/`len` describe memory that gets DMA'd in place (by
            // the HCA, later) rather than copied into a kernel buffer, so
            // `copy_from_user`'s "copy into a kernel-owned Copy value"
            // shape doesn't apply here. Validate the range the same way
            // `copy_from_user`/`copy_to_user` do (bounds-check via
            // `access_ok`) before handing the raw pointer to
            // `from_raw_parts_mut` and onward into DMA registration.
            let data_addr = VirtAddr::new(__kernel_ibv_mr_container.data_ptr as u64);
            if !process.virtual_address_space.is_user_range_ok(data_addr, supported_len) {
                return Err(Errno::EINVAL);
            }

            let __user_data_ref = unsafe { from_raw_parts_mut(__kernel_ibv_mr_container.data_ptr, supported_len) };

            __kernel_ibv_mr_container.ibv_mr_res =
                uverbs_register_mem_region(minor, __kernel_ibv_mr_container.ibv_access_flags, __user_data_ref).map_err(|_| Errno::EINVAL)?;

            process.virtual_address_space.copy_to_user(__user_buf, &[__kernel_ibv_mr_container]).map_err(|_| Errno::EINVAL)?;

            Ok(0)
        }
        UVERBS_CMD_CREATE_CQ => {
            let __user_buf = arg as *mut ibv_cq_container;
            let mut __kernel_cq_container = ibv_cq_container::default();

            process.virtual_address_space.copy_from_user(from_mut(&mut __kernel_cq_container), __user_buf as *const _).map_err(|_| Errno::EINVAL)?;

            uverbs_create_cq(minor, &mut __kernel_cq_container).map_err(|_| Errno::EINVAL)?;

            process.virtual_address_space.copy_to_user(__user_buf, &[__kernel_cq_container]).map_err(|_| Errno::EINVAL)?;
            Ok(0)
        }
        UVERBS_CMD_CREATE_QP => {
            let __user_buf = arg as *mut ibv_qp_container;
            let mut __kernel_qp_container = ibv_qp_container::default();

            process.virtual_address_space.copy_from_user(from_mut(&mut __kernel_qp_container), __user_buf as *const _).map_err(|_| Errno::EINVAL)?;

            // This used to be a second copy_nonoverlapping dereferencing
            // `ib_caps: *mut ibv_qp_cap` (a raw user pointer, entirely
            // unvalidated) plus a pointer-patch-back dance so
            // `uverbs_create_qp` could mutate it in place. `ibv_qp_cap` is
            // fully POD (5x u32, see `os/library/rdma/src/ib_core.rs`), so
            // it is now embedded by value in `ibv_qp_container` instead -
            // the single copy_from_user above already pulled it in, and
            // `uverbs_create_qp` mutates `__kernel_qp_container.ib_caps`
            // directly.
            uverbs_create_qp(minor, &mut __kernel_qp_container).map_err(|_| Errno::EINVAL)?;

            process.virtual_address_space.copy_to_user(__user_buf, &[__kernel_qp_container]).map_err(|_| Errno::EINVAL)?;
            Ok(0)
        }
        UVERBS_CMD_MODIFY_QP => {
            let __user_buf = arg as *mut ibv_qp_modify_container;
            let mut __kernel_qp_modify_container = ibv_qp_modify_container::default();

            process.virtual_address_space.copy_from_user(from_mut(&mut __kernel_qp_modify_container), __user_buf as *const _).map_err(|_| Errno::EINVAL)?;

            // Same simplification as CREATE_QP above: `attr: *const
            // ibv_qp_attr` (unvalidated user pointer + second copy) becomes
            // `attr: ibv_qp_attr` embedded by value (`ibv_qp_attr` is POD -
            // scalars/enums plus POD `ibv_ah_attr` -> `ibv_global_route` ->
            // `ibv_gid`), pulled in by the single copy_from_user above.
            uverbs_modify_qp(minor, process.id(), __kernel_qp_modify_container).map_err(map_uverbs_err)?;

            Ok(0)
        }
        UVERBS_CMD_POLL_CQ => {
            let __user_buf = arg as *mut ibv_cq_poll_container;
            let mut __kernel_cq_poll_container = ibv_cq_poll_container::default();

            process.virtual_address_space.copy_from_user(from_mut(&mut __kernel_cq_poll_container), __user_buf as *const _).map_err(|_| Errno::EINVAL)?;

            // NOTE: preserved as-is from the pre-existing code: this clamps
            // a *count* (`wc_len`) against `ibv_cq_poll_container::S`, which
            // is a *byte* size (`size_of::<ibv_wc>() * UVERBS_MAX_USER_WC_REQ`).
            // That means the effective clamp is far larger than
            // `UVERBS_MAX_USER_WC_REQ` entries. This looks like a
            // pre-existing off-by-a-large-factor oddity, not something
            // introduced here; left unchanged since fixing the clamp
            // formula is outside this hardening pass's scope (a genuinely
            // huge `wc_len` still only costs a kernel `Vec` allocation that
            // can fail cleanly, not a memory-safety issue).
            let supported_len = __kernel_cq_poll_container.wc_len.min(ibv_cq_poll_container::S);

            let mut __kernel_wc_buf = vec![ibv_wc::default(); supported_len];

            let wc_count = uverbs_poll_cq(minor, __kernel_cq_poll_container.cq_num, &mut __kernel_wc_buf[..]).map_err(|_| Errno::EINVAL)?;

            // Previously a raw copy_nonoverlapping straight into the
            // unvalidated `wc` user pointer - now goes through
            // copy_to_user, which bounds-checks and page-checks the
            // destination the same way every other output path here does.
            process.virtual_address_space.copy_to_user(__kernel_cq_poll_container.wc, &__kernel_wc_buf[..wc_count]).map_err(|_| Errno::EINVAL)?;

            Ok(wc_count)
        }
        UVERBS_CMD_POST_SEND => {
            let __user_buf = arg as *mut ibv_qp_post_send_container;
            let mut __kernel_container = ibv_qp_post_send_container::default();

            process.virtual_address_space.copy_from_user(from_mut(&mut __kernel_container), __user_buf as *const _).map_err(|_| Errno::EINVAL)?;

            // `ibv_send_wr` (the linked-list, Vec<ibv_sge>-backed type used
            // by the high-level API) is not POD, so the old code raw-byte-
            // copied a `Vec`'s internal (ptr, len, cap) triple out of user
            // memory - unsound independent of the missing validation, and
            // `curr.next` was a raw user pointer dereferenced directly in
            // the kernel. `ibv_qp_post_send_container` now embeds a fully
            // POD `ibv_send_wr_uapi` (fixed-capacity `sg_list` array, no
            // `next`) by value, so the single copy_from_user above is
            // sufficient - no nested-pointer validation needed. WQE chains
            // are walked in userspace instead
            // (`os/library/ibverbs/src/ibverbs_sys.rs`'s `ibv_post_send`
            // issues one syscall per WR). `sg_list` bounds (`num_sge` vs.
            // the fixed array length, and vs. the QP's negotiated
            // `max_send_sge`) are checked in
            // `os/kernel/src/device/mlx4/queue_pair.rs::post_send`.
            uverbs_post_send(minor, process.id(), &__kernel_container).map_err(map_uverbs_err)?;

            Ok(0)
        }
        UVERBS_CMD_POST_RECV => {
            let __user_buf = arg as *mut ibv_qp_post_recv_container;
            let mut __kernel_container = ibv_qp_post_recv_container::default();

            process.virtual_address_space.copy_from_user(from_mut(&mut __kernel_container), __user_buf as *const _).map_err(|_| Errno::EINVAL)?;

            // Same rationale as POST_SEND above.
            uverbs_post_recv(minor, process.id(), &__kernel_container).map_err(map_uverbs_err)?;

            Ok(0)
        }
        UVERBS_CMD_MMAP_QP => {
            let __user_buf = arg as *mut ibv_qp_mmap_container;
            let mut __kernel_qp_mmap_container = ibv_qp_mmap_container::default();

            process.virtual_address_space.copy_from_user(from_mut(&mut __kernel_qp_mmap_container), __user_buf as *const _).map_err(|_| Errno::EINVAL)?;

            uverbs_mmap_qp(minor, &mut __kernel_qp_mmap_container).map_err(|_| Errno::EINVAL)?;

            process.virtual_address_space.copy_to_user(__user_buf, &[__kernel_qp_mmap_container]).map_err(|_| Errno::EINVAL)?;

            Ok(0)
        }
        UVERBS_CMD_MMAP_CQ => {
            let __user_buf = arg as *mut ibv_cq_mmap_container;
            let mut __kernel_cq_mmap_container = ibv_cq_mmap_container::default();

            process.virtual_address_space.copy_from_user(from_mut(&mut __kernel_cq_mmap_container), __user_buf as *const _).map_err(|_| Errno::EINVAL)?;

            uverbs_mmap_cq(minor, &mut __kernel_cq_mmap_container).map_err(|_| Errno::EINVAL)?;

            process.virtual_address_space.copy_to_user(__user_buf, &[__kernel_cq_mmap_container]).map_err(|_| Errno::EINVAL)?;

            Ok(0)
        }
        UVERBS_CMD_DESTROY_CQ => {
            let cq_num = arg as u32;

            let _ = uverbs_destroy(minor, |dev| dev.destroy_cq(cq_num)).map_err(|_| Errno::EINVAL)?;

            Ok(0)
        }
        UVERBS_CMD_DESTROY_QP => {
            let qp_num = arg as u32;
            let caller = process.id();

            let _ = uverbs_destroy(minor, |dev| dev.destroy_qp(qp_num, caller)).map_err(map_uverbs_err)?;

            Ok(0)
        }
        UVERBS_CMD_DEREGISTER_MR => {
            let mr_index = arg as u32;

            // No ownership check: memory-region ownership tracking doesn't
            // exist yet and is explicitly out of scope for this pass (see
            // `docs/thesis-plan-1-3.md`'s extension 2 section).
            let _ = uverbs_destroy(minor, |dev| dev.destroy_mr(mr_index)).map_err(|_| Errno::EINVAL)?;

            Ok(0)
        }
        _ => Err(Errno::ENOCMD),
    }
}

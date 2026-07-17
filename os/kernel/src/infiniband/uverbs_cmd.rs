use alloc::vec::Vec;
use rdma::uverbs_uapi::{
    ibv_cq_container, ibv_cq_mmap_container, ibv_mr_res, ibv_qp_container, ibv_qp_mmap_container, ibv_qp_modify_container, ibv_qp_post_recv_container,
    ibv_qp_post_send_container,
};
use rdma::{ibv_access_flags, ibv_device, ibv_device_attr, ibv_port_attr, ibv_wc};
use uuid::Uuid;
use x86_64::structures::paging::frame::PhysFrameRange;
use x86_64::structures::paging::{PageTableFlags, PhysFrame};
use x86_64::PhysAddr;

use crate::device::mlx4::{get_dev_list, minor_to_idx, ConnectX3Nic};
use crate::memory::vma::VmaType;
use crate::memory::{MemorySpace, PAGE_SIZE};
use crate::process_manager;

pub fn uverbs_query_devices(max_len: usize) -> Vec<ibv_device> {
    get_dev_list().lock().iter()
        .map(|dev| ibv_device { nic: dev.minor } )
        .take(max_len)
        .collect()
}

pub fn uverbs_query_device(minor: usize) -> Result<ibv_device_attr, &'static str> {
    get_dev_list().lock()
        .get_mut(minor_to_idx(minor)).unwrap()
        .query_device()
}

pub fn uverbs_query_port(minor: usize, port_num: u8) -> Result<ibv_port_attr, &'static str> {
    get_dev_list().lock()
        .get_mut(minor_to_idx(minor)).unwrap()
        .query_port(port_num)
}

pub fn uverbs_register_mem_region(minor: usize, access_flags: ibv_access_flags, user_data_ref: &mut [u8]) -> Result<ibv_mr_res, &'static str> {
    get_dev_list().lock().get_mut(minor_to_idx(minor)).unwrap()
        .create_mr(user_data_ref, access_flags)
        .map(ibv_mr_res::from)
        .map_err(|_| "failed to create memory region")
}

pub fn uverbs_create_cq<'cq>(minor: usize, cq_container: &'cq mut ibv_cq_container) -> Result<&'cq u32, &'static str> {
    let number = get_dev_list().lock()
        .get_mut(minor_to_idx(minor)).unwrap()
        .create_cq(cq_container.cq_entries)?;
    cq_container.cq_num = number;

    Ok(&cq_container.cq_num)
}

pub fn uverbs_create_qp<'qp>(minor: usize, qp_container: &'qp mut ibv_qp_container) -> Result<&'qp u32, &'static str> {
    let number = get_dev_list().lock()
        .get_mut(minor_to_idx(minor)).unwrap()
        .create_qp(qp_container.qp_type, qp_container.send_cq_num, qp_container.recv_cq_num, &mut qp_container.ib_caps)?;

    qp_container.qp_num = number;
    Ok(&qp_container.qp_num)
}

pub fn uverbs_modify_qp(minor: usize, caller: Uuid, qp_modify_container: ibv_qp_modify_container) -> Result<(), &'static str> {
    get_dev_list().lock()
        .get_mut(minor_to_idx(minor)).unwrap()
        .modify_qp(
        qp_modify_container.qp_num,
        caller,
        &qp_modify_container.attr,
        qp_modify_container.attr_mask,
    )
}

pub fn uverbs_poll_cq(minor: usize, cq_num: u32, wc: &mut [ibv_wc]) -> Result<usize, &'static str> {
    get_dev_list().lock()
        .get_mut(minor_to_idx(minor)).unwrap()
        .poll_cq(cq_num, wc)
}

pub fn uverbs_post_send(minor: usize, caller: Uuid, send_container_wr: &ibv_qp_post_send_container) -> Result<(), &'static str> {
    get_dev_list().lock()
        .get_mut(minor_to_idx(minor)).unwrap()
        .post_send(send_container_wr.qp_num, caller, &send_container_wr.wr)
}

pub fn uverbs_post_recv(minor: usize, caller: Uuid, recv_container_wr: &ibv_qp_post_recv_container) -> Result<(), &'static str> {
    get_dev_list().lock()
        .get_mut(minor_to_idx(minor)).unwrap()
        .post_receive(recv_container_wr.qp_num, caller, &recv_container_wr.wr)
}

/// Generic destroy-verb helper shared by `DESTROY_CQ`/`DESTROY_QP`/
/// `DEREGISTER_MR`. Takes a closure rather than the previous bare
/// `fn(&mut ConnectX3Nic, u32) -> ...` pointer so each call site can capture
/// whatever it individually needs: `destroy_cq`/`destroy_qp` need the
/// calling process's `Uuid` for the ownership check added in this pass,
/// while `destroy_mr` doesn't (memory-region ownership tracking is out of
/// scope - see `os/kernel/src/device/mlx4.rs::destroy_mr`).
pub fn uverbs_destroy<F>(minor: usize, destroy_spec_fn: F) -> Result<(), &'static str>
where
    F: FnOnce(&mut ConnectX3Nic) -> Result<(), &'static str>,
{
    let mut device_list = get_dev_list().lock();
    let device = device_list.get_mut(minor_to_idx(minor)).unwrap();
    destroy_spec_fn(device)
}

/// Map a physical memory region into the calling process's user address
/// space, following the exact `alloc_vma` + `map_pfr_for_vma` pattern
/// `sys_map_frame_buffer` (`crate::syscall::sys_vmem`) uses for the
/// framebuffer. Returns the new region's user virtual address.
///
/// A fresh user-space VMA is required here rather than exposing the
/// existing kernel-space mapping of this memory (`create_cont_mapping_with_
/// dma_flags`/`pci_map_bar_mem` map DMA/MMIO pages into the shared kernel
/// address range, common to every process's page tables): flipping
/// `USER_ACCESSIBLE` there would leak the region into every process's user
/// mode, not just the caller's.
fn mmap_region_into_current_process(phys_addr: PhysAddr, byte_len: usize, flags: PageTableFlags, tag: &str) -> Result<usize, &'static str> {
    let process = process_manager().read().current_process();
    let num_pages = (byte_len as u64).div_ceil(PAGE_SIZE as u64);
    let start_frame = PhysFrame::from_start_address(phys_addr).map_err(|_| "region is not page aligned")?;
    let end_frame = start_frame + num_pages;

    let vma = process
        .virtual_address_space
        .alloc_vma(None, num_pages, MemorySpace::User, VmaType::DeviceMemory, tag)
        .ok_or("failed to allocate vma for mmap")?;

    process
        .virtual_address_space
        .map_pfr_for_vma(&vma, PhysFrameRange { start: start_frame, end: end_frame }, flags)
        .map_err(|_| "failed to map region")?;

    Ok(vma.start().as_u64() as usize)
}

/// Map a queue pair's ring buffer, doorbell record and UAR doorbell/
/// BlueFlame page(s) into the calling process, once, right after
/// `uverbs_create_qp`. This is what lets `post_send`/`post_recv` operate
/// without a syscall afterwards.
pub fn uverbs_mmap_qp(minor: usize, container: &mut ibv_qp_mmap_container) -> Result<(), &'static str> {
    let caller = process_manager().read().current_process().id();
    let res = get_dev_list()
        .lock()
        .get_mut(minor_to_idx(minor))
        .unwrap()
        .mmap_qp_resources(container.qp_num, caller)?;

    // Ring buffer and doorbell record are regular DMA-coherent DRAM, not
    // MMIO - no NO_CACHE.
    let dma_flags = PageTableFlags::PRESENT | PageTableFlags::WRITABLE | PageTableFlags::USER_ACCESSIBLE | PageTableFlags::NO_EXECUTE;
    // UAR doorbell / BlueFlame pages are genuine device MMIO.
    let mmio_flags = dma_flags | PageTableFlags::NO_CACHE;

    container.ring_buf_addr = mmap_region_into_current_process(res.ring_buf.0, res.ring_buf.1, dma_flags, "ib-qp-ring")?;
    container.ring_buf_len = res.ring_buf.1;
    container.sq_offset = res.sq_offset;
    container.sq_wqe_cnt = res.sq_wqe_cnt;
    container.sq_wqe_shift = res.sq_wqe_shift;
    container.sq_spare_wqes = res.sq_spare_wqes;
    container.sq_max_gs = res.sq_max_gs;
    container.sq_max_post = res.sq_max_post;
    container.rq_offset = res.rq_offset;
    container.rq_wqe_cnt = res.rq_wqe_cnt;
    container.rq_wqe_shift = res.rq_wqe_shift;
    container.rq_max_gs = res.rq_max_gs;
    container.rq_max_post = res.rq_max_post;

    container.qp_doorbell_addr = mmap_region_into_current_process(res.qp_doorbell, PAGE_SIZE, dma_flags, "ib-qp-db")?;
    container.uar_doorbell_addr = mmap_region_into_current_process(res.uar_doorbell, PAGE_SIZE, mmio_flags, "ib-uar-db")?;

    if let Some((bf_addr, bf_len)) = res.bf {
        container.bf_addr = mmap_region_into_current_process(bf_addr, bf_len, mmio_flags, "ib-bf")?;
        container.bf_len = bf_len;
    } else {
        container.bf_addr = 0;
        container.bf_len = 0;
    }
    container.bf_reg_size = res.bf_reg_size;

    Ok(())
}

/// Map a completion queue's CQE ring buffer and doorbell record into the
/// calling process, once, right after `uverbs_create_cq`. This is what
/// lets `poll_cq` operate without a syscall afterwards.
pub fn uverbs_mmap_cq(minor: usize, container: &mut ibv_cq_mmap_container) -> Result<(), &'static str> {
    let caller = process_manager().read().current_process().id();
    let res = get_dev_list()
        .lock()
        .get_mut(minor_to_idx(minor))
        .unwrap()
        .mmap_cq_resources(container.cq_num, caller)?;

    let dma_flags = PageTableFlags::PRESENT | PageTableFlags::WRITABLE | PageTableFlags::USER_ACCESSIBLE | PageTableFlags::NO_EXECUTE;

    container.cqe_ring_addr = mmap_region_into_current_process(res.ring_buf.0, res.ring_buf.1, dma_flags, "ib-cq-ring")?;
    container.cqe_ring_len = res.ring_buf.1;
    container.num_entries = res.num_entries;
    container.cq_doorbell_addr = mmap_region_into_current_process(res.cq_doorbell, PAGE_SIZE, dma_flags, "ib-cq-db")?;

    Ok(())
}

//! This module consists of functions that create, work with and destroy
//! completion queues. Furthermore its functions can consume and print
//! completion queue elements.

use core::{
    mem::size_of,
    sync::atomic::{compiler_fence, Ordering},
};

use super::queue_pair::QueuePair;
use super::utils;
use super::utils::{MappedPages, PageToFrameMapping};
use crate::device::mlx4::utils::{FillOperation, OperationArgs};
use crate::memory::PAGE_SIZE;
use crate::sync::wait_queue::WaitQueue;
use alloc::boxed::Box;
use alloc::sync::Arc;
use log::{error, trace, warn};
use modular_bitfield_msb::{
    bitfield,
    specifiers::{B2, B24, B3, B40, B48, B5, B6},
};
use rdma::{ibv_wc, ibv_wc_flags, ibv_wc_opcode, ibv_wc_status};
use rdma::mlx4_hw::{CompletionQueueDoorbell, CompletionQueueEntry, DoorbellPage, QueuePairOpcode, ReceiveOpcode, Syndrome};
use tock_registers::interfaces::Writeable;
use uuid::Uuid;
use x86_64::PhysAddr;

use crate::process_manager;

use super::{
    cmd::{CommandInterface, Opcode},
    device::{uar_index_to_hw, PAGE_SHIFT},
    event_queue::EventQueue,
    fw::Capabilities,
    icm::{MrTable, ICM_PAGE_SHIFT},
    Offsets,
};

#[derive(Debug)]
pub(super) struct CompletionQueue {
    number: u32,
    num_entries: u32,
    memory: Option<PageToFrameMapping>,
    uar_idx: usize,
    doorbell_page: MappedPages,
    doorbell_address: PhysAddr,
    // TODO: somehow free this on Drop
    _mtt: u64,
    arm_sequence_number: u32,
    consumer_index: u32,
    /// Process that created this completion queue. Used to reject
    /// cross-process mmap requests for this CQ's ring buffer / doorbell /
    /// UAR pages.
    creator: Uuid,
    // TODO: bind the lifetime to the one of the event queue
    eq_number: Option<usize>,
    /// Wait queue for threads genuinely blocked in `poll_cq`, waiting for a
    /// completion on this CQ. Woken by `Mlx4InterruptHandler::trigger()`
    /// once the associated EQ reports a `Completion` event naming this
    /// CQ's number (`ConnectX3Nic::handle_interrupt`).
    ///
    /// `Arc`-wrapped (rather than a bare `WaitQueue`, as the plan's text
    /// otherwise describes) so a caller can clone a cheap handle to it out
    /// of `DEV_LIST`'s lock and then block on it *after* releasing that
    /// lock (`ConnectX3Nic::arm_cq_for_wait`/`uverbs_cmd::uverbs_poll_cq`).
    /// Blocking while still holding `DEV_LIST` would deadlock: the
    /// interrupt handler that is supposed to wake this waiter also needs
    /// `DEV_LIST`'s lock to find this CQ in the first place.
    wq: Arc<WaitQueue>,
}

impl CompletionQueue {
    /// Create a new completion queue.
    ///
    /// This is quite like creating an event queue.
    pub(super) fn new(
        cmd: &mut CommandInterface, caps: &Capabilities, offsets: &mut Offsets, memory_regions: &mut MrTable, eq: Option<&EventQueue>, num_entries: u32,
    ) -> Result<Self, &'static str> {
        let number: u32 = offsets.alloc_cqn().try_into().unwrap();
        let uar_idx = offsets.alloc_scq_db();
        let num_pages = (usize::try_from(num_entries).unwrap() * size_of::<CompletionQueueEntry>()).next_multiple_of(PAGE_SIZE) / PAGE_SIZE;

        let mut operation_container = utils::Operations::default();
        let size = num_pages * PAGE_SIZE + size_of::<CompletionQueueEntry>() - 1;
        let mapped_page_to_frame = utils::create_cont_mapping_with_dma_flags(utils::pages_required(size))?.fetch_in_addr()?;

        let bytes = utils::start_page_as_mut_ptr::<u8>(mapped_page_to_frame.0.into_range().start);

        operation_container.add_operation(Box::new(FillOperation {}), OperationArgs::Fill(0u8, bytes, size));

        operation_container.perform();

        let mtt = memory_regions.alloc_mtt(cmd, caps, num_pages, mapped_page_to_frame.1)?;
        let (mut doorbell_page, doorbell_address) =
            utils::create_cont_mapping_with_dma_flags(utils::pages_required(size_of::<CompletionQueueDoorbell>()))?.fetch_in_addr()?;
        let doorbell: &mut CompletionQueueDoorbell = doorbell_page.as_type_mut(0)?;
        doorbell.update_consumer_index.set(0_u32.to_be());
        doorbell.arm_consumer_index.set(0_u32.to_be());
        let arm_sequence_number = 1;
        let consumer_index = 0;

        let mut ctx = CompletionQueueContext::new();
        ctx.set_log_size(num_entries.ilog2().try_into().unwrap());
        ctx.set_usr_page(uar_index_to_hw(uar_idx).try_into().unwrap());
        let mut eq_number = None;
        if let Some(eq) = eq {
            ctx.set_comp_eqn(eq.number().try_into().unwrap());
            eq_number = Some(eq.number());
        }
        ctx.set_log_page_size(PAGE_SHIFT - ICM_PAGE_SHIFT);
        ctx.set_mtt_base_addr(mtt);
        ctx.set_doorbell_record_addr(doorbell_address.as_u64());
        let _: () = cmd.execute_command(Opcode::Sw2HwCq, (), &ctx.bytes[..], number.try_into().unwrap())?;

        let creator = process_manager().read().current_process().id();
        let cq = Self {
            number,
            num_entries,
            memory: Some(mapped_page_to_frame),
            uar_idx,
            doorbell_page,
            doorbell_address,
            _mtt: mtt,
            arm_sequence_number,
            consumer_index,
            eq_number,
            creator,
            wq: Arc::new(WaitQueue::new()),
        };
        trace!("created new CQ: {:?}", cq);
        Ok(cq)
    }

    /// Destroy this completion queue.
    pub(super) fn destroy(mut self, cmd: &mut CommandInterface) -> Result<(), &'static str> {
        // TODO: should make sure to undo all card state tied to this CQ
        cmd.execute_command::<_, _, ()>(Opcode::Hw2SwCq, (), (), self.number.try_into().unwrap())?;
        // actually free the mememory
        self.memory.take().unwrap();
        Ok(())
    }

    /// Arm this completion queue by writing the consumer index to the
    /// appropriate doorbell.
    pub(super) fn arm(&mut self, doorbells: &mut [MappedPages]) -> Result<(), &'static str> {
        const _DOORBELL_REQUEST_NOTIFICATION_SOLICITED: u32 = 0x1;
        const DOORBELL_REQUEST_NOTIFICATION: u32 = 0x2;
        let sn = self.arm_sequence_number & 3;
        let ci = self.consumer_index & 0xffffff;
        let cmd = DOORBELL_REQUEST_NOTIFICATION;
        let doorbell_record: &mut CompletionQueueDoorbell = self.doorbell_page.as_type_mut(0)?;
        doorbell_record.arm_consumer_index.set((sn << 28 | cmd << 24 | ci).to_be());
        // Make sure that the doorbell record in host memory is
        // written before ringing the doorbell via PCI MMIO.
        compiler_fence(Ordering::SeqCst);
        let doorbell: &mut DoorbellPage = doorbells[self.uar_idx].as_type_mut(0)?;
        doorbell.cq_sn_cmd_num.set((sn << 28 | cmd << 24 | self.number).to_be());
        doorbell.cq_consumer_index.set(ci.to_be());
        Ok(())
    }

    /// Query this completion queue for debugging purposes.
    pub(super) fn query(&mut self, cmd: &mut CommandInterface) -> Result<(), &'static str> {
        let bytes: MappedPages = cmd.execute_command(Opcode::QueryCq, (), (), self.number)?;
        let ctx = CompletionQueueContext::from_bytes(bytes.as_slice(0, size_of::<CompletionQueueContext>())?.try_into().unwrap());
        trace!("current CQ state: {ctx:?}");
        Ok(())
    }

    /// Poll this completion queue and return the number of new completions.
    ///
    /// This is used by ibv_poll_cq. As of extension 1
    /// (`docs/thesis-plan-1-3.md`), this only ever drains this CQ's own CQE
    /// ring directly - it deliberately does *not* also drain the
    /// associated EQ inline anymore (the previous, commented-out attempt at
    /// that is why `_eqs`/`_doorbells` below are unused parameters kept
    /// only for call-site compatibility). CQEs are written directly to this
    /// ring by hardware, independent of the EQ - the EQ is a pure
    /// notification side channel ("go check CQ N"), not a data source, so
    /// polling it inline here was never actually necessary for correctness,
    /// only a leftover from before EQ draining had a real, async home.
    /// That home is now `Mlx4InterruptHandler::trigger()` ->
    /// `ConnectX3Nic::handle_interrupt()`, which drains the EQ and wakes
    /// `self.wq` (see that field's docs) entirely off of this call path.
    pub(super) fn poll(
        &mut self, _eqs: &mut [EventQueue], qps: &mut [QueuePair], _doorbells: &mut [MappedPages], wc: &mut [ibv_wc],
    ) -> Result<usize, &'static str> {
        let mut completions = 0;
        // poll one for as long as there are elements
        while completions < wc.len() {
            if self.poll_one(qps, &mut wc[completions])? {
                completions += 1;
            } else {
                break;
            }
        }
        let doorbell_record: &mut CompletionQueueDoorbell = self.doorbell_page.as_type_mut(0)?;
        doorbell_record.update_consumer_index.set((self.consumer_index & 0xffffff).to_be());
        Ok(completions)
    }

    /// Poll this completion queue for one work completion.
    ///
    /// Return true if there are more.
    #[allow(unreachable_patterns)]
    fn poll_one(&mut self, qps: &mut [QueuePair], wc: &mut ibv_wc) -> Result<bool, &'static str> {
        const CQE_OPCODE_ERROR: u8 = 0x1e;
        const _CQE_OPCODE_RESIZE: u8 = 0x16;
        // clear the wc first
        *wc = ibv_wc::default();
        if let Some(cqe) = self.get_next_cqe_sw()? {
            self.consumer_index += 1;
            // Make sure we read CQ entry contents after we've checked the
            // ownership bit.
            compiler_fence(Ordering::SeqCst);
            wc.qp_num = cqe.qp_number();
            if let Some(qp) = qps.iter_mut().find(|qp| qp.number() == cqe.qp_number()) {
                let chain_size = qp.query_chain_size(cqe.wqe_index() as usize, cqe.is_send());
                if cqe.is_send() {
                    qp.advance_send_queue_by(chain_size);
                } else {
                    qp.advance_receive_queue_by(chain_size);
                }
                wc.wr_id = qp.query_wr_id(cqe.wqe_index() as usize, cqe.is_send());
            } else {
                warn!("completion has invalid queue pair number {}", cqe.qp_number());
            }
            if cqe.opcode() == CQE_OPCODE_ERROR {
                let checksum_bytes = cqe.checksum().to_be_bytes();
                let vendor_err_syndrome = checksum_bytes[0];
                let syndrome = Syndrome::from_repr(checksum_bytes[1]).ok_or("invalid error syndrome")?;
                error!(
                    "work completion error: (QPN {}, WQE index {}, vendor syndrome {}, syndrome {:?}, opcode {})",
                    cqe.qp_number(),
                    cqe.wqe_index(),
                    vendor_err_syndrome,
                    syndrome,
                    cqe.opcode(),
                );
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
                    _ => ibv_wc_status::Type::IBV_WC_GENERAL_ERR,
                };
                wc.vendor_err = vendor_err_syndrome.into();
                return Ok(true);
            }
            wc.status = ibv_wc_status::IBV_WC_SUCCESS;
            wc.wc_flags = ibv_wc_flags::empty();
            if cqe.is_send() {
                let opcode = QueuePairOpcode::from_repr(cqe.opcode().into()).ok_or("invalid opcode")?;
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
                let opcode = ReceiveOpcode::from_repr(cqe.opcode().into()).ok_or("invalid opcode")?;
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
        } else {
            Ok(false)
        }
    }

    /// Get the next element.
    fn get_next_cqe_sw(&mut self) -> Result<Option<CompletionQueueEntry>, &'static str> {
        let index = self.consumer_index;
        // get the cqe
        let cqe_bytes: &[u8] = self.memory.as_mut().unwrap().0.as_slice(
            (
                // wrap around
                usize::try_from(index & (self.num_entries - 1)).unwrap()
            ) * size_of::<CompletionQueueEntry>(),
            size_of::<CompletionQueueEntry>(),
        )?;
        let cqe = CompletionQueueEntry::from_bytes(cqe_bytes.try_into().unwrap());
        // check if it's valid
        // the ownership bit is flipping every round
        if cqe.owner() ^ ((index & self.num_entries) != 0) {
            Ok(None)
        } else {
            Ok(Some(cqe))
        }
    }

    /// Get the number of this completion queue.
    pub(super) fn number(&self) -> u32 {
        self.number
    }

    /// Get the process that created this completion queue.
    pub(super) fn creator(&self) -> Uuid {
        self.creator
    }

    /// Clone a handle to this CQ's wait queue, for a caller that needs to
    /// block on it *after* releasing `DEV_LIST`'s lock (see `wq`'s docs).
    pub(super) fn wait_queue(&self) -> Arc<WaitQueue> {
        self.wq.clone()
    }

    /// Wake every thread genuinely blocked in `poll_cq` on this CQ. Called
    /// from `Mlx4InterruptHandler::trigger()` once the EQE getter added for
    /// extension 1 (`EventQueueEntry::completion_cqn`) identifies a
    /// `Completion` event naming this CQ's number.
    pub(super) fn notify_waiters(&self) {
        self.wq.notify_all();
    }

    /// Physical start address and byte length of this CQ's CQE ring buffer.
    /// Used to mmap it into the owning process.
    pub(super) fn ring_buffer_region(&self) -> (PhysAddr, usize) {
        let memory = self.memory.as_ref().unwrap();
        (memory.1, memory.0.into_range().len() as usize * PAGE_SIZE)
    }

    /// Physical address of this CQ's doorbell-record page (DMA host memory,
    /// not MMIO).
    pub(super) fn doorbell_phys_addr(&self) -> PhysAddr {
        self.doorbell_address
    }

    /// Index into the NIC's UAR doorbell page vector for this CQ.
    pub(super) fn uar_idx(&self) -> usize {
        self.uar_idx
    }

    /// Number of CQE entries in this CQ (a power of two).
    pub(super) fn entry_count(&self) -> u32 {
        self.num_entries
    }
}

impl Drop for CompletionQueue {
    fn drop(&mut self) {
        if self.memory.is_some() {
            panic!("please destroy instead of dropping")
        }
    }
}

#[bitfield]
#[derive(Debug)]
#[allow(dead_code)]
struct CompletionQueueContext {
    #[skip]
    flags: u32,
    #[skip]
    __: B48,
    #[skip]
    page_offset: u16,
    #[skip]
    __: B3,
    #[skip(getters)]
    log_size: B5,
    #[skip(getters)]
    usr_page: B24,
    #[skip]
    cq_period: u16,
    #[skip]
    cq_max_count: u16,
    #[skip]
    __: B24,
    #[skip(getters)]
    comp_eqn: u8,
    #[skip]
    __: B2,
    #[skip(getters)]
    log_page_size: B6,
    #[skip]
    __: u16,
    // the last three bits must be zero
    #[skip(getters)]
    mtt_base_addr: B40,
    #[skip]
    __: u8,
    #[skip]
    last_notified_index: B24,
    #[skip]
    __: u8,
    #[skip]
    solicit_producer_index: B24,
    #[skip]
    __: u8,
    #[skip]
    consumer_index: B24,
    #[skip]
    __: u8,
    #[skip]
    producer_index: B24,
    #[skip]
    __: u64,
    // the last three bits must be zero
    #[skip(getters)]
    doorbell_record_addr: u64,
}


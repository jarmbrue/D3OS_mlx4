//! A mlx3 driver for a ConnectX-3 card.
//!
//! This is (very) roughly based on [the Nautilus driver](https://github.com/HExSA-Lab/nautilus/blob/master/src/dev/mlx3_ib.c)
//! and the existing mlx5 driver.

mod cmd;
mod completion_queue;
mod device;
mod event_queue;
mod fw;
mod icm;
mod interrupt;
mod port;
mod profile;
mod queue_pair;
mod utils;

use alloc::boxed::Box;
use alloc::sync::Arc;
use alloc::vec::Vec;
use cmd::CommandInterface;
use completion_queue::CompletionQueue;
use event_queue::{EventQueue, init_eqs};
use fw::{Capabilities, Hca, MappedFirmwareArea};
use icm::MappedIcmTables;
use interrupt::Mlx4InterruptHandler;
use log::{info, trace, warn};
use pci_types::{CommandRegister, EndpointHeader};

use rdma::{ibv_access_flags, ibv_device_attr, ibv_port_attr, ibv_qp_attr, ibv_qp_attr_mask, ibv_qp_cap, ibv_qp_type, ibv_wc};
use rdma::uverbs_uapi::{ibv_recv_wr_uapi, ibv_send_wr_uapi};

use crate::interrupt::interrupt_dispatcher::InterruptVector;
use crate::sync::irqsave_spinlock::IrqSaveSpinlock;
use crate::sync::wait_queue::WaitQueue;
use crate::{apic, interrupt_dispatcher, pci_bus};
use port::Port;
use queue_pair::QueuePair;
use spin::{Once, RwLock};
use utils::MappedPages;

use device::{Ownership, ResetRegisters};
use fw::Firmware;
use profile::Profile;

use core::sync::atomic::AtomicUsize;
use core::sync::atomic::Ordering::Relaxed;

use crate::memory::PAGE_SIZE;
use uuid::Uuid;
use x86_64::{PhysAddr, VirtAddr};

/// Physical memory regions backing a queue pair's kernel-bypass fast path,
/// as resolved by [`ConnectX3Nic::mmap_qp_resources`].
pub struct QpMmapResources {
    /// (start address, byte length) of the combined SQ+RQ ring buffer.
    pub ring_buf: (PhysAddr, usize),
    /// Per-QP doorbell record (DMA host memory, cacheable).
    pub qp_doorbell: PhysAddr,
    /// UAR doorbell MMIO page for this QP (uncacheable).
    pub uar_doorbell: PhysAddr,
    /// (start address, byte length) of the BlueFlame MMIO page for this QP,
    /// if the HCA supports it.
    pub bf: Option<(PhysAddr, usize)>,
    pub bf_reg_size: u32,
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
}

/// Physical memory regions backing a completion queue's kernel-bypass fast
/// path, as resolved by [`ConnectX3Nic::mmap_cq_resources`].
pub struct CqMmapResources {
    /// (start address, byte length) of the CQE ring buffer.
    pub ring_buf: (PhysAddr, usize),
    /// Per-CQ doorbell record (DMA host memory, cacheable).
    pub cq_doorbell: PhysAddr,
    pub num_entries: u32,
}

/// Sentinel error returned by the QP/CQ verb handlers below (`modify_qp`,
/// `post_send`, `post_receive`, `destroy_qp`, `poll_cq`, `destroy_cq`) when
/// the calling process does not own the resource it is trying to touch.
/// Matched by string equality at the `uverbs.rs` boundary (`uverbs.rs`'s
/// `map_uverbs_err`) to distinguish "not your QP/CQ" (`Errno::EACCES`) from
/// every other driver error (`Errno::EINVAL`), without introducing a typed
/// error enum for what is otherwise a `&'static str`-error driver.
pub(crate) const ERR_NOT_OWNER: &str = "permission denied: caller does not own this resource";

/// Vendor ID for Mellanox
pub const MLX_VEND: u16 = 0x15b3;
/// Device ID for the ConnectX-3 NIC
pub const CONNECTX3_DEV: u16 = 0x1003;
pub const NUM_SPECIAL_QP: u32 = 8;

const DEVICE_END: usize = 10;
const DEVICE_START: usize = 1;

#[inline(always)]
pub const fn devices_supported() -> usize {
    (DEVICE_END - DEVICE_START) + 1
}

#[inline(always)]
pub fn device_in_range(minor: usize) -> bool {
    (DEVICE_START..=DEVICE_END).contains(&minor)
}

#[inline(always)]
pub fn minor_to_idx(minor: usize) -> usize {
    minor - DEVICE_START
}

static MINOR: AtomicUsize = AtomicUsize::new(DEVICE_START);

/// List of all initialized ConnectX-3 NICs.
///
/// **Critical invariant (extension 1, `docs/thesis-plan-1-3.md`)**: every
/// uverbs syscall handler locks this while running, and the syscall
/// trampoline runs with interrupts enabled
/// (`syscall_dispatcher.rs`, `"sti"`). Once `ConnectX3Nic::init()` registers
/// a legacy INTx handler (`Mlx4InterruptHandler`, `mlx4/interrupt.rs`), that
/// handler *also* needs this lock (to find the device a firing interrupt
/// belongs to and drain its EQ). A plain `spin::Mutex` would self-deadlock
/// the instant the interrupt fires on the same core while a syscall handler
/// already holds it - this was the single biggest correctness risk called
/// out for this extension, and had to be fixed before/alongside interrupt
/// registration, not as a follow-up.
///
/// Fixed by using `IrqSaveSpinlock` instead of `spin::Mutex`: it disables
/// this core's interrupts for the duration of the critical section
/// (`crate::device::cpu::disable_int_nested`), so the mlx4 IRQ line simply
/// cannot fire on this core while anything already holds this lock -  no
/// same-core reentrancy is possible by construction, unlike the
/// `try_lock()`/`force_unlock()` idiom used elsewhere in this codebase for
/// the same *class* of hazard (`InterruptDispatcher::dispatch`,
/// `Apic::end_of_interrupt`, and this extension's own
/// `Scheduler::force_try_lock` for `ready_state`/`blocked_list`, which
/// couldn't use this same stronger fix without a much larger, out-of-scope
/// change to `Scheduler`'s locking).
static DEV_LIST: Once<IrqSaveSpinlock<Vec<ConnectX3Nic>>> = Once::new();

fn next_minor() -> usize {
    MINOR.fetch_add(1, Relaxed)
}

/// List of all initialized ConnectX-3 NICs
pub fn get_dev_list() -> &'static IrqSaveSpinlock<Vec<ConnectX3Nic>> {
    DEV_LIST.call_once(|| IrqSaveSpinlock::new(Vec::with_capacity(devices_supported())))
}

/// Struct representing a ConnectX-3 card
pub struct ConnectX3Nic {
    config_regs: MappedPages,
    cmd: CommandInterface,
    firmware: Firmware,
    firmware_area: MappedFirmwareArea,
    capabilities: Capabilities,
    offsets: Offsets,
    icm_tables: MappedIcmTables,
    hca: Hca,
    doorbells: Vec<MappedPages>,
    blueflame: Vec<MappedPages>,
    eqs: Vec<EventQueue>,
    // TODO: find some way to bind this to the relevant EQ
    cqs: Vec<CompletionQueue>,
    qps: Vec<QueuePair>,
    ports: Vec<Port>,
    pub minor: usize,
}

/// Functions that setup the struct.
impl ConnectX3Nic {
    /// Initializes the ConnectX-3 card that is connected as the given PciDevice.
    /// Adds the device to the global List of ConnectX-3 NICs
    ///
    /// # Arguments
    /// * `mlx3_pci_dev`: Contains the pci device information.
    pub fn init(mlx3_pci_dev: &RwLock<EndpointHeader>) -> Result<usize, &'static str> {
        if MINOR.load(Relaxed) > DEVICE_END {
            return Err("Max devices reached !");
        }

        let config_space = pci_bus().config_space();
        let mut mlx3_pci_dev = mlx3_pci_dev.write();

        // Disable Memory Space decoding before reading the BARs. Sizing a BAR
        // transiently writes 0xFFFFFFFF to it, and if memory decoding is enabled
        // the device would briefly decode at that bogus (all-ones) address, which
        // the host (QEMU/KVM) tries to map and rejects.
        mlx3_pci_dev.update_command(config_space, |creg| creg & !CommandRegister::MEMORY_ENABLE);

        // map the Global Device Configuration registers
        let mut config_regs = utils::pci_map_bar_mem(&mlx3_pci_dev, 0, config_space)?;
        trace!("mlx3 configuration registers: {:?}", config_regs);

        // map the User Access Region
        let user_access_region = utils::pci_map_bar_mem(&mlx3_pci_dev, 2, &config_space)?;
        trace!("mlx3 user access region: {:?}", user_access_region);

        // set the memory space bit for this PciDevice
        // set the bus mastering bit for this PciDevice, which allows it to use DMA
        mlx3_pci_dev.update_command(config_space, |creg| creg | CommandRegister::MEMORY_ENABLE | CommandRegister::BUS_MASTER_ENABLE);

        ResetRegisters::reset(&mlx3_pci_dev, &mut config_regs)?;

        // TODO: This shouldn't be necessary.
        // We should be restoring the config space in reset(),
        // but even now these bits are always set.

        mlx3_pci_dev.update_command(config_space, |creg| creg | CommandRegister::MEMORY_ENABLE | CommandRegister::BUS_MASTER_ENABLE);

        // In linux driver ownership is taken before reset
        Ownership::get(&config_regs)?;
        let mut cmd = CommandInterface::new(&mut config_regs)?;
        let firmware = Firmware::query(&mut cmd)?;
        let mut firmware_area = firmware.map_area(&mut cmd)?;
        firmware_area.run(&mut cmd)?;
        let capabilities = firmware_area.repeat_query_capabilities(&mut cmd)?;

        // In the Nautilus driver, some of the port setup already happens here.

        let mut offsets = Offsets::init(&capabilities);
        let mut profile = Profile::new(&capabilities)?;
        let aux_pages = firmware_area.set_icm(&mut cmd, profile.total_size)?;
        let icm_aux_area = firmware_area.map_icm_aux(&mut cmd, aux_pages)?;
        let mut icm_tables = icm_aux_area.map_icm_tables(&mut cmd, &profile, &capabilities)?;
        let hca = profile.init_hca.init_hca(&mut cmd)?;

        // give us the interrupt pin
        hca.query_adapter(&mut cmd)?;

        // `minor` is needed both by the interrupt handler below (to know
        // which device in `DEV_LIST` a firing interrupt belongs to) and by
        // the EQ setup, so it has to be computed before both instead of
        // right before pushing into `DEV_LIST` as before. `next_minor()`
        // only touches its own atomic counter, so moving it earlier is
        // side-effect-free.
        let minor = next_minor();

        // Legacy INTx only - D3OS has no MSI-X support anywhere, matching
        // `rtl8139.rs`/`virtio/mod.rs`. Mirrors `Rtl8139::plugin()`
        // (`device/rtl8139.rs`) and `virtio/mod.rs`'s interrupt wiring: read
        // the PCI interrupt line, convert to a host `InterruptVector`,
        // assign our handler to it and unmask it at the APIC. A line value
        // of 0 or 0xFF means "no legacy interrupt assigned" (matching
        // `virtio/mod.rs`'s own check) - fall back to the pre-existing
        // always-polling EQ mode in that case rather than registering a
        // handler for a nonsensical vector.
        let (_, interrupt_line) = mlx3_pci_dev.interrupt(config_space);
        let interrupts_enabled = if interrupt_line != 0 && interrupt_line != 0xFF {
            match InterruptVector::try_from(interrupt_line + 32) {
                Ok(vector) => {
                    interrupt_dispatcher().assign(vector, Box::new(Mlx4InterruptHandler::new(minor)));
                    apic().allow(vector);
                    info!("mlx4: registered legacy INTx handler for device minor {minor} on vector {vector:?}");
                    true
                }
                Err(_) => {
                    warn!("mlx4: PCI interrupt line {interrupt_line} does not map to a valid host vector, falling back to polling EQ");
                    false
                }
            }
        } else {
            warn!("mlx4: no legacy PCI interrupt line assigned, falling back to polling EQ");
            false
        };

        // get the doorbells and the BlueFlame section
        let (mut doorbells, blueflame) = capabilities.get_doorbells_and_blueflame(user_access_region)?;
        let eqs = init_eqs(&mut cmd, &mut doorbells, &capabilities, &mut offsets, icm_tables.memory_regions(), interrupts_enabled)?;

        hca.config_mad_demux(&mut cmd, &capabilities)?;

        // TODO: Configure Special QPs (QP0, QP1) for SMI and GSI MAD packets
        //       before initializing the ports
        //let _: () = cmd.execute_command(cmd::Opcode::ConfSpecialQp, (), (), offsets.base_qpn)?;

        let ports = hca.init_ports(&mut cmd, &capabilities, offsets.base_qpn)?;

        let nic = Self {
            cmd,
            config_regs,
            firmware,
            firmware_area,
            capabilities,
            offsets,
            icm_tables,
            hca,
            doorbells,
            blueflame,
            eqs,
            cqs: Vec::new(),
            qps: Vec::new(),
            ports,
            minor,
        };
        get_dev_list().lock().push(nic);
        Ok(minor)
    }

    /// Get statistics about the device.
    ///
    /// This is used by ibv_query_device.
    pub fn query_device(&mut self) -> Result<ibv_device_attr, &'static str> {
        Ok(ibv_device_attr {
            fw_ver_major: self.firmware.major.get(),
            fw_ver_minor: self.firmware.minor.get(),
            fw_ver_subminor: self.firmware.sub_minor.get(),
            phys_port_cnt: self.ports.len().try_into().unwrap(),
        })
    }

    /// Get statistics about a port.
    ///
    /// This is used by ibv_query_port.
    pub fn query_port(&mut self, port_num: u8) -> Result<ibv_port_attr, &'static str> {
        let port: Option<&mut Port> = self.ports.get_mut(port_num as usize - 1);
        if let Some(port) = port {
            port.query(&mut self.cmd)
        } else {
            Err("port does not exist")
        }
    }

    /// Create a completion queue and return its number.
    ///
    /// This is used by ibv_create_cq.
    pub fn create_cq(&mut self, min_num_entries: i32) -> Result<u32, &'static str> {
        // TODO min_num_entries should be u32
        let mut cq = CompletionQueue::new(
            &mut self.cmd,
            &mut self.capabilities,
            &mut self.offsets,
            self.icm_tables.memory_regions(),
            self.eqs.get(0),
            min_num_entries.try_into().unwrap(),
        )?;
        cq.arm(&mut self.doorbells)?;
        cq.query(&mut self.cmd)?;
        let number = cq.number();
        self.cqs.push(cq);
        Ok(number)
    }

    /// Poll a completion queue and return the number of new completions.
    ///
    /// This is used by ibv_poll_cq. Refuses to poll a CQ created by a
    /// different process (`ERR_NOT_OWNER`) - this is also the check
    /// extension 1's eventual blocking `poll_cq` design gates
    /// "may this caller block-wait on this CQ" on, so keep it as the single,
    /// obviously-reusable choke point.
    pub fn poll_cq(&mut self, number: u32, caller: Uuid, wc: &mut [ibv_wc]) -> Result<usize, &'static str> {
        let cq = self.cqs.iter_mut().find(|cq| cq.number() == number).ok_or("invalid completion queue number")?;
        if cq.creator() != caller {
            return Err(ERR_NOT_OWNER);
        }
        cq.poll(&mut self.eqs, &mut self.qps, &mut self.doorbells, wc)
    }

    /// Look up `number`, verify `caller` owns it (the exact same
    /// `cq.creator() != caller` check `poll_cq` above uses - deliberately
    /// re-run here rather than trusted from an earlier call, since this is
    /// the gate that decides "may this caller block-wait on this CQ"), arm
    /// it for the next completion interrupt, and return a cloneable handle
    /// to its wait queue.
    ///
    /// This is the building block `uverbs_cmd::uverbs_poll_cq`'s blocking
    /// mode uses to wait *without* holding `DEV_LIST`'s lock - see
    /// `CompletionQueue::wq`'s docs for why blocking while still holding it
    /// would deadlock `Mlx4InterruptHandler::trigger()`, which also needs
    /// that lock to find this CQ.
    pub fn arm_cq_for_wait(&mut self, number: u32, caller: Uuid) -> Result<Arc<WaitQueue>, &'static str> {
        let cq = self.cqs.iter_mut().find(|cq| cq.number() == number).ok_or("invalid completion queue number")?;
        if cq.creator() != caller {
            return Err(ERR_NOT_OWNER);
        }
        cq.arm(&mut self.doorbells)?;
        Ok(cq.wait_queue())
    }

    /// Re-arm `number` for the next completion interrupt. Used by the
    /// blocking `poll_cq` loop between "found nothing, about to wait again"
    /// iterations - hardware CQ arming is effectively one-shot per
    /// completion, so every time the wait predicate is about to block
    /// again it must re-arm first, or a completion that arrives while
    /// "unarmed" would never generate an interrupt and the waiter would
    /// hang forever. No ownership check here: this is only ever called
    /// immediately after `arm_cq_for_wait` or `poll_cq` already validated
    /// ownership for the same `(caller, number)` pair in the same blocking
    /// call, with no intervening yield that could let the CQ change hands.
    pub fn rearm_cq(&mut self, number: u32) -> Result<(), &'static str> {
        let cq = self.cqs.iter_mut().find(|cq| cq.number() == number).ok_or("invalid completion queue number")?;
        cq.arm(&mut self.doorbells)
    }

    /// Drain this device's (single, `NUM_EQS == 1`) event queue and wake
    /// any thread blocked in `poll_cq` on a completion queue that just got
    /// a `Completion` EQE. Called from `Mlx4InterruptHandler::trigger()`,
    /// itself called by `InterruptDispatcher::dispatch()` while holding
    /// `get_dev_list()`'s `IrqSaveSpinlock` (so interrupts are already
    /// masked on this core for the whole call - see `DEV_LIST`'s docs).
    pub(crate) fn handle_interrupt(&mut self) {
        let minor = self.minor;
        let Self { eqs, cqs, doorbells, .. } = self;
        let Some(eq) = eqs.get_mut(0) else { return };
        if let Err(e) = eq.handle_events(doorbells, |cqn| {
            if let Some(cq) = cqs.iter().find(|cq| cq.number() == cqn) {
                cq.notify_waiters();
            } else {
                warn!("mlx4: interrupt on device minor {minor} named unknown CQ {cqn}");
            }
        }) {
            log::error!("mlx4: error draining event queue on device minor {minor}: {e}");
        }
    }

    /// Destroy a completion queue. Refuses to destroy a CQ created by a
    /// different process (`ERR_NOT_OWNER`).
    pub fn destroy_cq(&mut self, number: u32, caller: Uuid) -> Result<(), &'static str> {
        let (index, cq) = self
            .cqs
            .iter()
            .enumerate()
            .find(|(_, cq)| cq.number() == number)
            .ok_or("completion queue not found")?;
        if cq.creator() != caller {
            return Err(ERR_NOT_OWNER);
        }
        let cq = self.cqs.remove(index);
        cq.destroy(&mut self.cmd)?;
        Ok(())
    }

    /// Create a queue pair and return its number. Refuses to bind the new
    /// QP to a send/receive CQ created by a different process
    /// (`ERR_NOT_OWNER`) - without this, `post_send`/`post_receive`'s
    /// per-QP ownership check wouldn't be enough on its own, since a
    /// process could still create a QP that delivers completions into a
    /// CQ it doesn't own (readable via that other process's `poll_cq`,
    /// itself ownership-checked, but the *binding* is what needs to be
    /// prevented here).
    ///
    /// This is used by ibv_create_qp.
    pub fn create_qp(
        &mut self, qp_type: ibv_qp_type::Type, caller: Uuid, send_cq_number: u32, receive_cq_number: u32, ib_caps: &mut ibv_qp_cap,
    ) -> Result<u32, &'static str> {
        let send_cq = self
            .cqs
            .iter()
            .find(|cq| cq.number() == send_cq_number)
            .ok_or("invalid send completion queue number")?;
        if send_cq.creator() != caller {
            return Err(ERR_NOT_OWNER);
        }
        let receive_cq = self
            .cqs
            .iter()
            .find(|cq| cq.number() == receive_cq_number)
            .ok_or("invalid receive completion queue number")?;
        if receive_cq.creator() != caller {
            return Err(ERR_NOT_OWNER);
        }
        let qp = QueuePair::new(
            &mut self.cmd,
            &mut self.capabilities,
            &mut self.offsets,
            self.icm_tables.memory_regions(),
            qp_type,
            send_cq,
            receive_cq,
            ib_caps,
        )?;
        let number = qp.number();
        self.qps.push(qp);
        Ok(number)
    }

    /// Modify a queue pair.
    ///
    /// This is used by ibv_modify_qp. Refuses to modify a QP created by a
    /// different process (`ERR_NOT_OWNER`).
    pub fn modify_qp(&mut self, number: u32, caller: Uuid, attr: &ibv_qp_attr, attr_mask: ibv_qp_attr_mask) -> Result<(), &'static str> {
        let qp = self.qps.iter_mut().find(|qp| qp.number() == number).ok_or("invalid queue pair number")?;
        if qp.creator() != caller {
            return Err(ERR_NOT_OWNER);
        }
        qp.modify(&mut self.cmd, &mut self.capabilities, attr, attr_mask)
    }

    /// Destroy a queue pair. Refuses to destroy a QP created by a different
    /// process (`ERR_NOT_OWNER`).
    pub fn destroy_qp(&mut self, number: u32, caller: Uuid) -> Result<(), &'static str> {
        let (index, qp) = self
            .qps
            .iter()
            .enumerate()
            .find(|(_, qp)| qp.number() == number)
            .ok_or("queue pair not found")?;
        if qp.creator() != caller {
            return Err(ERR_NOT_OWNER);
        }
        let qp = self.qps.remove(index);
        qp.destroy(&mut self.cmd, &mut self.capabilities)?;
        Ok(())
    }

    /// Post a work request to receive data.
    ///
    /// This is used by ibv_post_recv. Refuses to post to a QP created by a
    /// different process (`ERR_NOT_OWNER`).
    pub fn post_receive(&mut self, qp_number: u32, caller: Uuid, wr: &ibv_recv_wr_uapi) -> Result<(), &'static str> {
        let qp = self.qps.iter_mut().find(|qp| qp.number() == qp_number).ok_or("invalid queue pair number")?;
        if qp.creator() != caller {
            return Err(ERR_NOT_OWNER);
        }
        qp.post_receive(wr)
    }

    /// Post a work request to send data.
    ///
    /// This is used by ibv_post_send. Refuses to post to a QP created by a
    /// different process (`ERR_NOT_OWNER`).
    pub fn post_send(&mut self, qp_number: u32, caller: Uuid, wr: &ibv_send_wr_uapi) -> Result<(), &'static str> {
        let qp = self.qps.iter_mut().find(|qp| qp.number() == qp_number).ok_or("invalid queue pair number")?;
        if qp.creator() != caller {
            return Err(ERR_NOT_OWNER);
        }
        // TODO: check if blue flame is available
        qp.post_send(&mut self.capabilities, &mut self.doorbells, Some(&mut self.blueflame), wr)
    }

    /// Resolve the physical memory regions backing `qp_number`'s ring
    /// buffer, doorbell record and UAR doorbell/BlueFlame page(s), for the
    /// `UVERBS_CMD_MMAP_QP` syscall handler to map into the calling
    /// process. Refuses to resolve a QP created by a different process.
    pub fn mmap_qp_resources(&mut self, qp_number: u32, caller: Uuid) -> Result<QpMmapResources, &'static str> {
        let qp = self.qps.iter().find(|qp| qp.number() == qp_number).ok_or("invalid queue pair number")?;
        if qp.creator() != caller {
            return Err("queue pair belongs to a different process");
        }
        let uar_idx = qp.uar_idx();
        let uar_doorbell = utils::get_physical_address(VirtAddr::new(self.doorbells[uar_idx].into_range().start.start_address().as_u64()));
        let bf = if self.capabilities.bf() {
            let page = &self.blueflame[uar_idx];
            let addr = utils::get_physical_address(VirtAddr::new(page.into_range().start.start_address().as_u64()));
            Some((addr, page.into_range().len() as usize * PAGE_SIZE))
        } else {
            None
        };
        let (sq_offset, sq_wqe_cnt, sq_wqe_shift, sq_spare_wqes, sq_max_gs, sq_max_post) = qp.sq_geometry();
        let (rq_offset, rq_wqe_cnt, rq_wqe_shift, rq_max_gs, rq_max_post) = qp.rq_geometry();
        Ok(QpMmapResources {
            ring_buf: qp.ring_buffer_region(),
            qp_doorbell: qp.doorbell_phys_addr(),
            uar_doorbell,
            bf,
            bf_reg_size: self.capabilities.bf_reg_size() as u32,
            sq_offset,
            sq_wqe_cnt,
            sq_wqe_shift,
            sq_spare_wqes,
            sq_max_gs,
            sq_max_post,
            rq_offset,
            rq_wqe_cnt,
            rq_wqe_shift,
            rq_max_gs,
            rq_max_post,
        })
    }

    /// Resolve the physical memory regions backing `cq_number`'s CQE ring
    /// buffer and doorbell record, for the `UVERBS_CMD_MMAP_CQ` syscall
    /// handler to map into the calling process. Refuses to resolve a CQ
    /// created by a different process.
    pub fn mmap_cq_resources(&mut self, cq_number: u32, caller: Uuid) -> Result<CqMmapResources, &'static str> {
        let cq = self.cqs.iter().find(|cq| cq.number() == cq_number).ok_or("invalid completion queue number")?;
        if cq.creator() != caller {
            return Err("completion queue belongs to a different process");
        }
        Ok(CqMmapResources {
            ring_buf: cq.ring_buffer_region(),
            cq_doorbell: cq.doorbell_phys_addr(),
            num_entries: cq.entry_count(),
        })
    }

    /// Create a memory region and return its index, physical address, lkey and rkey.
    ///
    /// This is used by ibv_reg_mr.
    pub fn create_mr<T>(&mut self, data: &mut [T], access: ibv_access_flags) -> Result<(u32, usize, u32, u32), &'static str> {
        // TODO: this fails for large memory regions (>= 64 MB)
        self.icm_tables.memory_regions().alloc_dmpt(
            &mut self.cmd,
            &mut self.capabilities,
            &mut self.offsets,
            data,
            None,
            access,
        )
    }

    /// Destroy a memory region.
    pub fn destroy_mr(&mut self, index: u32) -> Result<(), &'static str> {
        self.icm_tables.memory_regions().destroy(&mut self.cmd, index)
    }
}

impl Drop for ConnectX3Nic {
    fn drop(&mut self) {
        self.icm_tables.memory_regions().destroy_all(&mut self.cmd).unwrap();
        while let Some(qp) = self.qps.pop() {
            qp.destroy(&mut self.cmd, &mut self.capabilities).unwrap()
        }
        while let Some(cq) = self.cqs.pop() {
            cq.destroy(&mut self.cmd).unwrap()
        }
        while let Some(port) = self.ports.pop() {
            port.close(&mut self.cmd).unwrap()
        }
        while let Some(eq) = self.eqs.pop() {
            eq.destroy(&mut self.cmd).unwrap()
        }
        self.hca.close(&mut self.cmd).unwrap();
        self.icm_tables.unmap(&mut self.cmd).unwrap();
        self.firmware_area.unmap(&mut self.cmd).unwrap();
    }
}

struct Offsets {
    base_qpn: u32,
    next_cqn: usize,
    next_qpn: usize,
    next_dmpt: usize,
    next_eqn: usize,
    next_sqc_doorbell_index: usize,
    // TODO: EventQueue does not seem to need this.
    // Should it use this to be more similar to QueuePair?
    _next_eq_doorbell_index: usize,
}

impl Offsets {
    /// Initialize the queue offsets.
    pub(in crate::device::mlx4) fn init(caps: &Capabilities) -> Self {
        let end_reserved_cpn: u32 = 1 << caps.log2_rsvd_cqs();
        // Reserve numbers for special qp. Base_qpn must be naturally aliged
        let base_qpn = end_reserved_cpn.next_multiple_of(NUM_SPECIAL_QP);
        Self {
            base_qpn,
            // This should return the first non reserved cq, qp, eq number.
            next_cqn: (base_qpn + NUM_SPECIAL_QP) as usize,
            next_qpn: 1 << caps.log2_rsvd_qps(),
            next_dmpt: 1 << caps.log2_rsvd_mrws(),
            next_eqn: caps.num_rsvd_eqs().into(),
            // For SQ and CQ Uar Doorbell index starts from 128
            next_sqc_doorbell_index: 128,
            // Each UAR has 4 EQ doorbells; so if a UAR is reserved,
            // then we can't use any EQs whose doorbell falls on that page,
            // even if the EQ itself isn't reserved.
            _next_eq_doorbell_index: caps.num_rsvd_eqs() as usize / 4,
        }
    }

    /// Allocate an event queue number.
    pub(in crate::device::mlx4) fn alloc_eqn(&mut self) -> usize {
        let res = self.next_eqn;
        self.next_eqn += 1;
        res
    }

    /// Allocate a completion queue number.
    pub(in crate::device::mlx4) fn alloc_cqn(&mut self) -> usize {
        let res = self.next_cqn;
        self.next_cqn += 1;
        res
    }

    /// Allocate a queue pair number.
    pub(in crate::device::mlx4) fn alloc_qpn(&mut self) -> usize {
        let res = self.next_qpn;
        self.next_qpn += 1;
        res
    }

    /// Allocate a doorbell for SCQs.
    pub(in crate::device::mlx4) fn alloc_scq_db(&mut self) -> usize {
        let res = self.next_sqc_doorbell_index;
        self.next_sqc_doorbell_index += 1;
        res
    }

    /// Allocate a dmpt offset.
    pub(in crate::device::mlx4) fn alloc_dmpt(&mut self) -> usize {
        let res = self.next_dmpt;
        self.next_dmpt += 256;
        res
    }
}

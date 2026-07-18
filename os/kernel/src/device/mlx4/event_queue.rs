//! This module consists of functions that create, work with and destroy event queues.
//! Additionally it holds the interrupt handling function to consume EQEs.

use core::{
    mem::size_of,
    sync::atomic::{compiler_fence, Ordering},
};

use super::utils;
use super::utils::MappedPages;
use crate::memory::PAGE_SIZE;
use alloc::vec::Vec;
use bitflags::bitflags;
use log::trace;
use modular_bitfield_msb::{
    bitfield,
    specifiers::{B10, B16, B2, B22, B24, B4, B40, B5, B6, B60, B64, B7, B72, B96},
};
use rdma::mlx4_hw::DoorbellPage;
use strum_macros::FromRepr;
use tock_registers::interfaces::Writeable;

use super::{
    cmd::{CommandInterface, Opcode},
    device::PAGE_SHIFT,
    fw::Capabilities,
    icm::{MrTable, ICM_PAGE_SHIFT},
    Offsets,
};

const _NUM_ASYNC_EQE: u32 = 0x100;
const NUM_SPARE_EQE: u32 = 0x80;

/// Initialize the event queues.
/// This creates all of the EQs ahead of time,
/// passes their ownership to the hardware and calls MapEq.
///
/// `interrupts_enabled` reflects whether `ConnectX3Nic::init()` managed to
/// register a legacy INTx handler for this device with the interrupt
/// dispatcher/APIC (see `mlx4/interrupt.rs`). If so, the EQ is programmed
/// `EQ_STATE_ARMED` (hardware will raise the shared legacy INTx line on new
/// events) instead of `EQ_STATE_FIRED` (the old, permanently-polling mode);
/// see [`EventQueue::new`] for what value is actually programmed into the
/// EQ context's `intr` field either way.
pub(super) fn init_eqs(
    cmd: &mut CommandInterface, doorbells: &mut [MappedPages], caps: &Capabilities, offsets: &mut Offsets, memory_regions: &mut MrTable,
    interrupts_enabled: bool,
) -> Result<Vec<EventQueue>, &'static str> {
    const NUM_EQS: usize = 1;
    let mut eqs = Vec::with_capacity(NUM_EQS);
    for _ in 0..NUM_EQS {
        // Legacy INTx only (no MSI-X support in this driver): with a single
        // shared interrupt line and a single completion EQ, the hardware
        // "interrupt vector index" this device needs is always 0 - see
        // `EventQueue::new`'s docs on the `base_vector` parameter for the
        // derivation (cross-checked against Linux mlx4_core's
        // `mlx4_create_eq`/`mlx4_init_eq_table`).
        let eq = EventQueue::new(cmd, caps, offsets, memory_regions, interrupts_enabled.then_some(0u8))?;
        eqs.push(eq);
    }
    // map all events to the first (and only) event queue
    eqs[0].map(cmd)?;
    eqs[0].ring(doorbells, true)?;
    Ok(eqs)
}

#[derive(Debug)]
pub(super) struct EventQueue {
    number: usize,
    num_entries: u32,
    memory: Option<utils::PageToFrameMapping>,
    // TODO: somehow free this on Drop
    _mtt: u64,
    consumer_index: u32,
    /// IRQ number on bus
    _intr_vector: Option<u8>,
    /// IRQ we will see
    _base_vector: Option<u8>,
    /// event bitmask
    async_ev_mask: AsyncEventMask,
}

impl EventQueue {
    /// Create a new event queue. If `base_vector` is given, it will be
    /// interrupt driven, else it will be polled.
    ///
    /// `base_vector` is *not* an x86 IDT vector, a PCI IRQ line/pin, or any
    /// other host-side interrupt number - despite the field's name (kept
    /// from the pre-extension-1 scaffolding) it is the HCA's own
    /// "completion vector index", written verbatim into the EQ context's
    /// `intr` field (`ctx.set_intr(...)` below). Cross-checked against
    /// upstream Linux mlx4_core (`drivers/net/ethernet/mellanox/mlx4/eq.c`,
    /// `mlx4_create_eq`/`mlx4_init_eq_table`): `eq_context->intr = intr;`
    /// verbatim, and when the device has no MSI-X support (this driver's
    /// case, always - `NUM_EQS == 1`, one shared legacy INTx line), every
    /// caller of `mlx4_create_eq` passes `intr = 0`. The actual host-side
    /// interrupt wiring (x86 vector, `InterruptDispatcher::assign`,
    /// `Apic::allow`) happens once, at the whole-device level, in
    /// `ConnectX3Nic::init()` (`mlx4/interrupt.rs`) - entirely independent
    /// of this value.
    fn new(
        cmd: &mut CommandInterface, caps: &Capabilities, offsets: &mut Offsets, memory_regions: &mut MrTable, base_vector: Option<u8>,
    ) -> Result<Self, &'static str> {
        // EQE size is 32. There is 64 B support also available in CX3.
        const EQE_SIZE: usize = 32;
        const EQ_STATUS_OK: u8 = 0;
        const EQ_STATE_ARMED: u8 = 9;
        const EQ_STATE_FIRED: u8 = 0xa;
        let number = offsets.alloc_eqn();
        let num_entries: u32 = 4096; // NUM_ASYNC_EQE + NUM_SPARE_EQE
        let consumer_index = 0;
        let mut num_pages = (num_entries as usize * EQE_SIZE).next_multiple_of(PAGE_SIZE) / PAGE_SIZE;
        // not needed if 128 EQE entries
        if num_pages == 0 {
            num_pages = 1;
        }
        let mapped_page_to_frame = utils::create_cont_mapping_with_dma_flags(utils::pages_required(num_pages * PAGE_SIZE + EQE_SIZE - 1))?.fetch_in_addr()?;

        let mtt = memory_regions.alloc_mtt(cmd, caps, num_pages, mapped_page_to_frame.1)?;
        // TODO: Should use MSI-X instead of legacy INTs
        //
        // There is nothing left to "resolve" here beyond `base_vector`
        // itself: with a single EQ and no MSI-X, there is no separate
        // host-side vector to look up or register from inside this
        // constructor (that already happened once, device-wide, in
        // `ConnectX3Nic::init()` before this function was ever called) -
        // `_intr_vector`/`_base_vector` both just record the same
        // "completion vector index" value for debugging (see this
        // function's doc comment).
        let intr_vector = base_vector;

        let mut ctx = EventQueueContext::new();
        ctx.set_status(EQ_STATUS_OK);
        ctx.set_state(if base_vector.is_some() { EQ_STATE_ARMED } else { EQ_STATE_FIRED });
        ctx.set_log_eq_size(num_entries.ilog2().try_into().unwrap());
        if let Some(base_vector) = base_vector {
            ctx.set_intr(base_vector.try_into().unwrap());
        }
        ctx.set_log_page_size(PAGE_SHIFT - ICM_PAGE_SHIFT);
        ctx.set_mtt_base_addr(mtt);
        let _: () = cmd.execute_command(Opcode::Sw2HwEq, (), &ctx.bytes[..], number.try_into().unwrap())?;

        let async_ev_mask = AsyncEventMask::empty();
        let eq = Self {
            number,
            num_entries,
            memory: Some(mapped_page_to_frame),
            _mtt: mtt,
            consumer_index,
            _intr_vector: intr_vector,
            _base_vector: base_vector,
            async_ev_mask,
        };
        trace!("created new EQ: {:?}", eq);
        Ok(eq)
    }

    /// Map all event types to this EQ.
    // TODO: should parameterize the types of events given to this EQ
    fn map(&mut self, cmd: &mut CommandInterface) -> Result<(), &'static str> {
        // TODO: unmask IRQ
        self.async_ev_mask = AsyncEventMask::all();
        let unmap = false;
        let _: () = cmd.execute_command(
            Opcode::MapEq,
            (),
            self.async_ev_mask.bits(),
            ((unmap as u32) << 31) | u32::try_from(self.number).unwrap(),
        )?;
        Ok(())
    }

    /// Unmap all events from this EQ.
    fn unmap(&mut self, cmd: &mut CommandInterface) -> Result<(), &'static str> {
        let unmap = true;
        let _: () = cmd.execute_command(
            Opcode::MapEq,
            (),
            self.async_ev_mask.bits(),
            ((unmap as u32) << 31) | u32::try_from(self.number).unwrap(),
        )?;
        self.async_ev_mask = AsyncEventMask::empty();
        Ok(())
    }

    /// Destroy the event queue.
    pub(super) fn destroy(mut self, cmd: &mut CommandInterface) -> Result<(), &'static str> {
        if !self.async_ev_mask.is_empty() {
            self.unmap(cmd)?;
        }
        let _: () = cmd.execute_command(Opcode::Hw2SwEq, (), (), self.number.try_into().unwrap())?;
        // actually free the memory
        self.memory.take().unwrap();
        Ok(())
    }

    /// Ring this event queue by writing the consumer index to the appropriate
    /// doorbell.
    ///
    /// If armed, events will generate interrupts.
    fn ring(&mut self, doorbells: &mut [MappedPages], arm: bool) -> Result<(), &'static str> {
        // for the EQ number n the relevant doorbell is in
        // DoorbellPage (n / 4) and eq (n % 4)
        let doorbell: &mut DoorbellPage = doorbells[self.number / 4].as_type_mut(0)?;
        doorbell.eqs[self.number % 4].val.set(((self.consumer_index & 0xffffff) | (arm as u32) << 31).to_be());
        // We still want ordering, just not swabbing, so add a barrier
        compiler_fence(Ordering::SeqCst);
        Ok(())
    }

    /// Handle events, draining every EQE currently visible in the ring.
    ///
    /// This can be called manually (polling) or from an interrupt - as of
    /// extension 1, `Mlx4InterruptHandler::trigger()` is the only caller,
    /// via `ConnectX3Nic::handle_interrupt()`.
    ///
    /// `on_completion` is invoked once per `Completion`-type EQE drained,
    /// with the CQ number it names (see [`EventQueueEntry::completion_cqn`]).
    /// Every other EQE type is drained (so the EQ doesn't stall) but
    /// otherwise ignored for now - async/error events (`CqError`,
    /// `PortChange`, ...) aren't wired to anything yet, matching this
    /// driver's pre-existing scope (see `EventType`'s variants below).
    ///
    /// Always re-arms the EQ for further interrupts before returning
    /// (`self.ring(doorbells, true)`), matching the pre-existing behavior
    /// this function already had.
    pub(super) fn handle_events<F: FnMut(u32)>(&mut self, doorbells: &mut [MappedPages], mut on_completion: F) -> Result<(), &'static str> {
        let mut set_ci: u32 = 0;
        loop {
            match self.poll_one()? {
                Some(ConsumedEvent::Completion(cqn)) => {
                    on_completion(cqn);
                    set_ci += 1;
                }
                Some(ConsumedEvent::Other) => {
                    set_ci += 1;
                }
                None => break,
            }
            if set_ci >= NUM_SPARE_EQE {
                self.ring(doorbells, false)?;
                set_ci = 0;
            }
        }
        self.ring(doorbells, true)?;
        Ok(())
    }

    /// Poll this event queue for one event, returning what kind it was (if
    /// any) so [`handle_events`](Self::handle_events) can act on
    /// `Completion` events without leaking the raw [`EventQueueEntry`]
    /// bitfield type outside this module.
    fn poll_one(&mut self) -> Result<Option<ConsumedEvent>, &'static str> {
        if let Some(eqe) = self.get_next_eqe_sw()? {
            self.consumer_index += 1;
            trace!("got eqe: {:?}", eqe);
            // Make sure we read CQ entry contents after we've checked the
            // ownership bit.
            compiler_fence(Ordering::SeqCst);
            let event = match EventType::from_repr(eqe.event_type().into()) {
                Some(EventType::Completion) => ConsumedEvent::Completion(eqe.completion_cqn()),
                _ => ConsumedEvent::Other,
            };
            Ok(Some(event))
        } else {
            Ok(None)
        }
    }

    /// Get the next entry.
    fn get_next_eqe_sw(&mut self) -> Result<Option<EventQueueEntry>, &'static str> {
        let index = self.consumer_index;
        // get the eqe
        let eqe_bytes: &[u8] = self.memory.as_mut().unwrap().0.as_slice(
            (
                // wrap around
                usize::try_from(index & (self.num_entries - 1)).unwrap()
            ) * size_of::<EventQueueEntry>(),
            // TODO: CX3 is capable of extending the EQE from 32 to 64 bytes
            // with strides of 64B, 128B and 256B. When 64B EQE is used, the
            // first (in the lower addresses) 32 bytes in the 64 byte EQE are
            // reserved and the next 32 bytes contain the legacy EQE information.
            // In all other cases, the first 32B contains the legacy EQE info.
            size_of::<EventQueueEntry>(),
        )?;
        let eqe = EventQueueEntry::from_bytes(eqe_bytes.try_into().unwrap());
        // check if it's valid
        // the ownership bit is flipping every round
        if eqe.owner() ^ ((index & self.num_entries) != 0) {
            Ok(None)
        } else {
            Ok(Some(eqe))
        }
    }

    /// Get the number of this event queue.
    pub(super) fn number(&self) -> usize {
        self.number
    }
}

impl Drop for EventQueue {
    fn drop(&mut self) {
        if self.memory.is_some() {
            panic!("please destroy instead of dropping")
        }
    }
}

/// Result of consuming a single EQE via [`EventQueue::poll_one`].
enum ConsumedEvent {
    /// A `Completion` event, naming the CQ number that has new CQEs ready.
    Completion(u32),
    /// Any other event type - drained so the EQ doesn't stall, but not
    /// otherwise acted on (async/error events aren't wired to anything in
    /// this driver yet).
    Other,
}

#[bitfield]
struct EventQueueContext {
    #[skip(getters)]
    status: B4,
    #[skip]
    __: B16,
    #[skip(getters)]
    state: B4,
    #[skip]
    __: B60,
    #[skip]
    page_offset: B7,
    #[skip]
    __: u8,
    #[skip(getters)]
    log_eq_size: B5,
    #[skip]
    __: B24,
    #[skip]
    eq_period: u16,
    #[skip]
    eq_max_count: u16,
    #[skip]
    __: B22,
    #[skip(getters)]
    intr: B10,
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
    __: B72,
    #[skip]
    consumer_index: B24,
    #[skip]
    __: u8,
    #[skip]
    producer_index: B24,
    #[skip]
    __: B96,
}

/// A single event queue entry. EQE size is 32 bytes on ConnectX-3 (64 B
/// EQEs are also supported by the hardware but not used here) - matches
/// upstream Linux mlx4_core's `struct mlx4_eqe`
/// (`include/linux/mlx4/device.h`), verified against the current upstream
/// source (`torvalds/linux`, `include/linux/mlx4/device.h` and
/// `drivers/net/ethernet/mellanox/mlx4/eq.c`'s `mlx4_eq_int`) rather than
/// derived from local memory of the layout, given how easy it is to get a
/// bitfield offset like this subtly wrong.
///
/// `struct mlx4_eqe`'s 4-byte header (`reserved1, type, reserved2,
/// subtype`) is `event_type`/`event_subtype` below (plus two skipped
/// reserved bytes). What follows is a 24-byte `union event` whose exact
/// interpretation depends on `event_type`; this driver only models the
/// `Completion` variant (`union.comp: struct { __be32 cqn; }`) so far -
/// every other event type's payload is left as opaque, skipped padding
/// (`event_data1`'s low 64 bits, and the whole of `event_data2`).
#[bitfield(bytes = 32)]
struct EventQueueEntry {
    #[skip]
    __: u8,
    event_type: u8,
    #[skip]
    __: u8,
    event_subtype: u8,
    // `event_data1` is the first 12 of the union's 24 bytes. For a
    // `Completion` event this is `struct { __be32 cqn; }` followed by 8
    // reserved bytes - i.e. only the first 32-bit word (this struct's next
    // two fields) is meaningful; the trailing `B64` is unused padding for
    // every event type this driver understands. Mirrors Linux
    // mlx4_eq_int's `cqn = be32_to_cpu(eqe->event.comp.cqn) & 0xffffff;`:
    // one reserved/unused leading byte, then the CQ number in the low 24
    // bits of that first big-endian word - the exact same "skip 1 byte,
    // B24" shape as `CompletionQueueEntry::qp_number`
    // (`completion_queue.rs`/`os/library/rdma/src/mlx4_hw.rs`).
    #[skip]
    __: u8,
    completion_cqn: B24,
    #[skip]
    __: B64,
    #[skip]
    event_data2: B96,
    #[skip]
    __: B24,
    owner: bool,
    #[skip]
    __: B7,
}

impl core::fmt::Debug for EventQueueEntry {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("EventQueueEntry")
            .field("owner", &self.owner())
            .field("type", &EventType::from_repr(self.event_type().into()))
            .finish_non_exhaustive()
    }
}

#[repr(u64)]
#[derive(Debug, FromRepr)]
enum EventType {
    // completion
    Completion = 0x00,

    // IB affiliated events
    PathMigrationSucceeded = 0x01,
    CommunicationEstablished = 0x02,
    SendQueueDrained = 0x03,
    SrqLastWqe = 0x13,
    SrqLimit = 0x14,

    // QP affiliated errors
    CqError = 0x04,
    WqCatastrophicError = 0x05,
    EecCatastrophicError = 0x06,
    PathMigrationFailed = 0x07,
    WqInvalidRequestError = 0x10,
    WqAccessViolation = 0x11,
    SrqCatastropicError = 0x12,

    // unaffiliated events and errors
    InternalError = 0x08,
    PortChange = 0x09,
    // EqOverflow = 0x0f,
    // EccDetect = 0x0e,
    // VepUpdate = 0x19,
    // OpRequired = 0x1a,
    FatalWarning = 0x1b,
    FlrEvent = 0x1c,
    PortManagementChange = 0x1d,
    RecoverableEvent = 0x3e,
    // None = 0xff,

    // HCA interface
    CommandInterfaceCompletion = 0x0a,
    CommunicationChannelWritten = 0x18,
}

bitflags! {
    #[derive(Debug)]
    pub struct AsyncEventMask: u64 {
        // IB affiliated
        const PATH_MIGRATION_SUCCEEDED = 1 << EventType::PathMigrationSucceeded as u64;
        const COMMUNICATION_ESTABLISHED = 1 << EventType::CommunicationEstablished as u64;
        const SEND_QUEUE_DRAINED = 1 << EventType::SendQueueDrained as u64;
        const SRQ_LAST_WQE = 1 << EventType::SrqLastWqe as u64;
        const SRQ_LIMIT = 1 << EventType::SrqLimit as u64;

        // QP affiliated errors
        const CQ_ERROR = 1 << EventType::CqError as u64;
        const WQ_CATASTROPHIC_ERROR = 1 << EventType::WqCatastrophicError as u64;
        const EEC_CATASTROPHIC_ERROR = 1 << EventType::EecCatastrophicError as u64;
        const PATH_MIGRATION_FAILED = 1 << EventType::PathMigrationFailed as u64;
        const WQ_INVALID_REQUEST_ERROR = 1 << EventType::WqInvalidRequestError as u64;
        const WQ_ACCESS_VIOLATION = 1 << EventType::WqAccessViolation as u64;
        const SRQ_CATASTROPHIC_ERROR = 1 << EventType::SrqCatastropicError as u64;

        // unaffiliated events and errors
        const INTERNAL_ERROR = 1 << EventType::InternalError as u64;
        const PORT_CHANGE = 1 << EventType::PortChange as u64;
        const FATAL_WARNING = 1 << EventType::FatalWarning as u64;
        const FLR_EVENT = 1 << EventType::FlrEvent as u64;
        const PORT_MANAGEMENT_CHANGE = 1 << EventType::PortManagementChange as u64;
        const RECOVERABLE_EVENT = 1 << EventType::RecoverableEvent as u64;

        // HCA interface
        const COMMAND_INTERFACE_COMPLETION = 1 << EventType::CommandInterfaceCompletion as u64;
        const COMMUNICATION_CHANNEL_WRITTEN = 1 << EventType::CommunicationChannelWritten as u64;
    }
}

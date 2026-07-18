//! Legacy INTx interrupt handler for a ConnectX-3 device, wired up in
//! `ConnectX3Nic::init()`. Mirrors `device/rtl8139.rs`'s
//! `Rtl8139InterruptHandler`/`Rtl8139::plugin()` and `device/virtio/
//! interrupt.rs`'s `VirtioInterruptHandler` - the two existing precedents
//! for this codebase's (MSI-X-less) interrupt registration pattern.

use log::trace;

use crate::interrupt::interrupt_handler::InterruptHandler;

use super::{get_dev_list, minor_to_idx};

/// Identifies which `ConnectX3Nic` (by `minor`) this handler belongs to.
///
/// Unlike `Rtl8139`/virtio's device structs, `ConnectX3Nic`s are not
/// individually `Arc`-owned - they live in a flat `Vec` behind `DEV_LIST`,
/// addressed by `minor` (see `mlx4.rs`'s module doc on the minor scheme).
/// So this handler captures the minor, not a reference to the device
/// itself, and re-resolves it through `DEV_LIST` on every interrupt.
pub(super) struct Mlx4InterruptHandler {
    minor: usize,
}

impl Mlx4InterruptHandler {
    pub(super) fn new(minor: usize) -> Self {
        Self { minor }
    }
}

impl InterruptHandler for Mlx4InterruptHandler {
    fn trigger(&self) {
        // `get_dev_list()` is an `IrqSaveSpinlock` specifically so this is
        // safe: locking it here cannot self-deadlock against a syscall
        // handler on this same core, because acquiring it disables this
        // core's interrupts for the duration of whichever critical section
        // holds it first (see `DEV_LIST`'s doc comment in `mlx4.rs`).
        let mut list = get_dev_list().lock();
        let idx = minor_to_idx(self.minor);
        let Some(dev) = list.get_mut(idx) else {
            // Can legitimately happen for a brief window during
            // `ConnectX3Nic::init()`: the interrupt handler is registered
            // with the dispatcher/APIC before the device is pushed into
            // `DEV_LIST` (the device doesn't exist to look up yet at that
            // point). Not an error - just nothing to do.
            trace!("mlx4: interrupt fired for minor {} before/after it was in DEV_LIST", self.minor);
            return;
        };
        dev.handle_interrupt();
    }
}

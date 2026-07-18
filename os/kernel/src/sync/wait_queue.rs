/* ╔═════════════════════════════════════════════════════════════════════════╗
   ║ Module: wait_queue                                                      ║
   ╟─────────────────────────────────────────────────────────────────────────╢
   ║ Wait queues for blocking i/o.                                           ║
   ║                                                                         ║
   ║ Public functions:                                                       ║
   ║   - wait:       Blocks calling thread if the given predicate is true.   ║
   ║   - notify_one: Deblocks one waiting thread (if any).                   ║
   ║   - notify_all: Deblocks all waiting threads (if any).                  ║
   ╟─────────────────────────────────────────────────────────────────────────╢
   ║ Author: Michael Schoettner, Univ. Duesseldorf, 16.02.2026               ║
   ╚═════════════════════════════════════════════════════════════════════════╝
*/

use alloc::collections::VecDeque;
use log::trace;
use uuid::Uuid;

use crate::{
    process::core_local_storage::scheduler,
    process::scheduler::is_thread_alive,
    sync::irqsave_spinlock::IrqSaveSpinlock,
};

pub struct WaitQueue {
    queue: IrqSaveSpinlock<VecDeque<(Uuid, usize)>>,
}

impl core::fmt::Debug for WaitQueue {
    // `IrqSaveSpinlock` doesn't implement `Debug` (its `UnsafeCell` interior
    // can't be inspected without locking, which a `Debug` impl shouldn't do
    // - especially not from this codebase's typical panic/interrupt-time
    // debug-printing use of `Debug`, where locking could itself deadlock).
    // Manual no-field impl so callers embedding a `WaitQueue` (e.g.
    // `CompletionQueue`'s `#[derive(Debug)]`) can still derive `Debug`.
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("WaitQueue").finish_non_exhaustive()
    }
}

impl WaitQueue {
    pub fn new() -> WaitQueue {
        WaitQueue {
            queue: IrqSaveSpinlock::new(VecDeque::<(Uuid, usize)>::new()),
        }
    }

    /// Block until `pred()` becomes true.
    ///
    /// This used to be a cooperative busy-spin dressed up as a blocking
    /// call: it called `scheduler().park_current()` (which only flips the
    /// thread's state to `Parking`) and then unconditionally
    /// `scheduler().yield_now()`, which immediately sets the thread back to
    /// `Ready` and re-pushes it onto `ready_queue` regardless of the
    /// `Parking` state - so the thread never actually left the run queue.
    /// Harmless-but-wasteful for `os/kernel/src/naming/tmpfs.rs`'s pipe
    /// blocking (its only prior consumer), but it defeats the entire point
    /// of extension 1 (a genuinely off-CPU blocking `poll_cq`) if reused
    /// unmodified.
    ///
    /// Fixed to use `Scheduler::block()` (actually switches away, pushing
    /// onto `blocked_list`) paired with `Scheduler::deblock()` (moves the
    /// thread back onto `ready_queue`, falling back to a cross-core IPI if
    /// it isn't blocked on this core). The remaining race - a notifier
    /// firing after we register ourselves in `queue` but before `block()`
    /// has actually parked us in `blocked_list` (in practice, exactly the
    /// window `Mlx4InterruptHandler::trigger()` can land in, since it can
    /// run on the same core between those two steps) - is closed with
    /// `Thread::reset_wake_pending()`/`set_wake_pending()`/
    /// `should_block_or_consume_wake()`: `deblock()` latches the wakeup on
    /// the thread itself instead of losing it when the thread isn't in
    /// `blocked_list` yet, and this function consumes that latch instead of
    /// blocking when it's already set.
    pub fn wait<F>(&self, mut pred: F, message: &str)
    where
        F: FnMut() -> bool,
    {
        let current = scheduler().current_thread();
        let pid = current.process().id();
        let tid = current.id();

        loop {
            if pred() {
                return;
            }

            // Clear any stale latch from a previous loop iteration *before*
            // we become visible to notifiers (pushed into `queue` below), so
            // a wakeup racing us between here and `block()` is recorded
            // rather than lost.
            current.reset_wake_pending();

            {
                let mut guard = self.queue.lock();

                // Re-check under the queue lock: a notifier may have already
                // flipped the underlying condition right before we got here.
                if pred() {
                    return;
                }

                guard.push_back((pid, tid));
            }

            trace!("WaitQueue::wait: PID={}, TID={} about to block, message = {}", pid, tid, message);

            // If a notifier already ran (and thus set our wake_pending latch
            // via `deblock`, because we were not yet in `blocked_list` for it
            // to find), skip blocking entirely - otherwise we would sleep
            // forever, since that notifier already popped our entry out of
            // `queue` and will not call us again for the same event.
            if current.should_block_or_consume_wake() {
                scheduler().block();
            }

            // Either way, loop back and re-check the predicate: this also
            // absorbs spurious wakeups (e.g. `notify_all` waking multiple
            // waiters for one event, or a duplicate `queue` entry from an
            // earlier loop iteration).
        }
    }

    /// Wake up exactly one waiter (if any). Returns true if someone was woken up.
    pub fn notify_one(&self) -> bool {
        let mut guard = self.queue.lock();

        while let Some((pid, tid)) = guard.pop_front() {
            if is_thread_alive(tid) {
                scheduler().deblock(pid, tid);
                return true;
            }
            // else: stale waiter (killed/exited) -> keep going
        }

        false
    }

    /// Wake up all waiters currently queued.
    ///
    /// Returns the number of `deblock` calls issued. Unlike `notify_one`,
    /// this does not filter out stale (exited) waiters via
    /// `is_thread_alive` first: it must not stop early regardless, so
    /// there is nothing to gain by skipping a doomed `deblock` call other
    /// than saving a little work, and every current caller (`tmpfs.rs`'s
    /// pipes, and `CompletionQueue::notify_waiters` from
    /// `Mlx4InterruptHandler::trigger()`) ignores the return value.
    pub fn notify_all(&self) -> usize {
        let mut guard = self.queue.lock();
        let mut woke = 0;

        while let Some((pid, tid)) = guard.pop_front() {
            scheduler().deblock(pid, tid);
            woke += 1;
        }

        woke
    }
}

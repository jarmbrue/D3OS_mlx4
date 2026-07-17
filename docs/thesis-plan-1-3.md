# Implementation plan: mlx4/ibverbs thesis extensions 1-3

Phased implementation plans for the first three extensions listed in `THESIS.md`'s "Thesis
notes" section (interrupts instead of polling, per-process QP protection, user/kernel
separation hardening). Produced by a software-architect planning pass over the actual
`os/kernel/src/infiniband/`, `os/kernel/src/device/mlx4/`, and `os/library/{rdma,ibverbs}`
source — every file/function/struct referenced below exists in the repo as described unless
otherwise noted.

## 1. Interrupts instead of polling

**Current behavior + gap**

Completion delivery is 100% synchronous polling today. Userspace calls `poll_cq` → syscall
`Uverb`/`UVERBS_CMD_POLL_CQ` → `uverbs_ctl` (`os/kernel/src/infiniband/uverbs.rs:181-206`) →
`uverbs_poll_cq` (`os/kernel/src/infiniband/uverbs_cmd.rs:63-67`) →
`ConnectX3Nic::poll_cq` (`os/kernel/src/device/mlx4.rs:240-243`) →
`CompletionQueue::poll`/`poll_one` (`os/kernel/src/device/mlx4/completion_queue.rs:147-172`),
which returns immediately with however many CQEs are currently visible. There is no
kernel-side blocking anywhere in this path; "waiting for a completion" is implemented purely
as a userspace spin loop re-issuing the syscall.

An `EventQueue` already exists (`os/kernel/src/device/mlx4/event_queue.rs`) and is created at
bring-up via `init_eqs()` (called from `ConnectX3Nic::init()`, `mlx4.rs:160`), but it is
permanently in polling mode:

- `init_eqs()` always calls `EventQueue::new(cmd, caps, offsets, memory_regions, None)` —
  `base_vector` is hardcoded `None` (`event_queue.rs:43`, `// TODO: use interrupts here`), so
  `EQ_STATE_FIRED` is programmed into the HCA instead of `EQ_STATE_ARMED`.
- `EventQueue::new`'s own vector-registration path is dead:
  `let intr_vector = base_vector.and_then(|_| todo!());` (`event_queue.rs:92`).
- `CompletionQueue::poll` documents the gap in its own comment: *"the event queue should be
  polled async and not while polling here !!! consider moving to seperate thread or impl.
  interrupts !"* (`completion_queue.rs:150-159`), and the code that would drain the associated
  EQ is commented out.
- `ConnectX3Nic::init()` never reads the PCI interrupt line and never registers with the
  kernel's interrupt subsystem — unlike `os/kernel/src/device/rtl8139.rs`, mlx4 has zero
  interrupt wiring today.
- `CompletionQueue::arm()` (rings `DOORBELL_REQUEST_NOTIFICATION`, `completion_queue.rs:117-134`)
  and `EventQueue::ring(doorbells, arm: bool)` (sets the "generate interrupt" bit,
  `event_queue.rs:163-171`) already exist but are only called from device bring-up/CQ
  creation, never in a genuine "arm → interrupt fires → rearm" loop.
- `EventQueueEntry`'s `event_data1`/`event_data2` fields are `#[skip]` (no getters generated)
  — the CQ number carried by a `Completion`-type EQE currently cannot be read out at all
  (`event_queue.rs:298-316`). A getter (analogous to `CompletionQueueEntry::qp_number`'s `B24`
  extraction) is a prerequisite; the exact bit layout isn't currently encoded anywhere in this
  port and needs to come from the ConnectX-3 PRM / Linux `mlx4_core`'s `mlx4_eq_int`.

**Proposed design**

Two independent pieces: (a) real interrupt delivery from the device into the kernel, and (b) a
genuine off-CPU blocking wait for a thread polling a CQ.

*(a) Interrupt registration* — mirror `Rtl8139::plugin()`, the closest existing precedent:

```rust
// rtl8139.rs:392, 448-451 — the pattern to copy
let interrupt = InterruptVector::try_from(pci_device.interrupt(pci_config_space).1 + 32).unwrap();
interrupt_dispatcher().assign(interrupt, Box::new(Rtl8139InterruptHandler::new(device)));
apic().allow(interrupt);
```

In `ConnectX3Nic::init()` (`mlx4.rs`), read the PCI interrupt line, convert to an
`InterruptVector`, implement `InterruptHandler for Mlx4InterruptHandler { fn trigger(&self) }`
(`os/kernel/src/interrupt/interrupt_handler.rs`) capturing the device's `minor`, and call
`interrupt_dispatcher().assign(...)` / `apic().allow(...)`. D3OS has no MSI-X support anywhere
— this is legacy shared-INTx only, matching `rtl8139.rs` and `virtio/mod.rs:148-154` (and the
`// TODO: Should use MSI-X instead of legacy INTs` already in `event_queue.rs:91`). `NUM_EQS`
is hardcoded to `1` (`event_queue.rs:39`), so there's exactly one EQ/interrupt line per device.

Then flip `init_eqs()` to pass `Some(vector)` so `EQ_STATE_ARMED`/`ctx.set_intr(...)` get
programmed, resolve the `todo!()` at `event_queue.rs:92`, and after each
`EventQueue::handle_events()` drain, re-arm via `ring(doorbells, true)` and re-arm the relevant
`CompletionQueue::arm()` after each `poll()` that returns work.

*(b) Blocking wait in `poll_cq`* — `os/kernel/src/sync/wait_queue.rs`'s
`WaitQueue { wait(pred, msg), notify_one(), notify_all() }` is already used by
`os/kernel/src/naming/tmpfs.rs` for pipe blocking. **This must be fixed before reuse**:
`wait()` calls `scheduler().park_current()` (only flips state to `Parking`,
`scheduler.rs:562-567`) then unconditionally calls `scheduler().yield_now()`
(`wait_queue.rs:56-59`), but `yield_now()` (`scheduler.rs:932-964`) immediately sets the
thread back to `Ready` and re-pushes it onto `ready_queue` regardless of `Parking` state — so
the thread never actually leaves the run queue. `WaitQueue::wait` today is a cooperative
busy-spin, not a true off-CPU block; `notify_one`/`notify_all`'s call to
`scheduler().unblock(pid, tid)` is mostly a no-op since these waiters are never in
`blocked_list`. Fine for tmpfs pipes (correct, just wastes CPU), but defeats the point of this
extension if reused unmodified.

Fix `WaitQueue::wait`/`notify_one`/`notify_all` to use `Scheduler::block()`
(`scheduler.rs:306-326`, pushes onto `blocked_list` and actually switches away) and
`Scheduler::deblock(pid, tid)` (`scheduler.rs:329-339`, removes from `blocked_list`, requeues,
falls back to a cross-core IPI (`MessageCmd::Deblock`) if the thread isn't blocked on this
core). Close the lost-wakeup race ("interrupt fires and wakes before the thread finishes
calling block") using `Thread::reset_wake_pending()`/`set_wake_pending()`/
`should_block_or_consume_wake()` (`os/kernel/src/process/thread.rs:533-555`) — this latch
exists but has zero callers today.

With `WaitQueue` fixed, give `CompletionQueue` a `wq: WaitQueue` field, and have
`Mlx4InterruptHandler::trigger()` — after `EventQueue::handle_events()` drains EQEs — look up
the completed CQ (once the `event_data1`/`event_data2` getter exists) via `get_dev_list()` and
call `cq.wq.notify_all()`/`notify_one()`. The blocking `poll_cq` path calls
`cq.wq.wait(|| /* CQE ready */, "poll_cq: waiting for completion")` instead of returning
empty-handed.

**Critical invariant — locking from interrupt context**: `DEV_LIST`
(`static DEV_LIST: Once<Mutex<Vec<ConnectX3Nic>>>`, `mlx4.rs:66`) is a plain `spin::Mutex`, not
`IrqSaveSpinlock`. Every uverbs syscall handler already holds `get_dev_list().lock()` while
running, and the syscall trampoline runs with interrupts enabled
(`syscall_dispatcher.rs:197`, `"sti"`). If the mlx4 interrupt fires on the same core while a
syscall handler holds that lock, and the ISR also tries to lock it, that's an immediate
same-core spinlock self-deadlock. None of D3OS's current ISRs touch shared mutable state
reachable from a syscall path this way — this would be the first. Fix by converting `DEV_LIST`
(and anything the ISR touches inside `CompletionQueue`/`EventQueue`) to `IrqSaveSpinlock`, or
by adopting the `try_lock()` + `force_unlock()` idiom `InterruptDispatcher::dispatch()` and
`Apic::end_of_interrupt()` already use for this exact scenario
(`interrupt_dispatcher.rs:305-314`, `apic.rs:404-411`). **Must land before/alongside interrupt
registration, not as a follow-up.**

**Files to change**
- `os/kernel/src/device/mlx4.rs` (`ConnectX3Nic::init` — interrupt read + assign/allow;
  `DEV_LIST` lock type)
- `os/kernel/src/device/mlx4/event_queue.rs` (`init_eqs`, `EventQueue::new`'s `todo!()`,
  `EventQueueEntry` CQ-number getter)
- `os/kernel/src/device/mlx4/completion_queue.rs` (new `wq` field, re-arm on drain)
- `os/kernel/src/sync/wait_queue.rs` (fix `wait`/`notify_one`/`notify_all`)
- New: `Mlx4InterruptHandler` (e.g. `os/kernel/src/device/mlx4/interrupt.rs`, mirroring
  `device/virtio/interrupt.rs`)
- `os/kernel/src/infiniband/uverbs_cmd.rs`/`uverbs.rs` if a blocking entry point is added

**Invariants to preserve**
- Syscall enum-order contract untouched if blocking is a *mode* of the existing
  `UVERBS_CMD_POLL_CQ`/`Uverb` path rather than a new syscall variant.
- No dynamic driver framework beyond the existing `DEV_LIST` + minor scheme.
- `NUM_EQS = 1` / single shared legacy INTx line per device; no MSI-X.

**Risks / open questions**
- The `spin::Mutex` → IRQ-safe change is the single biggest correctness risk — test in
  isolation (deliberately trigger an interrupt while a syscall holds the lock) before building
  on top.
- `WaitQueue`'s fix changes behavior for its only existing consumer (tmpfs pipes) —
  regression-test pipe/FIFO blocking after the fix.
- Whether QEMU's VFIO-passthrough ConnectX-3 (per `ib2`'s setup) delivers legacy INTx
  correctly needs to be verified before trusting real hardware runs.
- Whether to expose blocking as a new `UverbsInnerCmd` variant vs. an implicit blocking-mode
  flag on `PollCq` — real ibverbs uses a separate completion channel + `ibv_get_cq_event`,
  which doesn't exist in `os/library/ibverbs` yet. Scope this decision explicitly.

**Suggested verification**
- `os/kernel/src/tests/test_runner.rs`'s harness is currently entirely unpopulated (no
  `impl TestPlugin` anywhere) — a plugin creating a loopback QP pair, issuing a blocking wait,
  and asserting the thread actually left `ready_queue` would validate both the interrupt
  wiring and the `WaitQueue` fix.
- Manual QEMU boot to confirm clean init with interrupts enabled.
- Manual `ib1`/`ib2` hardware testing per `CLAUDE.md`'s "Testing on real InfiniBand hardware"
  section, comparing CPU utilization and completion latency (old polling vs. new blocking)
  using the existing `os/application/rdma/mlx4/bench.rs` harness.

## 2. Protection for queue pairs between different processes

**Current behavior + gap**

`ConnectX3Nic` tracks QPs in a flat, unauthenticated vector: `qps: Vec<QueuePair>`
(`mlx4.rs:92`). Every uverbs entry point that touches a QP — `modify_qp`, `post_receive`,
`post_send`, `destroy_qp` (`mlx4.rs:292-325`) — looks it up purely by `qp_number: u32` via
`.find(|qp| qp.number() == number)`, with no notion of which process created it. `QueuePair`
(`os/kernel/src/device/mlx4/queue_pair.rs:50-68`) has no owner field. Meanwhile `uverbs_ctl`
(`uverbs.rs:35`) already resolves the caller's identity on every call —
`let process = process_manager().read().current_process();` — but that `Arc<Process>` is only
used for `copy_to_user` calls today, never threaded into `uverbs_cmd.rs`/`mlx4.rs`.
`Process::id()` returns a `Uuid` (`process/process.rs:26-73`), the right token to tag
ownership with.

Concretely: any process that knows (or guesses) a `qp_num` can currently call `MODIFY_QP`,
`POST_SEND`, `POST_RECV`, or `DESTROY_QP` against a QP another process created on the same
device. `CompletionQueue`s (`cqs: Vec<CompletionQueue>`, `mlx4.rs:91`) and memory regions
(`icm.rs`'s `MrTable`/`DmptEntry`) have the structurally identical gap.

**Proposed design**

Tag `QueuePair` with its creator and enforce it at the point `ConnectX3Nic`'s methods resolve a
`qp_num` — the one choke point every QP-touching verb already funnels through.

1. Add `owner: Uuid` to `QueuePair` (`queue_pair.rs:50-68`); set it in `QueuePair::new`
   (`queue_pair.rs:78`, needs a new `owner: Uuid` param). `uuid` is already a kernel
   dependency, used identically by `Process`/`Thread`/`WaitQueue`.
2. Thread `process.id()` from `uverbs_ctl` down through `uverbs_cmd::uverbs_create_qp`
   (`uverbs_cmd.rs:42-51`) into `ConnectX3Nic::create_qp` (`mlx4.rs:261-287`), and likewise
   into `uverbs_modify_qp`, `uverbs_post_send`, `uverbs_post_recv`, and
   `uverbs_destroy(minor, ConnectX3Nic::destroy_qp, qp_num)` (`uverbs_cmd.rs:81-85` — currently
   `fn(&mut ConnectX3Nic, u32) -> ...`, needs an added `Uuid` param or a per-call-site wrapper
   closure).
3. In `modify_qp`/`post_receive`/`post_send`/`destroy_qp` (`mlx4.rs:292-325,298-308`), after
   the existing lookup, add `if qp.owner != caller { return Err("permission denied"); }`.
4. At the `uverbs.rs` call sites, stop blanket-mapping every driver error to `Errno::EINVAL`
   for these ownership failures — map to `Errno::EACCES`
   (`os/library/syscall/src/return_vals.rs:21`, defined, currently unused by infiniband) so
   userspace can distinguish "not your QP" from "malformed request." Requires the driver-side
   `Result<_, &'static str>` errors to be distinguishable at the `uverbs.rs` boundary — pick
   whichever is the smaller diff (sentinel string match vs. a small typed error).

**Files to change**
- `os/kernel/src/device/mlx4/queue_pair.rs` (`owner` field, `QueuePair::new` signature)
- `os/kernel/src/device/mlx4.rs` (`create_qp`/`modify_qp`/`post_receive`/`post_send`/
  `destroy_qp` signatures + ownership checks)
- `os/kernel/src/infiniband/uverbs_cmd.rs` (thread `Uuid` through QP-touching wrappers)
- `os/kernel/src/infiniband/uverbs.rs` (pass `process.id()` at each relevant arm; map
  ownership errors to `Errno::EACCES`)

**Invariants to preserve**
- QP numbers are allocated monotonically via `Offsets::alloc_qpn()` (`mlx4.rs:418-422`, only
  ever increments) — no ABA/reuse race between destroy and a stale ownership check.
- Syscall enum-order contract untouched — this is a logic change inside `sys_uverbs_ctl`'s
  existing handler.
- `UVERBS_SUPPORTED_MINOR_TABLE`/`device_in_range`/magic checks stay as-is; ownership is an
  additional, later-stage check.
- The global `DEV_LIST`/minor-allocation scheme is unaffected — ownership is per-QP; a process
  can still own QPs on multiple minors.

**Risks / open questions**
- `CompletionQueue`s and memory regions have the identical gap and are out of explicit scope
  here — but see "Cross-extension interactions": extension 1's real blocking on `poll_cq`
  turns an unprotected CQ into an active timing side-channel. **Recommend extending the same
  `owner: Uuid` pattern to `CompletionQueue` (`completion_queue.rs:36-48`) in the same change**,
  since extension 1 will need it regardless.
- Deciding the exact error-signaling mechanism from `&'static str`-returning driver methods to
  `Errno::EACCES` is a small design call — keep it minimal.
- `send_cq_number`/`receive_cq_number` on `QueuePair` (`queue_pair.rs:61-62`) reference CQs by
  number with no ownership check either — should `create_qp` verify the caller owns (or may
  bind to) the CQs it's attaching to? Worth deciding explicitly.

**Suggested verification**
- Extend `os/kernel/src/tests/test_runner.rs` with a `TestPlugin` creating two synthetic
  `Uuid`s, creating a QP as "process A," and asserting `modify_qp`/`post_send`/`post_recv`/
  `destroy_qp` called with "process B"'s id return the ownership error.
- Manual QEMU test: run two userspace apps concurrently, each creating a QP, and have one
  attempt to `ibv_modify_qp`/`ibv_destroy_qp` a QP number it didn't create — confirm rejection.
- Manual `ib1`/`ib2` hardware run per `CLAUDE.md`, confirming legitimate single-process RDMA
  flows (`os/application/rdma/mlx4`) are unaffected.

## 3. Improve the separation between user and kernel space

**Current behavior + gap**

`uverbs_ctl` (`os/kernel/src/infiniband/uverbs.rs:32-283`) is the single entry point for every
verb. Only `UVERBS_CMD_QUERY_DEVICES`/`UVERBS_CMD_QUERY_DEVICE` (`uverbs.rs:48-60`) go through
the kernel's safe user-copy primitives, `process.virtual_address_space.copy_to_user`/
`copy_from_user` (`os/kernel/src/memory/vmm.rs:417-453`), which validate the destination range
via `access_ok` (bounds check, `vmm.rs:395-405`) and per-page mapping via
`ensure_user_page_is_mapped` (`vmm.rs:407-415`) before touching the pointer.

Every other arm — `QUERY_PORT`, `REGISTER_MR`, `CREATE_CQ`, `CREATE_QP`, `MODIFY_QP`,
`POLL_CQ`, `POST_SEND`, `POST_RECV` (`uverbs.rs:61-259`) — casts the raw syscall `arg: usize`
straight to a typed pointer and calls `core::ptr::copy_nonoverlapping` directly, with no
`access_ok`/page-mapped check beyond the earlier `UVERBS_MAGIC`/`device_in_range` gate. Note:
the `size` used in these copies is not independently attacker-controllable — `cmd` must
exactly equal a precomputed `UVERBS_CMD_*` constant for the arm to be selected at all
(`rdma::uverbs_uapi`, `uverbs_uapi.rs:70-83`) — so this is a missing-pointer-validation bug,
not a length-confusion bug.

Severity increases further in:

- **Nested user pointers inside already-copied containers are dereferenced with zero
  validation**: `ibv_qp_container.ib_caps: *mut ibv_qp_cap` (`uverbs.rs:145-149`),
  `ibv_qp_modify_container.attr: *const ibv_qp_attr` (`uverbs.rs:169-173`),
  `ibv_qp_post_send_container.ibv_send_wr`/`ibv_qp_post_recv_container.ibv_recv_wr`
  (`uverbs.rs:221-225,247-251`).
- **Worst case**: `UVERBS_CMD_REGISTER_MR` reads `data_ptr: *mut u8`/`len: usize` straight out
  of the unvalidated `ibv_mr_container` and builds
  `unsafe { from_raw_parts_mut(container.data_ptr, len) }` (`uverbs.rs:104-106`) directly from
  that untrusted pointer, then hands it to `uverbs_register_mem_region` → `create_mr` → DMA
  registration (`icm.rs`'s `alloc_dmpt`). A process can point `data_ptr` at kernel memory and
  get the driver to register it as a remotely-read/writable RDMA memory region — an
  RDMA-reachable arbitrary-kernel-memory primitive once the fabric can address it.
- `ibv_send_wr`/`ibv_recv_wr`'s `sg_list`/`next` chains are walked
  (`queue_pair.rs:470-476,599-605`) with no bounds/pointer validation — the code's own
  comments flag this: *"for now we just check the ibv_send_wr struct, not the internal
  pointers it points to which needs to be done to prevent security issues !"* and
  *"TODO next, sg_list, have to be checked before proceding"* (`uverbs.rs:207-208,228,254`).

**Proposed design**

No new kernel facility is needed for the outer copies — `vmm.rs`'s `copy_to_user`/
`copy_from_user` already implement the right checks; generalize the pattern already used
correctly for `QUERY_DEVICES`/`QUERY_DEVICE`. The nested-pointer cases split into two distinct
fixes, refined after checking the actual field types in `os/library/rdma/src/ib_core.rs`:

1. **Outer container copies**: replace every raw
   `copy_nonoverlapping(user_buf.cast(), &mut container as *mut _ as *mut u8, size)` with
   `process.virtual_address_space.copy_from_user(kernel_slice, user_buf)`.

2. **`ibv_qp_container.ib_caps` / `ibv_qp_modify_container.attr` — embed by value, don't
   validate-and-dereference.** `ibv_qp_cap` (5×`u32`, `ib_core.rs:23-29`) and `ibv_qp_attr`
   (`ib_core.rs:119-141` — scalars/enums, plus POD `ibv_ah_attr`→`ibv_global_route`→`ibv_gid`
   with a fixed `[u8;16]`) are fully POD, no internal pointers. There is no reason these sit
   behind a pointer in `ibv_qp_container`/`ibv_qp_modify_container`
   (`rdma/src/uverbs_uapi.rs:177-212`) at all — change the field from `*mut ibv_qp_cap`/
   `*const ibv_qp_attr` to the struct by value. One `copy_from_user` of the container then
   pulls in everything; the second copy and the nested dereference disappear rather than
   needing to be hardened. This also deletes the second-copy/pointer-patch-back dance at
   `uverbs.rs:145-151,169-175`, and simplifies `os/library/ibverbs/src/ibverbs_sys.rs` on the
   userspace side (build the container with the struct inline instead of a pointer to it).

3. **`ibv_qp_post_send_container.ibv_send_wr` / `ibv_qp_post_recv_container.ibv_recv_wr` — a
   wire-format change, not a validate-in-place fix.** `ibv_send_wr`/`ibv_recv_wr` are **not**
   POD (`ib_core.rs:189-201,244-249`): `sg_list: Vec<ibv_sge>` is a real heap-allocated `Vec`
   (pointer + len + cap), and `next: *mut ibv_send_wr` chains WQEs. This makes the *current*
   code more severe than a bare unvalidated pointer: the existing second copy
   (`uverbs.rs:221-225`) raw-byte-copies a `Vec`'s internal `(ptr, len, cap)` triple out of user
   memory into a kernel-side value — unsound independent of security, since `copy_nonoverlapping`
   over a non-`Copy` type violates its invariants outright — and `queue_pair.rs`'s
   `post_send`/`post_receive` (`queue_pair.rs:455,553`) then iterate that `Vec`'s buffer
   pointer as if it were valid kernel memory, when it's still a userspace address. Embedding
   by value doesn't fix this — the `Vec` *is* the value.

   The fix is a wire-format type distinct from the in-memory `ibv_send_wr`/`ibv_recv_wr` used
   by `os/library/ibverbs/src/ibverbs.rs`'s high-level API, fully POD:
   ```rust
   #[repr(C)]
   pub struct ibv_send_wr_uapi {
       pub wr_id: u64,
       pub sg_list: [ibv_sge; UVERBS_MAX_SGE],  // fixed cap, e.g. 16 or 32
       pub num_sge: u32,
       pub opcode: ibv_wr_opcode,
       pub send_flags: ibv_send_flags,
       pub wr: ibv_send_wr_wr,
       // no `next` — see chaining decision below
   }
   ```
   - **SGE count**: bound to a fixed max instead of `Vec`. The driver already imposes a hard
     per-QP cap via `WorkQueue::new_send_queue`/`new_receive_queue`'s negotiated
     `max_send_sge`/`max_recv_sge` (`queue_pair.rs:729-808`), so a generous fixed array gives
     up no real capability, just matches a bound that already exists.
   - **WQE chaining (`next`)**: inherently variable-length, so "embed by value" can't preserve
     it as-is. Two options: **(a)** drop kernel-side chain-walking — `ibverbs_sys.rs` issues
     one `Uverb` syscall per WR instead of one per chain; the loop currently in
     `queue_pair.rs::post_send`/`post_receive` (walking `curr.next`) moves to userspace, where
     iterating a real `Vec`/slice is fine since it never crosses the boundary. **(b)** keep
     one-syscall-per-batch but bound the chain to a fixed-size array of WRs instead of a linked
     list. (a) is the smaller change and removes an entire class of pointer-walking risk at the
     cost of one extra syscall per WR in a chain (not per byte) — recommended, since this is a
     from-scratch fix anyway.

   Once this wire struct is POD, `UVERBS_CMD_POST_SEND`/`POST_RECV` collapse to the same
   single-`copy_from_user` shape as the fixed `CREATE_QP`/`MODIFY_QP` — no second copy, no
   nested-pointer validation needed anywhere in `uverbs_ctl`. This also folds in the
   `sg_list`/`next`-walking TODOs already flagged in the code as a separate fix, rather than
   deferring them.

4. **`REGISTER_MR`'s `data_ptr`/`len`**: this data is DMA'd, not read into a kernel buffer, so
   a full copy isn't the right shape — validate with an `access_ok`-style range check before
   constructing `from_raw_parts_mut`, sized against the existing
   `ibv_mr_container::S = UVERBS_MAX_USER_TRUST_SIZE` clamp (`uverbs_uapi.rs:108`). `vmm.rs`'s
   `access_ok` is currently private (`vmm.rs:395`); expose a `pub fn is_user_range_ok` (or
   equivalent) alongside `copy_to_user`/`copy_from_user` — minimal, additive.

**Files to change**
- `os/library/rdma/src/uverbs_uapi.rs` (embed `ibv_qp_cap`/`ibv_qp_attr` by value; add POD
  `ibv_send_wr_uapi`/`ibv_recv_wr_uapi` with bounded `sg_list`, no/bounded `next`)
- `os/kernel/src/infiniband/uverbs.rs` (route every arm through `copy_from_user`; delete the
  second-copy blocks entirely once containers are POD)
- `os/kernel/src/memory/vmm.rs` (expose `access_ok`/add a thin validation wrapper)
- `os/kernel/src/device/mlx4/queue_pair.rs` (`post_send`/`post_receive` consume the bounded
  array instead of walking `Vec`/`next`)
- `os/library/ibverbs/src/ibverbs_sys.rs` (build the new POD containers; if chaining moves to
  userspace, loop and issue N syscalls there)

**Invariants to preserve**
- `UVERBS_MAGIC`/`device_in_range`/`UVERBS_SUPPORTED_MINOR_TABLE` checks (`uverbs.rs:37-45`)
  stay exactly as-is — this hardening is additive, layered after them.
- Dispatch via exact-equality match against precomputed `UVERBS_CMD_*` constants (what makes
  `size` trustworthy) must not be loosened.
- Existing `TypeSize`/`::S` constants (`uverbs_uapi.rs:95-129`) remain the authoritative
  max-length bounds for whichever `access_ok`-style check is added.
- No syscall enum/ordering change — entirely inside `sys_uverbs_ctl`'s existing handler body.

**Risks / open questions**
- `ensure_user_page_is_mapped` (`vmm.rs:407-415`) currently just translates and, on a miss,
  **rejects** rather than mapping on demand (`// TODO: Map`). Routing previously-unchecked
  calls through `copy_to_user`/`copy_from_user` for the first time could start rejecting
  buffers that haven't been first-touched — check this against real client code
  (`os/library/ibverbs`, `os/application/rdma/mlx4`) before rollout, or fill in the `TODO: Map`
  as part of this work.
- The wire-format change to `POST_SEND`/`POST_RECV` changes per-call work (extra copies,
  possibly N syscalls instead of 1 for chains) — worth a before/after latency check against
  `os/application/rdma/mlx4/bench.rs`, since this is a hot path.
- Whether to expose `access_ok` publicly vs. wrapping it is a small API-surface call — either
  is fine, keep it minimal.

**Suggested verification**
- `TestPlugin`(s) in `os/kernel/src/tests/test_runner.rs` calling `uverbs_ctl` directly with
  deliberately invalid pointers (null, kernel address, unmapped user address, one-past-end) for
  each hardened command, asserting `Errno::EINVAL`/`Errno::EACCES` rather than a page
  fault/panic.
- Manual QEMU run of the existing, unmodified `ibping`/`ibstat`/`os/application/rdma/mlx4` to
  confirm no functional regression for well-behaved callers.
- Manual `ib1`/`ib2` hardware test per `CLAUDE.md`, exercising the full
  `bench.rs`/`rdma_read.rs`/`rdma_write.rs` paths post-hardening.

## Cross-extension interactions and suggested order

- **Extensions 2 and 1 are coupled through `CompletionQueue`, not just `QueuePair`.**
  Extension 1's wakeup is keyed by CQ number, and `CompletionQueue`s currently have the same
  unprotected-lookup-by-number gap as QPs. Pure polling today already lets any process poll
  any `cq_num`, so this isn't a new hole in the abstract — but once extension 1 makes
  `poll_cq` genuinely block, an unprotected CQ becomes an active side channel: process B can
  enroll as a waiter on process A's CQ and learn the *timing* of A's completions without
  needing read access to A's data. Fold the `owner: Uuid` mechanism from extension 2 onto
  `CompletionQueue` in the same change — the diff is small once the QP version exists, and it
  closes a hole extension 1 would otherwise open.
- **Extension 3 touches the exact same container-copy call sites extension 1 and 2's new
  fields flow through.** Extension 2's `CREATE_QP` handler reads `process.id()` at the same
  point extension 3 hardens the container copy; extension 1's eventual blocking-mode argument
  will be copied in via the same `uverbs_ctl` machinery extension 3 rewrites. Landing 3 first
  means 1 and 2 build on already-safe copy primitives instead of adding more raw pointer
  arithmetic that then needs re-touching.
- **Recommended implementation order: 3, then 2 (incl. CQ ownership), then 1** — despite the
  priority order in `THESIS.md` reflecting thesis-benchmark priority, not implementation risk:
  1. **Extension 3 first** — mechanical, self-contained, no cross-cutting locking changes, and
     de-risks the other two by giving them a safe copy-based pattern to extend.
  2. **Extension 2 next** (QP + CQ ownership) — still self-contained (no new locking/interrupt
     concerns), and its CQ ownership check becomes a hard prerequisite for extension 1's
     wait-registration to be safe.
  3. **Extension 1 last** — by far the highest-risk piece (the `spin::Mutex` → IRQ-safe change,
     the `WaitQueue`/`Scheduler::block`/`deblock` fix, and real interrupt delivery are new
     ground for this codebase), and it can safely gate "may this caller block-wait on this CQ"
     on the ownership check extension 2 already added.
- All three share one invariant to double-check at each step: the `SystemCall` enum-variant-order
  contract (`os/library/syscall/src/lib.rs`) is untouched as long as blocking/ownership/
  hardening stay inside the existing `Uverb`/`sys_uverbs_ctl` dispatch rather than adding new
  top-level syscalls — only decide otherwise deliberately, and if so, update
  `docs/new-syscall.howto.md`'s checklist on both the `SystemCall` enum and `SyscallTable`
  sides together.

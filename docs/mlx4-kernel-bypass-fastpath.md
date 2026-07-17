# Kernel-bypass fast path for mlx4 post_send / post_receive / poll_cq

## Summary

Previously, every `post_send`, `post_receive`, and `poll_cq` call in the mlx4/ibverbs stack went
through a full syscall round-trip (the single multiplexed `Uverb` syscall), even though this is
exactly the data-plane hot loop kernel-bypass RDMA is meant to avoid a context switch on.

This change adds a zero-syscall fast path: at `create_qp`/`create_cq` time (still a syscall — a
one-time control-plane setup), the QP/CQ's ring buffer, doorbell record(s), and UAR doorbell/
BlueFlame MMIO page(s) are mapped directly into the owning userspace process. From then on,
userspace builds WQEs, rings doorbells, and parses CQEs by reading/writing that mapped memory
directly — no kernel involvement, no context switch. `create`/`destroy`/`modify_qp`/`reg_mr`/query
verbs are unaffected and remain syscalls.

The original syscall-based `ibv_poll_cq`/`ibv_post_send`/`ibv_post_recv` path was **kept**, not
deleted, selectable via a Cargo feature (see below), as a known-good reference for real-hardware
bring-up and for direct before/after comparisons.

Full design rationale and phase breakdown: `/Users/juliusarmbruster/.claude/plans/hashed-inventing-sunset.md`
(local plan file, not checked into the repo).

## What changed, by phase

**Phase 0 — bug fix.** `ibv_post_recv` (`os/library/ibverbs/src/ibverbs_sys.rs`) issued its syscall
with `UVERBS_CMD_POST_SEND` instead of `UVERBS_CMD_POST_RECV`. Fixed.

**Phase 1 — QP/CQ ownership tagging.** Added a `creator: Uuid` field to `QueuePair` and
`CompletionQueue` (`os/kernel/src/device/mlx4/{queue_pair,completion_queue}.rs`), populated from
`process_manager().read().current_process().id()` at creation time. The new mmap syscall (Phase 3)
refuses to map a QP/CQ's memory into any process other than its creator. Also added the
`doorbell_address: PhysAddr` field to `CompletionQueue`, which the code already computed at
creation but never stored (`QueuePair` already had the equivalent field).

**Phase 2 — shared hardware-layout module.** Moved the WQE/CQE/doorbell hardware-layout structs
(`WqeControlSegment`, `WqeDataSegment`, `WqeRemoteAddressSegment`, `QueuePairOpcode`,
`QueuePairDoorbell`, `CompletionQueueDoorbell`, `CompletionQueueEntry`, `Syndrome`,
`ReceiveOpcode`, `DoorbellPage`) out of the kernel driver
(`os/kernel/src/device/mlx4/{queue_pair,completion_queue,fw}.rs`) into a new `rdma::mlx4_hw` module
(`os/library/rdma/src/mlx4_hw.rs`), shared by both the kernel driver and the new userspace fast
path. A single definition avoids the two sides' WQE/CQE byte layouts silently drifting apart.
`WqeDatagramSegment` (UD-only) stayed kernel-private — the fast path is RC/UC only.

**Phase 3 — mmap syscalls.** Implemented the previously-stubbed `uverbs_mmap_uar()`
(`os/kernel/src/infiniband/uverbs_cmd.rs`) for real, as two new `Uverb`-multiplexed commands,
`UVERBS_CMD_MMAP_QP`/`UVERBS_CMD_MMAP_CQ` (`os/library/rdma/src/uverbs_uapi.rs`). Each resolves the
target QP/CQ's physical memory regions (`ConnectX3Nic::mmap_qp_resources`/`mmap_cq_resources` in
`os/kernel/src/device/mlx4.rs`), checks the caller against the `creator` field from Phase 1, and
maps each region into a fresh user-space VMA via `alloc_vma` + `map_pfr_for_vma` — the same pattern
`sys_map_frame_buffer` (`os/kernel/src/syscall/sys_vmem.rs`) already uses for the framebuffer.
Ring buffers and doorbell records get cacheable DMA flags; the UAR doorbell/BlueFlame pages get
`NO_CACHE` MMIO flags.

**Phase 4 — userspace fast path.** Added `fastpath_post_send`/`fastpath_post_recv`/
`fastpath_poll_cq` to `os/library/ibverbs/src/ibverbs_sys.rs`, mirroring the kernel driver's
`QueuePair::post_send`/`post_receive` and `CompletionQueue::poll`/`poll_one`/`get_next_cqe_sw`
field-for-field, operating directly on the memory mapped in Phase 3. Per-QP ring state
(`QpRingState`) and per-CQ ring state (`CqRingState`) — local head/tail indices and per-WQE
wr_id/chain_size metadata — live in userspace now instead of the kernel, since poster and poller
are always the same process for a given QP/CQ once bypassed. SGE virtual→physical address
translation (previously a kernel page-table walk) is done via a per-process lkey→(phys,virt)
registry populated at `ibv_reg_mr` time.

**Phase 5 — wiring.** `ibv_context`/`ibv_qp`/`ibv_cq` gained the new ring-state fields; `ibv_open_device`,
`ibv_create_qp`, `ibv_create_cq`, `ibv_reg_mr`, and the `Drop` impls for `ibv_qp`/`ibv_mr` were
updated accordingly (mmap calls, registry inserts/removes). The `IBV_CONTEXT_OPS` vtable is now
built under `#[cfg(feature = "fastpath-verbs")]` (fast path, default-enabled) vs.
`#[cfg(not(feature = "fastpath-verbs"))]` (original syscall path, build with
`--no-default-features` to select). `ibverbs.rs` and every consumer in
`os/application/rdma/mlx4` needed zero source changes.

**Phase 6 — verification.** See below.

## Two bugs found and fixed along the way (not the main point of this change, but adjacent)

1. `ibv_post_recv` used `UVERBS_CMD_POST_SEND` instead of `UVERBS_CMD_POST_RECV` (Phase 0).
2. `QueuePair::post_receive` (`os/kernel/src/device/mlx4/queue_pair.rs`) wrote the posted `wr_id`
   into the **send**-queue metadata table (`self.sq.update_id(...)`) instead of the **receive**-
   queue one. Since `poll_one`'s `query_wr_id` correctly reads from `self.rq` for receive
   completions, this meant every receive completion's `wc.wr_id` was always 0 on the (still
   existing) syscall path. Fixed to `self.rq.update_id(...)`; the new userspace fast path was
   written directly with the correct behavior.

A third issue was **found but deliberately left alone**: `WorkQueue::new_receive_queue` never
calls `update_chain_size` for the receive queue, so `advance_receive_queue_by`'s `chain_size` is
always 0 on the existing syscall path — meaning the kernel's receive-queue tail never advances,
and `would_overflow` will eventually always report the RQ full. This is pre-existing, in a part of
`post_receive` not otherwise touched by this change, and fixing it changes behavior beyond scope.
The new userspace fast path was written with the obviously-correct value (chain_size = 1 per
receive WR) since it's new code, not a modification of existing kernel logic — so this bug does
**not** affect the fast path, only the (kept-as-reference) slow path.

## Known limitations / explicit scope decisions

- **RC/UC only.** UD (datagram) QPs are not supported by the fast path — `WqeDatagramSegment`
  stayed kernel-private. No current consumer in `os/application/rdma/mlx4` creates UD QPs.
- **Single-threaded per QP/CQ.** A given QP/CQ pair must only be posted/polled from one thread
  within its owning process. The fast-path ring state uses `Rc<RefCell<...>>`, not
  `Arc<spin::Mutex<...>>`; concurrent use from multiple threads is undefined behavior (a `RefCell`
  double-borrow panic at best).
- **Address translation assumes physical contiguity**, same as the existing (kernel) design:
  `ibv_reg_mr` already returns a single physical base address for the whole region (see the
  existing `// TODO: this fails for large memory regions (>= 64 MB)` comment on `create_mr` in
  `os/kernel/src/device/mlx4.rs`) — the fast path's `fastpath_translate_sge` derives an SGE's
  physical address as `mr.phys_base + (sge.addr - mr.virt_base)`, which is only correct under that
  same pre-existing contiguity assumption.

## Verification performed in this session

- `cargo make --no-workspace check-members` — clean (only pre-existing, unrelated warnings).
- `cargo make --no-workspace clippy-members` — pre-existing failures in unrelated files
  (`sys_terminal.rs`, `sys_concurrent.rs`, `sys_vmem.rs`, `sys_graphic.rs`,
  `sys_system_info.rs`, `sys_logger.rs`, `sys_shm.rs`); confirmed present before this change too
  and not touched by it. No clippy errors in any mlx4/infiniband/rdma/ibverbs file.
- `cargo check -p ibverbs --no-default-features` — the original syscall path still compiles clean.
- `cargo make --no-workspace image` — full workspace build (kernel + every application) succeeds.
- Booted the resulting image in QEMU: boots cleanly through PCI/VirtIO/IDE/network init, DHCP,
  `terminal_emulator`, and `shell`, with the expected `No ConnectX-3 card found !` (QEMU has no
  real ConnectX-3). No panics.

## What still needs to happen (requires real hardware, not available in this session)

QEMU has no real ConnectX-3, so none of the above exercises actual WQE/CQE hardware semantics —
the BlueFlame MMIO path, the CQE ownership-bit protocol against real completions, or ring-
wraparound under real traffic. Per `CLAUDE.md`'s documented ib1/ib2 workflow:

1. `cargo make --no-workspace image` (already produces a working `d3os.img`).
2. Serve the repo root over HTTP (e.g. `caddy file-server --browse`).
3. `scripts/infiniband-remote-run.sh` — boots the image on `ib2` via VFIO passthrough to the real
   ConnectX-3, against `ib1` as the known-good Linux subnet-manager peer.
4. Run `rdma_write`/`rdma_read`/`bench` (`os/application/rdma/mlx4`) end-to-end.
5. Run the same apps built with `cargo make --no-workspace image --no-default-features` (or
   equivalent per-crate override) to exercise the kept syscall-based path, and compare
   latency/throughput against the default fast-path build — that before/after comparison is the
   actual acceptance criterion for this change, since eliminating syscall overhead was the whole
   point.

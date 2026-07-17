# Thesis notes: mlx4 driver + ibverbs

This file tracks the parts of D3OS relevant to the master thesis: the mlx4 (ConnectX-3)
InfiniBand kernel driver and the userspace `ibverbs` library built on top of it. It is
separate from `CLAUDE.md`, which orients any contributor/agent to the whole repo — this file
is scoped to thesis context and is expected to grow with research questions, planned changes,
and benchmark notes over time.

## 1. mlx4 kernel driver

### Kernel InfiniBand core — `os/kernel/src/infiniband/`

| File | Role |
|---|---|
| `mod.rs` | Entry point for the InfiniBand subsystem. `init()` calls a feature-gated `_init()` that PCI-probes for Mellanox devices (vendor `0x15b3`) and constructs `ConnectX3Nic` instances when `infiniband_mlx4` is enabled; stub/no-op branches exist for `infiniband_mlx5` and the "no feature selected" case. |
| `uverbs.rs` | Kernel-side handler for the `uverbs_ctl` syscall (`uverbs_ctl(minor, cmd, arg)`). Decodes the `UverbsCmd` bitpacked command word, validates the magic number and minor number, unsafely copies fixed-size "container" structs to/from user space (`ibv_mr_container`, `ibv_cq_container`, `ibv_qp_container`, `ibv_qp_modify_container`, `ibv_cq_poll_container`, `ibv_qp_post_send_container`/`_recv_container`), and dispatches to the per-verb functions in `uverbs_cmd.rs`. `UVERBS_SUPPORTED_MINOR_TABLE` lists supported commands. |
| `uverbs_cmd.rs` | Thin per-verb wrapper functions (`uverbs_query_devices`, `uverbs_query_device`, `uverbs_query_port`, `uverbs_register_mem_region`, `uverbs_create_cq`, `uverbs_create_qp`, `uverbs_modify_qp`, `uverbs_poll_cq`, `uverbs_post_send`, `uverbs_post_recv`, `uverbs_destroy`) that look up the target NIC by minor number in the global `get_dev_list()` and call the corresponding `ConnectX3Nic` method. `uverbs_mmap_uar` is a TODO stub. |

### mlx4 driver — `os/kernel/src/device/mlx4.rs` + `os/kernel/src/device/mlx4/`

| File | Role |
|---|---|
| `mlx4.rs` | Top-level driver module and public API surface. Defines `MLX_VEND` (`0x15b3`) / `CONNECTX3_DEV` (`0x1003`) PCI IDs, the `ConnectX3Nic` struct (config registers, `CommandInterface`, firmware/ICM state, HCA, doorbells, BlueFlame buffer, event/completion/queue-pair/port vectors, minor number), the global `DEV_LIST`/minor-allocation machinery (`get_dev_list`, `next_minor`, `device_in_range`, `minor_to_idx`), and `ConnectX3Nic::init()` (full PCI/BAR mapping + firmware bring-up). Methods `query_device`, `query_port`, `create_cq`, `poll_cq`, `destroy_cq`, `create_qp`, `modify_qp`, `destroy_qp`, `post_receive`, `post_send`, `create_mr`, `destroy_mr` back the uverbs handlers directly; `Drop` tears everything down in reverse order. Internal `Offsets` struct allocates CQ/QP/EQ/DMPT numbers and doorbell indices. |
| `device/mlx4/cmd.rs` | `CommandInterface` — issues firmware commands via the mailbox/command register mechanism (`OpcodeModifier`, `InputParameter`, `OutputParameter` traits, opcodes). |
| `device/mlx4/fw.rs` | Firmware query/init: `Firmware`, `MappedFirmwareArea`, `Capabilities`, `InitHcaParameters`, `Hca`, `DoorbellEq` — queries FW version, maps firmware area, queries capabilities, initializes the HCA and ports. |
| `device/mlx4/icm.rs` | ICM (InfiniHost Context Memory) table management: `MappedIcmAuxiliaryArea`, `IcmTable`, `MrTable`/`MemoryRegion`/`DmptEntry`, `MappedIcm`, `MappedIcmTables` — backing store for QP/CQ/MR contexts, including memory-region allocation (`alloc_dmpt`) used by `create_mr`. |
| `device/mlx4/completion_queue.rs` | `CompletionQueue` — creation, arming, querying, polling (`poll`) and destruction of CQs; backend for `create_cq`/`poll_cq`/`destroy_cq`. |
| `device/mlx4/queue_pair.rs` | `QueuePair`, `WorkQueue`, WQE segment types (`WqeControlSegment`, `WqeDataSegment`, `WqeDatagramSegment`, `WqeRemoteAddressSegment`) — QP creation/modify/destroy and posting of send/receive work requests, including BlueFlame fast-path send. |
| `device/mlx4/event_queue.rs` | `EventQueue`, `EventQueueEntry`, `init_eqs` — interrupt/event queue setup used during device init and by CQ polling. |
| `device/mlx4/port.rs` | `Port`, `PortCapabilities`, `MadPacket` — port query/init/close, backend for `query_port`. |
| `device/mlx4/profile.rs` | `Resource`, `Profile` — computes ICM resource sizing profile used during `init()`. |
| `device/mlx4/device.rs` | `ResetRegisters`, `Ownership` — low-level device reset and ownership-taking sequence. |
| `device/mlx4/utils.rs` | `MappedPages`, PCI BAR mapping (`pci_map_bar_mem`), DMA page allocation/flag helpers used throughout the driver. |

### Feature gates

`os/kernel/Cargo.toml`: `infiniband_mlx4 = []`, `infiniband_mlx5 = []`, with
`default = ["frame_alloc_lockfree", "infiniband_mlx4"]` — mlx4 is built by default; mlx5
exists only as an empty placeholder feature/`_init` stub, no implementation.

### Data flow

A PCI ConnectX-3 card (vendor `0x15b3`, device `0x1003`) is discovered at boot via
`infiniband::init()` → `_init()` → PCI bus search by ID. Each match becomes a `ConnectX3Nic`
via `ConnectX3Nic::init()`, which maps BARs, resets the device, brings up
firmware/ICM/HCA/ports, and registers itself in the global `DEV_LIST` (indexed by an
allocated `minor`). At runtime, userspace issues verbs through the syscall path described in
§3 below, which is dispatched to `uverbs_cmd.rs`, which invokes the addressed `ConnectX3Nic`'s
methods, which drive the hardware through `CommandInterface`/ICM/QP/CQ/EQ. Results are copied
back into user-space buffers before returning through the syscall boundary.

## 2. ibverbs userspace library + shared rdma crate

### `os/library/ibverbs/` — structure

| File | Role |
|---|---|
| `src/ibverbs.rs` (~1720 lines) | Public high-level Rust "verbs" API, modeled on the `rust-ibverbs` crate and upstream `libibverbs`. Exposes `devices()`, `DeviceList`, `Device`, `Context`, `ProtectionDomain`, `CompletionQueue`, `QueuePair`, `MemoryRegion`, work-request helpers, etc. |
| `src/ibverbs_sys.rs` (~433 lines) | Low-level FFI-equivalent layer (`ibv_context`, `ibv_cq`, `ibv_pd`, `ibv_mr`, `ibv_qp`, `ibv_qp_init_attr`, functions `ibv_get_device_list`, `ibv_open_device`, `ibv_query_device/port`, `ibv_alloc_pd`, `ibv_reg_mr`, `ibv_create_cq/qp`, `ibv_modify_qp`, `ibv_poll_cq`, `ibv_post_send/recv`). This is the module that actually issues syscalls; `ibverbs.rs` builds on top of it (`use ibverbs_sys as ffi;`). |
| `src/sliceindex.rs` | Hand-rolled `SliceIndex`-like trait (workaround for an unstable std trait), used internally for SGE/buffer slicing. |
| `src/examples/loopback.rs` | Small example/demo program (loopback RDMA test). |

The crate is `no_std`, depends on internal crates `rdma`, `mm`, `syscall`, and external
`bincode` (optional, feature `serialize`) + `core3`. Unlike `syscall`/`naming`/`mm`, it has no
kernel-side feature gate — it's consumed only by userspace apps.

### Relationship to `os/library/rdma/`

`rdma` is a **shared, dependency-free contract crate** (no internal path deps, only
`bitflags`/`strum_macros`/`num_enum`), used independently by **both** the userspace `ibverbs`
crate and the kernel's `infiniband` module (`os/kernel/Cargo.toml` depends on it directly). It
is not a case of `ibverbs` being layered on top of `rdma` within one address space — `rdma` is
the type/wire-format vocabulary both sides depend on separately, the same pattern CLAUDE.md
describes for `syscall`/`naming`/`mm`, just without a `userspace` feature gate.

| File | Role |
|---|---|
| `src/ib_core.rs` | Core IB/RDMA type vocabulary shared by both sides: `ibv_qp_type`, `ibv_qp_cap`, `ibv_access_flags` (bitflags), `ibv_device`, `ibv_device_attr`, `ibv_wc`, `ibv_send_wr`/`ibv_recv_wr`, `ibv_qp_attr`, `ibv_port_attr`, `ibv_gid`, etc. — analogous to Linux's `verbs.h`. |
| `src/uverbs_uapi.rs` | Wire-format ("uAPI") layer: per-command "container" structs (`ibv_mr_container`, `ibv_cq_container`, `ibv_qp_container`, `ibv_qp_modify_container`, `ibv_qp_post_send_container`, `ibv_cq_poll_container`, `ibv_port_attr_container`, etc.) plus the `UverbsCmd`/`UverbsInnerCmd` command-encoding scheme (`UVERBS_CMD_*` constants, `UVERBS_MAGIC`) that packs a command number + payload size + magic + "minor present" flag into one `usize`, Linux-ioctl-style. |

### Syscall path: single multiplexed syscall

ibverbs does **not** have per-verb dedicated syscall numbers on mainline. It goes through the
general mechanism described in `docs/new-syscall.howto.md`:

1. `os/library/syscall/src/lib.rs`'s `enum SystemCall` has one variant, `Uverb`.
2. `ibverbs_sys.rs` calls `syscall(Uverb, &[dev_fd, UVERBS_CMD_XXX, arg_ptr])` for every
   operation. The command argument is an *encoded* `usize` from `rdma::uverbs_uapi::UverbsCmd`
   (command id + payload size + `UVERBS_MAGIC` + a "minor present" flag).
3. Kernel side: `SyscallTable` in `os/kernel/src/syscall/syscall_dispatcher.rs` maps `Uverb` to
   `sys_uverbs_ctl` (`os/kernel/src/syscall/sys_uverbs.rs`), which calls `uverbs_ctl(minor,
   cmd, arg)` in `os/kernel/src/infiniband/uverbs.rs`. That decodes `cmd`, validates
   magic/minor, copies structs between user/kernel memory, and dispatches into
   `uverbs_cmd.rs`, which operates on the addressed `ConnectX3Nic`.

Note: there is an abandoned/WIP branch `ibverbs_as_syscall` (commit `6907bf12`, "WIP: create
separate syscalls for ibverbs", adding `sys_ibverbs.rs`) exploring one-syscall-per-verb, but it
was never merged — mainline uses the multiplexed `Uverb` syscall described above.

### Consuming applications

- `os/application/infiniband-diags/ibping`, `.../ibstat` — diagnostic CLIs (`no_std`, use
  `runtime::*`, `terminal::println`) that call `ibverbs::devices()`, open a device context, and
  query device/port attributes (firmware version, port state, LID, SM LID, capability mask,
  link layer). Analogous to Linux's `ibping`/`ibstat`.
- `os/application/rdma/mlx4` (`bench.rs`, `handshake.rs`, `integrity.rs`, `rdma_read.rs`,
  `rdma_write.rs`, `session.rs`) — a more elaborate RDMA benchmark/demo app exercising real
  data-path RDMA read/write, checksummed payload integrity checks, and a benchmark harness,
  built on the same `ibverbs` crate.

## 3. End-to-end architecture

An application (`ibping`, `ibstat`, `rdma/mlx4`) links `ibverbs` and calls its high-level API
(`devices()`, `Device::open()`, `Context::create_cq/create_qp`, `MemoryRegion::register`,
`QueuePair::post_send/post_recv`, `CompletionQueue::poll`, ...). These forward to
`ibverbs_sys.rs`, which builds a command-specific "container" struct (defined in the shared
`rdma::uverbs_uapi` module) and issues a single general-purpose syscall, `SystemCall::Uverb`,
with a device fd, an encoded command id, and a pointer to the container. The kernel's syscall
dispatcher routes `Uverb` → `sys_uverbs_ctl` → `uverbs_ctl` in
`os/kernel/src/infiniband/uverbs.rs`, which decodes the command, copies data between
user/kernel address spaces, and dispatches into `uverbs_cmd.rs` functions that manipulate real
IB resources (memory regions, completion queues, queue pairs, work completions — protection
domains are effectively stubbed) on the `ConnectX3Nic` (mlx4) hardware driver. The verbs
abstraction itself (device, device attributes, port attributes, access flags, QP types/caps,
send/receive work requests, work completions) is defined once in the shared `rdma` crate and
used identically on both sides of the user/kernel boundary, avoiding duplication of the verbs
data model.

## Thesis notes

_(Research questions, planned modifications, and benchmark plans go here.)_

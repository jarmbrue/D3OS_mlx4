# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

D3OS is a research operating system (kernel + userspace) for data centers, written in Rust, developed by the
operating systems group at Heinrich Heine University Düsseldorf. It targets `x86_64` bare metal / QEMU, boots via
UEFI (through the `towboot` bootloader) using Multiboot2, and has no POSIX compatibility layer — the kernel and all
applications are `#![no_std]`.

## Build, run, test

All builds go through `cargo-make` (`cargo install --no-default-features cargo-make cargo-license`), never plain
`cargo build`, because the kernel and every application are each compiled against their own custom JSON target
spec (`d3os_kernel.json` / `d3os_application.json`) and then linked by hand with `ld`/`x86_64-elf-ld` using a
project-specific linker script — there is no single "host" target that produces a working binary. Always pass
`--no-workspace` at the top level so cargo-make treats the repo root as the driving Makefile instead of trying to
apply workspace defaults per member crate.

```bash
# Build + boot in QEMU (debug profile)
cargo make --no-workspace

# Same, release profile (much faster runtime)
cargo make --no-workspace --profile production

# Only produce the bootable d3os.img, don't launch QEMU
cargo make --no-workspace image

# Run the test suite (defined as `cargo test` in Makefile.toml, runs against host-compatible crates)
cargo make --no-workspace test

# Clean all build artifacts (kernel, every app, workspace-level images)
cargo make --no-workspace clean
```

There is no per-crate/per-test filtering wired through cargo-make; for iterating on a single library crate's unit
tests, `cd` into it and use plain `cargo test` (only crates that don't disable `test`/`doctest` in `Cargo.toml`
support this — the kernel crate and all application crates set `test = false`).

To type-check or lint an individual kernel/application crate against its real target (useful since a plain
`cargo check` at the repo root won't use the right target spec), run cargo-make inside that crate's directory:

```bash
cd os/kernel && cargo make --no-workspace check
cd os/kernel && cargo make --no-workspace clippy
cd os/application/<name> && cargo make --no-workspace check
```

### Debugging

```bash
cargo make --no-workspace clean
cargo make --no-workspace debug   # builds and starts QEMU halted, waiting for gdb
cargo make --no-workspace gdb     # in a second terminal, attaches and stops at boot.rs::start
```

Set breakpoints like `break kernel::naming::api::init`. To debug a userspace app, load its symbols too:
`add-symbol-file loader/initrd/bin/<app>` then `break main`. See `docs/gdb-commands.pdf` for more. Debug configs
for RustRover (`.idea/runConfigurations`), VS Code (`.vscode`), and Zed (`.zed`) mirror these cargo-make tasks.

The Rust toolchain is pinned via `rust-toolchain.toml` (nightly, with `rust-src`) since the build uses
`-Z build-std=core,alloc`; no manual `rustup` setup should be needed beyond what's listed in `README.md`.

## Architecture

### Workspace layout

- `os/kernel` — the kernel itself, one crate, `crate-type = ["staticlib"]`, linked with `os/kernel/link.ld` plus
  hand-written `boot.asm`/`boot_ap.asm` into `loader/kernel.elf`.
- `os/library/*` — crates shared between kernel and/or userspace. Some are kernel-only (`graphic`, `stream`,
  `drawer`, `system_info`, `input` — pulled into the kernel with `default-features = false`), some are
  userspace-only (`runtime`, `concurrent`, `terminal`, `libc`), and a few are compiled into *both* sides with
  differing feature sets (notably `syscall`, `naming`, `mm`) — the `userspace` feature/`#[cfg]` gates decide
  whether the arch-specific inline-asm syscall trap is compiled in versus a kernel-side no-op.
- `os/application/*` — one crate per userspace program, each `crate-type = ["staticlib"]`, linked with
  `os/application/link.ld` into a flat binary and copied into `loader/initrd/bin/`, which is tarred up as
  `initrd.tar` and embedded as a Multiboot2 module the kernel reads at boot (see `INIT_RAMDISK` in
  `os/kernel/src/lib.rs`). Every app must also be listed as a member in the root `Cargo.toml` workspace.
- `loader/` — bootloader staging area (`towboot`, `initrd/`, the assembled `kernel.elf`/`initrd.tar` before they're
  packed into `d3os.img`).

### Boot path

`boot.asm` → `boot.rs` sets up the GDT/IDT/paging, parses the Multiboot2 info (incl. ACPI RSDP and the initrd
module), then hands off into the statics initialized in `os/kernel/src/lib.rs` (APIC, PIT, PS/2, PCI bus, process
manager, etc., each behind a `Once<...>` with a `init_x()` / accessor pair). `boot_ap.rs`/`ipi.rs` handle bringing
up additional APs (multicore). Per-CPU state is threaded through `PER_CPU_REF`/`per_cpu_ref()`.

### Syscall mechanism

Userspace and kernel agree on syscall numbers purely by **enum-variant order**, not by name — there is no ID
constant shared explicitly:

- `os/library/syscall/src/lib.rs` — `enum SystemCall` (user-mode side). The `syscall()` fn issues the `syscall`
  x86_64 instruction with the enum value in `rax` and up to 6 args in `rdi`/`rsi`/`rdx`/`r10`/`r8`/`r9`.
- `os/kernel/src/syscall/syscall_dispatcher.rs` — `SyscallTable::handle`, a fixed-size array of function pointers,
  indexed by that same `rax` value via naked-asm dispatch (`syscall_handler`). Function order in the array **must
  match** `SystemCall` variant order exactly.
- Kernel-side handlers live in `os/kernel/src/syscall/sys_*.rs`, named `sys_*`, returning `SyscallResult`
  (`os/library/syscall/src/return_vals.rs`), converted to the raw `isize` ABI return value.

When adding a syscall, both sides need a change in the same relative position — see `docs/new-syscall.howto.md`.
Missing/misordering one side silently miscalls a different handler rather than failing to compile.

### Kernel module map (`os/kernel/src/`)

- `memory/` — physical frame allocation (`frames.rs`/`frames_lf.rs`, lock-free via the `llfree` crate, feature-gated
  against a locking fallback), paging/VMA/VMM, kernel heap, shared memory (`shm.rs`), NVDIMM support (`nvmem.rs`).
- `process/` — `ProcessManager`, `Process`/`Thread`, the scheduler, and core-local storage used by the syscall
  trampoline to find each core's TSS/kernel stack.
- `naming/` — the naming/VFS-like service: `tmpfs`, `procfs`, mount points, and the lookup/open-object machinery
  apps go through for `Open`/`Read`/`Write`/`Mkdir`/etc. syscalls.
- `network/` — smoltcp-backed networking stack, exposed to userspace via `sys_net.rs` (`SockOpen`/`SockBind`/...).
- `device/` — driver instances (APIC, PIT, PS/2, PCI, serial, speaker, IDE, rtl8139, virtio, mlx4). There is no
  dynamic driver framework yet; drivers are constructed once and stored as globals in `lib.rs`, with a stated
  long-term plan to move them into the naming service instead.
- `infiniband/` — InfiniBand/RDMA verbs support (`uverbs.rs`, `uverbs_cmd.rs`), paired with the userspace
  `os/library/ibverbs` and `os/library/rdma` crates and exercised by `os/application/infiniband-diags/*` and
  `os/application/rdma/mlx4`. Gated by the `infiniband_mlx4`/`infiniband_mlx5` Cargo features.
- `sync/` — IRQ-safe spinlocks and wait queues used throughout the kernel instead of std sync primitives.
- `tests/` — a custom kernel test runner (built only under the `kernel_test`/`kernel_bench` cfgs), since the
  kernel can't use the standard `#[test]` harness in a `no_std` freestanding binary.

### Adding a new application

Copy the `hello` app directory, rename it, update `name`/`path` in its `Cargo.toml`, add it to the root
`Cargo.toml` workspace `members`, and optionally register it in `os/library/globals/src/application.rs` for shell
autocompletion. Details in `docs/new-app.howto.md`.

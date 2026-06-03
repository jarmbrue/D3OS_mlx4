pub mod vmm;
pub mod vma;
pub mod pages;
pub mod frames;
pub mod frames_lf;

pub mod nvmem;
pub mod dram;
pub mod shm;

pub mod heap;
pub mod stack;
pub mod acpi_handler;

use core::sync::atomic::{AtomicUsize, Ordering};
use x86_64::structures::paging::frame::PhysFrameRange;


#[derive(PartialEq)]
#[derive(Clone, Copy, Debug)]
pub enum MemorySpace {
    Kernel,
    User
}

pub const PAGE_SIZE: usize = 0x1000;


static FREE_FRAMES: AtomicUsize = AtomicUsize::new(0);                   

pub fn init_total_free_frames() {
    FREE_FRAMES.store(get_total_free_frames(), Ordering::SeqCst);
}

pub fn get_free_frames() -> usize {
    FREE_FRAMES.load(Ordering::SeqCst)
}


/// Wrapper functions for the page frame allocator in `frames.rs` or `frames_lf.rs` (news lockfree implementation)

/// Wrapper function
pub fn init() {
    frames::init();
}

/// Wrapper function
pub fn dump() {
    frames::dump();
}

/// Wrapper function
/// Allocate `frame_count` contiguous page frames.
pub fn alloc_frames(frame_count: usize) -> PhysFrameRange {
    FREE_FRAMES.fetch_sub(frame_count, Ordering::SeqCst);
    frames::alloc(frame_count)
}

/// Wrapper function
/// Free a contiguous range of page `frames`.
pub fn free_frames(frames: PhysFrameRange) {
    FREE_FRAMES.fetch_add((frames.end - frames.start) as usize, Ordering::SeqCst);
    unsafe {
        frames::free(frames);
    }
}

/// Wrapper function
pub fn frame_allocator_locked() -> bool {
    frames::allocator_locked()
}

/// Wrapper function
pub fn get_total_free_frames() -> usize {
    frames::get_total_free_frames()
}
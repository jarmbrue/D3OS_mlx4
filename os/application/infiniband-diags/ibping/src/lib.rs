#![no_std]

use terminal::{print, println};
use rdma::ibverbs;
use runtime::*;

#[unsafe(no_mangle)]
pub fn main() {
    let devices = ibverbs::devices().expect("failed to get device list");
    println!("Found {:} devices!", devices.len());
}

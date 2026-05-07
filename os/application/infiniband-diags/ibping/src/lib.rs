#![no_std]

use terminal::{print, println};
use ibverbs::devices;
use runtime::*;

#[unsafe(no_mangle)]
pub fn main() {
    let devices = devices().expect("failed to get device list");
    println!("Found {:} devices!", devices.len());
}

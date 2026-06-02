#![no_std]

use runtime::env::args;

mod read_bw;
mod comm;

extern crate alloc;

#[unsafe(no_mangle)]
pub fn main() {
    let args = args();
    let config = read_bw::Config::default();
    read_bw::run(config).unwrap();
}

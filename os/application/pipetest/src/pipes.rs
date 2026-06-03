#![no_std]

extern crate alloc;

mod test1;
mod test2;


use naming::shared_types::OpenOptions;
use naming::{close, mkfifo, open, read, write};
use syscall::return_vals::Errno;

use concurrent::{thread,process};
#[allow(unused_imports)]
use runtime::*;
use terminal::println;


const NR_OF_ITERATIONS: u32 = 6;
const FIFO_PATH: &str = "/mypipe";


#[unsafe(no_mangle)]
pub fn main() {
    let process = process::current().unwrap();
    let thread = thread::current().unwrap();
    let main_tid = thread.id();
    println!("MAIN: pid={}, tid={}", process.id(), main_tid);

    let res = mkfifo(FIFO_PATH);
    match res { 
        Ok(_) =>  println!("MAIN:  mkfifo created"),
        Err(e) => {
            if e == Errno::EEXIST {
                println!("MAIN:  mkfifo pipe already exists");
            } else {
                println!("MAIN:  mkfifo failed, error: {:?}", e);
                return;
            }
        }
    }

    test1::test1_run();
    
    test2::test2_run();
}

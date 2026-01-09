use std::fs::File;
use std::io::Write;

fn main() {
    // Write build information to a file
    built::write_built_file().expect("Failed to acquire build-time information");
}

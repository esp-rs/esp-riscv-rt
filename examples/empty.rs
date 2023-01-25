#![no_std]
#![no_main]

use panic_halt as _;

#[esp_riscv_rt::entry]
fn main() -> ! {
    // do something here
    loop {}
}

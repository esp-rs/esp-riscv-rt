#![no_std]
#![no_main]

use panic_halt as _;
use riscv::{
    asm::wfi,
    register::{mie, mip},
};

#[export_name = "_mp_hook"]
#[rustfmt::skip]
pub extern "Rust" fn user_mp_hook(hartid: usize) -> bool {
    if hartid == 0 {
        true
    } else {
        let addr = 0x02000000 + hartid * 4;
        unsafe {
            // Clear IPI
            (addr as *mut u32).write_volatile(0);

            // Start listening for software interrupts
            mie::set_msoft();

            loop {
                wfi();
                if mip::read().msoft() {
                    break;
                }
            }

            // Stop listening for software interrupts
            mie::clear_msoft();

            // Clear IPI
            (addr as *mut u32).write_volatile(0);
        }
        false
    }
}

#[esp_riscv_rt::entry]
fn main(hartid: usize) -> ! {
    if hartid == 0 {
        // Waking hart 1...
        let addr = 0x02000004;
        unsafe {
            (addr as *mut u32).write_volatile(1);
        }
    }

    loop {}
}

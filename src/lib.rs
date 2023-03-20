//! Minimal startup / runtime for RISC-V CPUs from Espressif
//!
//! # Minimum Supported Rust Version (MSRV)
//!
//! This crate is guaranteed to compile on stable Rust 1.60 and up. It *might*
//! compile with older versions but that may change in any new patch release.
//!
//! # Features
//!
//! This crate provides:
//!
//! - Before main initialization of the `.bss` and `.data` sections
//! - `#[entry]` to declare the entry point of the program
//! - `#[pre_init]` to run code *before* `static` variables are initialized

// NOTE: Adapted from riscv-rt/src/lib.rs
#![no_std]

use core::arch::global_asm;

pub use riscv;
use riscv::register::{
    mstatus,
    mcause,
    mhartid,
    mtvec::{self, TrapMode},
};
pub use riscv_rt_macros::{entry, pre_init};


pub use self::Interrupt as interrupt;

#[export_name = "error: esp-riscv-rt appears more than once in the dependency graph"]
#[doc(hidden)]
pub static __ONCE__: () = ();

extern "C" {
    // Boundaries of the .bss section
    static mut _ebss: u32;
    static mut _sbss: u32;

    // Boundaries of the .data section
    static mut _edata: u32;
    static mut _sdata: u32;

    // Initial values of the .data section (stored in Flash)
    static _sidata: u32;
}

/// Rust entry point (_start_rust)
///
/// Zeros bss section, initializes data section and calls main. This function
/// never returns.
#[link_section = ".init.rust"]
#[export_name = "_start_rust"]
pub unsafe extern "C" fn start_rust(a0: usize, a1: usize, a2: usize) -> ! {
    extern "Rust" {
        // This symbol will be provided by the user via `#[entry]`
        fn main(a0: usize, a1: usize, a2: usize) -> !;

        // This symbol will be provided by the user via `#[pre_init]`
        fn __pre_init();

        fn _setup_interrupts();

        fn _mp_hook(hartid: usize) -> bool;
    }

    // sbi passes hartid as first parameter (a0)
    let hartid = mhartid::read();

    if _mp_hook(hartid) {
        __pre_init();

        r0::zero_bss(&mut _sbss, &mut _ebss);
        r0::init_data(&mut _sdata, &mut _edata, &_sidata);
    }

    _setup_interrupts();

    main(a0, a1, a2);
}

/// Registers saved in trap handler
#[allow(missing_docs)]
#[derive(Debug, Default, Clone, Copy)]
#[repr(C)]
pub struct TrapFrame {
    pub ra: usize,
    pub t0: usize,
    pub t1: usize,
    pub t2: usize,
    pub t3: usize,
    pub t4: usize,
    pub t5: usize,
    pub t6: usize,
    pub a0: usize,
    pub a1: usize,
    pub a2: usize,
    pub a3: usize,
    pub a4: usize,
    pub a5: usize,
    pub a6: usize,
    pub a7: usize,
    pub s0: usize,
    pub s1: usize,
    pub s2: usize,
    pub s3: usize,
    pub s4: usize,
    pub s5: usize,
    pub s6: usize,
    pub s7: usize,
    pub s8: usize,
    pub s9: usize,
    pub s10: usize,
    pub s11: usize,
    pub gp: usize,
    pub tp: usize,
    pub sp: usize,
    pub pc: usize,
    pub mstatus: usize,
    pub mcause: usize,
    pub mtval: usize,
}

/// Trap entry point rust (_start_trap_rust)
///
/// `scause`/`mcause` is read to determine the cause of the trap. XLEN-1 bit
/// indicates if it's an interrupt or an exception. The result is examined and
/// ExceptionHandler or one of the core interrupt handlers is called.
#[link_section = ".trap.rust"]
#[export_name = "_start_trap_rust"]
pub unsafe extern "C" fn start_trap_rust(trap_frame: *const TrapFrame) {
    extern "C" {
        fn ExceptionHandler(trap_frame: &TrapFrame);
        fn DefaultHandler();
    }

    unsafe {
        let cause = mcause::read();
        if cause.is_exception() {
            ExceptionHandler(&*trap_frame)
        } else if cause.code() < __INTERRUPTS.len() {
            let h = &__INTERRUPTS[cause.code()];
            if h.reserved == 0 {
                DefaultHandler();
            } else {
                mstatus::set_mie();
                (h.handler)();
            }
        } else {
            DefaultHandler();
        }
    }
}

#[doc(hidden)]
#[no_mangle]
#[allow(unused_variables, non_snake_case)]
pub fn DefaultExceptionHandler(trap_frame: &TrapFrame) -> ! {
    loop {
        // Prevent this from turning into a UDF instruction
        // see rust-lang/rust#28728 for details
        continue;
    }
}

#[doc(hidden)]
#[no_mangle]
#[allow(unused_variables, non_snake_case)]
pub fn DefaultInterruptHandler() {
    loop {
        // Prevent this from turning into a UDF instruction
        // see rust-lang/rust#28728 for details
        continue;
    }
}

// Interrupts
#[doc(hidden)]
pub enum Interrupt {
    UserSoft,
    SupervisorSoft,
    MachineSoft,
    UserTimer,
    SupervisorTimer,
    MachineTimer,
    UserExternal,
    SupervisorExternal,
    MachineExternal,
}

extern "C" {
    fn UserSoft();
    fn SupervisorSoft();
    fn MachineSoft();
    fn UserTimer();
    fn SupervisorTimer();
    fn MachineTimer();
    fn UserExternal();
    fn SupervisorExternal();
    fn MachineExternal();
}

#[doc(hidden)]
pub union Vector {
    pub handler: unsafe extern "C" fn(),
    pub reserved: usize,
}

#[doc(hidden)]
#[no_mangle]
pub static __INTERRUPTS: [Vector; 12] = [
    Vector { handler: UserSoft },
    Vector {
        handler: SupervisorSoft,
    },
    Vector { reserved: 0 },
    Vector {
        handler: MachineSoft,
    },
    Vector { handler: UserTimer },
    Vector {
        handler: SupervisorTimer,
    },
    Vector { reserved: 0 },
    Vector {
        handler: MachineTimer,
    },
    Vector {
        handler: UserExternal,
    },
    Vector {
        handler: SupervisorExternal,
    },
    Vector { reserved: 0 },
    Vector {
        handler: MachineExternal,
    },
];

#[doc(hidden)]
#[no_mangle]
#[rustfmt::skip]
pub unsafe extern "Rust" fn default_pre_init() {}

#[doc(hidden)]
#[no_mangle]
#[rustfmt::skip]
pub extern "Rust" fn default_mp_hook(hartid: usize) -> bool {
    match hartid {
        0 => true,
        _ => loop {
            unsafe { riscv::asm::wfi() }
        },
    }
}

/// Default implementation of `_setup_interrupts` that sets `mtvec`/`stvec` to a
/// trap handler address.
#[doc(hidden)]
#[no_mangle]
#[rustfmt::skip]
pub unsafe extern "Rust" fn default_setup_interrupts() {
    extern "C" {
        fn _start_trap();
    }

    mtvec::write(_start_trap as usize, TrapMode::Direct);
}

global_asm!(
    r#"

/*
    Entry point of all programs (_start).

    It initializes DWARF call frame information, the stack pointer, the
    frame pointer (needed for closures to work in start_rust) and the global
    pointer. Then it calls _start_rust.
*/

.section .init, "ax"
.global _start

_start:
    /* Jump to the absolute address defined by the linker script. */
    lui ra, %hi(_abs_start)
    jr %lo(_abs_start)(ra)

// .section .text

_abs_start:
    .option norelax
    .cfi_startproc
    .cfi_undefined ra

    // Unsupported on ESP32-C2/C3
    // csrw mie, 0
    // csrw mip, 0

    li  x1, 0
    li  x2, 0
    li  x3, 0
    li  x4, 0
    li  x5, 0
    li  x6, 0
    li  x7, 0
    li  x8, 0
    li  x9, 0
    li  x10,0
    li  x11,0
    li  x12,0
    li  x13,0
    li  x14,0
    li  x15,0
    li  x16,0
    li  x17,0
    li  x18,0
    li  x19,0
    li  x20,0
    li  x21,0
    li  x22,0
    li  x23,0
    li  x24,0
    li  x25,0
    li  x26,0
    li  x27,0
    li  x28,0
    li  x29,0
    li  x30,0
    li  x31,0

    .option push
    .option norelax
    la gp, __global_pointer$
    .option pop

    // Check hart ID
    csrr t2, mhartid
    lui t0, %hi(_max_hart_id)
    add t0, t0, %lo(_max_hart_id)
    bgtu t2, t0, abort

    // Allocate stacks
    la sp, _stack_start
    lui t0, %hi(_hart_stack_size)
    add t0, t0, %lo(_hart_stack_size)

    beqz t2, 2f  // Jump if single-hart
    mv t1, t2
    mv t3, t0
1:
    add t0, t0, t3
    addi t1, t1, -1
    bnez t1, 1b
2:
    sub sp, sp, t0

    // Set frame pointer
    add s0, sp, zero

    jal zero, _start_rust

    .cfi_endproc

/*
    Trap entry point (_start_trap)

    Saves caller saved registers ra, t0..6, a0..7, calls _start_trap_rust,
    restores caller saved registers and then returns.
*/
.section .trap, "ax"
.global default_start_trap

// .option norelax
// .align 6

default_start_trap:
    addi sp, sp, -40*4
    sw ra, 0*4(sp)
    sw t0, 1*4(sp)
    sw t1, 2*4(sp)
    sw t2, 3*4(sp)
    sw t3, 4*4(sp)
    sw t4, 5*4(sp)
    sw t5, 6*4(sp)
    sw t6, 7*4(sp)
    sw a0, 8*4(sp)
    sw a1, 9*4(sp)
    sw a2, 10*4(sp)
    sw a3, 11*4(sp)
    sw a4, 12*4(sp)
    sw a5, 13*4(sp)
    sw a6, 14*4(sp)
    sw a7, 15*4(sp)
    sw s0, 16*4(sp)
    sw s1, 17*4(sp)
    sw s2, 18*4(sp)
    sw s3, 19*4(sp)
    sw s4, 20*4(sp)
    sw s5, 21*4(sp)
    sw s6, 22*4(sp)
    sw s7, 23*4(sp)
    sw s8, 24*4(sp)
    sw s9, 25*4(sp)
    sw s10, 26*4(sp)
    sw s11, 27*4(sp)
    sw gp, 28*4(sp)
    sw tp, 29*4(sp)
    csrrs t1, mepc, x0
    sw t1, 31*4(sp)
    csrrs t1, mstatus, x0
    sw t1, 32*4(sp)
    csrrs t1, mcause, x0
    sw t1, 33*4(sp)
    csrrs t1, mtval, x0
    sw t1, 34*4(sp)
    addi s0, sp, 40*4
    sw s0, 30*4(sp)
    
    jal ra, set_prio
    /* returned old prio, stack it*/
    sw a0, 35*4(sp)
    
    add a0, sp, zero

    jal ra, _start_trap_rust_hal

    /* load stacked interrupt id so we may restore it*/
    lw a0, 35*4(sp) 
    
    jal ra, restore_prio

    lw t1, 31*4(sp)
    csrrw x0, mepc, t1
    lw t1, 32*4(sp)
    csrrw x0, mstatus, t1
    lw ra, 0*4(sp)
    lw t0, 1*4(sp)
    lw t1, 2*4(sp)
    lw t2, 3*4(sp)
    lw t3, 4*4(sp)
    lw t4, 5*4(sp)
    lw t5, 6*4(sp)
    lw t6, 7*4(sp)
    lw a0, 8*4(sp)
    lw a1, 9*4(sp)
    lw a2, 10*4(sp)
    lw a3, 11*4(sp)
    lw a4, 12*4(sp)
    lw a5, 13*4(sp)
    lw a6, 14*4(sp)
    lw a7, 15*4(sp)
    lw s0, 16*4(sp)
    lw s1, 17*4(sp)
    lw s2, 18*4(sp)
    lw s3, 19*4(sp)
    lw s4, 20*4(sp)
    lw s5, 21*4(sp)
    lw s6, 22*4(sp)
    lw s7, 23*4(sp)
    lw s8, 24*4(sp)
    lw s9, 25*4(sp)
    lw s10, 26*4(sp)
    lw s11, 27*4(sp)
    lw gp, 28*4(sp)
    lw tp, 29*4(sp)
    lw sp, 30*4(sp)
    # SP was restored from the original SP
    mret

/* Make sure there is an abort when linking */
.section .text.abort
.globl abort
abort:
    j abort

/*
    Interrupt vector table (_vector_table)
*/

.section .trap, "ax"
.global _vector_table
.type _vector_table, @function

.option push
.balign 0x100
.option norelax
.option norvc

_vector_table:
    j _start_trap
    .rept 31
    j _start_trap
    .endr

.option pop
"#

);

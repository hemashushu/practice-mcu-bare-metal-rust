// Copyright (c) 2022 Hemashushu <hippospark@gmail.com>, All rights reserved.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

use core::ptr;

// PM0214 2.3.2 Exception types
// Table 17. Properties of the different exception types
extern "C" {
    /**
     * A NonMaskable Interrupt (NMI) can be signalled by a peripheral or
     * triggered by software. This is the highest priority exception other than
     * reset. It is permanently enabled and has a fixed priority of -2. NMIs
     * cannot be:
     *
     * - Masked or prevented from activation by any other exception
     * - Preempted by any exception other than Reset.
     */
    fn NMI_Handler();

    /**
     * A hard fault is an exception that occurs because of an error during
     * exception processing, or because an exception cannot be managed by
     * any other exception mechanism. Hard faults have a fixed priority of -1,
     * meaning they have higher priority than any exception with configurable
     * priority.
     */
    fn HardFault_Handler();
    fn MemManage_Handler();
    fn BusFault_Handler();
    fn UsageFault_Handler();

    /**
     * A supervisor call (SVC) is an exception that is triggered by the SVC
     * instruction. In an OS environment, applications can use SVC
     * instructions to access OS kernel functions and device drivers.
     */
    fn SVC_Handler();

    /**
     * PendSV is an interrupt-driven request for system-level service. In an
     * OS environment, use PendSV for context switching when no other
     * exception is active.
     */
    fn PendSV_Handler();

    /**
     * A SysTick exception is an exception the system timer generates when
     * it reaches zero. Software can also generate a SysTick exception. In an
     * OS environment, the processor can use this exception as system tick.
     */
    fn SysTick_Handler();
}

extern "C" {
    fn main() -> !;
}

// $ arm-none-eabi-objdump -j .vector_table -s target/thumbv7m-none-eabi/debug/bare-metal-blinky
//
// ```
// Contents of section .vector_table:
// 8000000 00500020 41040008 b9040008 b9040008  .P. A...........
// ```
//
// Note:
// the second entry `41040008` (i.e. `0x0800_0441`) in the `.vector_table`
// is the address of the function `Reset_Handler`. note this address is
// 1 byte behind of actual function address `0x0800_0440` because of the
// executed in thumb mode.
//
// running check
//
// (gdb) x/2xw 0x08000000
// 0x8000000:      0x20005000      0x08000441
//
#[no_mangle]
pub extern "C" fn Reset_Handler() -> ! {
    // $ arm-none-eabi-nm target/thumbv7m-none-eabi/debug/bare-metal-blinky
    //
    // ```
    // 08000004 R Reset_Vector
    // 08000008 R Exceptions
    // 080002a6 t rust_begin_unwind
    // 080002ae T main
    // 080004ca T Reset_Handler
    // 080005a2 T HardFault_Handler
    // 080005a2 T MemManage_Handler
    // 080005a2 T NMI_Handler
    // 080005a2 T PendSV_Handler
    // 08001c98 A _sidata
    //
    // 20000000 D _sdata
    // 20000000 D SHOULD_LOCATED_IN_DATA_SECTION
    // 20000020 D _edata
    // 20000020 B _sbss
    // 20000020 B SHOULD_LOCATED_BSS_SECTION
    // 20000048 B _ebss
    // 20005000 A _estack
    // 20000048 B _heap_start
    // ```
    //
    // running check
    //
    // (gdb) i variables
    //
    // ```
    // Non-debugging symbols:
    // 0x08000004  Reset_Vector
    // 0x08000008  Exceptions
    // 0x20000000  _sdata
    // 0x20000000  SHOULD_LOCATED_IN_DATA_SECTION
    // 0x2000000c  bare_metal_blinky::common::SYSCLK
    // 0x20000010  bare_metal_blinky::common::HCLK
    // 0x20000014  bare_metal_blinky::common::PCLK1
    // 0x20000018  bare_metal_blinky::common::PCLK2
    // 0x2000001c  bare_metal_blinky::common::ADCCLK
    // 0x20000020  _edata
    // 0x20000020  _sbss
    // 0x20000020  SHOULD_LOCATED_BSS_SECTION
    // 0x20000040  bare_metal_blinky::common::SYSTICKS
    // 0x20000048  _ebss
    // 0x20000048  _heap_start
    // ```
    //
    // These symbols come from `linker.ld`
    //
    // note:
    // the variable (e.g. `_sbss`) following the `extern "C" { static ...` is the
    // value of the address, but what we need is the address itself.
    // e.g. the `_sbss` value is the memory content from address `0x2000_0100`.
    extern "C" {
        static mut _sbss: u8; // Start of .bss section
        static _ebss: u8; // End of .bss section
        static mut _sdata: u8; // Start of .data section
        static _edata: u8; // End of .data section
        static _sidata: u8; // Start of .rodata section
        static _estack: u8; // Stack bottom
        static _heap_start: u8; // Heap start
    }

    // set initial stack pointer
    //
    // note::
    // since the stack point is already included in the vector_table (in the first entry),
    // this statement is not necessary.
    //
    // unsafe {
    //     asm!("ldr sp, = _estack",);
    // }

    // running check
    //
    // (gdb) i r sp
    //
    // ```
    // sp             0x20004f30          0x20004f30
    // ```

    // for testing purposes only
    unsafe {
        // running check
        //
        // (gdb) set output-radix 16
        // Output radix now set to decimal 16, hex 10, octal 20.
        //
        // (gdb) i locals
        // o = 0x20000048
        // n = 0x8001c98
        // m = 0x20005000
        // l = 0x20000020 <SHOULD_LOCATED_BSS_SECTION>
        // k = 0x20000000 <SHOULD_LOCATED_IN_DATA_SECTION>
        // j = 0x20000048
        // i = 0x20000020 <SHOULD_LOCATED_BSS_SECTION>
        let i = &_sbss;
        let j = &_ebss;
        let k = &_sdata;
        let l = &_edata;
        let m = &_estack;
        let n = &_sidata;
        let o = &_heap_start;
        let r = 0;
    }

    // running check
    //
    // (gdb) x/10xw 0x20000000
    // 0x20000000 :    0x2e006816      0x6855d018      0xd0f942b5      0x8026882e
    // 0x20000010 :    0x34023502      0x270168c6      0xd1fb423e      0x423e2714
    // 0x20000020 :    0x429dd108      0x4615d301
    //
    // note:
    // these are the random numbers in memory that
    // have not been initialized.

    // initialize `BSS`
    // set all bytes within `.bss` to `0`
    //
    // note::
    // `write_volatile` means volatile write of a memory
    //
    // unsafe {
    //     (&_sbss as *const u8 as u32..&_ebss as *const u8 as u32)
    //         .for_each(|dest_addr| (dest_addr as *mut u8).write_volatile(0))
    // }
    //
    unsafe {
        let count = &_ebss as *const u8 as usize - &_sbss as *const u8 as usize;
        ptr::write_bytes(&mut _sbss as *mut u8, 0, count);
    }

    // initialize `Data`
    // copy the content of `.data` from `flash` to `RAM`
    //
    // unsafe {
    //     let data_source_addr = &_sidata as *const u8 as u32;
    //     (&_sdata as *const u8 as u32..&_edata as *const u8 as u32)
    //         .enumerate()
    //         .for_each(|(index, dest_addr)| {
    //             (dest_addr as *mut u8)
    //                 .write_volatile(*((index as u32 + data_source_addr) as *const u8))
    //         })
    // }

    unsafe {
        let count = &_edata as *const u8 as usize - &_sdata as *const u8 as usize;
        ptr::copy_nonoverlapping(&_sidata as *const u8, &mut _sdata as *mut u8, count);
    }

    // running check
    //
    // (gdb) x/10xw 0x20000000
    // 0x20000000 :    0x00000001      0x00000002      0x00000003      0x007a1200
    // 0x20000010 :    0x007a1200      0x007a1200      0x007a1200      0x003d0900
    // 0x20000020 :    0x00000000      0x00000000

    // running check
    //
    // (gdb) i addr bare_metal_blinky::bare_main
    // Symbol "bare_metal_blinky::bare_main" is a function at address 0x80002ae.
    //
    // (gdb) x/10i 0x80004c4
    // 0x80002ae <main>:    push    {r7, lr}
    // 0x80002b0 <main+2>:  mov     r7, sp
    // 0x80002b2 <main+4>:  sub     sp, #40 ; 0x28
    // 0x80002b4 <main+6>:  movw    r1, #6248       ; 0x1868
    // 0x80002b8 <main+10>: movt    r1, #2048       ; 0x800
    // 0x80002bc <main+14>: str     r1, [sp, #24]
    // ...

    // Call user's main function
    unsafe { main() }
}

pub union Vector {
    reserved: u32,
    handler: unsafe extern "C" fn(),
}

#[link_section = ".vector_table.reset_vector"]
#[no_mangle]
pub static Reset_Vector: extern "C" fn() -> ! = Reset_Handler;

#[link_section = ".vector_table.exceptions"]
#[no_mangle]
pub static Exceptions: [Vector; 14] = [
    Vector { handler: NMI_Handler },
    Vector { handler: HardFault_Handler },
    Vector { handler: MemManage_Handler },
    Vector { handler: BusFault_Handler },
    Vector {
        handler: UsageFault_Handler,
    },
    Vector { reserved: 0 },
    Vector { reserved: 0 },
    Vector { reserved: 0 },
    Vector { reserved: 0 },
    Vector { handler: SVC_Handler },
    Vector { reserved: 0 },
    Vector { reserved: 0 },
    Vector { handler: PendSV_Handler },
    Vector { handler: SysTick_Handler },

    // IRQ starts here

    //  An interrupt, or IRQ, is an exception signalled by a peripheral, or
    //  generated by a software request. All interrupts are asynchronous to
    //  instruction execution. In the system, peripherals use interrupts to
    //  communicate with the processor.
];

#[no_mangle]
pub extern "C" fn Default_Handler() -> ! {
    loop {}
}

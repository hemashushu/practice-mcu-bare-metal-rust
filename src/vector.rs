// Copyright (c) 2022 Hemashushu <hippospark@gmail.com>, All rights reserved.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

use core::ptr;

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
// is the address of the function `ResetHandler`. note this address is
// 1 byte behind of actual function address `0x0800_0440` because of the
// executed in thumb mode.
//
// running check
//
// (gdb) x/2xw 0x08000000
// 0x8000000:      0x20005000      0x08000441
//
#[no_mangle]
pub extern "C" fn ResetHandler() -> ! {
    // $ arm-none-eabi-nm target/thumbv7m-none-eabi/debug/bare-metal-blinky
    //
    // ```
    // 20000038 B __ebss
    // 2000000c D __edata
    // 20000038 B __heap_start
    // 080004d8 T main
    // 08000440 T ResetHandler
    // 20000010 B __sbss
    // 20000000 D __sdata
    // 20000010 B SHOULD_LOCATED_BSS_SECTION
    // 20000000 D SHOULD_LOCATED_IN_DATA_SECTION
    // 080017b0 A __sidata
    // 20005000 A __stack_start
    // ```
    //
    // running check
    //
    // (gdb) i variables
    //
    // ```
    // Non-debugging symbols:
    // 0x20000000  SHOULD_LOCATED_IN_DATA_SECTION
    // 0x20000000  __sdata
    // 0x2000000c  __edata
    // 0x20000010  SHOULD_LOCATED_BSS_SECTION
    // 0x20000010  __sbss
    // 0x20000038  __ebss
    // 0x20000038  __heap_start
    // ```
    //
    // These symbols come from `linker.ld`
    //
    // extern "C" {
    //     fn __sbss(); // Start of .bss section
    //     fn __ebss(); // End of .bss section
    //     fn __sdata(); // Start of .data section
    //     fn __edata(); // End of .data section
    //     fn __sidata(); // Start of .rodata section
    //     fn __stack_start(); // Stack bottom
    //     fn __heap_start(); // Heap start
    // }
    //
    // note:
    // the name (e.g. `__sbss`) following the `function` is the variable address of the symbol.
    // the name (e.g. `__sbss`) following the `static` is the variable value of the address.
    // e.g. the `fn __sbss` value is u32 `0x2000_0100`, the `static __sbss` value is
    // the memory content from address `0x2000_0100`.
    extern "C" {
        static mut __sbss: u8; // Start of .bss section
        static __ebss: u8; // End of .bss section
        static mut __sdata: u8; // Start of .data section
        static __edata: u8; // End of .data section
        static __sidata: u8; // Start of .rodata section
        static __stack_start: u8; // Stack bottom
        static __heap_start: u8; // Heap start
    }

    // set initial stack pointer
    //
    // note::
    // since the stack point is already included in the vector_table (in the first entry),
    // this statement is not necessary.
    //
    // unsafe {
    //     asm!("ldr sp, = __stack_start",);
    // }

    // running check
    //
    // (gdb) i r sp
    //
    // ```
    // sp             0x20004fc0          0x20004fc0
    // ```

    // for testing purposes only
    unsafe {
        // running check
        //
        // (gdb) set output-radix 16
        // Output radix now set to decimal 16, hex 10, octal 20.
        // (gdb) i locals
        // o = 0x20000038
        // n = 0x80017b0
        // m = 0x20005000
        // l = 0x2000000c
        // k = 0x20000000
        // j = 0x20000038
        // i = 0x20000010
        //
        // (gdb) i locals
        // o = 0x20000038
        // n = 0x8001738
        // m = 0x20005000
        // l = 0x2000000c
        // k = 0x20000000 <SHOULD_LOCATED_IN_DATA_SECTION>
        // j = 0x20000038
        // i = 0x20000010 <SHOULD_LOCATED_BSS_SECTION>
        let i = &__sbss;
        let j = &__ebss;
        let k = &__sdata;
        let l = &__edata;
        let m = &__stack_start;
        let n = &__sidata;
        let o = &__heap_start;
        let r = 0;
    }

    // running check
    //
    // (gdb) x/10xw 0x20000000
    // 0x20000000 <SHOULD_LOCATED_IN_DATA_SECTION>:    0x2e006816      0x6855d018      0xd0f942b5      0x8026882e
    // 0x20000010 <SHOULD_LOCATED_BSS_SECTION>:        0x34023502      0x270168c6      0xd1fb423e      0x423e2714
    // 0x20000020 <SHOULD_LOCATED_BSS_SECTION+16>:     0x429dd108      0x4615d301
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
    //     (&__sbss as *const u8 as u32..&__ebss as *const u8 as u32)
    //         .for_each(|dest_addr| (dest_addr as *mut u8).write_volatile(0))
    // }
    //
    unsafe {
        let count = &__ebss as *const u8 as usize - &__sbss as *const u8 as usize;
        ptr::write_bytes(&mut __sbss as *mut u8, 0, count);
    }

    // initialize `Data`
    // copy the content of `.data` from `flash` to `RAM`
    //
    // unsafe {
    //     let data_source_addr = &__sidata as *const u8 as u32;
    //     (&__sdata as *const u8 as u32..&__edata as *const u8 as u32)
    //         .enumerate()
    //         .for_each(|(index, dest_addr)| {
    //             (dest_addr as *mut u8)
    //                 .write_volatile(*((index as u32 + data_source_addr) as *const u8))
    //         })
    // }

    unsafe {
        let count = &__edata as *const u8 as usize - &__sdata as *const u8 as usize;
        ptr::copy_nonoverlapping(&__sidata as *const u8, &mut __sdata as *mut u8, count);
    }

    // running check
    //
    // (gdb) x/10xw 0x20000000
    // 0x20000000 <SHOULD_LOCATED_IN_DATA_SECTION>:    0x00000001      0x00000002      0x00000003      0x00000000
    // 0x20000010 <SHOULD_LOCATED_BSS_SECTION+4>:      0x00000000      0x00000000      0x00000000      0x00000000
    // 0x20000020 <SHOULD_LOCATED_BSS_SECTION+20>:     0x00000000      0x00000000

    // running check
    //
    // (gdb) i addr bare_metal_blinky::bare_main
    // Symbol "bare_metal_blinky::bare_main" is a function at address 0x80004c4.
    //
    // (gdb) x/10i 0x80004c4
    // 0x80004c4 <main>:    push    {r7, lr}
    // 0x80004c6 <main+2>:  mov     r7, sp

    // Call user's main function
    unsafe { main() }
}

pub union Vector {
    reserved: u32,
    handler: unsafe extern "C" fn(),
}

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
    fn NMI();

    /**
     * A hard fault is an exception that occurs because of an error during
     * exception processing, or because an exception cannot be managed by
     * any other exception mechanism. Hard faults have a fixed priority of -1,
     * meaning they have higher priority than any exception with configurable
     * priority.
     */
    fn HardFault();
    fn MemManage();
    fn BusFault();
    fn UsageFault();

    /**
     * A supervisor call (SVC) is an exception that is triggered by the SVC
     * instruction. In an OS environment, applications can use SVC
     * instructions to access OS kernel functions and device drivers.
     */
    fn SVCall();

    /**
     * PendSV is an interrupt-driven request for system-level service. In an
     * OS environment, use PendSV for context switching when no other
     * exception is active.
     */
    fn PendSV();

    /**
     * A SysTick exception is an exception the system timer generates when
     * it reaches zero. Software can also generate a SysTick exception. In an
     * OS environment, the processor can use this exception as system tick.
     */
    fn SysTick();
}

#[link_section = ".vector_table.reset_vector"]
#[no_mangle]
pub static RESET_VECTOR: extern "C" fn() -> ! = ResetHandler;

#[link_section = ".vector_table.exceptions"]
#[no_mangle]
pub static EXCEPTIONS: [Vector; 14] = [
    Vector { handler: NMI },
    Vector { handler: HardFault },
    Vector { handler: MemManage },
    Vector { handler: BusFault },
    Vector {
        handler: UsageFault,
    },
    Vector { reserved: 0 },
    Vector { reserved: 0 },
    Vector { reserved: 0 },
    Vector { reserved: 0 },
    Vector { handler: SVCall },
    Vector { reserved: 0 },
    Vector { reserved: 0 },
    Vector { handler: PendSV },
    Vector { handler: SysTick },

    // IRQ starts here

    //  An interrupt, or IRQ, is an exception signalled by a peripheral, or
    //  generated by a software request. All interrupts are asynchronous to
    //  instruction execution. In the system, peripherals use interrupts to
    //  communicate with the processor.
];

#[no_mangle]
pub extern "C" fn DefaultExceptionHandler() -> ! {
    loop {}
}

// Copyright (c) 2022 Hemashushu <hippospark@gmail.com>, All rights reserved.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

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
// `41040008` = `0x0800_0441`, the Rust `debug` profile build
// the `.vector_table` the second entry (i.e. the `reset_handler`)
// with one byte offset to the address for unknown reasons,
// the correct address should be `0x0800_0440`.
//
//
// running check
//
// (gdb) x/2xw 0x08000000
// 0x8000000:      0x20005000      0x08000441
//
#[no_mangle]
pub extern "C" fn reset_handler() -> ! {

    // $ arm-none-eabi-nm target/thumbv7m-none-eabi/debug/bare-metal-blinky
    //
    // ```
    // 20000038 B __ebss
    // 2000000c D __edata
    // 20000038 B __heap_start
    // 080004d8 T main
    // 08000440 T reset_handler
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
    extern "C" {
        // These symbols come from `linker.ld`
        fn __sbss(); // Start of .bss section
        fn __ebss(); // End of .bss section
        fn __sdata(); // Start of .data section
        fn __edata(); // End of .data section
        fn __sidata(); // Start of .rodata section
        fn __stack_start(); // Stack bottom
        fn __heap_start(); // Heap start
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
    {
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
        let i = __sbss as u32;
        let j = __ebss as u32;
        let k = __sdata as u32;
        let l = __edata as u32;
        let m = __stack_start as u32;
        let n = __sidata as u32;
        let o = __heap_start as u32;
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
    unsafe {
        (__sbss as u32..__ebss as u32)
            .for_each(|dest_addr| (dest_addr as *mut u8).write_volatile(0))
    }

    // initialize `Data`
    // copy the content of `.data` from `flash` to `RAM`
    unsafe {
        (__sdata as u32..__edata as u32)
            .enumerate()
            .for_each(|(index, dest_addr)| {
                (dest_addr as *mut u8)
                    .write_volatile(*((index as u32 + __sidata as u32) as *const u8))
            })
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

extern "C" {
    fn NMI();
    fn HardFault(); // fn HardFaultTrampoline() ;
    fn MemManage();
    fn BusFault();
    fn UsageFault();
    fn SVCall();
    fn PendSV();
    fn SysTick();
}

#[link_section = ".vector_table.reset_vector"]
#[no_mangle]
pub static RESET_VECTOR: extern "C" fn() -> ! = reset_handler;

#[link_section = ".vector_table.exceptions"]
#[no_mangle]
pub static EXCEPTIONS: [Vector; 14] = [
    Vector { handler: NMI },
    Vector { handler: HardFault },
    // Vector {
    //     handler: HardFaultTrampoline,
    // },
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
];

#[no_mangle]
pub extern "C" fn default_exception_handler() -> ! {
    loop {}
}

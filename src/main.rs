// Copyright (c) 2022 Hemashushu <hippospark@gmail.com>, All rights reserved.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

#![no_std]
#![no_main]

#[macro_use]
mod utils;

mod common;
mod peripheral;
mod pin;
mod register_flash;
mod register_gpio;
mod register_rcc;
mod register_systick;
mod register_usart;
mod startup;

use core::panic::PanicInfo;
use peripheral::{
    clock_init, gpio_clock_on, gpio_init, gpio_read, gpio_write, systick_init_with_millisecond,
    tick_delay, uart1_init, uart_write_str,
};
use register_gpio::{GPIO_CNF, GPIO_CNF_INPUT, GPIO_CNF_OUTPUT, GPIO_MODE};
use register_usart::get_usart1_register;
use utils::spin;

// The following variables are used for testing purposes only:
//
// - SHOULD_LOCATED_IN_DATA_SECTION
// - SHOULD_LOCATED_BSS_SECTION
// - SHOULD_LOCATED_IN_RODATA_SECTION
//
// $ arm-none-eabi-nm target/thumbv7m-none-eabi/debug/bare-metal-blinky
//
// ```
// 2000000c B SHOULD_LOCATED_BSS_SECTION
// 20000000 D SHOULD_LOCATED_IN_DATA_SECTION
// ```
//
// $ arm-none-eabi-objdump -j .data -s target/thumbv7m-none-eabi/debug/bare-metal-blinky
//
// ```
// Contents of section .data:
//  20000000 01000000 02000000 03000000           ............
// ```
//
#[no_mangle] // preventing name changes by the compiler
pub static mut SHOULD_LOCATED_IN_DATA_SECTION: [u32; 3] = [1, 2, 3];

#[no_mangle]
pub static mut SHOULD_LOCATED_BSS_SECTION: [u32; 7] = [0; 7];

// $ arm-none-eabi-objdump -j .rodata -s target/thumbv7m-none-eabi/debug/bare-metal-blinky
//
// ```
// 8001440 69746820 6f766572 666c6f77 48454c4c  ith overflowHELL
// 8001450 4f20574f 524c4473 72632f6d 61696e2e  O WORLDsrc/main.
// ```
//
pub const SHOULD_LOCATED_IN_RODATA_SECTION: &str = "HELLO WORLD";

#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    loop {
        //
    }
}

#[no_mangle]
#[export_name = "main"]
pub extern "C" fn bare_main() -> ! {
    // note:
    // the following statements is used to prevent these test variables
    // from being removed by the optimizer
    //
    // - SHOULD_LOCATED_IN_RODATA_SECTION
    // - SHOULD_LOCATED_IN_DATA_SECTION
    // - SHOULD_LOCATED_BSS_SECTION
    //
    unsafe {
        // running check
        //
        // (gdb) i locals
        // i = 72
        // j = 1
        // k = 0
        let i = SHOULD_LOCATED_IN_RODATA_SECTION.as_bytes()[0];
        let j = SHOULD_LOCATED_IN_DATA_SECTION[0];
        let k = SHOULD_LOCATED_BSS_SECTION[0];
        let r = 0;
    }

    test_set_clock();
    test_blink(); // Test A
    test_button(); // Test B
    test_systick(); // Test C
    test_uart(); // Test D

    // !Only one of the test A,B,C,D can be selected at a time.

    loop {
        spin(1);
    }
}

fn test_blink() {
    // turn on GPIO clock
    gpio_clock_on(pin::BUILTIN_LED_PIN.port);

    // set builtin LED to output mode
    gpio_init(
        &pin::BUILTIN_LED_PIN,
        GPIO_MODE::GPIO_MODE_OUTPUT_50MHZ,
        GPIO_CNF::Output(GPIO_CNF_OUTPUT::GPIO_CNF_OUTPUT_PUSH_PULL),
        None,
    );

    loop {
        gpio_write(&pin::BUILTIN_LED_PIN, false); // set `false` to turn on builtin LED
        spin(50000);
        gpio_write(&pin::BUILTIN_LED_PIN, true);
        spin(50000);
    }
}

fn test_set_clock() {
    clock_init();
}

fn test_button() {
    // turn on GPIO clock
    gpio_clock_on(pin::EXTERNAL_LED_PIN.port);
    gpio_clock_on(pin::BUTTON_1_PIN.port);

    // set external LED pin to output mode
    gpio_init(
        &pin::EXTERNAL_LED_PIN,
        GPIO_MODE::GPIO_MODE_OUTPUT_50MHZ,
        GPIO_CNF::Output(GPIO_CNF_OUTPUT::GPIO_CNF_OUTPUT_PUSH_PULL),
        None,
    );

    // set button pin to input mode
    gpio_init(
        &pin::BUTTON_1_PIN,
        GPIO_MODE::GPIO_MODE_INPUT,
        GPIO_CNF::Input(GPIO_CNF_INPUT::GPIO_CNF_INPUT_PULL_UP_AND_PULL_DOWN),
        Some(true),
    );

    gpio_write(&pin::EXTERNAL_LED_PIN, true);

    loop {
        let value = gpio_read(&pin::BUTTON_1_PIN);
        // button value:
        // - false: pressed
        // - true: released
        //
        // set external LED value:
        // - false: turn off
        // - true: turn on
        gpio_write(&pin::EXTERNAL_LED_PIN, !value);
    }
}

fn test_systick() {
    // set tick every 1 ms
    systick_init_with_millisecond();

    // turn on GPIO clock
    gpio_clock_on(pin::BUILTIN_LED_PIN.port);

    // set builtin LED to output mode
    gpio_init(
        &pin::BUILTIN_LED_PIN,
        GPIO_MODE::GPIO_MODE_OUTPUT_50MHZ,
        GPIO_CNF::Output(GPIO_CNF_OUTPUT::GPIO_CNF_OUTPUT_PUSH_PULL),
        None,
    );

    loop {
        gpio_write(&pin::BUILTIN_LED_PIN, false); // set `false` to turn on builtin LED
        tick_delay(100);
        gpio_write(&pin::BUILTIN_LED_PIN, true);
        tick_delay(900);
    }
}

// connect the STM32F103 by a cp2102/ft232/ch340 USB dongle and run command:
// `$ picocom -b 115200 /dev/ttyUSB0`
fn test_uart() {
    // set tick every 1 ms
    systick_init_with_millisecond();

    // turn on GPIO clock
    gpio_clock_on(pin::EXTERNAL_LED_PIN.port);

    // set external LED to output mode
    gpio_init(
        &pin::EXTERNAL_LED_PIN,
        GPIO_MODE::GPIO_MODE_OUTPUT_50MHZ,
        GPIO_CNF::Output(GPIO_CNF_OUTPUT::GPIO_CNF_OUTPUT_PUSH_PULL),
        None,
    );

    // set UART baud rate to 115200.
    let baudrate = 115200;
    uart1_init(baudrate);

    let usart1_register = unsafe { &mut *get_usart1_register() };

    loop {
        // toggle builtin LED
        gpio_write(&pin::EXTERNAL_LED_PIN, true); // set `true` to turn on external LED
        uart_write_str(usart1_register, "LED on\r\n");
        tick_delay(100);

        gpio_write(&pin::EXTERNAL_LED_PIN, false);
        uart_write_str(usart1_register, "LED off\r\n");
        tick_delay(400);
    }
}

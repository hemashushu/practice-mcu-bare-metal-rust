// Copyright (c) 2022 Hemashushu <hippospark@gmail.com>, All rights reserved.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

//! Documents:
//!
//! - PM0214
//!   Programming manual
//!   STM32 Cortex®-M4 MCUs and MPUs programming manual
//!
//! - STM32F103c8
//!   Datasheet
//!
//! - RM0008
//!   Reference manual
//!   STM32F101xx, STM32F102xx, STM32F103xx, STM32F105xx and
//!   STM32F107xx advanced Arm®-based 32-bit MCUs

#![no_std]
#![no_main]

#[macro_use]
mod types;
mod vector;

use core::{arch::asm, panic::PanicInfo};

use types::{
    getGPIORegister, getRCCRegister, getSysTickRegister, Port, GPIO_CNF_Output, GPIO_Mode,
    PeripheralBus2Enable, Pin, SysTickCtrl, GPIO_CNF, SYS_CLOCK_FREQUENCY,
};

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

    let builtin_led_pin: Pin = Pin::new(Port::B, 12);
    let external_led_pin: Pin = Pin::new(Port::C, 13);
    let button_pin: Pin = Pin::new(Port::B, 9);

    // set builtin LED to output mode
    gpio_init(
        &builtin_led_pin,
        GPIO_Mode::GPIO_MODE_OUTPUT_50MHZ,
        GPIO_CNF::Output(GPIO_CNF_Output::GPIO_CNF_OUTPUT_PUSH_PULL),
    );

    // set external LED pin to output mode
    gpio_init(
        &external_led_pin,
        GPIO_Mode::GPIO_MODE_OUTPUT_50MHZ,
        GPIO_CNF::Output(GPIO_CNF_Output::GPIO_CNF_OUTPUT_PUSH_PULL),
    );

    // set button pin to input mode
    gpio_init(
        &button_pin,
        GPIO_Mode::GPIO_MODE_INPUT,
        GPIO_CNF::Input(types::GPIO_CNF_Input::GPIO_CNF_INPUT_FLOATING),
    );

    // tick every 1 ms
    systick_init(SYS_CLOCK_FREQUENCY / 1000);

    // loop {
    //     gpio_write(&builtin_led_pin, false);
    //     spin(999999);
    //     gpio_write(&builtin_led_pin, true);
    //     spin(999999);
    // }

    // for testing purpose only
    spin(1);

    let period = 500;   // 500 ms
    let mut expired_time = get_next_expired_time(unsafe { TOTAL_TICKS }, period);
    let mut builtin_led_state = false;

    loop {
        // toggle builtin LED
        if is_time_expired(expired_time, unsafe { TOTAL_TICKS }) {
            gpio_write(&builtin_led_pin, builtin_led_state);
            builtin_led_state = !builtin_led_state;
            expired_time = get_next_expired_time(unsafe { TOTAL_TICKS }, period);
        }

        // change external LED by the button status
        let state = gpio_read(&button_pin);
        gpio_write(&external_led_pin, !state);
    }
}

fn spin(count: u32) {
    (0..count).for_each(|_| unsafe { asm!("nop") });
}

fn is_high_pin(number: u8) -> bool {
    if number >= 8 {
        true
    } else {
        false
    }
}

fn get_next_expired_time(current_time: u64, period: u32) -> u64 {
    current_time + period as u64
}

fn is_time_expired(expired_time: u64, current_time: u64) -> bool {
    current_time > expired_time
}

fn gpio_init(pin: &Pin, gpio_mode: GPIO_Mode, gpio_cnf: GPIO_CNF) {
    gpio_set_mode(pin, gpio_mode, gpio_cnf)
}

fn gpio_set_mode(pin: &Pin, gpio_mode: GPIO_Mode, gpio_cnf: GPIO_CNF) {
    // RM0008 8.3.7 APB2 peripheral clock enable register (RCC_APB2ENR)
    let rcc_register = unsafe { &mut *getRCCRegister() };

    let pbus2_enable_value = match pin.port {
        Port::A => PeripheralBus2Enable::GPIO_BANK_A_ENABLE,
        Port::B => PeripheralBus2Enable::GPIO_BANK_B_ENABLE,
        Port::C => PeripheralBus2Enable::GPIO_BANK_C_ENABLE,
        Port::D => PeripheralBus2Enable::GPIO_BANK_D_ENABLE,
        Port::E => PeripheralBus2Enable::GPIO_BANK_E_ENABLE,
        _ => panic!(),
    };

    rcc_register.APB2ENR |= pbus2_enable_value as u32;

    let (crl_idx, relative_number) = match is_high_pin(pin.number) {
        true => (1, pin.number - 8),
        false => (0, pin.number),
    };

    let cnf_value = match gpio_cnf {
        GPIO_CNF::Input(i) => i as u8,
        GPIO_CNF::Output(o) => o as u8,
    };

    let mode_value = gpio_mode as u8;
    let value = (cnf_value << 2 | mode_value) & 0xf;

    let gpio_register = unsafe { &mut *getGPIORegister(pin.port) };

    // each pin takes up 4 bits
    // - bit 0..1 for mode
    // - bit 2..3 for cnf
    //
    // so left shift (n*4) bits by pin number
    gpio_register.CRL[crl_idx] &= !((0xf as u32) << (relative_number * 4));
    gpio_register.CRL[crl_idx] |= (value as u32) << (relative_number * 4);
}

fn gpio_write(pin: &Pin, val: bool) {
    let gpio_register = unsafe { &mut *getGPIORegister(pin.port) };

    // RM0008 9.2.5 Port bit set/reset register (GPIOx_BSRR)
    if val {
        // `set` means `set output to 1`
        gpio_register.BSRR |= bit_to_u32!(pin.number as u32)
    } else {
        // `reset` means `set output to 0`
        gpio_register.BSRR |= bit_to_u32!(pin.number as u32) << 16;
    }
}

fn gpio_read(pin: &Pin) -> bool {
    let gpio_register = unsafe { &mut *getGPIORegister(pin.port) };

    // RM0008 9.2.3 Port input data register (GPIOx_IDR)
    gpio_register.IDR & bit_to_u32!(pin.number as u32) != 0
}

fn systick_init(ticks: u32) {
    if (ticks - 1) > 0xffffff {
        // systick timer is 24 bit
        panic!();
    }

    // PM0214 4.5.1 SysTick control and status register (STK_CTRL)
    let systick_register = unsafe { &mut *getSysTickRegister() };

    systick_register.CTRL =
        SysTickCtrl::ENABLE as u32 | SysTickCtrl::TICKINT as u32 | SysTickCtrl::CLKSOURCE as u32;

    systick_register.LOAD = ticks - 1; // the ending number
    systick_register.VAL = 0; // the start number

    // RM0008 8.3.7 APB2 peripheral clock enable register (RCC_APB2ENR)
    let rcc_register = unsafe { &mut *getRCCRegister() };

    rcc_register.APB2ENR |= PeripheralBus2Enable::TIMER1_ENABLE as u32;
}

// the current tick value (unit: ms)
pub static mut TOTAL_TICKS: u64 = 0;

// this function will be triggered (executed) every millisecond
#[no_mangle]
pub extern "C" fn SysTick() {
    unsafe {
        TOTAL_TICKS += 1;
    }
}

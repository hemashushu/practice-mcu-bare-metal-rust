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

use core::{
    arch::asm,
    fmt::{self, Write},
    panic::PanicInfo,
};

use types::{
    getGPIORegister, get_bit_range_value, get_flash_register, get_rcc_register,
    get_systick_register, get_usart1_register, AHBPrescTable, Pin, Port, USART_Register, FLASH_ACR,
    FLASH_ACR_LATENCY, GPIO_CNF, GPIO_CNF_OUTPUT, GPIO_MODE, HSE_VALUE, HSI_VALUE,
    RCC_APB2_CLOCK_ENABLE, RCC_CFGR, RCC_CFGR_PLLMUL, RCC_CFGR_PLLSRC, RCC_CFGR_PLLXTPRE,
    RCC_CFGR_PPRE1, RCC_CFGR_SW, RCC_CFGR_SWS, RCC_CR, RCC_CR_HSEON, RCC_CR_PLLON, SYS_TICK_CTRL,
    USART_CR1, USART_SR,
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

    // set clock
    // init_clock();

    // get the current system clock value
    let clock = get_system_core_clock();
    update_system_core_clock(clock);

    // for testing purpose only
    spin(1);

    let builtin_led_pin: Pin = Pin::new(Port::B, 12);
    let external_led_pin: Pin = Pin::new(Port::C, 13);
    let button_pin: Pin = Pin::new(Port::B, 9);

    // set builtin LED to output mode
    gpio_init(
        &builtin_led_pin,
        GPIO_MODE::GPIO_MODE_OUTPUT_50MHZ,
        GPIO_CNF::Output(GPIO_CNF_OUTPUT::GPIO_CNF_OUTPUT_PUSH_PULL),
    );

    // set external LED pin to output mode
    gpio_init(
        &external_led_pin,
        GPIO_MODE::GPIO_MODE_OUTPUT_50MHZ,
        GPIO_CNF::Output(GPIO_CNF_OUTPUT::GPIO_CNF_OUTPUT_PUSH_PULL),
    );

    // set button pin to input mode
    gpio_init(
        &button_pin,
        GPIO_MODE::GPIO_MODE_INPUT,
        GPIO_CNF::Input(types::GPIO_CNF_INPUT::GPIO_CNF_INPUT_FLOATING),
    );

    // tick every 1 ms
    systick_init(unsafe { SystemCoreClock } / 1000);

    // loop {
    //     gpio_write(&builtin_led_pin, false);
    //     spin(99999);
    //     gpio_write(&builtin_led_pin, true);
    //     spin(99999);
    // }

    // loop {
    //     gpio_write(&builtin_led_pin, false);
    //     delayMilliseconds(500);
    //     gpio_write(&builtin_led_pin, true);
    //     delayMilliseconds(500);
    // }

    let baudrate = 9600;
    init_uart1(baudrate);

    let usart1_register = unsafe { &mut *get_usart1_register() };

    let period = 500; // 500 ms
    let mut expired_time = get_next_expired_time(get_current_time(), period);
    let mut builtin_led_state = false;

    loop {
        // toggle builtin LED
        if is_time_expired(expired_time, get_current_time()) {
            builtin_led_state = !builtin_led_state;
            expired_time = get_next_expired_time(get_current_time(), period);

            gpio_write(&builtin_led_pin, builtin_led_state);

            if builtin_led_state {
                uart_write_str(usart1_register, "on\r\n");
            } else {
                uart_write_str(usart1_register, "off\r\n");
            }
        }

        // change external LED by the button status
        let state = gpio_read(&button_pin);
        gpio_write(&external_led_pin, !state);

        if !state {
            uart_write_str(usart1_register, "press\r\n");
        }

        // check user input
        let input_char = {
            if uart_read_ready(usart1_register) {
                Some(uart_read_byte(usart1_register))
            } else {
                None
            }
        };

        if let Some(b) = input_char {
            uart_write_str(usart1_register, "input is ");
            uart_write_byte(usart1_register, b);
            uart_write_str(usart1_register, "\r\n");
        }
    }
}

fn spin(count: u32) {
    (0..count).for_each(|_| spin_one());
}

#[inline]
fn spin_one() {
    unsafe { asm!("nop") }
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

fn delayMilliseconds(milliseconds: u32) {
    let expired_time = get_next_expired_time(get_current_time(), milliseconds);
    while !is_time_expired(expired_time, get_current_time()) {
        spin_one();
    }
}

fn gpio_init(pin: &Pin, gpio_mode: GPIO_MODE, gpio_cnf: GPIO_CNF) {
    gpio_set_mode(pin, gpio_mode, gpio_cnf)
}

fn gpio_set_mode(pin: &Pin, gpio_mode: GPIO_MODE, gpio_cnf: GPIO_CNF) {
    // RM0008 8.3.7 APB2 peripheral clock enable register (RCC_APB2ENR)
    let rcc_register = unsafe { &mut *get_rcc_register() };

    let pbus2_enable_value = match pin.port {
        Port::A => RCC_APB2_CLOCK_ENABLE::GPIO_PORT_A_ENABLE,
        Port::B => RCC_APB2_CLOCK_ENABLE::GPIO_PORT_B_ENABLE,
        Port::C => RCC_APB2_CLOCK_ENABLE::GPIO_PORT_C_ENABLE,
        Port::D => RCC_APB2_CLOCK_ENABLE::GPIO_PORT_D_ENABLE,
        Port::E => RCC_APB2_CLOCK_ENABLE::GPIO_PORT_E_ENABLE,
        Port::F => RCC_APB2_CLOCK_ENABLE::GPIO_PORT_F_ENABLE,
        Port::G => RCC_APB2_CLOCK_ENABLE::GPIO_PORT_G_ENABLE,
    };

    // enable GPIOx clock
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
    gpio_register.CR[crl_idx] &= !((0xf as u32) << (relative_number * 4));
    gpio_register.CR[crl_idx] |= (value as u32) << (relative_number * 4);
}

fn gpio_write(pin: &Pin, val: bool) {
    let gpio_register = unsafe { &mut *getGPIORegister(pin.port) };

    // RM0008 9.2.5 Port bit set/reset register (GPIOx_BSRR)
    if val {
        // set the `set` bit
        // `set` means `set output to 1`
        gpio_register.BSRR |= bit_to_u32!(pin.number as u32)
    } else {
        // set the `reset` bit
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
    let systick_register = unsafe { &mut *get_systick_register() };

    systick_register.LOAD = ticks - 1; // the ending number
    systick_register.VAL = 0; // the start number

    systick_register.CTRL = SYS_TICK_CTRL::ENABLE as u32
        | SYS_TICK_CTRL::TICKINT as u32
        | SYS_TICK_CTRL::CLKSOURCE as u32;

    // RM0008 8.3.7 APB2 peripheral clock enable register (RCC_APB2ENR)
    // let rcc_register = unsafe { &mut *getRCCRegister() };
    //
    // enable timer clock
    // rcc_register.APB2ENR |= PERIPHERAL_BUS_2_CLOCK_ENABLE::TIMER1_ENABLE as u32;
}

// the current tick value (unit: ms)
pub static mut TOTAL_TICKS: u64 = 0;

pub fn get_current_time() -> u64 {
    let ticks = unsafe { TOTAL_TICKS };
    ticks
}

// this function will be triggered (executed) every millisecond
#[no_mangle]
pub extern "C" fn SysTick() {
    unsafe {
        TOTAL_TICKS += 1;
    }
}

// System Clock Frequency (Core Clock)
pub static mut SystemCoreClock: u32 = HSI_VALUE;

fn update_system_core_clock(clock: u32) {
    // update variable `SystemCoreClock`
    unsafe {
        SystemCoreClock = clock;
    }
}

fn get_system_core_clock() -> u32 {
    let mut clock: u32 = 0;

    // Get SYSCLK source
    let rcc_register = unsafe { &mut *get_rcc_register() };
    let sws = rcc_register.CFGR & RCC_CFGR::SWS as u32;

    if sws == RCC_CFGR_SWS::HSI as u32 {
        // HSI used as system clock
        clock = HSI_VALUE;
    } else if sws == RCC_CFGR_SWS::HSE as u32 {
        // HSE used as system clock
        clock = HSE_VALUE;
    } else if sws == RCC_CFGR_SWS::PLL as u32 {
        // PLL used as system clock

        // Get PLL clock source and multiplication factor
        let pllmul = get_bit_range_value(rcc_register.CFGR, RCC_CFGR::PLLMUL as u32);
        let pllsource = get_bit_range_value(rcc_register.CFGR, RCC_CFGR::PLLSRC as u32);

        let pllmul_number = pllmul + 2;

        if pllsource == RCC_CFGR_PLLSRC::HSI as u32 {
            // HSI oscillator clock divided by 2 selected as PLL clock entry
            clock = HSI_VALUE / 2 * pllmul_number;
        } else {
            // HSE selected as PLL clock entry
            if (rcc_register.CFGR & RCC_CFGR::PLLXTPRE as u32) != RCC_CFGR_PLLXTPRE::RESET as u32 {
                // HSE oscillator clock divided by 2
                clock = HSE_VALUE / 2 * pllmul_number;
            } else {
                // HSE oscillator clock NOT divided by 2
                clock = HSE_VALUE * pllmul_number;
            }
        }
    } else {
        clock = HSI_VALUE;
    }

    // Compute HCLK clock frequency

    // Get HCLK prescaler
    let hpre = get_bit_range_value(rcc_register.CFGR, RCC_CFGR::HPRE as u32);
    let prescaler = AHBPrescTable[hpre as usize];

    // HCLK clock frequency
    clock >> prescaler
}

// set system clock to 72MHz
fn init_clock() {
    let flash_register = unsafe { &mut *get_flash_register() };

    // 1. set flash latency to `2 wait states`
    flash_register.ACR &= !(FLASH_ACR::LATENCY as u32);
    flash_register.ACR |= FLASH_ACR_LATENCY::Two as u32;

    let rcc_register = unsafe { &mut *get_rcc_register() };

    // 2. set APB low-speed prescaler to `HCLK / 2`
    rcc_register.CFGR &= !(RCC_CFGR::PPRE1 as u32);
    rcc_register.CFGR |= RCC_CFGR_PPRE1::DIVIDED_2 as u32;

    // 3. enable HSE clock
    rcc_register.CR |= RCC_CR_HSEON::ON as u32;

    // 4. wait for the HSE READY flag
    while (rcc_register.CR & RCC_CR::HSERDY as u32) == 0 {
        // note:
        // here need to write some statements, or rustc may ignore the condition and
        // optimize into an empty loop (infinite loop)
        spin_one();
    }

    // 5. set PLL source to HSE
    rcc_register.CFGR &= !(RCC_CFGR::PLLSRC as u32);
    rcc_register.CFGR |= RCC_CFGR_PLLSRC::HSE as u32;

    // 6. multiply by 9
    rcc_register.CFGR &= !(RCC_CFGR::PLLMUL as u32);
    rcc_register.CFGR |= RCC_CFGR_PLLMUL::MULX9 as u32;

    // 7. enable the PLL
    rcc_register.CR |= RCC_CR_PLLON::ON as u32;

    // 8. wait for the PLL READY flag
    while (rcc_register.CR & RCC_CR::PLLRDY as u32) == 0 {
        spin_one();
    }

    // 9. set clock source to pll
    rcc_register.CFGR &= !(RCC_CFGR::SW as u32);
    rcc_register.CFGR |= RCC_CFGR_SW::PLL as u32;

    // 10. wait for PLL as source
    while (rcc_register.CFGR & RCC_CFGR_SWS::PLL as u32) == 0 {
        spin_one();
    }

    // note:
    // It is necessary to update the `SystemCoreClock` variable
}

fn init_uart1(baudrate: u32) {
    // STM32F103 datasheet Table 5. Medium-density STM32F103xx pin definitions
    // Pinouts
    // - PA9: USART1_TX
    // - PA10: USART1_RX
    //
    // note:
    // The connection wires between the MCU and USB-TTL(CP2102, CH340 etc.) sometimes
    // need to have the TX and RX crossed, e.g.
    // MCU          CP210x/CH340
    // TX  <-------> RX
    // RX  <-------> TX
    // GND <-------> GND
    // 3.3 <-------> 3.3V

    // to check out the UART output, run the following command in host:
    // `$ picocom -b 115200 /dev/ttyUSB1`
    // the USB-UART device path may be `/dev/ttyUSB0` or other else.
    // press `Ctrl+a, Ctrl+x` to exit `picocom`.

    let tx_pin: Pin = Pin::new(Port::A, 9);
    let rx_pin: Pin = Pin::new(Port::A, 10);

    // TX (PA9) pin is configured as 50MHz output, push-pull and alternate function.
    gpio_set_mode(
        &tx_pin,
        GPIO_MODE::GPIO_MODE_OUTPUT_10MHZ,
        GPIO_CNF::Output(GPIO_CNF_OUTPUT::GPIO_CNF_OUTPUT_AF_PUSH_PULL),
    );
    // RX (PA10) pin is configured input mode and floating.
    gpio_set_mode(
        &rx_pin,
        GPIO_MODE::GPIO_MODE_INPUT,
        GPIO_CNF::Input(types::GPIO_CNF_INPUT::GPIO_CNF_INPUT_FLOATING),
    );

    // enable USART1 clock
    let rcc_register = unsafe { &mut *get_rcc_register() };
    rcc_register.APB2ENR |= RCC_APB2_CLOCK_ENABLE::USART1_ENABLE as u32;

    // configure USART1 registers
    let clock = get_system_core_clock();
    let baud = clock / baudrate;
    let usart1_register = unsafe { &mut *get_usart1_register() };

    // usart1_register.CR1 = 0;
    usart1_register.BRR = baud;
    usart1_register.CR1 = USART_CR1::TE as u32 | USART_CR1::RE as u32 | USART_CR1::UE as u32;
}

fn uart_write_byte(usart_register: &mut USART_Register, byte: u8) {
    usart_register.DR = byte as u32;

    // 19 Universal synchronous asynchronous receiver transmitter (USART)
    // Figure 167. USART block diagram
    //
    // waiting for the data transfer from `transmit data register (TDR)` to
    // `transmit shift register`
    while (usart_register.SR & USART_SR::TXE as u32) == 0 {
        spin(1);
    }
}

fn uart_write_buf(usart_register: &mut USART_Register, buf: &[u8], len: usize) {
    (0..len).for_each(|idx| uart_write_byte(usart_register, buf[idx]));
}

fn uart_write_str(usart_register: &mut USART_Register, s: &str) {
    let bytes = s.as_bytes();
    uart_write_buf(usart_register, bytes, bytes.len());
}

fn uart_read_ready(usart_register: &USART_Register) -> bool {
    // If RXNE bit is set, data is ready
    (usart_register.SR & USART_SR::RXNE as u32) != 0
}

fn uart_read_byte(usart_register: &USART_Register) -> u8 {
    (usart_register.DR & 255) as u8
}

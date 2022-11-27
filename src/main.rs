// Copyright (c) 2022 Hemashushu <hippospark@gmail.com>, All rights reserved.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

//! Reference Documents:
//!
//! - PM0214 Programming manual
//!   STM32 Cortex®-M4 MCUs and MPUs programming manual
//!   https://www.st.com/resource/en/programming_manual/pm0214-stm32-cortexm4-mcus-and-mpus-programming-manual-stmicroelectronics.pdf
//!
//! - STM32F103c8 Datasheet
//!   https://www.st.com/resource/en/datasheet/cd00161566.pdf
//!
//! - RM0008 Reference manual
//!   STM32F101xx, STM32F102xx, STM32F103xx, STM32F105xx and
//!   STM32F107xx advanced Arm®-based 32-bit MCUs
//!   https://www.st.com/resource/en/reference_manual/cd00171190-stm32f101xx-stm32f102xx-stm32f103xx-stm32f105xx-and-stm32f107xx-advanced-arm-based-32-bit-mcus-stmicroelectronics.pdf

#![no_std]
#![no_main]

#[macro_use]
mod type_common;
mod type_gpio;
mod type_rcc;
mod type_systick;
mod type_usart;
mod vector;

use core::{arch::asm, panic::PanicInfo};

use type_common::{Pin, Port};
use type_gpio::{getGPIORegister, GPIO_CNF, GPIO_CNF_INPUT, GPIO_CNF_OUTPUT, GPIO_MODE};
use type_rcc::{get_rcc_register, HSI_VALUE, RCC_APB2_CLOCK_ENABLE};
use type_systick::{get_systick_register, SYS_TICK_CTRL};
use type_usart::{get_usart1_register, USART_Register, USART_CR1, USART_SR};

// the current tick value (unit: ms)
pub static mut TOTAL_TICKS: u64 = 0;

// System Clock Frequency (Core Clock)
pub static mut SystemCoreClock: u32 = HSI_VALUE;

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

    // for testing purpose only
    spin(1);

    let builtin_led_pin: Pin = Pin::new(Port::B, 2);
    let external_led_pin: Pin = Pin::new(Port::B, 13);
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
        GPIO_CNF::Input(GPIO_CNF_INPUT::GPIO_CNF_INPUT_FLOATING),
    );

    // tick every 1 ms
    systick_init(get_system_core_clock() / 1000);

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

    // set UART baud rate to 9600.
    // connect the STM32F103 by a cp2102 USB dongle and run command:
    // `$ picocom -b 9600 /dev/ttyUSB0`
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

        // print button status
        if !state {
            uart_write_str(usart1_register, "press\r\n");
        }

        // check the user input char
        let input_char = {
            if uart_read_ready(usart1_register) {
                Some(uart_read_byte(usart1_register))
            } else {
                None
            }
        };

        // print the user input char
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

fn gpio_init(pin: &Pin, gpio_mode: GPIO_MODE, gpio_config: GPIO_CNF) {
    gpio_set_mode(pin, gpio_mode, gpio_config)
}

fn gpio_set_mode(pin: &Pin, gpio_mode: GPIO_MODE, gpio_config: GPIO_CNF) {
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

    let cnf_value = match gpio_config {
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

fn get_system_core_clock() -> u32 {
    unsafe { SystemCoreClock }
}

fn init_uart1(baudrate: u32) {
    // STM32F103 datasheet Table 5. Medium-density STM32F103xx pin definitions
    // Pinouts
    // - PA9: USART1_TX
    // - PA10: USART1_RX
    //
    // note:
    // The connection wires between the MCU and some USB-TTL dongle (CP2102, FT232, CH340 etc.)
    // need to have the TX and RX crossed, e.g.
    //
    // MCU          CP210x/FT232/CH340 Dongle
    // ---          -------------------
    // TX  <-------> RX  (or TX)
    // RX  <-------> TX  (or RX)
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
        GPIO_CNF::Input(GPIO_CNF_INPUT::GPIO_CNF_INPUT_FLOATING),
    );

    // enable USART1 clock
    let rcc_register = unsafe { &mut *get_rcc_register() };
    rcc_register.APB2ENR |= RCC_APB2_CLOCK_ENABLE::USART1_ENABLE as u32;

    // configure USART1 registers
    let clock = get_system_core_clock();
    let baud = clock / baudrate;
    let usart1_register = unsafe { &mut *get_usart1_register() };

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

fn init_clock() {
    todo!()

    // F103 clock tree
    //
    //                                  Mux
    // 1. HSI (8MHz) ------------------->T--> SYSCLK --> AHB prescaler (/1, /2, /4, /8 or /16)
    //                                   |                 |--> HCLK, max: 72MHz
    // 2. HSE (8MHz) ------------------->|                        |--> AHB bus, core, memory, DMA
    //                                   |                        |--> (/1 /8) sys timer (systick)
    //              /------------------->/                        |--> FCLK (free-running clock)
    //              |                                             |--> APB1 prescaler (/1, /2, /4, /8 or /16), max: 36HMz
    // 3. PLL --> PLL Mul (x1 .. x16)                             |      |--> PCLK1 --> APB1 peripheral clock
    //              |                                             |      |--> <x2>  --> APB1 timer clock
    //              \--> USB prescaler </1> --> USB               |
    //                                                            |--> APB2 prescaler (/1, /2, /4, /8 or /16), max: 72HMz
    //                                                                   |--> PCLK2 --> APB2 peripheral clock
    // note:                                                             |--> <x1>  --> APB2 timer clock
    // - PLL source: HSI </2> or HSE (/1 or /2)                            |--> ADC prescaler </2> --> ADC 1,2
    //
    //
    // Config example
    //
    // Initializes the RCC Oscillators
    // RCC_OscInitStruct
    // - OscillatorType = RCC_OSCILLATORTYPE_HSE;
    // - HSEState = RCC_HSE_ON;
    // - HSEPredivValue = RCC_HSE_PREDIV_DIV1;
    // - HSIState = RCC_HSI_ON;
    // - PLL.PLLState = RCC_PLL_ON;
    // - PLL.PLLSource = RCC_PLLSOURCE_HSE;
    // - PLL.PLLMUL = RCC_PLL_MUL9;
    //
    // Initializes the CPU, AHB and APB buses clocks
    // RCC_ClkInitStruct
    // - ClockType = RCC_CLOCKTYPE_HCLK
    //              |RCC_CLOCKTYPE_SYSCLK
    //              |RCC_CLOCKTYPE_PCLK1
    //              |RCC_CLOCKTYPE_PCLK2;
    // - SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    // - AHBCLKDivider = RCC_SYSCLK_DIV1;
    // - APB1CLKDivider = RCC_HCLK_DIV2;
    // - APB2CLKDivider = RCC_HCLK_DIV1;
    // - FLASH_LATENCY_2


    // RCC Set System Clock PLL at 72MHz from HSE at 8MHz
    //
    // source from:
    // libopencm3/lib/stm32/f1/rcc.c
    // function:
    // rcc_clock_setup_in_hse_8mhz_out_72mhz


// TODO:: convert C code to Rust code

    //     /* Enable internal high-speed oscillator. */
    // 	rcc_osc_on(RCC_HSI);
    // 	rcc_wait_for_osc_ready(RCC_HSI);
    //
    // 	/* Select HSI as SYSCLK source. */
    // 	rcc_set_sysclk_source(RCC_CFGR_SW_SYSCLKSEL_HSICLK);
    //
    // 	/* Enable external high-speed oscillator 8MHz. */
    // 	rcc_osc_on(RCC_HSE);
    // 	rcc_wait_for_osc_ready(RCC_HSE);
    // 	rcc_set_sysclk_source(RCC_CFGR_SW_SYSCLKSEL_HSECLK);
    //
    // 	/*
    // 	 * Set prescalers for AHB, ADC, ABP1, ABP2.
    // 	 * Do this before touching the PLL (TODO: why?).
    // 	 */
    // 	rcc_set_hpre(RCC_CFGR_HPRE_SYSCLK_NODIV);    /* Set. 72MHz Max. 72MHz */
    // 	rcc_set_ppre1(RCC_CFGR_PPRE1_HCLK_DIV2);     /* Set. 36MHz Max. 36MHz */
    // 	rcc_set_ppre2(RCC_CFGR_PPRE2_HCLK_NODIV);    /* Set. 72MHz Max. 72MHz */
    // 	rcc_set_adcpre(RCC_CFGR_ADCPRE_PCLK2_DIV8);  /* Set.  9MHz Max. 14MHz */
    //
    // 	/*
    // 	 * Sysclk runs with 72MHz -> 2 waitstates.
    // 	 * 0WS from 0-24MHz
    // 	 * 1WS from 24-48MHz
    // 	 * 2WS from 48-72MHz
    // 	 */
    // 	flash_set_ws(FLASH_ACR_LATENCY_2WS);
    //
    // 	/*
    // 	 * Set the PLL multiplication factor to 9.
    // 	 * 8MHz (external) * 9 (multiplier) = 72MHz
    // 	 */
    // 	rcc_set_pll_multiplication_factor(RCC_CFGR_PLLMUL_PLL_CLK_MUL9);
    //
    // 	/* Select HSE as PLL source. */
    // 	rcc_set_pll_source(RCC_CFGR_PLLSRC_HSE_CLK);
    //
    // 	/*
    // 	 * External frequency undivided before entering PLL
    // 	 * (only valid/needed for HSE).
    // 	 */
    // 	rcc_set_pllxtpre(RCC_CFGR_PLLXTPRE_HSE_CLK);
    //
    // 	/* Enable PLL oscillator and wait for it to stabilize. */
    // 	rcc_osc_on(RCC_PLL);
    // 	rcc_wait_for_osc_ready(RCC_PLL);
    //
    // 	/* Select PLL as SYSCLK source. */
    // 	rcc_set_sysclk_source(RCC_CFGR_SW_SYSCLKSEL_PLLCLK);
    //
    // 	/* Set the peripheral clock frequencies used */
    // 	rcc_ahb_frequency = 72000000;
    // 	rcc_apb1_frequency = 36000000;
    // 	rcc_apb2_frequency = 72000000;
    //
    //     // note:
    //     // There is a full-speed USB module in the STM32, and its serial interface engine requires a
    //     // clock source with a frequency of 48MHz. The clock source can only be obtained from the PLL output.
    //     // when a USB module is required, the PLL must be enabled, and the clock frequency
    //     // is configured to 48MHz or 72MHz.
}
// Copyright (c) 2022 Hemashushu <hippospark@gmail.com>, All rights reserved.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

use crate::{
    common::{Pin, Port, ADCCLK, HCLK, PCLK1, PCLK2, SYSCLK, SYSTICKS},
    register_flash::{get_flash_register, FLASH_ACR_LATENCY, FLASH_ACR_LATENCY_MASK},
    register_gpio::{get_GPIO_register, GPIO_CNF, GPIO_CNF_INPUT, GPIO_CNF_OUTPUT, GPIO_MODE},
    register_rcc::{
        get_rcc_register, RCC_APB2_CLOCK_ENABLE, RCC_CFGR_ADCPRE, RCC_CFGR_ADCPRE_MASK,
        RCC_CFGR_HPRE, RCC_CFGR_HPRE_MASK, RCC_CFGR_PLLMUL, RCC_CFGR_PLLMUL_MASK,
        RCC_CFGR_PLLSRC_HSE, RCC_CFGR_PPRE1, RCC_CFGR_PPRE1_MASK, RCC_CFGR_PPRE2,
        RCC_CFGR_PPRE2_MASK, RCC_CFGR_SW, RCC_CFGR_SWS, RCC_CR_HSEON, RCC_CR_HSERDY, RCC_CR_PLLON,
        RCC_CR_PLLRDY,
    },
    register_systick::{get_systick_register, SYS_TICK_CTRL},
    register_usart::{get_usart1_register, USART_CR1, USART_Register, USART_SR}, utils::spin,
};

pub fn gpio_clock_on(port: Port) {
    // RM0008 8.3.7 APB2 peripheral clock enable register (RCC_APB2ENR)
    let rcc_register = unsafe { &mut *get_rcc_register() };

    let pbus2_enable_value = match port {
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
}

pub fn gpio_init(
    pin: &Pin,
    gpio_mode: GPIO_MODE,
    gpio_config: GPIO_CNF,
    option_input_pull_up_down: Option<bool>,
) {
    let cr_idx = pin.number / 8;
    let relative_pin_number = pin.number % 8;

    let cnf_value = match gpio_config {
        GPIO_CNF::Input(i) => i as u8,
        GPIO_CNF::Output(o) => o as u8,
    };

    const MODE_AND_CNF_VALUE_MASK: u8 = 0xf;

    let mode_value = gpio_mode as u8;
    let value = (cnf_value << 2 | mode_value) & MODE_AND_CNF_VALUE_MASK;

    let gpio_register = unsafe { &mut *get_GPIO_register(pin.port) };

    // each pin takes up 4 bits
    // - bit 0..1 for mode
    // - bit 2..3 for cnf
    //
    // so left shift (n*4) bits by pin number
    gpio_register.CR[cr_idx as usize] &=
        !((MODE_AND_CNF_VALUE_MASK as u32) << (relative_pin_number * 4));
    gpio_register.CR[cr_idx as usize] |= (value as u32) << (relative_pin_number * 4);

    // RM0008 Table 20. Port bit configuration table
    if let Some(input_pull_up_down) = option_input_pull_up_down {
        if input_pull_up_down {
            // pull up
            gpio_register.ODR |= bit_to_u32!(pin.number);
        } else {
            // pull down
            gpio_register.ODR &= !bit_to_u32!(pin.number);
        }
    }
}

pub fn gpio_write(pin: &Pin, val: bool) {
    let gpio_register = unsafe { &mut *get_GPIO_register(pin.port) };

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

pub fn gpio_read(pin: &Pin) -> bool {
    let gpio_register = unsafe { &mut *get_GPIO_register(pin.port) };

    // RM0008 9.2.3 Port input data register (GPIOx_IDR)
    gpio_register.IDR & bit_to_u32!(pin.number as u32) != 0
}

pub fn clock_init() {
    // RM0008 8.2 Clocks, Figure 11. Clock tree
    //
    // clock tree
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

    // RCC Set System Clock PLL at 72MHz from HSE at 8MHz
    //
    // reference `libopencm3/lib/stm32/f1/rcc.c` function `rcc_clock_setup_in_hse_8mhz_out_72mhz`

    // enable external high-speed oscillator 8MHz.
    let rcc_register = unsafe { &mut *get_rcc_register() };
    rcc_register.CR |= RCC_CR_HSEON;
    while (rcc_register.CR & RCC_CR_HSERDY) == 0 {
        //
    }

    // set flash latency
    // Sysclk runs with 72MHz -> 2 waitstates.
    // 0WS from 0-24MHz
    // 1WS from 24-48MHz
    // 2WS from 48-72MHz

    let flash_register = unsafe { &mut *get_flash_register() };
    flash_register.ACR &= !FLASH_ACR_LATENCY_MASK;
    flash_register.ACR |= FLASH_ACR_LATENCY::Two as u32;

    // RM0008 8.3.12
    // Clock configuration register2 (RCC_CFGR2)
    // rcc_register.CFGR2 &= !RCC_CFGR2_PREDIV1_MASK; // set PREDIV1[3:0] = 0b0000
    // rcc_register.CFGR2 &= !RCC_CFGR2_PREDIV1SRC_MASK; // set PREDIV1SRC = 0 (HSE)

    // set PLL source
    rcc_register.CFGR |= RCC_CFGR_PLLSRC_HSE;

    // rcc_register.CFGR &= !RCC_CFGR_PLLXTPRE_MASK;
    rcc_register.CFGR &= !RCC_CFGR_PLLMUL_MASK; // clean PLLMUL[3:0]
    rcc_register.CFGR |= RCC_CFGR_PLLMUL::MULX9 as u32;

    // turn on PLL
    rcc_register.CR |= RCC_CR_PLLON;
    while (rcc_register.CR & RCC_CR_PLLRDY) == 0 {
        //
    }

    // set prescalers for AHB, ADC, ABP1, ABP2.

    rcc_register.CFGR &= !RCC_CFGR_HPRE_MASK; // clear bits 4:4
    rcc_register.CFGR |= RCC_CFGR_HPRE::NOT_DIVIDED as u32; /* Set. 72MHz Max. 72MHz */

    rcc_register.CFGR &= !RCC_CFGR_PPRE1_MASK; // clear bits 8:3
    rcc_register.CFGR |= RCC_CFGR_PPRE1::DIVIDED_2 as u32; /* Set. 36MHz Max. 36MHz */

    rcc_register.CFGR &= !RCC_CFGR_PPRE2_MASK; // clear bits 11:3
    rcc_register.CFGR |= RCC_CFGR_PPRE2::NOT_DIVIDED as u32; /* Set. 72MHz Max. 72MHz */

    rcc_register.CFGR &= !RCC_CFGR_ADCPRE_MASK; // clear bits 14:2
    rcc_register.CFGR |= RCC_CFGR_ADCPRE::DIVIDED_8 as u32; /* Set.  9MHz Max. 14MHz */

    // select PLL as SYSCLK source.
    rcc_register.CFGR |= RCC_CFGR_SW::PLL as u32;
    while (rcc_register.CFGR & RCC_CFGR_SWS::PLL as u32) != RCC_CFGR_SWS::PLL as u32 {
        //
    }

    // update the peripheral clock frequencies used
    unsafe {
        SYSCLK = 72_000_000;
        HCLK = 72_000_000;
        PCLK1 = 36_000_000;
        PCLK2 = 72_000_000;
        ADCCLK = 9_000_000;
    }

    // note:
    // There is a full-speed USB module in the STM32, and its serial interface engine requires a
    // clock source with a frequency of 48MHz. The clock source can only be obtained from the PLL output.
    // when a USB module is required, the PLL must be enabled, and the clock frequency
    // is configured to 48MHz or 72MHz.
}

pub fn systick_init(ticks: u32) {
    // ticks:
    // how many ticks to trigger the `SysTick` interrupt,

    if (ticks - 1) > 0xffffff {
        // systick timer is 24 bit
        panic!();
    }

    // PM0214 4.5.1 SysTick control and status register (STK_CTRL)
    let systick_register = unsafe { &mut *get_systick_register() };

    systick_register.LOAD = ticks - 1; // the ending number
    systick_register.VAL = 0; // the start number

    // PM0214 4.5.1 SysTick control and status register (STK_CTRL)
    //
    // - `SYS_TICK_CTRL::CLKSOURCE` indicates use HCLK as clock source.
    //   RM0008 7.2 Clocks
    //   The RCC feeds the System Timer (SysTick) external clock with the AHB clock
    //   (HCLK) divided by 8.
    //   The SysTick can work either with this clock or with the clock
    //   (HCLK), configurable in the SysTick Control and Status register.
    // - `SYS_TICK_CTRL::TICKINT` indicates trigger SysTick exception.

    systick_register.CTRL = SYS_TICK_CTRL::ENABLE as u32
        | SYS_TICK_CTRL::TICKINT as u32
        | SYS_TICK_CTRL::CLKSOURCE as u32;

    // for STM32F4 and STM32F0 etc., the SYSCFG should be enable by RCC->APB2ENR
    // to make SysTick works.
}

pub fn systick_init_with_millisecond() {
    // The SysTick clock source is the HCLK
    let ticks = unsafe { HCLK } / 1000;
    systick_init(ticks);
}

// this function will be triggered (executed) when `systick_register.LOAD`
// downcount to `0`.
#[no_mangle]
pub extern "C" fn SysTick_Handler() {
    unsafe {
        SYSTICKS += 1;
    }
}

pub fn get_current_ticks() -> u64 {
    unsafe { SYSTICKS }
}

pub fn get_expired_tick(current_tick: u64, period_ticks: u64) -> u64 {
    current_tick + period_ticks
}

pub fn is_tick_expired(current_tick: u64, expired_tick: u64) -> bool {
    current_tick > expired_tick
}

pub fn tick_delay(period_ticks: u64) {
    let expired_tick = get_expired_tick(get_current_ticks(), period_ticks);
    while !is_tick_expired(get_current_ticks(), expired_tick) {
        //
    }
}

pub fn uart1_init(baudrate: u32) {
    // STM32F103 datasheet Table 5. Medium-density STM32F103xx pin definitions
    // Pinouts
    // - PA9: USART1_TX
    // - PA10: USART1_RX
    //
    // note:
    // The connection wires between the MCU and USB-TTL dongle (CP2102, FT232, CH340 etc.)
    // need to have the TX and RX crossed, e.g.
    //
    // MCU                CP210x/FT232/CH340 Dongle
    // -------            -----------
    // PA9  TX  <-------> RX  (or TX)
    // PA10 RX  <-------> TX  (or RX)
    //     GND  <-------> GND
    //     3.3  <-------> 3.3V

    // to check out the UART output, run the following command in host:
    // `$ picocom -b 115200 /dev/ttyUSB1`
    // the USB-UART device path may be `/dev/ttyUSB0` or other else.
    // press `Ctrl+a, Ctrl+x` to exit `picocom`.

    let tx_pin: Pin = Pin::new(Port::A, 9);
    let rx_pin: Pin = Pin::new(Port::A, 10);

    // turn on GPIO clock
    gpio_clock_on(tx_pin.port);

    // TX (PA9) pin is configured as 50MHz output, push-pull and alternate function.
    gpio_init(
        &tx_pin,
        GPIO_MODE::GPIO_MODE_OUTPUT_50MHZ,
        GPIO_CNF::Output(GPIO_CNF_OUTPUT::GPIO_CNF_OUTPUT_AF_PUSH_PULL),
        None,
    );

    // RX (PA10) pin is configured as input mode and floating.
    gpio_init(
        &rx_pin,
        GPIO_MODE::GPIO_MODE_INPUT,
        GPIO_CNF::Input(GPIO_CNF_INPUT::GPIO_CNF_INPUT_FLOATING),
        None,
    );

    // enable USART1 clock
    let rcc_register = unsafe { &mut *get_rcc_register() };
    rcc_register.APB2ENR |= RCC_APB2_CLOCK_ENABLE::USART1_ENABLE as u32;

    // configure USART1 registers
    let clock = unsafe { PCLK2 };

    // let baud = clock / baudrate;

    // round() the value rather than floor()ing it, for more
    // accurate divisor selection.
    let baud = (clock + baudrate / 2) / baudrate;

    let usart1_register = unsafe { &mut *get_usart1_register() };

    usart1_register.BRR = baud;
    usart1_register.CR1 = USART_CR1::TE as u32 | USART_CR1::RE as u32 | USART_CR1::UE as u32;
}

pub fn uart_write_byte(usart_register: &mut USART_Register, byte: u8) {
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

pub fn uart_write_buf(usart_register: &mut USART_Register, buf: &[u8], len: usize) {
    (0..len).for_each(|idx| uart_write_byte(usart_register, buf[idx]));
}

pub fn uart_write_str(usart_register: &mut USART_Register, s: &str) {
    let bytes = s.as_bytes();
    uart_write_buf(usart_register, bytes, bytes.len());
}

pub fn uart_read_ready(usart_register: &USART_Register) -> bool {
    // If RXNE bit is set, data is ready
    (usart_register.SR & USART_SR::RXNE as u32) != 0
}

pub fn uart_read_byte(usart_register: &USART_Register) -> u8 {
    (usart_register.DR & 255) as u8
}

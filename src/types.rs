// Copyright (c) 2022 Hemashushu <hippospark@gmail.com>, All rights reserved.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

// default internal oscillator frequency
// STM32F103C8 is 8 Mhz
pub const SYS_CLOCK_FREQUENCY: u32 = 8_000_000;

#[repr(u8)]
#[derive(Clone, Copy)]
pub enum Port {
    A = 0, // default starts at 0 also
    B,
    C,
    D,
    E,
    F,
    G,
}

pub struct Pin {
    pub port: Port,
    pub number: u8,
}

impl Pin {
    pub fn new(bank: Port, number: u8) -> Self {
        Self { port: bank, number }
    }
}

macro_rules! bit_to_u32 {
    ($bit:expr) => {{
        1u32 << $bit
    }};
}

// RM0008 9.2.1 Port configuration register low (GPIOx_CRL)
// RM0008 9.2.2 Port configuration register high (GPIOx_CRH)
// low 2-bits
#[repr(u8)]
#[derive(Clone, Copy)]
pub enum GPIO_Mode {
    GPIO_MODE_INPUT,
    GPIO_MODE_OUTPUT_10MHZ,
    GPIO_MODE_OUTPUT_2MHZ,
    GPIO_MODE_OUTPUT_50MHZ,
}

// high 2-bits
pub enum GPIO_CNF {
    Input(GPIO_CNF_Input),
    Output(GPIO_CNF_Output),
}

#[repr(u8)]
#[derive(Clone, Copy)]
pub enum GPIO_CNF_Input {
    GPIO_CNF_INPUT_ANALOG,
    GPIO_CNF_INPUT_FLOATING,
    GPIO_CNF_INPUT_PULL_UP_AND_PULL_DOWN,
}

#[repr(u8)]
#[derive(Clone, Copy)]
pub enum GPIO_CNF_Output {
    GPIO_CNF_OUTPUT_PUSH_PULL,
    GPIO_CNF_OUTPUT_OPEN_DRAIN,
    GPIO_CNF_OUTPUT_AF_PUSH_PULL,
    GPIO_CNF_OUTPUT_AF_OPEN_DRAIN,
}

// RM0008 9.5 GPIO and AFIO register maps
#[repr(C)]
pub struct GPIO_Register {
    // RM0008 9.2.1 Port configuration register low
    // RM0008 9.2.2 Port configuration register high
    pub CRL: [u32; 2], // index 0 for pin 0..7, index 1 for pin 8..15
    pub IDR: u32,
    pub ODR: u32,
    pub BSRR: u32, // bit 16..31 for `reset`, bit 0..15 for `set`
    pub BRR: u32,
    pub LCKR: u32,
}

// RM0008 3.3 Memory map
//
// 0x4001 2000 - 0x4001 23FF  GPIO Port G
// 0x4001 1C00 - 0x4001 1FFF  GPIO Port F
// 0x4001 1800 - 0x4001 1BFF  GPIO Port E
// 0x4001 1400 - 0x4001 17FF  GPIO Port D
// 0x4001 1000 - 0x4001 13FF  GPIO Port C
// 0x4001 0C00 - 0x4001 0FFF  GPIO Port B
// 0x4001 0800 - 0x4001 0BFF  GPIO Port A
pub fn getGPIORegister(bank: Port) -> *mut GPIO_Register {
    let addr: u32 = 0x4001_0800 + 0x400 * bank as u32;
    addr as *mut GPIO_Register
}

// RM0008 8.3.7 APB2 peripheral clock enable register (RCC_APB2ENR)
//
// bit 2: GPIO A Enable
// bit 3: GPIO B Enable
// bit 4: GPIO C Enable
// bit 5: GPIO D Enable
// bit 6: GPIO E Enable
#[repr(u32)]
pub enum PeripheralBus2Enable {
    AFIO_ENABLE = bit_to_u32!(0),
    GPIO_BANK_A_ENABLE = bit_to_u32!(2),
    GPIO_BANK_B_ENABLE = bit_to_u32!(3),
    GPIO_BANK_C_ENABLE = bit_to_u32!(4),
    GPIO_BANK_D_ENABLE = bit_to_u32!(5),
    GPIO_BANK_E_ENABLE = bit_to_u32!(6),
    ADC1_ENABLE = bit_to_u32!(9),
    ADC2_ENABLE = bit_to_u32!(10),
    TIMER1_ENABLE = bit_to_u32!(11),
    SPI1_ENABLE = bit_to_u32!(12),
    USART1_ENABLE = bit_to_u32!(14),
}

// RM0008 8.3.13 RCC register map
#[repr(C)]
pub struct RCC_Register {
    pub CR: u32,
    pub CFGR: u32,
    pub CIR: u32,
    pub APB2RSTR: u32,
    pub APB1RSTR: u32,
    pub AHBENR: u32,
    pub APB2ENR: u32,
    pub APB1ENR: u32,
    pub BDCR: u32,
    pub CSR: u32,
    pub AHBSTR: u32,
    pub CFGR2: u32,
}

// RM0008 3.3 Memory map
// 0x4002 1000 - 0x4002 13FF Reset and clock control RCC
pub fn getRCCRegister() -> *mut RCC_Register {
    let addr: u32 = 0x4002_1000;
    addr as *mut RCC_Register
}

/**
 * PM0214 4.5 SysTick timer (STK)
 *
 * - 0xE000E010 STK_CTRL     RW      SysTick control and status register (STK_CTRL)
 * - 0xE000E014 STK_LOAD     RW      SysTick reload value register (STK_LOAD)
 * - 0xE000E018 STK_VAL      RW      SysTick current value register (STK_VAL)
 * - 0xE000E01C STK_CALIB    RO      SysTick calibration value register (STK_CALIB)
 */
#[repr(C)]
pub struct SysTick_Register {
    pub CTRL: u32,
    pub LOAD: u32,
    pub VAL: u32,
    pub CALIB: u32,
}

// PM0214 4.5 SysTick timer (STK)
// Table 54. System timer registers summary
// 0xE000E010 STK_CTRL
pub fn getSysTickRegister() -> *mut SysTick_Register {
    let addr: u32 = 0xe000_e010;
    addr as *mut SysTick_Register
}

// PM0214 4.5.1 SysTick control and status register (STK_CTRL)
// Bit 16 COUNTFLAG:
// Bit 2 CLKSOURCE: Clock source selection
// Bit 1 TICKINT: SysTick exception request enable
// Bit 0 ENABLE: Counter enable
#[repr(u32)]
pub enum SysTickCtrl {
    ENABLE = bit_to_u32!(0),
    TICKINT = bit_to_u32!(1),
    CLKSOURCE = bit_to_u32!(2),
    COUNTFLAG = bit_to_u32!(16),
}

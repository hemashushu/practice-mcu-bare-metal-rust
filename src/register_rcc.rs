// Copyright (c) 2022 Hemashushu <hippospark@gmail.com>, All rights reserved.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

// Default value of the External oscillator in Hz.
// pub const HSE_VALUE: u32 = 8000000;
//
// Default value of the Internal oscillator in Hz.
// pub const HSI_VALUE: u32 = 8000000;
//
// pub const AHBPrescTable: [u8; 16] = [0, 0, 0, 0, 0, 0, 0, 0, 1, 2, 3, 4, 6, 7, 8, 9];

// RM0008 7.3.11 Low-, medium-, high- and XL-density reset and clock control (RCC)
// RCC register map
//
// note:
// RM0008 Section 7 Low-, medium-, high- and XL-density reset and clock control (RCC)
//
// STM32F101xx, STM32F102xx and STM32F103xx
// - Low-density flash between 16 and 32 Kbytes.
// - Medium-density between 64 and 128 Kbytes.
// - High-density between 256 and 512 Kbytes.
// - XL-density between 768 Kbytes and 1 Mbyte.
//
// STM32F105xx and STM32F107xx see RM0008 Section 8.
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
    pub AHBSTR:u32,
    pub CFGR2:u32
}

// RM0008 7.3.1 Clock control register (RCC_CR)
#[repr(u32)]
pub enum RCC_CR {
    HSION = bit_to_u32!(0),
    HSIRDY = bit_to_u32!(1),
    HSITRIM = ones_to_u32!(3, 5),
    HSICAL = ones_to_u32!(8, 8),
    HSEON = bit_to_u32!(16),
    HSERDY = bit_to_u32!(17),
    HSEBYP = bit_to_u32!(18),
    CSSON = bit_to_u32!(19),
    PLLON = bit_to_u32!(24),
    PLLRDY = bit_to_u32!(25),
}

// HSEON: HSE clock enable
pub const RCC_CR_HSEON: u32 = bit_to_u32!(16);
pub const RCC_CR_HSERDY: u32 = bit_to_u32!(17);

// PLLON: PLL enable
pub const RCC_CR_PLLON: u32 = bit_to_u32!(24);
pub const RCC_CR_PLLRDY: u32 = bit_to_u32!(25);

// RM0008 7.3.2 Clock configuration register (RCC_CFGR)
#[repr(u32)]
pub enum RCC_CFGR {
    SW = ones_to_u32!(0, 2),
    SWS = ones_to_u32!(2, 2),
    HPRE = ones_to_u32!(4, 4),
    PPRE1 = ones_to_u32!(8, 3),
    PPRE2 = ones_to_u32!(11, 3),
    ADCPRE = ones_to_u32!(14, 2),
    PLLSRC = bit_to_u32!(16),
    PLLXTPRE = bit_to_u32!(17),
    PLLMUL = ones_to_u32!(18, 4),
    USBPRE = bit_to_u32!(22),
    MCO = ones_to_u32!(24, 3),
}

// SW: System clock switch
#[repr(u32)]
pub enum RCC_CFGR_SW {
    HSI = 0b00, // selected as system clock
    HSE = 0b01, // selected as system clock
    PLL = 0b10, // selected as system clock
}

// SWS: System clock switch status
#[repr(u32)]
pub enum RCC_CFGR_SWS {
    HSI = bits_to_u32!(0b00, 2),
    HSE = bits_to_u32!(0b01, 2),
    PLL = bits_to_u32!(0b10, 2),
}

// HPRE: AHB prescaler
// Set and cleared by software to control the division factor of the AHB clock.
#[repr(u32)]
pub enum RCC_CFGR_HPRE {
    // 0xxx: SYSCLK not divided
    NOT_DIVIDED = bits_to_u32!(0b0000, 4),

    // 1000: SYSCLK divided by 2
    // 1001: SYSCLK divided by 4
    // 1010: SYSCLK divided by 8
    // 1011: SYSCLK divided by 16
    // 1100: SYSCLK divided by 64
    // 1101: SYSCLK divided by 128
    // 1110: SYSCLK divided by 256
    // 1111: SYSCLK divided by 512
    DIVIDED_2 = bits_to_u32!(0b1000, 4),
    DIVIDED_4 = bits_to_u32!(0b1001, 4),
    DIVIDED_8 = bits_to_u32!(0b1010, 4),
    DIVIDED_16 = bits_to_u32!(0b1011, 4),
    DIVIDED_64 = bits_to_u32!(0b1100, 4),
    DIVIDED_128 = bits_to_u32!(0b1101, 4),
    DIVIDED_256 = bits_to_u32!(0b1110, 4),
    DIVIDED_512 = bits_to_u32!(0b1111, 4),
}

pub const RCC_CFGR_HPRE_MASK:u32 = ones_to_u32!(4, 4);

// PPRE1: APB low-speed prescaler (APB1)
// Set and cleared by software to control the division factor of the APB low-speed clock
// (PCLK1).
#[repr(u32)]
pub enum RCC_CFGR_PPRE1 {
    // 0xx: HCLK not divided
    NOT_DIVIDED = bits_to_u32!(0b000, 8),

    // 100: HCLK divided by 2
    // 101: HCLK divided by 4
    // 110: HCLK divided by 8
    // 111: HCLK divided by 16
    DIVIDED_2 = bits_to_u32!(0b100, 8),
    DIVIDED_4 = bits_to_u32!(0b101, 8),
    DIVIDED_8 = bits_to_u32!(0b110, 8),
    DIVIDED_16 = bits_to_u32!(0b111, 8),
}

pub const RCC_CFGR_PPRE1_MASK:u32 = ones_to_u32!(8, 3);

#[repr(u32)]
pub enum RCC_CFGR_PPRE2 {
    // 0xx: HCLK not divided
    NOT_DIVIDED = bits_to_u32!(0b000, 11),

    // 100: HCLK divided by 2
    // 101: HCLK divided by 4
    // 110: HCLK divided by 8
    // 111: HCLK divided by 16
    DIVIDED_2 = bits_to_u32!(0b100, 11),
    DIVIDED_4 = bits_to_u32!(0b101, 11),
    DIVIDED_8 = bits_to_u32!(0b110, 11),
    DIVIDED_16 = bits_to_u32!(0b111, 11),
}

pub const RCC_CFGR_PPRE2_MASK:u32 = ones_to_u32!(11, 3);

#[repr(u32)]
pub enum RCC_CFGR_ADCPRE {
    // Set and cleared by software to select the frequency of the clock to the ADCs.
    // 00: PCLK2 divided by 2
    // 01: PCLK2 divided by 4
    // 10: PCLK2 divided by 6
    // 11: PCLK2 divided by 8
    DIVIDED_2 = bits_to_u32!(0b00, 14),
    DIVIDED_4 = bits_to_u32!(0b01, 14),
    DIVIDED_6 = bits_to_u32!(0b10, 14),
    DIVIDED_8 = bits_to_u32!(0b11, 14),
}

pub const RCC_CFGR_ADCPRE_MASK:u32 = ones_to_u32!(14, 2);

// PLLMUL: PLL multiplication factor
// These bits are written by software to define the PLL multiplication factor. These bits can be
// written only when PLL is disabled.
#[repr(u32)]
pub enum RCC_CFGR_PLLMUL {
    // 0010: PLL input clock x 4
    // 0011: PLL input clock x 5
    // 0100: PLL input clock x 6
    // 0101: PLL input clock x 7
    // 0110: PLL input clock x 8
    // 0111: PLL input clock x 9
    MULX4 = bits_to_u32!(0b0010, 18),
    MULX5 = bits_to_u32!(0b0011, 18),
    MULX6 = bits_to_u32!(0b0100, 18),
    MULX7 = bits_to_u32!(0b0101, 18),
    MULX8 = bits_to_u32!(0b0110, 18),
    MULX9 = bits_to_u32!(0b0111, 18),

//     // 0000: PLL input clock x 2
//     // 0001: PLL input clock x 3
//     // 0010: PLL input clock x 4
//     // 0011: PLL input clock x 5
//     // 0100: PLL input clock x 6
//     // 0101: PLL input clock x 7
//     // 0110: PLL input clock x 8
//     // 0111: PLL input clock x 9
//     MULX2 = bits_to_u32!(0b0000, 18),
//     MULX3 = bits_to_u32!(0b0001, 18),
//     MULX4 = bits_to_u32!(0b0010, 18),
//     MULX5 = bits_to_u32!(0b0011, 18),
//     MULX6 = bits_to_u32!(0b0100, 18),
//     MULX7 = bits_to_u32!(0b0101, 18),
//     MULX8 = bits_to_u32!(0b0110, 18),
//     MULX9 = bits_to_u32!(0b0111, 18),
//
//     // 1000: PLL input clock x 10
//     // 1001: PLL input clock x 11
//     // 1010: PLL input clock x 12
//     // 1011: PLL input clock x 13
//     // 1100: PLL input clock x 14
//     // 1101: PLL input clock x 15
//     // 1110: PLL input clock x 16
//     // 1111: PLL input clock x 16
//     MULX10 = bits_to_u32!(0b1000, 18),
//     MULX11 = bits_to_u32!(0b1001, 18),
//     MULX12 = bits_to_u32!(0b1010, 18),
//     MULX13 = bits_to_u32!(0b1011, 18),
//     MULX14 = bits_to_u32!(0b1100, 18),
//     MULX15 = bits_to_u32!(0b1101, 18),
//     MULX16 = bits_to_u32!(0b1110, 18),
//     MULX16DUP = bits_to_u32!(0b1111, 18),
}

pub const RCC_CFGR_PLLMUL_MASK:u32 = ones_to_u32!(18,4);

pub const RCC_CFGR_PLLXTPRE_MASK:u32 = bit_to_u32!(17);

pub const RCC_CFGR2_PREDIV1_MASK:u32 = ones_to_u32!(0,4);
pub const RCC_CFGR2_PREDIV1SRC_MASK:u32 = bit_to_u32!(16);

// PLLSRC: PLL entry clock source
// Set and cleared by software to select PLL clock source. This bit can be written only when
// PLL is disabled.
pub const RCC_CFGR_PLLSRC_HSE:u32 = bit_to_u32!(16); // 1: HSE oscillator clock selected as PLL input clock


// RM0008 7.3.7 APB2 peripheral clock enable register (RCC_APB2ENR)
#[repr(u32)]
pub enum RCC_APB2_CLOCK_ENABLE {
    AFIO_ENABLE = bit_to_u32!(0),
    // bit 1 reserved
    GPIO_PORT_A_ENABLE = bit_to_u32!(2),
    GPIO_PORT_B_ENABLE = bit_to_u32!(3),
    GPIO_PORT_C_ENABLE = bit_to_u32!(4),
    GPIO_PORT_D_ENABLE = bit_to_u32!(5),
    GPIO_PORT_E_ENABLE = bit_to_u32!(6),
    GPIO_PORT_F_ENABLE = bit_to_u32!(7),
    GPIO_PORT_G_ENABLE = bit_to_u32!(8),
    ADC1_ENABLE = bit_to_u32!(9),
    ADC2_ENABLE = bit_to_u32!(10),
    TIMER1_ENABLE = bit_to_u32!(11),
    SPI1_ENABLE = bit_to_u32!(12),
    TIMER8_ENABLE = bit_to_u32!(13),
    USART1_ENABLE = bit_to_u32!(14),
    ADC3_ENABLE = bit_to_u32!(15),

    TIMER9_ENABLE = bit_to_u32!(19),
    TIMER10_ENABLE = bit_to_u32!(20),
    TIMER11_ENABLE = bit_to_u32!(21),
}

// RM0008 3.3 Memory map
// 0x4002 1000 - 0x4002 13FF Reset and clock control RCC
pub fn get_rcc_register() -> *mut RCC_Register {
    let addr: u32 = 0x4002_1000;
    addr as *mut RCC_Register
}

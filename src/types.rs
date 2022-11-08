// Copyright (c) 2022 Hemashushu <hippospark@gmail.com>, All rights reserved.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

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
    pub fn new(port: Port, number: u8) -> Self {
        Self { port, number }
    }
}

// #[repr(u32)]
// pub enum FlagStatus {
//     RESET = 0,
//     SET = !0,
// }

/**
 * convert the bit number to uint32
 * e.g.
 *
 * bit_to_u32(1) => 0x...0000_0010
 * bit_to_u32(4) => 0x...0000_1000
 * bit_to_u32(7) => 0x...0100_0000
 */
macro_rules! bit_to_u32 {
    ($bit:expr) => {{
        1u32 << $bit
    }};
}

/**
 * convert the bit range to uint32
 * e.g.
 *
 * bit_range_to_u32(0, 2) => 0x...0000_0011
 * bit_range_to_u32(2, 2) => 0x...0000_1100
 * bit_range_to_u32(2, 4) => 0x...0011_1100
 */
macro_rules! bit_range_to_u32 {
    ($bit_start:expr, $bit_length:expr) => {{
        let a: u32 = 2u32.pow($bit_length) - 1;
        a << $bit_start
    }};
}

/**
 * convert the value of a bit range to integer
 *
 * e.g.
 * get_bit_range_value(0x????_11??, 0x0000_001100),
 * return `0x????_11?? & 0x0000_001100 >> 2`
 */
pub fn get_bit_range_value(src: u32, mask: u32) -> u32 {
    let tailing_zeros = mask.trailing_zeros();

    // `src` is unsigned integer, so `>>` is logic right shift
    (src & mask) >> tailing_zeros
}

// RM0008 9.2.1 Port configuration register low (GPIOx_CRL)
// RM0008 9.2.2 Port configuration register high (GPIOx_CRH)
// low 2-bits
#[repr(u8)]
#[derive(Clone, Copy)]
pub enum GPIO_MODE {
    GPIO_MODE_INPUT,
    GPIO_MODE_OUTPUT_10MHZ,
    GPIO_MODE_OUTPUT_2MHZ,
    GPIO_MODE_OUTPUT_50MHZ,
}

// RM0008 9.2.1 Port configuration register low (GPIOx_CRL)
// RM0008 9.2.2 Port configuration register high (GPIOx_CRH)
// high 2-bits
pub enum GPIO_CNF {
    Input(GPIO_CNF_INPUT),
    Output(GPIO_CNF_OUTPUT),
}

#[repr(u8)]
#[derive(Clone, Copy)]
pub enum GPIO_CNF_INPUT {
    GPIO_CNF_INPUT_ANALOG,
    GPIO_CNF_INPUT_FLOATING,
    GPIO_CNF_INPUT_PULL_UP_AND_PULL_DOWN,
}

#[repr(u8)]
#[derive(Clone, Copy)]
pub enum GPIO_CNF_OUTPUT {
    GPIO_CNF_OUTPUT_PUSH_PULL,
    GPIO_CNF_OUTPUT_OPEN_DRAIN,
    GPIO_CNF_OUTPUT_AF_PUSH_PULL,
    GPIO_CNF_OUTPUT_AF_OPEN_DRAIN,
}

// RM0008 9.5 GPIO and AFIO register maps
#[repr(C)]
pub struct GPIO_Register {
    // CR
    // - 0: RM0008 9.2.1 Port configuration register low
    // - 1: RM0008 9.2.2 Port configuration register high
    pub CR: [u32; 2], // index 0 for pin 0..7, index 1 for pin 8..15
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
pub fn getGPIORegister(port: Port) -> *mut GPIO_Register {
    let addr: u32 = 0x4001_0800 + 0x400 * port as u32;
    addr as *mut GPIO_Register
}

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
}

// RM0008 3.3 Memory map
// 0x4002 1000 - 0x4002 13FF Reset and clock control RCC
pub fn get_rcc_register() -> *mut RCC_Register {
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
pub fn get_systick_register() -> *mut SysTick_Register {
    let addr: u32 = 0xe000_e010;
    addr as *mut SysTick_Register
}

// PM0214 4.5.1 SysTick control and status register (STK_CTRL)
//
// Bit 16 COUNTFLAG:
// Bit 2 CLKSOURCE: Clock source selection
// Bit 1 TICKINT: SysTick exception request enable
// Bit 0 ENABLE: Counter enable
#[repr(u32)]
pub enum SYS_TICK_CTRL {
    ENABLE = bit_to_u32!(0),
    TICKINT = bit_to_u32!(1),
    CLKSOURCE = bit_to_u32!(2),
    COUNTFLAG = bit_to_u32!(16),
}

// Default value of the External oscillator in Hz.
pub const HSE_VALUE: u32 = 8000000;

// Default value of the Internal oscillator in Hz.
pub const HSI_VALUE: u32 = 8000000;

// RM0008 7.3.1 Clock control register (RCC_CR)
#[repr(u32)]
pub enum RCC_CR {
    HSION = bit_to_u32!(0),
    HSIRDY = bit_to_u32!(1),
    HSITRIM = bit_range_to_u32!(3, 5),
    HSICAL = bit_range_to_u32!(8, 8),
    HSEON = bit_to_u32!(16),
    HSERDY = bit_to_u32!(17),
    HSEBYP = bit_to_u32!(18),
    CSSON = bit_to_u32!(19),
    PLLON = bit_to_u32!(24),
    PLLRDY = bit_to_u32!(25),
}

// HSEON: HSE clock enable
#[repr(u32)]
pub enum RCC_CR_HSEON {
    OFF = 0 << 16,
    ON = 1 << 16,
}

// PLLON: PLL enable
#[repr(u32)]
pub enum RCC_CR_PLLON {
    OFF = 0 << 24, // PLL OFF
    ON = 1 << 24,  // PLL ON
}

// RM0008 7.3.2 Clock configuration register (RCC_CFGR)
#[repr(u32)]
pub enum RCC_CFGR {
    SW = bit_range_to_u32!(0, 2),
    SWS = bit_range_to_u32!(2, 2),
    HPRE = bit_range_to_u32!(4, 4),
    PPRE1 = bit_range_to_u32!(8, 3),
    PPRE2 = bit_range_to_u32!(11, 3),
    ADCPRE = bit_range_to_u32!(14, 2),
    PLLSRC = bit_to_u32!(16),
    PLLXTPRE = bit_to_u32!(17),
    PLLMUL = bit_range_to_u32!(18, 4),
    USBPRE = bit_to_u32!(22),
    MCO = bit_range_to_u32!(24, 3),
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
    HSI = 0b00 << 2,
    HSE = 0b01 << 2,
    PLL = 0b10 << 2,
}

// PPRE1: APB low-speed prescaler (APB1)
// Set and cleared by software to control the division factor of the APB low-speed clock
// (PCLK1).
#[repr(u32)]
pub enum RCC_CFGR_PPRE1 {
    // 0xx: HCLK not divided
    NOT_DIVIDED = 0b000 << 8,

    // 100: HCLK divided by 2
    // 101: HCLK divided by 4
    // 110: HCLK divided by 8
    // 111: HCLK divided by 16
    DIVIDED_2 = 0b100 << 8,
    DIVIDED_4 = 0b101 << 8,
    DIVIDED_8 = 0b110 << 8,
    DIVIDED_16 = 0b111 << 8,
}

// PLLMUL: PLL multiplication factor
// These bits are written by software to define the PLL multiplication factor. These bits can be
// written only when PLL is disabled.
#[repr(u32)]
pub enum RCC_CFGR_PLLMUL {
    // 0000: PLL input clock x 2
    // 0001: PLL input clock x 3
    // 0010: PLL input clock x 4
    // 0011: PLL input clock x 5
    // 0100: PLL input clock x 6
    // 0101: PLL input clock x 7
    // 0110: PLL input clock x 8
    // 0111: PLL input clock x 9
    MULX2 = 0b0000 << 18,
    MULX3 = 0b0001 << 18,
    MULX4 = 0b0010 << 18,
    MULX5 = 0b0011 << 18,
    MULX6 = 0b0100 << 18,
    MULX7 = 0b0101 << 18,
    MULX8 = 0b0110 << 18,
    MULX9 = 0b0111 << 18,

    // 1000: PLL input clock x 10
    // 1001: PLL input clock x 11
    // 1010: PLL input clock x 12
    // 1011: PLL input clock x 13
    // 1100: PLL input clock x 14
    // 1101: PLL input clock x 15
    // 1110: PLL input clock x 16
    // 1111: PLL input clock x 16
    MULX10 = 0b1000 << 18,
    MULX11 = 0b1001 << 18,
    MULX12 = 0b1010 << 18,
    MULX13 = 0b1011 << 18,
    MULX14 = 0b1100 << 18,
    MULX15 = 0b1101 << 18,
    MULX16 = 0b1110 << 18,
    MULX16ALSO = 0b1111 << 18,
}

// PLLSRC: PLL entry clock source
// Set and cleared by software to select PLL clock source. This bit can be written only when
// PLL is disabled.
#[repr(u32)]
pub enum RCC_CFGR_PLLSRC {
    HSI = 0 << 16, // 0: HSI oscillator clock / 2 selected as PLL input clock
    HSE = 1 << 16, // 1: HSE oscillator clock selected as PLL input clock
}

// PLLXTPRE: HSE divider for PLL entry
// Set and cleared by software to divide HSE before PLL entry. This bit can be written only
// when PLL is disabled.
#[repr(u32)]
pub enum RCC_CFGR_PLLXTPRE {
    RESET = 0 << 17, // 0: HSE clock not divided
    SET = 1 << 17,   // 1: HSE clock divided by 2
}

// HPRE: AHB prescaler
// Set and cleared by software to control the division factor of the AHB clock.
#[repr(u32)]
pub enum RCC_CFGR_HPRE {
    // 0xxx: SYSCLK not divided
    NOT_DIVIDED = 0b0000 << 4,

    // 1000: SYSCLK divided by 2
    // 1001: SYSCLK divided by 4
    // 1010: SYSCLK divided by 8
    // 1011: SYSCLK divided by 16
    // 1100: SYSCLK divided by 64
    // 1101: SYSCLK divided by 128
    // 1110: SYSCLK divided by 256
    // 1111: SYSCLK divided by 512
    DIVIDED_2 = 0b1000 << 4,
    DIVIDED_4 = 0b1001 << 4,
    DIVIDED_8 = 0b1010 << 4,
    DIVIDED_16 = 0b1011 << 4,
    DIVIDED_64 = 0b1100 << 4,
    DIVIDED_128 = 0b1101 << 4,
    DIVIDED_256 = 0b1110 << 4,
    DIVIDED_512 = 0b1111 << 4,
}

pub const AHBPrescTable: [u8; 16] = [0, 0, 0, 0, 0, 0, 0, 0, 1, 2, 3, 4, 6, 7, 8, 9];

// RM0008 3.3 Memory map
// Table 5. Flash module organization (medium-density devices)
// Flash memory interface registers
// FLASH_ACR 0x4002 2000 - 0x4002 2003 4
// FLASH_KEYR 0x4002 2004 - 0x4002 2007 4
// FLASH_OPTKEYR 0x4002 2008 - 0x4002 200B 4
// FLASH_SR 0x4002 200C - 0x4002 200F 4
// FLASH_CR 0x4002 2010 - 0x4002 2013 4
// FLASH_AR 0x4002 2014 - 0x4002 2017 4
// Reserved 0x4002 2018 - 0x4002 201B 4
// FLASH_OBR 0x4002 201C - 0x4002 201F 4
// FLASH_WRPR 0x4002 2020 - 0x4002 2023 4
#[repr(C)]
pub struct Flash_Register {
    pub ACR: u32,
    pub KEYR: u32,
    pub OPTKEYR: u32,
    pub SR: u32,
    pub CR: u32,
    pub AR: u32,
    pub Reserved: u32,
    pub OBR: u32,
    pub WRPR: u32,
}

pub fn get_flash_register() -> *mut Flash_Register {
    let addr: u32 = 0x4002_2000;
    addr as *mut Flash_Register
}

// PM0075 3.1 Flash access control register (FLASH_ACR)
// 3.1 Flash access control register (FLASH_ACR)
#[repr(u32)]
pub enum FLASH_ACR {
    // Bits 2:0 LATENCY: Latency
    // These bits represent the ratio of the SYSCLK (system clock) period to the Flash access
    // time.
    LATENCY = bit_range_to_u32!(0, 3),

    // Bit 3 HLFCYA: Flash half cycle access enable
    // 0: Half cycle is disabled
    // 1: Half cycle is enabled
    HLFCYA = bit_to_u32!(3),

    // Bit 4 PRFTBE: Prefetch buffer enable
    // 0: Prefetch is disabled
    // 1: Prefetch is enabled
    PRFTBE = bit_to_u32!(4),

    // Bit 5 PRFTBS: Prefetch buffer status
    // This bit provides the status of the prefetch buffer.
    // 0: Prefetch buffer is disabled
    // 1: Prefetch buffer is enabled
    PRFTBS = bit_to_u32!(5),
}

#[repr(u32)]
pub enum FLASH_ACR_LATENCY {
    // PM0075 3.1 Flash access control register (FLASH_ACR)
    // 000 Zero wait state, if 0 < SYSCLK≤ 24 MHz
    // 001 One wait state, if 24 MHz < SYSCLK ≤ 48 MHz
    // 010 Two wait states, if 48 MHz < SYSCLK ≤ 72 MHz
    //
    // note:
    // in project CMSIS/Device/STM32F1xx/include/stm32f103xb.h, these value is define as another values:
    // #define FLASH_ACR_LATENCY_0                 (0x1U << FLASH_ACR_LATENCY_Pos)    /*!< 0x00000001 */
    // #define FLASH_ACR_LATENCY_1                 (0x2U << FLASH_ACR_LATENCY_Pos)    /*!< 0x00000002 */
    // #define FLASH_ACR_LATENCY_2                 (0x4U << FLASH_ACR_LATENCY_Pos)    /*!< 0x00000004 */
    Zero = 0b000,
    One = 0b001,
    Two = 0b010,
}

// RM0008 27.6.8 USART register map
#[repr(C)]
pub struct USART_Register {
    pub SR: u32,
    pub DR: u32,
    pub BRR: u32,
    pub CR1: u32,
    pub CR2: u32,
    pub CR3: u32,
    pub GTPR: u32,
}

// RM0008 27.6.4 Control register 1 (USART_CR1)
#[repr(u32)]
pub enum USART_CR1 {
    SBK = bit_to_u32!(0),
    RWU = bit_to_u32!(1),
    RE = bit_to_u32!(2), // Receiver enable
    TE = bit_to_u32!(3), // Transmitter enable

    IDLEIE = bit_to_u32!(4),
    RXNEIE = bit_to_u32!(5),
    TCIE = bit_to_u32!(6),
    TXEIE = bit_to_u32!(7),

    PEIE = bit_to_u32!(8),
    PS = bit_to_u32!(9),
    PCE = bit_to_u32!(10),
    WAKE = bit_to_u32!(11),

    M = bit_to_u32!(12),
    UE = bit_to_u32!(13), // USART enable
}

// RM0008 27.6.1 Status register (USART_SR)
#[repr(u32)]
pub enum USART_SR {
    PE = bit_to_u32!(0),
    FE = bit_to_u32!(1),
    NE = bit_to_u32!(2),
    ORE = bit_to_u32!(3),

    IDLE = bit_to_u32!(4),

    // RXNE: Read data register not empty
    // This bit is set by hardware when the content of the RDR shift register has been transferred to
    // the USART_DR register. An interrupt is generated if RXNEIE=1 in the USART_CR1 register.
    // It is cleared by a read to the USART_DR register. The RXNE flag can also be cleared by
    // writing a zero to it. This clearing sequence is recommended only for multibuffer
    // communication.
    // 0: Data is not received
    // 1: Received data is ready to be read.
    RXNE = bit_to_u32!(5),

    TC = bit_to_u32!(6),

    // TXE: Transmit data register empty
    // This bit is set by hardware when the content of the TDR register has been transferred into
    // the shift register. An interrupt is generated if the TXEIE bit =1 in the USART_CR1 register. It
    // is cleared by a write to the USART_DR register.
    // 0: Data is not transferred to the shift register
    // 1: Data is transferred to the shift register)
    // Note: This bit is used during single buffer transmission.
    TXE = bit_to_u32!(7),

    LBD = bit_to_u32!(8),
    CTS = bit_to_u32!(9),
}

// RM0008 3.3 Memory map
// Table 3. Register boundary addresses
// 0x4001 3800 - 0x4001 3BFF USART1
// 0x4000 5000 - 0x4000 53FF UART5
// 0x4000 4C00 - 0x4000 4FFF UART4
// 0x4000 4800 - 0x4000 4BFF USART3
// 0x4000 4400 - 0x4000 47FF USART2

pub fn get_usart1_register() -> *mut USART_Register {
    let addr: u32 = 0x4001_3800;
    addr as *mut USART_Register
}
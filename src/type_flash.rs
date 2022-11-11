// Copyright (c) 2022 Hemashushu <hippospark@gmail.com>, All rights reserved.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

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

pub fn get_flash_register() -> *mut Flash_Register {
    let addr: u32 = 0x4002_2000;
    addr as *mut Flash_Register
}

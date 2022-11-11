// Copyright (c) 2022 Hemashushu <hippospark@gmail.com>, All rights reserved.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

use crate::type_common::Port;

// RM0008 9.5 GPIO and AFIO register maps
#[repr(C)]
pub struct GPIO_Register {
    // CR
    // - 0: CRL RM0008 9.2.1 Port configuration register low
    // - 1: CRH RM0008 9.2.2 Port configuration register high
    pub CR: [u32; 2], // index 0 for pin 0..7, index 1 for pin 8..15
    pub IDR: u32,
    pub ODR: u32,
    pub BSRR: u32, // bit 16..31 for `reset`, bit 0..15 for `set`
    pub BRR: u32,
    pub LCKR: u32,
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

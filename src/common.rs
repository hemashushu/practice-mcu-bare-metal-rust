// Copyright (c) 2022 Hemashushu <hippospark@gmail.com>, All rights reserved.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

// the total systicks
pub static mut SYSTICKS: u64 = 0;

pub static mut SYSCLK: u32 = 8_000_000; // 8MHz, max 72MHz
pub static mut HCLK: u32 = 8_000_000; // for AHB bus, core, memory and DMA
pub static mut PCLK1: u32 = 8_000_000; // for APB1, max 36MHz
pub static mut PCLK2: u32 = 8_000_000; // for APB2, max 72MHz
pub static mut ADCCLK:u32 = 4_000_000; // for ADC1,2, max 14MHz

#[repr(u8)]
#[derive(Clone, Copy)]
pub enum Port {
    A = 0, // the `enum` entry value starts from 0 by default also
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
    pub const fn new(port: Port, number: u8) -> Self {
        Self { port, number }
    }
}

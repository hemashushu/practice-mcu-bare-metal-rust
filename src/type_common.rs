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

// Copyright (c) 2022 Hemashushu <hippospark@gmail.com>, All rights reserved.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

use core::arch::asm;

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
 * shift left
 * e.g.
 *
 * bits_to_u32(0x11, 4) => 0x...11_0000
 */
macro_rules! bits_to_u32 {
    ($bits:expr, $offset:expr) => {{
        $bits << $offset
    }};
}

/**
 * set a range bits to `1` in uint32
 * e.g.
 *
 * ones_to_u32(0, 2) => 0x...0000_0011
 * ones_to_u32(2, 2) => 0x...0000_1100
 * ones_to_u32(2, 4) => 0x...0011_1100
 *
 * Bit `1` grows from low to high.
 */
macro_rules! ones_to_u32 {
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

pub fn spin(count: u32) {
    (0..count).for_each(|_| spin_one());
}

#[inline]
pub fn spin_one() {
    unsafe { asm!("nop") }
}

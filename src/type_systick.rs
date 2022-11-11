// Copyright (c) 2022 Hemashushu <hippospark@gmail.com>, All rights reserved.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

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

// PM0214 4.5 SysTick timer (STK)
// Table 54. System timer registers summary
// 0xE000E010 STK_CTRL
pub fn get_systick_register() -> *mut SysTick_Register {
    let addr: u32 = 0xe000_e010;
    addr as *mut SysTick_Register
}

// Copyright (c) 2022 Hemashushu <hippospark@gmail.com>, All rights reserved.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

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
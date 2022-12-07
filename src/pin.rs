// Copyright (c) 2022 Hemashushu <hippospark@gmail.com>, All rights reserved.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

use crate::common::{Pin, Port};

/**
 * note: set output to `0` to turn on builtin LED
 */
pub static BUILTIN_LED_PIN: Pin = Pin::new(Port::C, 13);

/**
 * note: set output to `1` to turn on external LED
 */
pub static EXTERNAL_LED_PIN: Pin = Pin::new(Port::B, 5);

pub static BUTTON_1_PIN: Pin = Pin::new(Port::A, 0);

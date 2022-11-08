#!/bin/bash

# # flash by `cargo-flash`:
#
# 1. install cargo-flash
# 2. check the program size
#    `$ cargo size`
#    or
#    `$ cargo size --release`
#    make sure the size of the firmware does not exceed the capacity of the flash
# 3. check probe connection
#    `$ cargo flash --list-probes`
# 4. chip name can be found by
#    `$ cargo flash --list-chips`
# 5. flash
#    `$ cargo flash --chip STM32F301C6Tx`
#    or
#    `$ cargo flash --release --chip STM32F301C6Tx``

# # flash by OpenOCD
#
# note:
# it seems there are some bugs with the 'release' profile, such as unable
# to configure the system clock, and UART also does not work properly.
cargo clean
cargo build
openocd -f interface/cmsis-dap.cfg  -f target/stm32f1x.cfg -s "/usr/share/openocd/scripts" -c "program target/thumbv7m-none-eabi/debug/bare-metal-blinky verify reset exit"
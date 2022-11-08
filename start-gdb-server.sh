#!/bin/bash

# can also start gdb server by `cargo-embed`
# https://github.com/probe-rs/cargo-embed

# start gdb server by OpenOCD
openocd -f interface/cmsis-dap.cfg  -f target/stm32f1x.cfg -s "/usr/share/openocd/scripts"


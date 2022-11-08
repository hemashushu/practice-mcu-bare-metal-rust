#!/bin/bash
cargo build
arm-none-eabi-gdb target/thumbv7m-none-eabi/debug/bare-metal-blinky -x debug-with-svd.gdb

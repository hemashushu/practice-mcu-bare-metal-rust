#!/bin/bash
# note:
#     build project first
arm-none-eabi-gdb target/thumbv7m-none-eabi/debug/bare-metal-blinky -x debug.gdb

#!/bin/bash
openocd -f interface/cmsis-dap.cfg  -f target/stm32f1x.cfg -s "/usr/share/openocd/scripts"


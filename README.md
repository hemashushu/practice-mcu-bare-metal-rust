# Practice - MCU (STM32F103C8T6) Bare-metal in Rust

A bare metal (register level) STM32F103C8T6 MCU program in Rust without any SDK, IDE, libraries, the only necessary is the Rust compiler.

![stm32f103 rust bare-metal](images/stm32f103-title.jpg)

## Implemented Features

- GPIO read and write
- SysTick
- UART read and write

[Demo](https://user-images.githubusercontent.com/394223/201260032-29f48a6d-eabd-4531-9e00-344b9ef80781.mp4)

<video width="480" height="270" controls>
  <source src="images/stm32f103.mp4" type="video/mp4">
</video>

## Source Code Structure

- `src/type_*.rs` Definition of register structures and values
- `src/vector.rs` MCU's vector array
- `src/main.rs` demo program
- `.vscode/launch.json` configuration file for VSCode debug
- `svd/` Data file for displaying various register names and values during GDB debugging
- `flash.sh` script for compiling and downloading firmware

## Reference Documents

Hardware description and register structure and value description documentations:

- PM0214 Programming manual
  STM32 Cortex®-M4 MCUs and MPUs programming manual
  https://www.st.com/resource/en/programming_manual/pm0214-stm32-cortexm4-mcus-and-mpus-programming-manual-stmicroelectronics.pdf
- STM32F103c8 Datasheet
  https://www.st.com/resource/en/datasheet/cd00161566.pdf
- RM0008 Reference manual
  STM32F101xx, STM32F102xx, STM32F103xx, STM32F105xx and
  STM32F107xx advanced Arm®-based 32-bit MCUs
  https://www.st.com/resource/en/reference_manual/cd00171190-stm32f101xx-stm32f102xx-stm32f103xx-stm32f105xx-and-stm32f107xx-advanced-arm-based-32-bit-mcus-stmicroelectronics.pdf

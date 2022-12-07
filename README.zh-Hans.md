# 使用 Rust 编写微处理单元 (MCU）STM32F103C8T6 的 "裸机程序"

[EN](README.md) | [中文](README.zh-Hans.md)

一个用 Rust 编写的微处理单元（MCU）STM32F103C8T6/STM32F103的 "裸机程序"，不依赖任何 IDE、SDK、HAL 或者库，唯一需要的是 Rust 编译器。通过直接读写硬件的寄存器实现信号的输入和输出、定时器、串口通信等功能。

![stm32f103 rust bare-metal](images/stm32f103-title-v2.jpg)

## 已实现的功能

- GPIO 读和写
- 设置系统时钟
- SysTick 和中断
- UART 读和写

<!--
[Demo](https://user-images.githubusercontent.com/394223/201260032-29f48a6d-eabd-4531-9e00-344b9ef80781.mp4)
-->

[演示视频](images/stm32f103-v2.mp4)

<video width="480" height="270" controls>
  <source src="images/stm32f103-v2.mp4" type="video/mp4">
</video>

## 源代码目录结构

- `src/register_*.rs` 寄存器结构和常量的定义
- `src/startup.rs` 启动过程的 `vector` 数列构造
- `src/main.rs` 主程序
- `.vscode/launch.json` VSCode 调试器的配置文件
- `svd/` 用于 GDB 调试时显示各个寄存器名称和地址数值的 `系统视图描述文件`
- `build.sh` 编译脚本
- `flash.sh` 编译和下载固件的脚本

## 连线

- PC13: 内置 LED -
- PB5: 外接 LED +
- PA0: 按钮（按钮另一脚接 GND）
- PA9: USART1_TX -> CP2012 RX
- PA10: USART1_RX -> CP2012 TX

## 编译

先在你的平台上安装 `Rust`，然后运行：

`$ ./build.sh`

## 固件下载

使用 `CMSIS_DAP 调试器/下载器`（硬件）连接 MCU 到你的电脑，然后运行：

`$ ./flash.sh`

## 调试

`$ ./server-gdb-server.sh`

然后打开另一个虚拟终端（terminal）运行：

`$ ./start-gdb-client-with-svd.sh`

## 参考文档

程序当中的寄存器结构和常量来自 STM32 的硬件数据手册：

- STM32F103c8 Datasheet
  https://www.st.com/resource/en/datasheet/cd00161566.pdf

- RM0008 Reference manual
  https://www.st.com/resource/en/reference_manual/cd00171190-stm32f101xx-stm32f102xx-stm32f103xx-stm32f105xx-and-stm32f107xx-advanced-arm-based-32-bit-mcus-stmicroelectronics.pdf

- PM0214 Programming manual
  https://www.st.com/resource/en/programming_manual/pm0214-stm32-cortexm4-mcus-and-mpus-programming-manual-stmicroelectronics.pdf

对于想了解工作原理，以及想进一步实现更多功能的，请参考：

- Bare metal programming guide (C lang on STM32F429)
  https://github.com/cpq/bare-metal-programming-guide

- 项目 libopencm3
  https://github.com/libopencm3/libopencm3


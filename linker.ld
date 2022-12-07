/* the entry point */
ENTRY(Reset_Handler);

/* memory layout of STM32F103C8T6 */
MEMORY
{
  FLASH : ORIGIN = 0x08000000, LENGTH = 64K
  RAM : ORIGIN = 0x20000000, LENGTH = 20K
}

/*
 * vector table entry list and item name from
 * `STM32Cube/Repository/STM32Cube_FW_F1_V1.8.4/Drivers/CMSIS/Device/ST/STM32F1xx/Source/Templates/gcc/startup_stm32f103xb.s`
 *
 * the keyword `PROVIDE` means:
 * set LHS = RHS only when the left symbol does not exists,
 * i.e. the RHS is the default value.
 */

/* ------ index 0-15 ------ */
/* idx 0: _estack */
/* idx 1: Reset_Handler */
PROVIDE(NMI_Handler = Default_Handler);
PROVIDE(HardFault_Handler = Default_Handler);
PROVIDE(MemManage_Handler = Default_Handler);
PROVIDE(BusFault_Handler = Default_Handler);
PROVIDE(UsageFault_Handler = Default_Handler);
/* idx 7,8,9,10: 0 */
PROVIDE(SVC_Handler = Default_Handler);
PROVIDE(DebugMon_Handler = Default_Handler);
/* idx 13: 0 */
PROVIDE(PendSV_Handler = Default_Handler);
PROVIDE(SysTick_Handler = Default_Handler);

/* ------ index 16-31 ------ */
PROVIDE(WWDG_IRQHandler = Default_Handler);
PROVIDE(PVD_IRQHandler = Default_Handler);
PROVIDE(TAMPER_IRQHandler = Default_Handler);
PROVIDE(RTC_IRQHandler = Default_Handler);
PROVIDE(FLASH_IRQHandler = Default_Handler);
PROVIDE(RCC_IRQHandler = Default_Handler);
PROVIDE(EXTI0_IRQHandler = Default_Handler);
PROVIDE(EXTI1_IRQHandler = Default_Handler);
PROVIDE(EXTI2_IRQHandler = Default_Handler);
PROVIDE(EXTI3_IRQHandler = Default_Handler);
PROVIDE(EXTI4_IRQHandler = Default_Handler);
PROVIDE(DMA1_Channel1_IRQHandler = Default_Handler);
PROVIDE(DMA1_Channel2_IRQHandler = Default_Handler);
PROVIDE(DMA1_Channel3_IRQHandler = Default_Handler);
PROVIDE(DMA1_Channel4_IRQHandler = Default_Handler);
PROVIDE(DMA1_Channel5_IRQHandler = Default_Handler);

/* ------ index 32-66 ------ */
PROVIDE(DMA1_Channel6_IRQHandler = Default_Handler);
PROVIDE(DMA1_Channel7_IRQHandler = Default_Handler);
PROVIDE(ADC1_2_IRQHandler = Default_Handler);
PROVIDE(USB_HP_CAN1_TX_IRQHandler = Default_Handler);
PROVIDE(USB_LP_CAN1_RX0_IRQHandler = Default_Handler);
PROVIDE(CAN1_RX1_IRQHandler = Default_Handler);
PROVIDE(CAN1_SCE_IRQHandler = Default_Handler);
PROVIDE(EXTI9_5_IRQHandler = Default_Handler);
PROVIDE(TIM1_BRK_IRQHandler = Default_Handler);
PROVIDE(TIM1_UP_IRQHandler = Default_Handler);
PROVIDE(TIM1_TRG_COM_IRQHandler = Default_Handler);
PROVIDE(TIM1_CC_IRQHandler = Default_Handler);
PROVIDE(TIM2_IRQHandler = Default_Handler);
PROVIDE(TIM3_IRQHandler = Default_Handler);
PROVIDE(TIM4_IRQHandler = Default_Handler);
PROVIDE(I2C1_EV_IRQHandler = Default_Handler);
PROVIDE(I2C1_ER_IRQHandler = Default_Handler);
PROVIDE(I2C2_EV_IRQHandler = Default_Handler);
PROVIDE(I2C2_ER_IRQHandler = Default_Handler);
PROVIDE(SPI1_IRQHandler = Default_Handler);
PROVIDE(SPI2_IRQHandler = Default_Handler);
PROVIDE(USART1_IRQHandler = Default_Handler);
PROVIDE(USART2_IRQHandler = Default_Handler);
PROVIDE(USART3_IRQHandler = Default_Handler);
PROVIDE(EXTI15_10_IRQHandler = Default_Handler);
PROVIDE(RTC_Alarm_IRQHandler = Default_Handler);
PROVIDE(USBWakeUp_IRQHandler = Default_Handler);
/* idx 59,60,61,62,63,64,65: 0 */
/* idx 66: BootRAM */

/*
 * use the `EXTERN` statement to preserve symbols that NOT directly or
 * indirectly referenced by the entry point.
 */
EXTERN(Reset_Vector);
EXTERN(Exceptions);

/*
 * $ arm-none-eabi-objdump -h target/thumbv7m-none-eabi/debug/bare-metal-blinky
 *
 * ```text
 * Sections:
 * Idx Name          Size      VMA       LMA       File off  Algn
 *   0 .vector_table 00000040  08000000  08000000  00010000  2**2
 *                   CONTENTS, ALLOC, LOAD, READONLY, DATA
 *   1 .text         000012bc  08000040  08000040  00010040  2**2
 *                   CONTENTS, ALLOC, LOAD, READONLY, CODE
 *   2 .rodata       00000440  08001300  08001300  00011300  2**4
 *                   CONTENTS, ALLOC, LOAD, READONLY, DATA
 *   3 .data         0000000c  20000000  08001740  00020000  2**2
 *                   CONTENTS, ALLOC, LOAD, DATA
 *   4 .bss          00000020  2000000c  2000000c  0002000c  2**2
 *                   ALLOC
 * ```
 *
 * - `.vector_table` starts from LMA `0x0800_0000`
 * - `.text` starts from LMA `B` (`B` = ADDR(.vector_table) + SIZEOF(.vector_table))
 * - `.rodata` starts from LMA `C` (`C` = ADDR(.text) + SIZEOF(.text))
 * - `.data` starts from VMA 0x2000_0000 and LMA `D` (`D` = ADDR(.rodata) + SIZEOF(.rodata))
 * - `.bss` starts from VMA `E` (`E` = ADDR(.data) + SIZEOF(.data))
 */

/* the stack point is the end address of RAM, i.e. `0x20000000 + 0x5000` */
_estack = ORIGIN(RAM) + LENGTH(RAM);

SECTIONS
{
  /*
   * PM0214 Programming manual
   * 2.3.4 Vector table
   * Vector table 16 standard and 91 STM32-specific exception handlers/functions,
   * size is `0x1ac`
   */
  .vector_table ORIGIN(FLASH) :
  {
    /* first entry: the initial Stack Pointer (SP) value */
    LONG(_estack);

    /*
     * second entry: reset handler
     *
     * use the `KEEP` statement to force the linker to insert
     * all input sections named, otherwise the linker will
     * discard the unused sections
     */
    KEEP(*(.vector_table.reset_vector));

    /* the next 14 entries are exception vectors */
    KEEP(*(.vector_table.exceptions));
  } > FLASH

  /* The 'text' section contains the main program code. */
  .text : ALIGN(4)
  {
    *(.text .text.*);
  } > FLASH

  /* The 'rodata' section contains read-only data,
   * constants, strings, information that won't change. */
  .rodata : ALIGN(4)
  {
    *(.rodata .rodata.*);
  } > FLASH

  /* The 'data' section is space set aside in RAM for
   * things like variables, which can change.
   *
   * set the Load Memory Address (LMA) of the `.data` section by
   * `AT(...)` at header line or `AT > FLASH` at footer line
   */
  .data : ALIGN(4) /* AT(ADDR(.rodata) + SIZEOF(.rodata)) */
  {
    _sdata = .;
    *(.data .data.*);
    _edata = .;
  }  > RAM AT > FLASH

  /**
   * get the LMA of the section `.data`
   * - 3.1 Basic Linker Script Concepts
   *   https://sourceware.org/binutils/docs/ld/Basic-Script-Concepts.html
   * - 3.6.8.2 Output Section LMA
   *   https://sourceware.org/binutils/docs/ld/Output-Section-LMA.html
   * - LOADADDR(section)
   *   https://sourceware.org/binutils/docs/ld/Builtin-Functions.html#index-LOADADDR_0028section_0029
   */
  /* LMA of .data */
  _sidata = LOADADDR(.data);

  /* The 'bss' section is similar to the 'data' section,
   * but its space is initialized to all 0s at the
   * start of the program. */
  .bss : ALIGN(4)
  {
    _sbss = .;
    *(.bss .bss.*);
    _ebss = .;
  } > RAM

  . = ALIGN(4);
  _heap_start = .;

  /* Discarded sections */
  /DISCARD/ :
  {
    *(.ARM.exidx .ARM.exidx.* .ARM.extab.*);
  }
}
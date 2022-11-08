/* the entry point */
ENTRY(ResetHandler);

/* memory layout of STM32F103C8T6 */
MEMORY
{
    FLASH : ORIGIN = 0x08000000, LENGTH = 64K
    RAM : ORIGIN = 0x20000000, LENGTH = 20K
}

/*
 * set LHS = RHS only when the left symbol does not exists,
 * so the RHS is the default value.
 */
PROVIDE(NMI = DefaultExceptionHandler);
PROVIDE(HardFault = DefaultExceptionHandler);
PROVIDE(MemManage = DefaultExceptionHandler);
PROVIDE(BusFault = DefaultExceptionHandler);
PROVIDE(UsageFault = DefaultExceptionHandler);
PROVIDE(SVCall = DefaultExceptionHandler);
PROVIDE(PendSV = DefaultExceptionHandler);
PROVIDE(SysTick = DefaultExceptionHandler);

/*
 * note:
 * use the `EXTERN` statement to preserve symbols that NOT directly or
 * indirectly referenced by the entry point.
 */
EXTERN(RESET_VECTOR);
EXTERN(EXCEPTIONS);

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

SECTIONS
{
  /* the stack point is the end address of RAM, i.e. `0x20000000 + 0x5000` */
  __stack_start = ORIGIN(RAM) + LENGTH(RAM);

  /*
   * PM0214 Programming manual
   * 2.3.4 Vector table
   * Vector table 16 standard and 91 STM32-specific exception handlers/functions,
   * size is `0x1ac`
   */
  .vector_table ORIGIN(FLASH) :
  {
    /* first entry: the initial Stack Pointer (SP) value */
    LONG(__stack_start);

    /*
     * second entry: reset handler
     *
     * note:
     * use the `KEEP` statement to force the linker to insert
     * all input sections named, otherwise the linker will
     * discard the unused sections
     */
    KEEP(*(.vector_table.reset_vector));

    /* the next 14 entries are exception vectors */
    KEEP(*(.vector_table.exceptions));

    /* . = 0x080001ac; */
  } > FLASH

  .text : ALIGN(4)
  {
    *(.text .text.*);
  } > FLASH

  .rodata : ALIGN(4)
  {
    *(.rodata .rodata.*);
  } > FLASH

  /*
   * set the Load Memory Address (LMA) of the `.data` section by
   * `AT(...)` or `AT > FLASH`
   */
  .data : ALIGN(4) /* AT(ADDR(.rodata) + SIZEOF(.rodata)) */
  {
    __sdata = .;
    *(.data .data.*);
    __edata = .;
  }  > RAM AT > FLASH

  /* LMA of .data */
  __sidata = LOADADDR(.data);

  .bss : ALIGN(4)
  {
    __sbss = .;
    *(.bss .bss.*);
    __ebss = .;
  } > RAM

  . = ALIGN(4);
  __heap_start = .;

  /* Discarded sections */
  /DISCARD/ :
  {
    *(.ARM.exidx .ARM.exidx.* .ARM.extab.*);
  }
}
/*
******************************************************************************
**
**  File        : LinkerScript.ld (debug in RAM dedicated)
**
**  Author      : STM32CubeIDE
**
**  Abstract    : Linker script for STM32H7 series
**                      1024Kbytes FLASH
**                      800Kbytes RAM
**
**                Set heap size, stack size and stack location according
**                to application requirements.
**
**                Set memory bank area and size if external memory is used.
**
**  Target      : STMicroelectronics STM32
**
**  Distribution: The file is distributed as is without any warranty
**                of any kind.
**
*****************************************************************************
** @attention
**
** Copyright (c) 2024 STMicroelectronics.
** All rights reserved.
**
** This software is licensed under terms that can be found in the LICENSE file
** in the root directory of this software component.
** If no LICENSE file comes with this software, it is provided AS-IS.
**
*****************************************************************************
*/
/* try -ffunction-sections -fdata-sections -Wl,--gc-sections */
/* Entry Point */
ENTRY(Reset_Handler)


  /* Highest address of the user mode stack */
  _estack = ORIGIN(RAM_D1) + LENGTH(RAM_D1); /* end of "RAM_D1" Ram type memory */

  /* Define heap and stack sizes */
  _Min_Heap_Size  = 0x400; /* 1 KB heap */
  _Min_Stack_Size  = 0x800; /* 2 KB stack */
  
/* Memories definition */
MEMORY
{
  RAM_D1 (xrw)   : ORIGIN = 0x24000000, LENGTH =  512K
  FLASH   (rx)   : ORIGIN = 0x08000000, LENGTH = 1024K    /* Memory is divided. Actual start is 0x8000000 and actual length is 2048K */
  DTCMRAM (xrw)  : ORIGIN = 0x20000000, LENGTH = 128K
  RAM_D2 (xrw)   : ORIGIN = 0x30000000, LENGTH = 288K
  /*SRAM1   (xrw)  : ORIGIN = 0x30000000, LENGTH = 192K      SRAM1 is part of RAM_D2 */
  /*SRAM2   (xrw)  : ORIGIN = 0x30030000, LENGTH = 96K       SRAM2 is the rest of RAM_D2 */
  RAM_D3 (xrw)   : ORIGIN = 0x38000000, LENGTH = 64K
  ITCMRAM (xrw)  : ORIGIN = 0x00000000, LENGTH = 64K
}



/* Sections */
SECTIONS
{

  /* The startup code into "RAM" Ram type memory */
  .isr_vector :
  {
    . = ALIGN(4);
    KEEP(*(.isr_vector)) /* Startup code */
    . = ALIGN(4);
  } >RAM_D1

  /* Declare before .data or other initialized sections */
 /* -Wl,--section-start=.dma_rx_buffer1=0x24060000 */ 
 /* -Wl,--section-start=.dma_rx_buffer2=0x30040000 */



  /* The program code and other data into "RAM" Ram type memory */
  .text :
  {
  
    . = ALIGN(4);
    *(.text)           /* .text sections (code) */
    *(.text*)          /* .text* sections (code) */
    *(.text .text.*) /* Includes all function-specific text sections */
    *(.data .data.*) /* Includes all variable-specific data sections */ 
    *(.glue_7)         /* glue arm to thumb code */
    *(.glue_7t)        /* glue thumb to arm code */
    *(.eh_frame)
    *(.RamFunc)        /* .RamFunc sections */
    *(.RamFunc*)       /* .RamFunc* sections */

    KEEP (*(.init))
    KEEP (*(.fini))

    . = ALIGN(4);
    _etext = .;        /* define a global symbols at end of code */
  } >RAM_D1

  /* Constant data into "RAM" Ram type memory */
  .rodata :
  {
    . = ALIGN(4);
    *(.rodata)         /* .rodata sections (constants, strings, etc.) */
    *(.rodata*)        /* .rodata* sections (constants, strings, etc.) */
    . = ALIGN(4);
  } >RAM_D1

  .fast_code : { *(.fast_code*) } > ITCMRAM
  /* to use: __attribute__((section(".fast_code"))) */

  /*
  learn how to use this
  OVERLAY
  {
    .overlay1 : { *(.overlay1_data) } > RAM_D2
    .overlay2 : { *(.overlay2_data) } > RAM_D2
  }

  */
  .ARM.extab (READONLY) : /* The READONLY keyword is only supported in GCC11 and later, remove it if using GCC10 or earlier. */
  {
    . = ALIGN(4);
    *(.ARM.extab* .gnu.linkonce.armextab.*)
    . = ALIGN(4);
  } >RAM_D1
  .ARM (READONLY) : /* The READONLY keyword is only supported in GCC11 and later, remove it if using GCC10 or earlier. */
  {
    . = ALIGN(4);
    __exidx_start = .;
    *(.ARM.exidx*)
    __exidx_end = .;
    . = ALIGN(4);
  } >RAM_D1

  .preinit_array (READONLY) : /* The READONLY keyword is only supported in GCC11 and later, remove it if using GCC10 or earlier. */
  {
    . = ALIGN(4);
    PROVIDE_HIDDEN (__preinit_array_start = .);
    KEEP (*(.preinit_array*))
    PROVIDE_HIDDEN (__preinit_array_end = .);
    . = ALIGN(4);
  } >RAM_D1

  .init_array (READONLY) : /* The READONLY keyword is only supported in GCC11 and later, remove it if using GCC10 or earlier. */
  {
    . = ALIGN(4);
    PROVIDE_HIDDEN (__init_array_start = .);
    KEEP (*(SORT(.init_array.*)))
    KEEP (*(.init_array*))
    PROVIDE_HIDDEN (__init_array_end = .);
    . = ALIGN(4);
  } >RAM_D1

  .fini_array (READONLY) : /* The READONLY keyword is only supported in GCC11 and later, remove it if using GCC10 or earlier. */
  {
    . = ALIGN(4);
    PROVIDE_HIDDEN (__fini_array_start = .);
    KEEP (*(SORT(.fini_array.*)))
    KEEP (*(.fini_array*))
    PROVIDE_HIDDEN (__fini_array_end = .);
    . = ALIGN(4);
  } >RAM_D1

  /* Used by the startup to initialize data */
  _sidata = LOADADDR(.data);

  /* Initialized data sections into "RAM" Ram type memory */
  .data :
  {
    . = ALIGN(4);
    _sdata = .;        /* create a global symbol at data start */
    *(.data)           /* .data sections */
    *(.data*)          /* .data* sections */

    . = ALIGN(4);
    _edata = .;        /* define a global symbol at data end */

  } >RAM_D1


  /* Uninitialized data section into "RAM" Ram type memory */
  . = ALIGN(4);
  .bss :
  {
    /* This is used by the startup in order to initialize the .bss section */
    _sbss = .;         /* define a global symbol at bss start */
    __bss_start__ = _sbss;
    *(.bss)
    *(.bss*)
    *(COMMON)

    . = ALIGN(4);
    _ebss = .;         /* define a global symbol at bss end */
    __bss_end__ = _ebss;
  } >RAM_D1


  
  /* Add ASSERT to ensure we do not exceed D1 memory */
  /* doesn't work ASSERT(__bss_end__ <= ORIGIN(RAM_D1) + LENGTH(RAM_D1), "D1 Overflow Detected"); */


  /* Heap section */
  .user_heap (NOLOAD) :
  {
    . = ALIGN(8);
    _end = .;
    . += _Min_Heap_Size;
    . = ALIGN(8);
  } > RAM_D1
  
   /* Stack section */
  .user_stack (NOLOAD) :
  {
    . = ALIGN(8);
    _stack = .;
    . += _Min_Stack_Size;
    . = ALIGN(8);
  } > RAM_D1

  
  .dma_tx_buffer2  (NOLOAD) : /* 0x30040000 */
  {
	. = ALIGN(32);          /* Ensure 4-byte alignment for the start of the buffer */
    *(.dma_tx_buffer2)      /* Use the .spi_buffers_32 section in code */
    . = ALIGN(32);          /* Maintain 4-byte alignment for subsequent sections */
  }  >RAM_D2 AT>RAM_D2


  .dma_rx_buffer2  (NOLOAD) : /* 0x30040100 */
  {
	. = ALIGN(32);          /* Ensure 4-byte alignment for the start of the buffer */
    *(.dma_rx_buffer2)      /* Use the .spi_buffers_32 section in code */
    . = ALIGN(32);          /* Maintain 4-byte alignment for subsequent sections */
  }  >RAM_D2 AT>RAM_D2



  /* Remove information from the compiler libraries */
  /DISCARD/ :
  {
    libc.a ( * )
    libm.a ( * )
    libgcc.a ( * )
  }

  .ARM.attributes 0 : { *(.ARM.attributes) }
}

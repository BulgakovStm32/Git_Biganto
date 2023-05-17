/*********************************************************************/
/*********************************************************************/
/* Entry Point */
ENTRY(Reset_Handler)

/* Highest address of the user mode stack */
_estack = ORIGIN(RAM) + LENGTH(RAM);	/* end of "RAM" Ram type memory */

_Min_Heap_Size  = 0x0;   /*0x200;	/* required amount of heap  */
_Min_Stack_Size = 0x200; /*0x400;	/* required amount of stack */
/*********************************************************************/
/*********************************************************************/
TARGET_STM32_RAM_SIZE   = 20K;		
TARGET_STM32_FLASH_SIZE = 128K;		

/* Memories definition */
MEMORY
{
	/* STM32 RAM */
	SHARED_RAM	(rw)	: ORIGIN = 0x20000000,								LENGTH = 256
  	RAM			(xrw)	: ORIGIN = ORIGIN(SHARED_RAM) + LENGTH(SHARED_RAM),	LENGTH = TARGET_STM32_RAM_SIZE - LENGTH(SHARED_RAM)
  	
  	/* STM32 FLASH */ 
 	/*Адрес и размер ЗАГРУЗЧИКА*/
 	BOOT	(rx) 	: ORIGIN = 0x08000000,					LENGTH = 9K	
 	/*Адрес и размер МЕТАДАННЫХ*/
 	META 	(r)		: ORIGIN = ORIGIN(BOOT) + LENGTH(BOOT),	LENGTH = 1K	
 	/*Адрес и размер ПРИЛОЖЕНИЯ*/
 	FLASH	(rx)	: ORIGIN = ORIGIN(META) + LENGTH(META),	LENGTH = TARGET_STM32_FLASH_SIZE - LENGTH(BOOT) - LENGTH(META) 
}
/*********************************************************************/
/*********************************************************************/
/* Определелние содержимого секции МЕТАДАННЫХ*/
SECTIONS
{
	.app_metadata :
	{
		KEEP (*(.app_metadata))
	} >META
}
/*********************************************************************/
/*********************************************************************/
/* Определелние содержимого секции памяти FLASH-памяти и ОЗУ ПРИЛОЖЕНИЯ*/
/* Sections */
SECTIONS
{

  /* The startup code into "FLASH" Rom type memory */
  .isr_vector :
  {
    . = ALIGN(4);
    KEEP(*(.isr_vector)) /* Startup code */
    . = ALIGN(4);
  } >FLASH

  /* The program code and other data into "FLASH" Rom type memory */
  .text :
  {
    . = ALIGN(4);
    *(.text)           /* .text sections (code) */
    *(.text*)          /* .text* sections (code) */
    *(.glue_7)         /* glue arm to thumb code */
    *(.glue_7t)        /* glue thumb to arm code */
    *(.eh_frame)

    KEEP (*(.init))
    KEEP (*(.fini))

    . = ALIGN(4);
    _etext = .;        /* define a global symbols at end of code */
  } >FLASH

  /* Constant data into "FLASH" Rom type memory */
  .rodata :
  {
    . = ALIGN(4);
    *(.rodata)         /* .rodata sections (constants, strings, etc.) */
    *(.rodata*)        /* .rodata* sections (constants, strings, etc.) */
    . = ALIGN(4);
  } >FLASH

  .ARM.extab   : {
    . = ALIGN(4);
    *(.ARM.extab* .gnu.linkonce.armextab.*)
    . = ALIGN(4);
  } >FLASH

  .ARM : {
    . = ALIGN(4);
    __exidx_start = .;
    *(.ARM.exidx*)
    __exidx_end = .;
    . = ALIGN(4);
  } >FLASH

  .preinit_array     :
  {
    . = ALIGN(4);
    PROVIDE_HIDDEN (__preinit_array_start = .);
    KEEP (*(.preinit_array*))
    PROVIDE_HIDDEN (__preinit_array_end = .);
    . = ALIGN(4);
  } >FLASH

  .init_array :
  {
    . = ALIGN(4);
    PROVIDE_HIDDEN (__init_array_start = .);
    KEEP (*(SORT(.init_array.*)))
    KEEP (*(.init_array*))
    PROVIDE_HIDDEN (__init_array_end = .);
    . = ALIGN(4);
  } >FLASH

  .fini_array :
  {
    . = ALIGN(4);
    PROVIDE_HIDDEN (__fini_array_start = .);
    KEEP (*(SORT(.fini_array.*)))
    KEEP (*(.fini_array*))
    PROVIDE_HIDDEN (__fini_array_end = .);
    . = ALIGN(4);
  } >FLASH
  
  
	.shared_memory (NOLOAD) :
  	{
   		KEEP(*(.shared_memory))
    } >SHARED_RAM
  
  
  /* Used by the startup to initialize data */
  _sidata = LOADADDR(.data);

  /* Initialized data sections into "RAM" Ram type memory */
  .data :
  {
    . = ALIGN(4);
    _sdata = .;        /* create a global symbol at data start */
    *(.data)           /* .data sections */
    *(.data*)          /* .data* sections */
    *(.RamFunc)        /* .RamFunc sections */
    *(.RamFunc*)       /* .RamFunc* sections */

    . = ALIGN(4);
    _edata = .;        /* define a global symbol at data end */

  } >RAM AT> FLASH

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
  } >RAM

  /* User_heap_stack section, used to check that there is enough "RAM" Ram  type memory left */
  ._user_heap_stack :
  {
    . = ALIGN(8);
    PROVIDE ( end = . );
    PROVIDE ( _end = . );
    . = . + _Min_Heap_Size;
    . = . + _Min_Stack_Size;
    . = ALIGN(8);
  } >RAM

  /* Remove information from the compiler libraries */
  /DISCARD/ :
  {
    libc.a ( * )
    libm.a ( * )
    libgcc.a ( * )
  }

  .ARM.attributes 0 : { *(.ARM.attributes) }
}
/*********************************************************************/
/*********************************************************************/










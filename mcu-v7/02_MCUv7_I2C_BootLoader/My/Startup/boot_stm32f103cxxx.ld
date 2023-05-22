/*********************************************************************/
/*********************************************************************/
/*Точка входа приложения*/
ENTRY(Reset_Handler) 

/* Highest address of the user mode stack */
_estack = ORIGIN(RAM) + LENGTH(RAM);	/* end of "RAM" Ram type memory */

_Min_Heap_Size  = 0;     /*0  байт     /* required amount of heap  */
_Min_Stack_Size = 1024;  /*1024 байта  /* required amount of stack */

/*********************************************************************/
/*********************************************************************/
/* Определение областей FLASH-памяти и ОЗУ */
MEMORY
{	
	SHARED_RAM	(rwx)	: ORIGIN = 0x20000000,			LENGTH = 256
  	RAM   		(rwx)	: ORIGIN = 0x20000000 + 256,	LENGTH = 20K-LENGTH(SHARED_RAM)
  	
	FLASH  		(rx)  	: ORIGIN = 0x08000000,			LENGTH = 10K 
}
/*********************************************************************/
/*********************************************************************/
/* Определелние содержимого секции разделяемой памяти ОЗУ для загрузчика и приложения*/
SECTIONS
{
	.shared_memory (NOLOAD) :
    {
    	KEEP(*(.shared_memory))
    } >SHARED_RAM
}
/*********************************************************************/
/*********************************************************************/
/* Определелние содержимого секции памяти FLASH-памяти и ОЗУ загрузчика*/
SECTIONS
{
	/***************************************************/
	/***************************************************/
	/*Секции, распологаемые во FLASH-памяти */
	 
	/* The startup code into "FLASH" Rom type memory */
  	.isr_vector :
  	{
    	. = ALIGN(4);			/*	С помощью директивы ALIGN(4) мы говорим, что секция будет
    								выровнена по словам (4 Байт) 
    							*/
    							
    	KEEP(*(.isr_vector))	/* 	KEEP(*(.isr_vector)) говорит компо-новщику LD удерживать 
    								таблицу векторов в конечном абсолютном файле, иначе секция
    								может быть «отброшена» другими инструментами, выполняющими
									оптимизацию ко-нечного файла
								*/
								
    	. = ALIGN(4);
  	} >FLASH  					/* директива >FLASH указывает, что секция .text будет помещена во Flash-память*/
  	
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

	.ARM.extab :
	{
    	. = ALIGN(4);
    	*(.ARM.extab* .gnu.linkonce.armextab.*)
    	. = ALIGN(4);
  	} >FLASH

  	.ARM :
  	{
    	. = ALIGN(4);
    	__exidx_start = .;
    	*(.ARM.exidx*)
    	__exidx_end = .;
    	. = ALIGN(4);
  	} >FLASH

  	.preinit_array :
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
	/***************************************************/
	/***************************************************/
	/*
		Секция копирования содержимого области Flash-памяти, содержащей значения
	  	инициализации, в область SRAM, выделенную для глобальных инициализированных
	   	переменных.
	*/
	
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
	/***************************************************/
	/***************************************************/
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
	/***************************************************/
	/***************************************************/

}
/*********************************************************************/
/*********************************************************************/





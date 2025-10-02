/* Linker script for STM32F427 Cortex-M4 (MODM project) */

/* Memory regions */
MEMORY
{
    FLASH  (rx)  : ORIGIN = 0x08000000, LENGTH = 512K
    RAM    (rwx) : ORIGIN = 0x20000000, LENGTH = 128K
    CCMRAM (rwx) : ORIGIN = 0x10000000, LENGTH = 64K  /* Core Coupled Memory */
}
/* You may want to add a default ENTRY(Reset_Handler) and OUTPUT_FORMAT here */

SECTIONS
{
    /* ================== 1. VECTOR & TEXT (FLASH) ================== */
    .isr_vector :
    {
        KEEP(*(.isr_vector))
    } > FLASH

    .text :
    {
        __text_start = .;
        *(*.text*)
        *(*.rodata*)
        __text_end = .;
        . = ALIGN(4);
    } > FLASH

    /* ================== 2. STARTUP TABLES (FLASH) ================== */
    /* These tables contain addresses for the startup routine (LMA) and must be stored in FLASH. */
    .tables :
    {
        . = ALIGN(4);
        
        __table_heap_start = .;
        KEEP(*(.heap_table*))
        __table_heap_end = .;

        __table_copy_extern_start = .;
        KEEP(*(.table_copy_extern*))
        __table_copy_extern_end = .;

        __table_copy_intern_start = .;
        KEEP(*(.table_copy_intern*))
        __table_copy_intern_end = .;
        
        __table_zero_extern_start = .;
        KEEP(*(.table_zero_extern*))
        __table_zero_extern_end = .;

        __table_zero_intern_start = .;
        KEEP(*(.table_zero_intern*))
        __table_zero_intern_end = .;
        
        . = ALIGN(4);
    } > FLASH

    /* ================== 3. INITIALIZED DATA (RAM, loads after TABLES) ================== */
    /* LMA: ADDR(.tables) + SIZEOF(.tables) */
    /* VMA: RAM */
    .data : AT(ADDR(.tables) + SIZEOF(.tables))
    {
        __data_start = .;
        *(*.data*)
        __data_end = .;
        . = ALIGN(4);
    } > RAM

    /* ================== 4. UNINITIALIZED DATA (VMA only) ================== */
    
    .bss (NOLOAD) :
    {
        __bss_start = .;
        *(*.bss*)
        __bss_end = .;
        . = ALIGN(4);
    } > RAM

    .bss_ccm (NOLOAD) :
    {
        __bss_ccm_start = .;
        *(*.bss_ccm*)
        __bss_ccm_end = .;
        . = ALIGN(4);
    } > CCMRAM

    /* ================== 5. STACK (VMA only) ================== */
    .stack (NOLOAD) :
    {
        __stack_start__ = .;
        /* Start at the top of RAM and grow downwards */
        . = ORIGIN(RAM) + LENGTH(RAM);
        __stack_end__ = .;
        PROVIDE(__StackTop = __stack_end__); /* Common symbol for startup */
    } > RAM
    
    /* REMOVED: .heap_table, .copy_table, .zero_table from here. 
       They are correctly defined in FLASH as part of the loading image (.tables). */
}

/* End of script */
/* Linker script for Cortex-M4, STM32 / modm project */

/* Memory regions */
MEMORY
{
    FLASH (rx) : ORIGIN = 0x08000000, LENGTH = 512K
    RAM   (rwx) : ORIGIN = 0x20000000, LENGTH = 128K
    CCMRAM (rwx) : ORIGIN = 0x10000000, LENGTH = 64K  /* Core Coupled Memory */
}

/* Sections */
SECTIONS
{
    /* ================== VECTOR & TEXT ================== */
    .isr_vector :
    {
        KEEP(*(.isr_vector))
    } > FLASH

    .text :
    {
        __text_start = .;
        *(.text*)
        *(.rodata*)
        *(.rodata*)
        __text_end = .;
        . = ALIGN(4);
    } > FLASH

    /* ================== INITIALIZED DATA ================== */
    .data : AT(ADDR(.text) + SIZEOF(.text))
    {
        __data_start = .;
        *(.data*)
        __data_end = .;
        . = ALIGN(4);
    } > RAM

    /* ================== ZEROED BSS ================== */
    .bss (NOLOAD) :
    {
        __bss_start = .;
        *(.bss*)
        __bss_end = .;
        . = ALIGN(4);
    } > RAM

    /* ================== CCM BSS ================== */
    .bss_ccm (NOLOAD) :
    {
        __bss_ccm_start = .;
        *(.bss_ccm*)
        __bss_ccm_end = .;
        . = ALIGN(4);
    } > CCMRAM

    /* ================== STACK ================== */
    .stack (NOLOAD) :
    {
        __stack_start__ = .;
        . = ORIGIN(RAM) + LENGTH(RAM);
        __stack_end__ = .;
    } > RAM

    /* ================== HEAP TABLE ================== */
    .heap_table (NOLOAD) :
    {
        __table_heap_start = .;
        KEEP(*(.heap_table*))
        __table_heap_end = .;
    } > RAM

    /* ================== COPY TABLES ================== */
    .copy_table :
    {
        __table_copy_extern_start = .;
        KEEP(*(.table_copy_extern*))
        __table_copy_extern_end = .;

        __table_copy_intern_start = .;
        KEEP(*(.table_copy_intern*))
        __table_copy_intern_end = .;
    } > RAM

    /* ================== ZERO TABLES ================== */
    .zero_table :
    {
        __table_zero_extern_start = .;
        KEEP(*(.table_zero_extern*))
        __table_zero_extern_end = .;

        __table_zero_intern_start = .;
        KEEP(*(.table_zero_intern*))
        __table_zero_intern_end = .;
    } > RAM
}

/* End of script */

/* Linker script for STM32F407 Cortex-M4 (MODM project) */

/* ================== MEMORY REGIONS ================== */
MEMORY
{
    FLASH      (rx)  : ORIGIN = 0x08000000, LENGTH = 512K
    CCM        (rwx) : ORIGIN = 0x10000000, LENGTH = 64K
    SRAM1      (rwx) : ORIGIN = 0x20000000, LENGTH = 114688
    SRAM2      (rwx) : ORIGIN = 0x2001C000, LENGTH = 16K
    BACKUP     (rwx) : ORIGIN = 0x40024000, LENGTH = 4K
    CONT_SRAM1 (rwx) : ORIGIN = 0x20000000, LENGTH = 131072
}

/* Define the base load address for all initialized data (just after .text) */
__data_load_base = ORIGIN(FLASH) + SIZEOF(.isr_vector) + SIZEOF(.text);
/* Note: SIZEOF(.text) includes .isr_vector and .text from the previous definitions.
 * A safer, more explicit approach is used below.
 */

/* ================== SECTIONS ================== */
SECTIONS
{
    /* ------------------ VECTOR TABLE ------------------ */
    .isr_vector :
    {
        KEEP(*(.isr_vector))
    } > FLASH

    /* ------------------ TEXT & RODATA ----------------- */
    .text :
    {
        __text_start = .;
        *(.text*)
        *(.rodata*)
        __text_end = .;
        . = ALIGN(4);
    } > FLASH

    /* Define a pointer to the load address of the first data section */
    __data_load_start = ADDR(.text) + SIZEOF(.text);

    /* ------------------ INITIALIZED DATA --------------- */
    /* FIRST data section: load at __data_load_start */
    .data : AT(__data_load_start)
    {
        __data_start = .;
        *(.data*)
        __data_end = .;
        . = ALIGN(4);
    } > CONT_SRAM1

    /* ------------------ FAST DATA IN CCM ------------------ */
    /* SECOND data section: load immediately after the previous section's load image (.data) */
    .fastdata : AT(LOADADDR(.data) + SIZEOF(.data))
    {
        __fastdata_load = LOADADDR(.fastdata);
        __fastdata_start = .;
        *(.fastdata*)
        . = ALIGN(4);
        __fastdata_end = .;
    } > CCM

    /* ------------------ DATA IN CCM ------------------ */
    /* THIRD data section: load immediately after the previous section's load image (.fastdata) */
    .data_ccm : AT(LOADADDR(.fastdata) + SIZEOF(.fastdata))
    {
        __data_ccm_start = .;
        *(.data_ccm*)
        . = ALIGN(4);
        __data_ccm_end = .;
    } > CCM
    
    /* ------------------ ZEROED BSS (NOLOAD) -------------------- */
    .bss (NOLOAD) :
    {
        __bss_start = .;
        *(.bss*)
        __bss_end = .;
        . = ALIGN(4);
    } > CONT_SRAM1

    /* ------------------ CCM BSS (NOLOAD) ----------------------- */
    .bss_ccm (NOLOAD) :
    {
        __bss_ccm_start = .;
        *(.bss_ccm*)
        __bss_ccm_end = .;
        . = ALIGN(4);
    } > CCM

    /* The remaining NOLOAD sections do not have load images in FLASH,
     * so their definitions are fine as is, relative to their RUN address.
     * The table sections at the end are also fine, as they are implicitly loaded
     * as part of the last preceding loadable section's load address.
     */

    /* The rest of the sections (copy_table, zero_table, heap, stack, noinit)
       remain unchanged, as they are either NOLOAD or the linker implicitly
       places the subsequent non-NOLOAD sections correctly after the last AT section.
       I will move the copy and zero tables up next to the other initialized data for clarity
       and to ensure they are loaded correctly if they contain non-BSS data.
    */
    
    /* The tables are often initialized data, so placing them where the linker
       can correctly calculate their load address (after the last AT section)
       is critical. Let's place them *after* the last AT section (.data_ccm)
       and rely on the linker to calculate their LOADADDR implicitly. */
       
    /* ------------------ COPY TABLES ------------------- */
    .copy_table :
    {
        __table_copy_extern_start = .;
        KEEP(*(.table_copy_extern*))
        __table_copy_extern_end = .;

        __table_copy_intern_start = .;
        KEEP(*(.table_copy_intern*))
        __table_copy_intern_end = .;
    } > CONT_SRAM1

    /* ------------------ ZERO TABLES ------------------- */
    .zero_table :
    {
        __table_zero_extern_start = .;
        KEEP(*(.table_zero_extern*))
        __table_zero_extern_end = .;

        __table_zero_intern_start = .;
        KEEP(*(.table_zero_intern*))
        __table_zero_intern_end = .;
    } > CONT_SRAM1
    
    /* ------------------ HEAP TABLE -------------------- */
    /* This is usually only referenced, so NOLOAD is appropriate */
    .heap_table (NOLOAD) :
    {
        __table_heap_start = .;
        KEEP(*(.heap_table*))
        __table_heap_end = .;
    } > CONT_SRAM1
    
    /* ------------------ STACK (NOLOAD) ------------------------- */
    .stack (NOLOAD) :
    {
        __stack_start__ = .;
        . = ORIGIN(CONT_SRAM1) + LENGTH(CONT_SRAM1);
        __stack_end__ = .;
    } > CONT_SRAM1

    /* ------------------ SRAM2 HEAP (NOLOAD) ------------------- */
    .heap_sram2 (NOLOAD) :
    {
        __heap_sram2_start = ORIGIN(SRAM2);
        __heap_sram2_end = ORIGIN(SRAM2) + LENGTH(SRAM2);
    } > SRAM2

    /* ------------------ BACKUP HEAP (NOLOAD) ------------------ */
    .heap_backup (NOLOAD) :
    {
        __heap_backup_start = ORIGIN(BACKUP);
        __heap_backup_end = ORIGIN(BACKUP) + LENGTH(BACKUP);
    } > BACKUP

    /* ------------------ NOINIT SECTIONS (NOLOAD) ------------------ */
    .noinit_sram1 (NOLOAD) :
    {
        __noinit_sram1_start = .;
        *(.noinit*)
        . = ALIGN(4);
        __noinit_sram1_end = .;
    } > CONT_SRAM1

    .noinit_sram2 (NOLOAD) :
    {
        __noinit_sram2_start = ORIGIN(SRAM2);
        *(.noinit_sram2*)
        . = ALIGN(4);
        __noinit_sram2_end = .;
    } > SRAM2

    .noinit_ccm (NOLOAD) :
    {
        __noinit_ccm_start = ORIGIN(CCM);
        *(.noinit_ccm*)
        . = ALIGN(4);
        __noinit_ccm_end = .;
    } > CCM

    .noinit_backup (NOLOAD) :
    {
        __noinit_backup_start = ORIGIN(BACKUP);
        *(.noinit_backup*)
        . = ALIGN(4);
        __noinit_backup_end = .;
    } > BACKUP
    
    /* ------------------ FAST DATA IN CCM ------------------ */
    .fastdata : AT(ADDR(.text) + SIZEOF(.text))
    {
        __fastdata_load = LOADADDR(.fastdata);
        __fastdata_start = .;
        *(.fastdata*)
        . = ALIGN(4);
        __fastdata_end = .;
    } > CCM

    /* ------------------ DATA IN CCM ------------------ */
    .data_ccm : AT(ADDR(.text) + SIZEOF(.text))
    {
        __data_ccm_start = .;
        *(.data_ccm*)
        . = ALIGN(4);
        __data_ccm_end = .;
    } > CCM
}

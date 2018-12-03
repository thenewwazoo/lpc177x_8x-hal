MEMORY
{
        /* Main ROM region - 512k for LPC1788 */
        FLASH (rx) : ORIGIN = 0x00000000, LENGTH = 512k

        /* local static RAM - 63k for LPC1788 */
        RAM (rwx) : ORIGIN = 0x10000000, LENGTH = 63k

        /* AHB SRAM - 256k for LPC1788 - often used for USB, unused here */
        AHBSRAM (rwx) : ORIGIN = 0x20000000, LENGTH = 256k

        /* External SDRAM - 32MB for LPC1788 */
        SDRAM (rwx) : ORIGIN = 0xA0000000, LENGTH = 32M
}

_stack_start = ORIGIN(RAM) + LENGTH(RAM) - 32;

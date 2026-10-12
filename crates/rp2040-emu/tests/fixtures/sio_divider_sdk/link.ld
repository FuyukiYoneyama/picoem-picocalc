ENTRY(_start)
SECTIONS {
    . = 0x20000000;
    .text : { *(.entry) *(.text*) }
}

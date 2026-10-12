.syntax unified
.cpu cortex-m0plus
.thumb
.section .entry,"ax"
.global _start
.type _start,%function
.thumb_func
_start:
    // Mailbox: numerator[2], denominator[2], signed selector.
    ldr r4, =0x20030000
    ldmia r4!, {r0-r3}
    ldr r5, [r4]
    cmp r5, #0
    beq 1f
    ldr r5, =__aeabi_ldivmod
    b 2f
1:
    ldr r5, =__aeabi_uldivmod
2:
    blx r5
    // Quotient[2], remainder[2], completion marker.
    ldr r4, =0x20030020
    stmia r4!, {r0-r3}
    movs r0, #1
    str r0, [r4]
3:
    wfe
    b 3b
.ltorg

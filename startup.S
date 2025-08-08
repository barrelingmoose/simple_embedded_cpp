.syntax unified
.thumb
.cpu cortex-m3

.section .isr_vector, "a", %progbits
.global _isr_vector
isr_vector:
    .word _stack_top
    .word Reset_Handler
    .word NMI_Handler
    .word HardFault_Handler
    .word 0
    .word 0
    .word 0
    .word 0
    .word 0
    .word 0
    .word 0
    .word SVC_Handler
    .word 0
    .word 0
    .word PendSV_Handler
    .word SysTick_Handler
    .space (0x40 - 4*4)

.size isr_vector, . - isr_vector

.section .text.Reset_Handler, "ax", %progbits
.global Reset_Handler
.type Reset_Handler, %function 
Reset_Handler:
    bl main
    
.LoopForever:
    b .LoopForever

.size Reset_Handler, . - Reset_Handler

.section .text.NMI_Handler, "ax", %progbits
.global NMI_Handler
.type NMI_Handler, %function
NMI_Handler:
    b .

.section .text.HardFault_Handler, "ax", %progbits
.global HardFault_Handler
.type HardFault_Handler, %function
HardFault_Handler: 
    b .

.section .stack, "a", %progbits
.global _stack_top
_stack_top = 0x20002000

.thumb_func
.global PendSV_Handler
PendSV_Handler:
    b xPortPendSVHandler

.thumb_func
.global SysTick_Handler
SysTick_Handler:
    b xPortSysTickHandler

.thumb_func
.global SVC_Handler
SVC_Handler:
    b vPortSVCHandler
# STM32VL QEMU Embedded Project Documentation

## Overview

This document covers a minimal embedded C++ project targeting the STM32VLDiscovery (STM32F100RB Cortex-M3) board, simulated using QEMU. It walks through the source code, linker scripts, build system, and simulation steps, with detailed line-by-line explanations, diagrams, and references.

## Memory Map Diagram

## Interrupt Vector Table Layout

## USART1 Peripheral Register Block

## GPIO Pin Configuration Bits

## Semihosting Flow Overview


## main.cpp – Line-by-Line Explanation

This file simulates an embedded "Hello, World" style message print using semihosting on STM32VLDiscovery in QEMU.

```cpp
extern "C" void _exit(int status) {
    asm volatile (
        "mov r0, %0\n"
        "mov r1, #0x20026\n"
        "bkpt 0xAB"
        : : "r" (status) : "r0", "r1"
    );
    while (1) {}
}
```
- `extern "C"` ensures the C++ compiler doesn’t mangle the function name.
- `asm volatile`: inlines ARM assembly to interact with QEMU semihosting.
- `r0` is loaded with the exit status, and `r1` with 0x20026 (semihosting syscall for `SYS_EXIT`).
- `bkpt 0xAB`: triggers the QEMU semihosting handler.
- Infinite loop ensures no execution past semihosting call.

```cpp
extern "C" void main() {
    const char msg[] = "Hello from STM32 in QEMU!\n";
    asm volatile (
        "mov r0, #1\n"
        "ldr r1, =msg\n"
        "ldr r2, =len\n"
        "mov r3, #0\n"
        "bkpt 0xAB"
    );
    _exit(0);
}
```
- `r0 = 1`: semihosting file descriptor for stdout.
- `r1 = &msg`: address of the message to print.
- `r2 = &len`: length of the message.
- `bkpt 0xAB`: semihosting call again (likely `SYS_WRITE`).
- Then it exits cleanly.

This illustrates a minimal, freestanding C++ application running on bare-metal ARM, printing a message via QEMU semihosting without using `libstdc++`.



## linker.ld – Line-by-Line Explanation

## startup.s – Line-by-Line Explanation

## Makefile – Line-by-Line Explanation

## Bit Manipulation Explained

## Assembly Instruction Breakdown

## Linker and .data Copy Behavior

## UART Baud Rate Calculation

## QEMU Tips and Semihosting Use

## Build and Run Instructions

## References
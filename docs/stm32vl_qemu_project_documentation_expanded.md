
# STM32VLDiscovery Bare-Metal C++ Project on QEMU

---

## main.cpp

```cpp
#define USART1_BASE 0x40013800
#define USART1_SR   (*(volatile unsigned int*)(USART1_BASE + 0x00))
#define USART1_DR   (*(volatile unsigned int*)(USART1_BASE + 0x04))

#define RCC_BASE    0x40021000
#define RCC_APB2ENR (*(volatile unsigned int*)(RCC_BASE + 0x18))

#define GPIOA_BASE  0x40010800
#define GPIOA_CRH   (*(volatile unsigned int*)(GPIOA_BASE + 0x04))
```

- **Peripheral base addresses:** These come from STM32F100RB memory map (RM0008).  
  - `USART1_BASE` is the fixed address of USART1 peripheral registers.  
  - The registers (`SR`, `DR`) are offsets 0x00 and 0x04 from base.  
- **Volatile keyword:** Prevents compiler optimizations since these are hardware registers that may change independently.  
- The cast `(volatile unsigned int*)` treats the address as a pointer to a 32-bit volatile unsigned integer.

---

```cpp
extern "C" void semihosting_exit(int code) {
    asm volatile (
        "mov r0, #0x18
"    // SYS_EXIT call number for semihosting
        "mov r1, %0
"       // Move exit code into r1
        "bkpt 0xAB
"        // Breakpoint to invoke semihosting
        :
        : "r" (code)
        : "r0", "r1"
    );
}
```

- **Semihosting exit:**  
  - `mov r0, #0x18` places the SYS_EXIT operation code in register r0.  
  - `mov r1, %0` moves the exit code argument into r1.  
  - `bkpt 0xAB` is a software breakpoint that triggers semihosting in QEMU.  
- When QEMU detects this breakpoint, it stops simulation and returns the exit code to the host.

---

```cpp
extern "C" void uart_init() {
    RCC_APB2ENR |= (1 << 2);   // Enable GPIOA clock (bit 2)
    RCC_APB2ENR |= (1 << 14);  // Enable USART1 clock (bit 14)

    GPIOA_CRH &= ~(0xF << 4);  // Clear bits 4..7 for PA9 config
    GPIOA_CRH |=  (0xB << 4);  // Set PA9 to AF push-pull, 50 MHz

    *(volatile unsigned int*)(USART1_BASE + 0x08) = 0x341;  // Baud rate register

    *(volatile unsigned int*)(USART1_BASE + 0x0C) = (1 << 3) | (1 << 13);  // TE + UE
}
```

- **Clock enabling:**  
  - RCC_APB2ENR controls clocks for APB2 peripherals.  
  - Bit 2 enables GPIOA peripheral clock; bit 14 enables USART1 clock.  
- **GPIO configuration:**  
  - GPIOA_CRH configures pins 8–15 (4 bits per pin).  
  - PA9 uses bits 4–7 (`4 * (pin - 8)`).  
  - `0xF << 4` mask clears these bits.  
  - `0xB << 4` sets:  
    - MODE9 = 0b11 (Output mode, max 50 MHz)  
    - CNF9 = 0b10 (Alternate function push-pull)  
- **USART Baud rate register (BRR):**  
  - Setting `0x341` corresponds to 9600 baud at 8 MHz clock (per STM32 manual formula).  
- **USART control register (CR1):**  
  - Bit 3 (TE) enables transmitter.  
  - Bit 13 (UE) enables USART peripheral.

---

```cpp
extern "C" void uart_send(char c) {
    while (!(USART1_SR & (1 << 7)));  // Wait for TXE (Transmit data register empty)
    USART1_DR = c;                    // Write char to data register
}
```

- **TXE flag:** Bit 7 of USART_SR indicates transmit buffer ready.  
- Loop waits until this bit is set, ensuring previous data sent.  
- Writing to USART_DR sends the character.

---

```cpp
extern "C" void uart_send_string(const char* str) {
    while (*str) {
        uart_send(*str++);
    }
}
```

- Sends null-terminated string byte-by-byte using `uart_send`.

---

```cpp
extern "C" void main() {
    uart_init();
    uart_send_string("Hello from STM32VL in QEMU!\n");
    semihosting_exit(0);
}
```

- Initializes UART, sends greeting, then calls semihosting exit to stop QEMU cleanly.

---

# linker.ld

```ld
ENTRY(Reset_Handler)

MEMORY
{
  FLASH (rx) : ORIGIN = 0x08000000, LENGTH = 128K
  RAM (rwx)  : ORIGIN = 0x20000000, LENGTH = 8K
}

SECTIONS
{
  .isr_vector :
  {
    KEEP(*(.isr_vector))
  } > FLASH

  .text :
  {
    *(.text*)
    *(.rodata*)
  } > FLASH

  .data :
  {
    *(.data*)
  } > RAM AT > FLASH

  .bss :
  {
    *(.bss*)
    *(COMMON)
  } > RAM
}
```

- **ENTRY:** Defines reset vector entry point symbol.  
- **MEMORY:** Defines memory regions used by the MCU: FLASH (code) and RAM (data).  
- **SECTIONS:** Assign code and data sections to memories:  
  - `.isr_vector`: interrupt vectors placed in FLASH, `KEEP` prevents removal.  
  - `.text`: code and read-only data in FLASH.  
  - `.data`: initialized variables stored in RAM at runtime but copied from FLASH on startup (`AT > FLASH`).  
  - `.bss`: zero-initialized variables in RAM.

---

# startup.s

```asm
.syntax unified
.cpu cortex-m3
.thumb

.section .isr_vector, "a", %progbits
.global _isr_vector
_isr_vector:
  .word _stack_top          /* Initial stack pointer */
  .word Reset_Handler       /* Reset vector */
  .word NMI_Handler         /* NMI Handler */
  .word HardFault_Handler   /* HardFault Handler */

.size _isr_vector, . - _isr_vector

.section .text.Reset_Handler, "ax", %progbits
.global Reset_Handler
.type Reset_Handler, %function
Reset_Handler:
  /* Normally initialize .data and .bss here if needed */

  bl main      /* Call main() */

.LoopForever:
  b .LoopForever  /* Infinite loop if main returns */

.size Reset_Handler, . - Reset_Handler

.section .text.NMI_Handler, "ax", %progbits
.global NMI_Handler
.type NMI_Handler, %function
NMI_Handler:
  b .  /* Loop forever */

.section .text.HardFault_Handler, "ax", %progbits
.global HardFault_Handler
.type HardFault_Handler, %function
HardFault_Handler:
  b .  /* Loop forever */

.section .stack, "a", %progbits
.global _stack_top
_stack_top = 0x20002000  /* End of 8KB RAM (0x20000000 + 8KB) */
```

- **`.isr_vector`** section contains the interrupt vector table:  
  - First word is initial stack pointer loaded into SP on reset.  
  - Second word is the address of the Reset_Handler (entry).  
  - Next are NMI and HardFault handlers.  
- **Reset_Handler:** Calls `main()`, then loops forever if main returns.  
- **Handlers:** Minimal infinite loops to catch exceptions.  
- **Stack top symbol:** Set to end of SRAM memory region (8KB after base).

---

# Makefile

```makefile
CC=arm-none-eabi-gcc
CXX=arm-none-eabi-g++
AS=arm-none-eabi-as
LD=arm-none-eabi-gcc

CFLAGS=-mcpu=cortex-m3 -mthumb -nostdlib -Wall -O2
LDFLAGS=-T linker.ld -nostdlib

all: firmware.elf

startup.o: startup.s
	$(AS) $(CFLAGS) -o $@ $<

main.o: main.cpp
	$(CXX) $(CFLAGS) -c $< -o $@

firmware.elf: startup.o main.o linker.ld
	$(LD) $(LDFLAGS) startup.o main.o -o firmware.elf

clean:
	rm -f *.o firmware.elf
```

- **Toolchain commands:** Uses ARM GCC toolchain for Cortex-M3.  
- **Flags:**  
  - `-mcpu=cortex-m3` targets the Cortex-M3 CPU.  
  - `-mthumb` compiles Thumb instructions.  
  - `-nostdlib` disables linking standard libs (bare metal).  
  - `-Wall` enables warnings, `-O2` optimizes code.  
- **Targets:**  
  - Assemble startup.s.  
  - Compile main.cpp.  
  - Link both with custom linker script.  
  - Clean removes generated files.

---

# Build and Run commands

```bash
arm-none-eabi-as -mcpu=cortex-m3 -mthumb -o startup.o startup.s
arm-none-eabi-g++ -mcpu=cortex-m3 -mthumb -nostdlib -c main.cpp -o main.o
arm-none-eabi-gcc -mcpu=cortex-m3 -mthumb -nostdlib -T linker.ld startup.o main.o -o firmware.elf
qemu-system-arm -M stm32vldiscovery -kernel firmware.elf -serial stdio -semihosting
```

- Assemble, compile, link.  
- Run in QEMU with STM32VLDiscovery machine, serial output on console, semihosting enabled.

---

# QEMU Notes

- Use only one `-serial stdio` or QEMU will error out.  
- `-nographic` disables GUI and redirects serial to stdio (careful if using multiple serial devices).  
- `bkpt 0xAB` triggers semihosting calls like exit.  
- Semihosting enables communication between target and host via breakpoints.

---

# References (at end)

- STM32F100RB Reference Manual (RM0008):  
  https://www.st.com/resource/en/reference_manual/cd00171190-stm32f100-advanced-armbased-32bit-mcus-stmicroelectronics.pdf

- STM32VLDiscovery User Manual (UM0462):  
  https://www.st.com/resource/en/user_manual/dm00028604-stm32-value-line-discovery-kit-stmicroelectronics.pdf

- STM32F100xB Datasheet:  
  https://www.st.com/resource/en/datasheet/stm32f100rb.pdf

- QEMU STM32VLDiscovery Board Source:  
  https://git.qemu.org/?p=qemu.git;a=tree;f=target/arm/boards

- ARM Cortex-M3 Technical Reference Manual:  
  https://developer.arm.com/documentation/ddi0337/latest/

- Arm Semihosting Spec:  
  https://developer.arm.com/documentation/den0028/latest/

- STM32CubeF1 Firmware Package:  
  https://www.st.com/en/embedded-software/stm32cubef1.html

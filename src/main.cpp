#include "FreeRTOS.h"
#include "task.h"
#define USART1_BASE 0x40013800
#define USART1_SR   (*(volatile unsigned int*)(USART1_BASE + 0x00))
#define USART1_DR   (*(volatile unsigned int*)(USART1_BASE + 0x04))

#define RCC_BASE    0x40021000
#define RCC_APB2ENR (*(volatile unsigned int*)(RCC_BASE + 0x18))

#define GPIOA_BASE  0x40010800
#define GPIOA_CRH   (*(volatile unsigned int*)(GPIOA_BASE + 0x04))

// QEMU semihosting SYS_EXIT call
extern "C" void semihosting_exit(int code) {
    asm volatile (
        "mov r0, #0x18\n"    // SYS_EXIT
        "mov r1, %0\n"
        "bkpt 0xAB\n"
        :
        : "r" (code)
        : "r0", "r1"
    );
}

extern "C" void uart_init() {
    RCC_APB2ENR |= (1 << 2); // Enable GPIOA
    RCC_APB2ENR |= (1 << 14); // Enable USART1

    // Configure PA9 (TX) as alternate function push-pull
    GPIOA_CRH &= ~(0xF << 4);  // Clear bits for pin 9
    GPIOA_CRH |=  (0xB << 4);  // 1011: output mode, AF push-pull

    // Set baud rate (e.g., 9600 assuming 8MHz PCLK2)
    *(volatile unsigned int*)(USART1_BASE + 0x08) = 0x341; // BRR
    *(volatile unsigned int*)(USART1_BASE + 0x0C) = (1 << 3) | (1 << 13); // TE + UE
}

extern "C" void uart_send(char c) {
    while (!(USART1_SR & (1 << 7)));  // Wait for TXE
    USART1_DR = c;
}

extern "C" void uart_task(void *pvParameters) {
    const char* str = "Hello from STM32VL in QEMU!\n";
    while (*str) {
        uart_send(*str++);
    }
    semihosting_exit(1);
    while(1);
}

int main() {
    uart_init();
    xTaskCreate(uart_task, "HelloWorld", 128, nullptr, 1, nullptr);
    vTaskStartScheduler();
    // Exit QEMU
}

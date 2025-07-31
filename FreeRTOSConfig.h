#define configCPU_CLOCK_HZ        ( 8000000UL )   // Assume 8 MHz for STM32
#define configTICK_RATE_HZ        ( 1000 )
#define configMAX_PRIORITIES      ( 5 )
#define configMINIMAL_STACK_SIZE  ( 128 )
#define configTOTAL_HEAP_SIZE     ( 4 * 1024 )

#define configUSE_PREEMPTION      1
#define configUSE_IDLE_HOOK       0
#define configUSE_TICK_HOOK       0
#define configUSE_16_BIT_TICKS    0

#define configUSE_MUTEXES         1
#define configUSE_COUNTING_SEMAPHORES 1
#define configUSE_RECURSIVE_MUTEXES   1

#define configUSE_NEWLIB_REENTRANT    0
#define configUSE_TIME_SLICING        1

#define configKERNEL_INTERRUPT_PRIORITY         255
#define configMAX_SYSCALL_INTERRUPT_PRIORITY    191
#define configLIBRARY_KERNEL_INTERRUPT_PRIORITY 15

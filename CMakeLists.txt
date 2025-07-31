cmake_minimum_required(VERSION 3.16)
project(STM32_FreeRTOS C CXX ASM)

# Set the target chip
set(CPU "-mcpu=cortex-m3")
set(FPU "")
set(FLOAT_ABI "")
set(MCU "${CPU} -mthumb")

# Set FreeRTOS Directory 
set(FreeRTOS_DIR "~/git_repos/FreeRTOS")

# Toolchain setup
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)

# Entry point
set(LINKER_SCRIPT ${CMAKE_SOURCE_DIR}/linker.ld)

# Flags
set(CMAKE_C_FLAGS "${MCU} -ffreestanding -fdata-sections -ffunction-sections -Os -g3 -Wall -Wextra")
set(CMAKE_CXX_FLAGS "${CMAKE_C_FLAGS} -fno-rtti -fno-exceptions -fno-threadsafe-statics")
set(CMAKE_ASM_FLAGS "${MCU} -x assembler-with-cpp")
set(CMAKE_EXE_LINKER_FLAGS "-T ${LINKER_SCRIPT} -nostartfiles -Wl,--gc-sections -Wl,-Map=${PROJECT_NAME}.map")

# Output format
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

# Source files
file(GLOB_RECURSE FREERTOS_SRC
    "${FreeRTOS_DIR}/FreeRTOS/Source/*.c"
    "${FreeRTOS_DIR}/FreeRTOS/Source/portable/GCC/ARM_CM3/*.c"
)

add_executable(${PROJECT_NAME}
    startup.s
    main.cpp
    ${FREERTOS_SRC}
)

# Include directories
target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_SOURCE_DIR}
    ${FreeRTOS_DIR}/FreeRTOS/Source/include
    ${FreeRTOS_DIR}/FreeRTOS/Source/portable/GCC/ARM_CM3
)

# Linker script as a source for dependency tracking
set_source_files_properties(${LINKER_SCRIPT} PROPERTIES GENERATED FALSE EXTERNAL_OBJECT TRUE)

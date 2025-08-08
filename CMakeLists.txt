cmake_minimum_required(VERSION 3.21)
project(simple_embedded LANGUAGES C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Specify the path to your linker script
set(LINKER_SCRIPT ${CMAKE_SOURCE_DIR}/linker.ld)

# Toolchain should be passed via command line or set in a separate file
# For example: cmake -DCMAKE_TOOLCHAIN_FILE=toolchain-arm-none-eabi.cmake ..

enable_language(C CXX ASM)

# === Fetch FreeRTOS ===
include(FetchContent)

FetchContent_Declare(
  freertos_kernel
  GIT_REPOSITORY https://github.com/FreeRTOS/FreeRTOS-Kernel.git
  GIT_TAG        main
)

add_library(freertos_config INTERFACE)

target_include_directories(freertos_config SYSTEM INTERFACE "${PROJECT_SOURCE_DIR}/include")

target_compile_definitions(freertos_config INTERFACE projCOVERAGE_TEST=0)

set( FREERTOS_HEAP "4" CACHE STRING "" FORCE)
# Select the native compile PORT
set( FREERTOS_PORT "GCC_POSIX" CACHE STRING "" FORCE)
# Select the cross-compile PORT
if (CMAKE_CROSSCOMPILING)
  set(FREERTOS_PORT "GCC_ARM_CM3" CACHE STRING "" FORCE)
endif()

FetchContent_MakeAvailable(freertos_kernel)

target_compile_definitions(freertos_config INTERFACE ${definitions})
target_compile_options(freertos_config INTERFACE ${options})

# === FreeRTOS kernel source files ===
set(FREERTOS_PORT_DIR ${freertos_kernel_SOURCE_DIR}/portable/GCC/ARM_CM3_MPU)

file(GLOB_RECURSE FREERTOS_SOURCES
    "${freertos_kernel_SOURCE_DIR}/*.c"
    "${FREERTOS_PORT_DIR}/*.c"
)

# Set startup and main sources
set(SOURCES
    ${CMAKE_SOURCE_DIR}/src/main.cpp
    ${CMAKE_SOURCE_DIR}/startup.S
)


# === Define the executable ===
add_executable(freertos_app
    ${SOURCES}
    ${FREERTOS_SOURCES}
)

# === Includes ===
target_include_directories(freertos_app PRIVATE
    ${CMAKE_SOURCE_DIR}/include
    ${freertos_kernel_SOURCE_DIR}/include
    ${FREERTOS_PORT_DIR}
)

# === Linker script ===
target_link_options(freertos_app PRIVATE
    "-T${LINKER_SCRIPT}"
    -Wl,--gc-sections
    -nostdlib
)

# === Compiler flags ===
target_compile_options(freertos_app PRIVATE
    $<$<COMPILE_LANGUAGE:CXX>:-mcpu=cortex-m3 -mthumb -fno-exceptions -ffreestanding -nostdlib -nostartfiles>
    $<$<COMPILE_LANGUAGE:ASM>:-x assembler-with-cpp -mcpu=cortex-m3 -mthumb -mno-thumb-interwork -mfpu=vfp -msoft-float -mfix-cortex-m3-ldrd>
)


target_link_libraries(freertos_app
    PRIVATE
    freertos_kernel
    freertos_config
)

set_source_files_properties(${LINKER_SCRIPT} PROPERTIES EXTERNAL_OBJECT TRUE)
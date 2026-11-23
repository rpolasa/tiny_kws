# Generated Cmake Pico project file
# kws_pico: tiny_kws rev B keyword spotter emulated on a Raspberry Pi Pico 2 W (see README.md)

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)

# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.3.1)
set(toolchainVersion 15_2_Rel1)
set(picotoolVersion 2.3.1)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico2_w CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(kws_pico C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

add_executable(kws_pico
        src/main.c
        src/kws_nn.c
        src/kws_fe_port.c        # includes src/ref/kws_fe.c (unchanged copy of model/kws_fe.c)
        src/kws_selftest.c
        )

pico_set_program_name(kws_pico "kws_pico")
pico_set_program_version(kws_pico "0.1")

# PIO program: PDM clock + data capture
pico_generate_pio_header(kws_pico ${CMAKE_CURRENT_LIST_DIR}/src/pdm_mic.pio)

# Output over USB serial (UART pins stay free)
pico_enable_stdio_uart(kws_pico 0)
pico_enable_stdio_usb(kws_pico 1)

target_include_directories(kws_pico PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
        ${CMAKE_CURRENT_LIST_DIR}/src
)

# Optimise the signal chain even in a Debug build (real-time margin is measured with 's')
target_compile_options(kws_pico PRIVATE -O2 -Wall)

target_link_libraries(kws_pico
        pico_stdlib
        pico_cyw43_arch_none     # on-board LED is on the CYW43 Wi-Fi chip
        hardware_pio
        hardware_dma
        hardware_clocks
        )

pico_add_extra_outputs(kws_pico)

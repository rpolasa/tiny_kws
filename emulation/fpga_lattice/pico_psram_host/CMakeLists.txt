# Generated Cmake Pico project file
# kws_fpga_host: Pico 2 W = QSPI PSRAM emulator + board-B host for the tiny_kws iCESugar FPGA emulation

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

project(kws_fpga_host C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

add_executable(kws_fpga_host
        src/main.c
        src/psram_emu.c          # core 1 + PIO0 + DMA: QSPI PSRAM B emulation
        src/kws_chip.c           # SPI0 host for the chip registers
        src/kws_nn.c             # bit-exact GRU (from kws_pico) - golden h' for the diagnostic
        )

pico_set_program_name(kws_fpga_host "kws_fpga_host")
pico_set_program_version(kws_fpga_host "0.1")

pico_generate_pio_header(kws_fpga_host ${CMAKE_CURRENT_LIST_DIR}/src/qspi_psram.pio)

pico_enable_stdio_uart(kws_fpga_host 0)
pico_enable_stdio_usb(kws_fpga_host 1)

target_include_directories(kws_fpga_host PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
        ${CMAKE_CURRENT_LIST_DIR}/src
)

target_compile_options(kws_fpga_host PRIVATE -O2 -Wall)

target_link_libraries(kws_fpga_host
        pico_stdlib
        pico_multicore
        pico_cyw43_arch_none
        hardware_pio
        hardware_dma
        hardware_spi
        hardware_clocks
        )

pico_add_extra_outputs(kws_fpga_host)

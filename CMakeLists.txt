# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.2.0)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.2.0-a4)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
cmake_minimum_required(VERSION 3.13)

set(PICO_BOARD pico CACHE STRING "Board type")

include(pico_sdk_import.cmake)

project(hub12_controller C CXX ASM)

pico_sdk_init()

add_executable(hub12_controller)

pico_generate_pio_header(hub12_controller ${CMAKE_CURRENT_LIST_DIR}/src/hub12.pio)

target_sources(hub12_controller PUBLIC
        ${CMAKE_CURRENT_LIST_DIR}/src/main.c
        ${CMAKE_CURRENT_LIST_DIR}/src/usb.c
        ${CMAKE_CURRENT_LIST_DIR}/src/hub12.c
        ${CMAKE_CURRENT_LIST_DIR}/src/debug.c)

target_link_libraries(hub12_controller PUBLIC pico_stdlib pico_unique_id pico_rand hardware_pio hardware_dma hardware_uart)

pico_add_extra_outputs(hub12_controller)

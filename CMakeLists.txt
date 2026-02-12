cmake_minimum_required(VERSION 3.13)

set(ENV{PICO_SDK_PATH} /home/radioegor146/pico-sdk)

include(pico_sdk_import.cmake)

project(hub12_controller C CXX ASM)

pico_sdk_init()

add_executable(hub12_controller)

pico_generate_pio_header(hub12_controller ${CMAKE_CURRENT_LIST_DIR}/src/hub12.pio)

target_sources(hub12_controller PUBLIC
        ${CMAKE_CURRENT_LIST_DIR}/src/main.c
        ${CMAKE_CURRENT_LIST_DIR}/src/usb.c
        ${CMAKE_CURRENT_LIST_DIR}/src/hub12.c)

target_link_libraries(hub12_controller PUBLIC pico_stdlib pico_unique_id pico_rand hardware_pio hardware_dma hardware_uart)

pico_add_extra_outputs(hub12_controller)

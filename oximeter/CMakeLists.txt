
cmake_minimum_required(VERSION 3.20.0)

set(DTC_OVERLAY_FILE "boards/nrf52832.overlay")

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(oximeter)


target_include_directories(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/inc)

target_sources(app PRIVATE src/main.c)
target_sources(app PRIVATE src/AFR4990.c)
target_sources(app PRIVATE src/spi.c)
target_sources(app PRIVATE src/bluetooth.c)
target_sources(app PRIVATE src/uart.c)


cmake_minimum_required(VERSION 3.20.0)


find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(oximeter)

target_sources(app PRIVATE src/main.c src/AFR4990.c src/spi.c)
include_directories(D:/zephyrproject/modules/lib/cmsis-dsp/Include)

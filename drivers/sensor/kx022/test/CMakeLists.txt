cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(kx022_test)

zephyr_library_include_directories(
        ${ZEPHYR_BASE}/drivers/sensor/kx022
)

FILE(GLOB app_sources src/*.c ${ZEPHYR_BASE}/drivers/sensor/kx022/kx022.c)

target_sources(app PRIVATE ${app_sources})
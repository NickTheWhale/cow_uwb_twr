cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(tag)

target_sources(app PRIVATE
    src/main.c

    drivers/dw1000/decadriver/deca_device.c
    drivers/dw1000/decadriver/deca_params_init.c

    drivers/dw1000/platform/deca_mutex.c
    drivers/dw1000/platform/deca_range_tables.c
    drivers/dw1000/platform/deca_sleep.c
    drivers/dw1000/platform/deca_spi.c
    drivers/dw1000/platform/port.c
)

target_include_directories(app PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)
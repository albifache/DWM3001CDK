# SPDX-License-Identifier: Apache-2.0
cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(DWM3001C)

zephyr_include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/..
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../port
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../deca_driver
    ${CMAKE_CURRENT_SOURCE_DIR}/../shared_data
)

# Add include path for header files only
target_include_directories(app PRIVATE ../examples_info)

# Explicit source files listing (Recommended for better control)
set(APP_SOURCES

    # main
    main.c

    # Congiguration options
    ../config_options.c

    # Shared example functions
    ../shared_data/shared_functions.c

    # Example selection
    ../examples_info/example_info.c

    # Example files
    ../ex_00a_reading_dev_id/read_dev_id.c
    ../ex_01a_simple_tx/simple_tx.c
    ../ex_02a_simple_rx/simple_rx.c
    ../ex_02a_simple_rx/simple_rx_nlos.c
   

    # App
    # ../../../app/app.c
    # ../../../app/app_utils.c

    # MAC
    ../../../mac/mac.c

    # PHY
    ../../../phy/phy.c

    # DW3000 API port to Zephyr RTOS
    ../../../port/deca_gpio.c
    ../../../port/deca_mutex.c
    ../../../port/deca_sleep.c
    ../../../port/deca_spi.c
    ../../../port/deca_probe_interface.c
    ../../../port/deca_utils.c

    # DW3000 driver (Qorvo API)
    ../../../deca_driver/deca_compat.c
    ../../../deca_driver/deca_interface.c
    ../../../deca_driver/deca_rsl.c
    ../../../deca_driver/deca_device.c
    ../../../deca_driver/qmath.c

    # LEDs
    ../../../port/led_gpio.c
)

# Add the sources to the target
target_sources(app PRIVATE ${APP_SOURCES})
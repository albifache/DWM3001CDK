# SPDX-License-Identifier: Apache-2.0
cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(DWM3001C)

# Explicit source files listing (Recommended for better control)
set(APP_SOURCES

    # main
    main.c

    # App
    ../../app/app.c
    ../../app/app_utils.c

    # MAC
    ../../mac/mac.c

    # PHY
    ../../phy/phy.c

    # DW3000 API port to Zephyr RTOS
    ../../port/deca_gpio.c
    ../../port/deca_mutex.c
    ../../port/deca_sleep.c
    ../../port/deca_spi.c
    ../../port/deca_probe_interface.c 
    ../../port/deca_init.c 

    # DW3000 driver (Qorvo API)
    ../../deca_driver/deca_compat.c
    ../../deca_driver/deca_interface.c
    ../../deca_driver/deca_rsl.c
    ../../deca_driver/deca_device.c
    ../../deca_driver/qmath.c
)

# Add the sources to the target
target_sources(app PRIVATE ${APP_SOURCES})
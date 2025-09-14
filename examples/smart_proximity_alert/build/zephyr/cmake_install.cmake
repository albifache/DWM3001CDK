# Install script for directory: /home/alberto/zephyrproject/zephyr

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "TRUE")
endif()

# Set path to fallback-tool for dependency-resolution.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/home/alberto/zephyr-sdk-0.17.0/arm-zephyr-eabi/bin/arm-zephyr-eabi-objdump")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/alberto/zephyrproject/zephyr/samples/basic/DWM3001CDK/examples/smart_proximity_alert/build/zephyr/arch/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/alberto/zephyrproject/zephyr/samples/basic/DWM3001CDK/examples/smart_proximity_alert/build/zephyr/lib/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/alberto/zephyrproject/zephyr/samples/basic/DWM3001CDK/examples/smart_proximity_alert/build/zephyr/soc/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/alberto/zephyrproject/zephyr/samples/basic/DWM3001CDK/examples/smart_proximity_alert/build/zephyr/boards/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/alberto/zephyrproject/zephyr/samples/basic/DWM3001CDK/examples/smart_proximity_alert/build/zephyr/subsys/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/alberto/zephyrproject/zephyr/samples/basic/DWM3001CDK/examples/smart_proximity_alert/build/zephyr/drivers/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/alberto/zephyrproject/zephyr/samples/basic/DWM3001CDK/examples/smart_proximity_alert/build/modules/acpica/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/alberto/zephyrproject/zephyr/samples/basic/DWM3001CDK/examples/smart_proximity_alert/build/modules/cmsis/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/alberto/zephyrproject/zephyr/samples/basic/DWM3001CDK/examples/smart_proximity_alert/build/modules/cmsis-dsp/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/alberto/zephyrproject/zephyr/samples/basic/DWM3001CDK/examples/smart_proximity_alert/build/modules/cmsis-nn/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/alberto/zephyrproject/zephyr/samples/basic/DWM3001CDK/examples/smart_proximity_alert/build/modules/fatfs/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/alberto/zephyrproject/zephyr/samples/basic/DWM3001CDK/examples/smart_proximity_alert/build/modules/adi/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/alberto/zephyrproject/zephyr/samples/basic/DWM3001CDK/examples/smart_proximity_alert/build/modules/altera/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/alberto/zephyrproject/zephyr/samples/basic/DWM3001CDK/examples/smart_proximity_alert/build/modules/hal_ambiq/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/alberto/zephyrproject/zephyr/samples/basic/DWM3001CDK/examples/smart_proximity_alert/build/modules/atmel/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/alberto/zephyrproject/zephyr/samples/basic/DWM3001CDK/examples/smart_proximity_alert/build/modules/hal_bouffalolab/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/alberto/zephyrproject/zephyr/samples/basic/DWM3001CDK/examples/smart_proximity_alert/build/modules/hal_espressif/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/alberto/zephyrproject/zephyr/samples/basic/DWM3001CDK/examples/smart_proximity_alert/build/modules/hal_ethos_u/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/alberto/zephyrproject/zephyr/samples/basic/DWM3001CDK/examples/smart_proximity_alert/build/modules/hal_gigadevice/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/alberto/zephyrproject/zephyr/samples/basic/DWM3001CDK/examples/smart_proximity_alert/build/modules/hal_infineon/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/alberto/zephyrproject/zephyr/samples/basic/DWM3001CDK/examples/smart_proximity_alert/build/modules/hal_intel/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/alberto/zephyrproject/zephyr/samples/basic/DWM3001CDK/examples/smart_proximity_alert/build/modules/microchip/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/alberto/zephyrproject/zephyr/samples/basic/DWM3001CDK/examples/smart_proximity_alert/build/modules/hal_nordic/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/alberto/zephyrproject/zephyr/samples/basic/DWM3001CDK/examples/smart_proximity_alert/build/modules/nuvoton/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/alberto/zephyrproject/zephyr/samples/basic/DWM3001CDK/examples/smart_proximity_alert/build/modules/hal_nxp/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/alberto/zephyrproject/zephyr/samples/basic/DWM3001CDK/examples/smart_proximity_alert/build/modules/openisa/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/alberto/zephyrproject/zephyr/samples/basic/DWM3001CDK/examples/smart_proximity_alert/build/modules/quicklogic/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/alberto/zephyrproject/zephyr/samples/basic/DWM3001CDK/examples/smart_proximity_alert/build/modules/hal_renesas/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/alberto/zephyrproject/zephyr/samples/basic/DWM3001CDK/examples/smart_proximity_alert/build/modules/hal_rpi_pico/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/alberto/zephyrproject/zephyr/samples/basic/DWM3001CDK/examples/smart_proximity_alert/build/modules/hal_silabs/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/alberto/zephyrproject/zephyr/samples/basic/DWM3001CDK/examples/smart_proximity_alert/build/modules/hal_st/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/alberto/zephyrproject/zephyr/samples/basic/DWM3001CDK/examples/smart_proximity_alert/build/modules/hal_stm32/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/alberto/zephyrproject/zephyr/samples/basic/DWM3001CDK/examples/smart_proximity_alert/build/modules/hal_tdk/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/alberto/zephyrproject/zephyr/samples/basic/DWM3001CDK/examples/smart_proximity_alert/build/modules/hal_telink/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/alberto/zephyrproject/zephyr/samples/basic/DWM3001CDK/examples/smart_proximity_alert/build/modules/ti/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/alberto/zephyrproject/zephyr/samples/basic/DWM3001CDK/examples/smart_proximity_alert/build/modules/hal_wch/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/alberto/zephyrproject/zephyr/samples/basic/DWM3001CDK/examples/smart_proximity_alert/build/modules/hal_wurthelektronik/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/alberto/zephyrproject/zephyr/samples/basic/DWM3001CDK/examples/smart_proximity_alert/build/modules/xtensa/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/alberto/zephyrproject/zephyr/samples/basic/DWM3001CDK/examples/smart_proximity_alert/build/modules/hostap/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/alberto/zephyrproject/zephyr/samples/basic/DWM3001CDK/examples/smart_proximity_alert/build/modules/liblc3/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/alberto/zephyrproject/zephyr/samples/basic/DWM3001CDK/examples/smart_proximity_alert/build/modules/libmctp/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/alberto/zephyrproject/zephyr/samples/basic/DWM3001CDK/examples/smart_proximity_alert/build/modules/libmetal/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/alberto/zephyrproject/zephyr/samples/basic/DWM3001CDK/examples/smart_proximity_alert/build/modules/littlefs/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/alberto/zephyrproject/zephyr/samples/basic/DWM3001CDK/examples/smart_proximity_alert/build/modules/loramac-node/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/alberto/zephyrproject/zephyr/samples/basic/DWM3001CDK/examples/smart_proximity_alert/build/modules/lvgl/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/alberto/zephyrproject/zephyr/samples/basic/DWM3001CDK/examples/smart_proximity_alert/build/modules/mbedtls/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/alberto/zephyrproject/zephyr/samples/basic/DWM3001CDK/examples/smart_proximity_alert/build/modules/mcuboot/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/alberto/zephyrproject/zephyr/samples/basic/DWM3001CDK/examples/smart_proximity_alert/build/modules/mipi-sys-t/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/alberto/zephyrproject/zephyr/samples/basic/DWM3001CDK/examples/smart_proximity_alert/build/modules/nrf_wifi/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/alberto/zephyrproject/zephyr/samples/basic/DWM3001CDK/examples/smart_proximity_alert/build/modules/open-amp/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/alberto/zephyrproject/zephyr/samples/basic/DWM3001CDK/examples/smart_proximity_alert/build/modules/openthread/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/alberto/zephyrproject/zephyr/samples/basic/DWM3001CDK/examples/smart_proximity_alert/build/modules/percepio/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/alberto/zephyrproject/zephyr/samples/basic/DWM3001CDK/examples/smart_proximity_alert/build/modules/picolibc/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/alberto/zephyrproject/zephyr/samples/basic/DWM3001CDK/examples/smart_proximity_alert/build/modules/segger/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/alberto/zephyrproject/zephyr/samples/basic/DWM3001CDK/examples/smart_proximity_alert/build/modules/tinycrypt/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/alberto/zephyrproject/zephyr/samples/basic/DWM3001CDK/examples/smart_proximity_alert/build/modules/trusted-firmware-a/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/alberto/zephyrproject/zephyr/samples/basic/DWM3001CDK/examples/smart_proximity_alert/build/modules/trusted-firmware-m/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/alberto/zephyrproject/zephyr/samples/basic/DWM3001CDK/examples/smart_proximity_alert/build/modules/uoscore-uedhoc/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/alberto/zephyrproject/zephyr/samples/basic/DWM3001CDK/examples/smart_proximity_alert/build/modules/zcbor/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/alberto/zephyrproject/zephyr/samples/basic/DWM3001CDK/examples/smart_proximity_alert/build/modules/nrf_hw_models/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/alberto/zephyrproject/zephyr/samples/basic/DWM3001CDK/examples/smart_proximity_alert/build/zephyr/kernel/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/alberto/zephyrproject/zephyr/samples/basic/DWM3001CDK/examples/smart_proximity_alert/build/zephyr/cmake/flash/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/alberto/zephyrproject/zephyr/samples/basic/DWM3001CDK/examples/smart_proximity_alert/build/zephyr/cmake/usage/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/alberto/zephyrproject/zephyr/samples/basic/DWM3001CDK/examples/smart_proximity_alert/build/zephyr/cmake/reports/cmake_install.cmake")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
if(CMAKE_INSTALL_LOCAL_ONLY)
  file(WRITE "/home/alberto/zephyrproject/zephyr/samples/basic/DWM3001CDK/examples/smart_proximity_alert/build/zephyr/install_local_manifest.txt"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
endif()

# DWM3001CDK

## License

This project is distributed under the **GNU AGPL v3.0 License**. See the [LICENSE](LICENSE) file for details.

## Overview

An advanced real-time positioning framework for DWM3001CDK modules, delivering precise location tracking via DS-TWR, over-the-air real-time network reconfigurability and massive collision-free scalability (60,000+ devices) through TDMA coordination.

## Project Components

- **Qorvo/Decawave DW3000 driver** (slightly modified to reduce unnecessary memory usage)
- **Port to Zephyr RTOS** for DWM3001CDK modules (nRF52833 MCU)
- **Low-level functions** to access precalibrated parameters stored in DW3000 OTP memory
- **Application protocol** based on IEEE 802.15.4 standard, relying on DS-TWR to measure distances and TDMA to handle multiple concurrent devices without collisions, while also featuring a novel, dynamic, real-time and over-the-air role assignment protocol 

## Features

- **Dynamic Role Assignment**: Nodes can be run-time configured over-the-air as tags, anchors, or listeners
- **Multi-node Support**: Handles multiple anchors and tags in the same PAN
- **TDMA Coordination**: Time-slotted multiple access prevents collisions and interference
- **DS-TWR Protocol**: Double-sided two-way ranging for accurate distance measurements
- **Real-time Processing**: Immediate distance calculations and alerts
- **LED Indicators**: Visual feedback for proximity alerts and system status

## Project Structure

The project folder is structured as follows:

```
DWM3001CDK/
│
├── app/
│   ├── app.c                               # Main application protocol implementation
│   ├── app.h                               # Application layer header
│   ├── app_utils.c                         # Application utility functions
│   └── app_utils.h                         # Application utility functions header
│
├── mac/
│   ├── mac.c                               # IEEE 802.15.4 MAC layer
│   └── mac.h                               # MAC layer header
│
├── phy/
│   ├── phy.c                               # PHY layer implementation
│   └── phy.h                               # PHY layer header
│
├── deca_driver/
│   ├── deca_compat.c
│   ├── deca_device_api.h
│   ├── deca_device.c
│   ├── deca_interface.c
│   ├── deca_interface.h
│   ├── deca_private.h
│   ├── deca_regs.h
│   ├── deca_rsl.c
│   ├── deca_rsl.h
│   ├── deca_types.h
│   ├── deca_vals.h
│   ├── deca_versions.h
│   ├── qmath.c
│   ├── qmath.h
│   └── LicenseRef-Qorvo-2.txt
│
├── port/
│   ├── deca_gpio.c                         # DW3000 RST pin interface
│   ├── deca_mutex.c                        # Just for compatibility with DW3000 API
│   ├── deca_probe_interface.c              # Device probe interface
│   ├── deca_probe_interface.h              # Device probe interface header
│   ├── deca_sleep.c                        # Blocking delays
│   ├── deca_spi.c                          # DW3000 SPI interface
│   ├── led_gpio.c                          # LEDs interface
│   └── port.h                              # Port interface
│
├── examples/
│   │
│   └── smart_proximity_alert/              # Smart proximity alert example
│       ├── main.c                          # Main file
│       ├── CMakeLists.txt                  # Build configuration
│       ├── prj.conf                        # Zephyr project configuration
│       └── README.md                       # Smart proximity alert documentation
│
├── README.md                               # Documentation
└── LICENSE                                 # License (GNU AGPL v3.0)
```

## Requirements

- **DWM3001CDK development boards** (minimum 2 for testing)
- **USB cables** for powering, flashing and debugging
- **Linux Host** with Zephyr installed

## Setup Instructions

### 1. Install J-Link Tools

Install J-Link tools following the instructions at [J-Link Download](https://www.segger.com/downloads/jlink/)

### 2. Install Zephyr

Follow the [Zephyr Getting Started Guide](https://docs.zephyrproject.org/latest/develop/getting_started/index.html) to set up your development environment. Choose Zephyr SDK version 0.17.0 or later

### 3. Clone This Repository

```bash
cd ~/zephyrproject/zephyr/samples
git clone https://github.com/albifache/DWM3001CDK
```

## API Reference

### Application Layer

```c
int app_run_ieee_802_15_4_schedule(void);                           // Run ranging protocol
int app_set_mac_addr(uint16_t mac_addr);                            // Set node MAC address
int app_set_pan_id(uint16_t pan_id);                                // Set node PAN ID
int app_set_tag_mac_addr(uint16_t mac_addr);                        // Select which node will behave as tag
void app_get_ranging_info(app_ranging_info_t *info);                // Read ranging session results
int app_set_anchor_mac_addr (uint16_t mac_addr[], uint8_t cnt);     // Select which nodes will behave as anchors
void app_sleep (uint16_t time_ms);                                  // Sleep for time_ms milliseconds
```

### PHY Layer

```c
int phy_device_init(void);                                          // Initialize DW3000
int phy_set_config(dwt_config_t* config);                           // Configure UWB channel parameters
int phy_set_tx_power(uint32_t tx_power);                            // Set transmission power
int phy_set_ant_delay (uint16_t ant_delay);                         // Set antenna delay
void phy_get_device_info (phy_device_info_t* device_info);          // Read device hardware info
```

## Support

For issues and questions:

- **GitHub Issues**: [Create an issue](https://github.com/albifache/DWM3001CDK/issues)
- **LinkedIn**: [Alberto Facheris](https://www.linkedin.com/in/alberto-facheris-9028ab357/)

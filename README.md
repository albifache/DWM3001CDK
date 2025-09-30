# DWM3001CDK

## License

This project is distributed under the **GNU AGPL v3.0 License** (Check [LICENSE](LICENSE) file for details). If you need a different license, please contact the author via [LinkedIn](https://www.linkedin.com/in/alberto-facheris-9028ab357/) or [E-mail](mailto:albi97.fache@gmail.com)

## Disclaimer

This software is provided "as is", without warranty of any kind, express or implied. The author shall not be liable for any damages, security issues, or malfunctions arising from the use of this software. Users are responsible for testing and validating the software for their specific use cases, especially in safety-critical or security-sensitive applications.

This project is intended for educational and research purposes. For production deployments, additional security audits and testing are strongly recommended.

## Overview

An advanced real-time positioning framework for DWM3001CDK modules, delivering precise location tracking via DS-TWR, over-the-air real-time network reconfigurability, secure ranging feature and massive collision-free scalability (60,000+ devices) through TDMA coordination.

## Project Components

- **Qorvo/Decawave DW3000 driver** (slightly modified to reduce unnecessary memory usage)
- **Port to Zephyr RTOS** for DWM3001CDK modules (nRF52833 MCU)
- **Low-level functions** to access pre-calibrated parameters stored in DW3000 OTP memory
- **Application protocol** based on IEEE 802.15.4z standard, relying on DS-TWR to measure distances and TDMA to handle multiple concurrent devices without collisions, while also featuring a novel, dynamic, real-time and over-the-air (OTA) role assignment protocol. The protocol also features secure ranging, resistant against replay attacks, thanks to STS and AES encryption

## Features

- **Dynamic Role Assignment**: Nodes can be runtime configured over-the-air (OTA) as tags, anchors, or listeners
- **Security**: Secure ranging resistant against replay attacks thanks to STS and AES encryption
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
│   ├── proximity_alert/                    # Proximity alert example
│   │   ├── main.c                          # Main file
│   │   ├── CMakeLists.txt                  # Build configuration
│   │   ├── prj.conf                        # Zephyr project configuration
│   │   └── README.md                       # Proximity alert documentation
│   │
│   └── multi_node_ranging/                 # Multi-node ranging example
│       ├── main.c                          # Main file
│       ├── CMakeLists.txt                  # Build configuration
│       ├── prj.conf                        # Zephyr project configuration
│       └── README.md                       # Multi-node ranging documentation
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

Install J-Link tools following the instructions at [J-Link Download](https://www.segger.com/downloads/jlink/).

### 2. Install Zephyr

Follow the [Zephyr Getting Started Guide](https://docs.zephyrproject.org/latest/develop/getting_started/index.html) to set up your development environment. Choose Zephyr SDK version 0.17.0 or later.

### 3. Clone This Repository

```bash
cd ~/zephyrproject/zephyr/samples
git clone https://github.com/albifache/DWM3001CDK
```

## API Reference

### Application Layer

```c
typedef struct
{
    uint16_t mac_addr;                                              // MAC address of the current node
    uint16_t pan_id;                                                // PAN ID of the network (same for all the nodes)
    dwt_aes_key_t aes_key;                                          // AES key (128 bits, same for all the nodes)
    dwt_sts_cp_key_t sts_key;                                       // STS key (128 bits, same for all the nodes)
}
app_init_obj_t;

typedef struct
{
    uint16_t tag_mac_addr;                                          // MAC address of tag node
    uint16_t anchor_mac_addr[MAX_NUM_ANCHORS];                      // MAC addresses of anchor nodes
    uint8_t num_anchors;                                            // Number of anchor nodes
}
app_ctrl_obj_t;

typedef struct
{
    uint64_t superframe_id;                                         // Superframe ID
    uint64_t ts_init;                                               // Timestamp of ranging session
    uint16_t tag_mac_addr;                                          // MAC address of tag node
    uint8_t num_anchors;                                            // Number of anchor nodes
    uint16_t anchor_mac_addr[MAX_NUM_ANCHORS];                      // MAC addresses of anchor nodes
    uint64_t dist[MAX_NUM_ANCHORS];                                 // Distances from tag to each of the anchors (DS-TWR)
}
app_log_info_t;

int app_init (app_init_obj_t *obj);                                 // Initialize app
int app_set_ctrl_params (app_ctrl_obj_t *obj);                      // Set runtime parameters for ranging session
int app_run_ieee_802_15_4z_schedule (void);                         // Run ranging protocol
void app_read_log_info (app_log_info_t *info);                      // Read ranging session results
void app_sleep (uint16_t ms);                                       // Sleep for ms milliseconds
```

### PHY Layer

```c
typedef struct
{
    dwt_pll_ch_type_e rf_chan;                                      // UWB channel (5 or 9)
    uint8_t preamble_code;                                          // Preamble code 
    uint16_t preamble_len;                                          // Preamble length (number of symbols)
    dwt_uwb_bit_rate_e bit_rate;                                    // UWB bit rate
    dwt_sts_lengths_e sts_len;                                      // STS length (number of symbols)
}
phy_init_obj_t;

int phy_init (phy_init_obj_t *obj);                                 // Set PHY parameters
```

## Support

For issues and questions:

- **GitHub Issues**: [Create an issue](https://github.com/albifache/DWM3001CDK/issues)
- **LinkedIn**: [Alberto Facheris](https://www.linkedin.com/in/alberto-facheris-9028ab357/)

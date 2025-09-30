# Proximity Alert

## License

This project is distributed under the **GNU AGPL v3.0 License** (Check [LICENSE](../../LICENSE) file for details). If you need a different license, please contact the author via [LinkedIn](https://www.linkedin.com/in/alberto-facheris-9028ab357/) or [E-mail](mailto:albi97.fache@gmail.com)

## Disclaimer

This software is provided "as is", without warranty of any kind, express or implied. The author shall not be liable for any damages, security issues, or malfunctions arising from the use of this software. Users are responsible for testing and validating the software for their specific use cases, especially in safety-critical or security-sensitive applications.

This project is intended for educational and research purposes. For production deployments, additional security audits and testing are strongly recommended.

## Overview

This example implements a proximity sensor where 1 tag equipped with 4 LEDs ranges up to 4 anchor nodes and blinks the LEDs associated to the anchor nodes whose distance from the tag is below a certain threshold. For this example you will need 2 to 5 DWM3001CDK modules. The code might contain some small inefficiencies since it is designed to be as readable and educational as possible.

## Building and Flashing

### 1. Clone the Repository

Clone the repository if you still haven't. Then navigate to this example folder.

```bash
cd ~/zephyrproject/zephyr/samples
git clone https://github.com/albifache/DWM3001CDK
cd examples/proximity_alert
```

### 2. Activate Python Virtual Environment

To use west you always need to activate the virtual environment first:

```bash
python3 -m venv ~/zephyrproject/.venv
source ~/zephyrproject/.venv/bin/activate
```

### 3. Build the Firmware

```bash
west build -b decawave_dwm3001cdk -d build
```

### 4. Flash the Firmware

```bash
west flash -r jlink
```

## Configuration

Edit the main configuration in `main.c`. Parameters can be modified but must be fixed for all the nodes apart from MAC_ADDR.

```c
#define RF_CHAN                                 5                           // UWB channel (5 or 9)
#define PREAMBLE_CODE                           9                           // Preamble code
#define BIT_RATE                                DWT_BR_850K                 // UWB bit rate
#define PREAMBLE_LEN                            DWT_PLEN_1024               // Preamble length (number of symbols)
#define STS_LEN                                 DWT_STS_LEN_1024            // STS length (number of symbols)

#define MAC_ADDR                                0x00u                       // MAC address of current node
#define PAN_ID                                  0x00u                       // PAN ID of current network

#define STS_KEY_0                               0ul                         // STS key (bits 0-31)
#define STS_KEY_1                               0ul                         // STS key (bits 32-63)
#define STS_KEY_2                               0ul                         // STS key (bits 64-95)
#define STS_KEY_3                               0ul                         // STS key (bits 96-127)

#define AES_KEY_0                               0ul                         // AES key (bits 0-31)
#define AES_KEY_1                               0ul                         // AES key (bits 32-63)
#define AES_KEY_2                               0ul                         // AES key (bits 64-95)
#define AES_KEY_3                               0ul                         // AES key (bits 96-127)

#define NUM_ANCHORS                             2                           // Number of anchors (max 4)

#define ANCHOR_MAC_ADDR_0                       0x06                        // MAC address of anchor 0
#define ANCHOR_MAC_ADDR_1                       0x07                        // MAC address of anchor 1
#define ANCHOR_MAC_ADDR_2                       0x00                        // MAC address of anchor 2 (unused in this case)
#define ANCHOR_MAC_ADDR_3                       0x00                        // MAC address of anchor 3 (unused in this case)

#define GREEN_LED                               0                           // LED that blinks when tag is too close to anchor 0
#define RED_LED_0                               1                           // LED that blinks when tag is too close to anchor 1
#define RED_LED_1                               2                           // LED that blinks when tag is too close to anchor 2
#define BLUE_LED                                3                           // LED that blinks when tag is too close to anchor 3

#define ALERT_DIST                              1.0f                        // Alert distance (m)
```

## Debugging

### RTT Console Output

Use J-Link RTT Viewer to monitor debug output:

```bash
JLinkRTTViewer
```

### Common Issues

- **Build failures**: 
  - Ensure Zephyr environment is properly activated
  - Check that all dependencies are installed
  - Check if Python virtual environment has been activated before using west
  - Try a pristine build:
```bash
west build -b decawave_dwm3001cdk -d build --pristine
```

- **Flash failures**: 
  - Check J-Link connection and USB port on DWM3001CDK (There are 2 USB ports but only one is connected to J-Link)
  - Verify the device is powered on
  - Try different USB cable or port on host machine

- **No RTT output**:
  - For this example it is normal, since there is no logging

## Support

For issues and questions:

- **GitHub Issues**: [Create an issue](https://github.com/albifache/DWM3001CDK/issues)
- **LinkedIn**: [Alberto Facheris](https://www.linkedin.com/in/alberto-facheris-9028ab357/)

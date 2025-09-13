# Smart Proximity Alert

## License

This project is distributed under the **GNU AGPL v3.0 License** (Check [LICENSE](LICENSE) file for details). If you need a different license please contact the author via [LinkedIn](https://www.linkedin.com/in/alberto-facheris-9028ab357/) or [E-mail](mailto:albi97.fache@gmail.com)

## Overview

This example implements a smart proximity sensor where 1 tag equipped with 4 LEDs ranges up to 4 anchor nodes and blinks the LEDs associated to the anchor nodes whose distance from the tag is below a certain threshold. For this example you will need 2 to 5 DWM3001CDK modules.

## Building and Flashing

### 1. Clone The Repository

Clone the repository if you still haven't. Then navigate to this example folder (examples/smart_proximity_alert)

```bash
cd ~/zephyrproject/zephyr/samples
git clone https://github.com/albifache/DWM3001CDK
cd examples/smart_proximity_alert
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

The example is configured to operate with:
- **One tag** (MAC address 0x00) 
- **Two anchors** (MAC addresses 0x06 and 0x07)
- **PAN ID** set to 0x00 (must be the same for all nodes)
- **Alert distance** set to 1 m
- **LED mapping**: 
  - LED 3 (blue) blinks for anchor node 0x06
  - LED 1 (red) blinks for anchor node 0x07
  - LED 0 is green, LEDs 1-2 are red, LED 3 is blue

### Customizing Configuration

Edit the main configuration in `main.c`:

```c
#define BLUE_LED        3                             // LED ID (0 to 3)
#define RED_LED         1                             // LED ID (0 to 3)  
#define ALERT_DIST      1.0f                          // Alert distance in meters

static uint16_t my_mac_addr = 0x00;                   // MAC address of this node
static uint16_t my_pan_id = 0x00;                     // PAN ID for all nodes
static uint16_t anchor_mac_addr[] = {0x06, 0x07};     // MAC addresses of anchor nodes
static uint16_t led_id[] = {BLUE_LED, RED_LED};       // LEDs associated to anchor nodes
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

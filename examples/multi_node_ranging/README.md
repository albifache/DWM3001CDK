# Proximity Alert

## License

This project is distributed under the **GNU AGPL v3.0 License** (Check [LICENSE](../../LICENSE) file for details). If you need a different license please contact the author via [LinkedIn](https://www.linkedin.com/in/alberto-facheris-9028ab357/) or [E-mail](mailto:albi97.fache@gmail.com)

## Overview

This example implements a RTLS where each node periodically ranges all the other nodes following a round-robin scheme. For this example you will need 2 to 9 DWM3001CDK modules. The code might contain some small inefficiencies since it is designed to be as readable and educational as possible.

## Building and Flashing

### 1. Clone The Repository

Clone the repository if you still haven't. Then navigate to this example folder (examples/smart_proximity_alert)

```bash
cd ~/zephyrproject/zephyr/samples
git clone https://github.com/albifache/DWM3001CDK
cd examples/multi_node_ranging
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
- **Six nodes** with MAC addresses 0x00, 0x01, 0x03, 0x06, 0x07, 0x09
- **PAN ID** set to 0x00 (must be the same for all nodes)


### Customizing Configuration

Edit the main configuration in `main.c`:

```c
static uint16_t my_mac_addr = 0x00;                                         // MAC address of this node
static uint16_t my_pan_id = 0x00;                                           // PAN ID for all nodes
static uint16_t node_mac_addr[] = {0x00, 0x01, 0x03, 0x06, 0x07, 0x09};     // MAC addresses of all the nodes
```

## Debugging

### RTT Console Output

Use J-Link RTT Viewer to monitor the serial output:

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
  - Check J-Link connection and USB port on DWM3001CDK (There are 2 USB ports but only one is connected to J-Link)
  - Verify the device is powered on
  - Try different USB cable or port on host machine

## Support

For issues and questions:

- **GitHub Issues**: [Create an issue](https://github.com/albifache/DWM3001CDK/issues)
- **LinkedIn**: [Alberto Facheris](https://www.linkedin.com/in/alberto-facheris-9028ab357/)

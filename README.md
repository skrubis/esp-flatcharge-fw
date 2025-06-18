# ESP32-S3 FlatCharge Firmware

Firmware for a custom ESP32-S3 board that controls up to 3 Eltek Flatpack2 power rectifiers via isolated CAN buses. This firmware provides a complete solution for EV charging with Type2 support.

## Features

- **WiFi Access Point**: Creates a WiFi network named "FLATCHARGE" with password "FLATCHARGE!"
- **Web Interface**: Mobile-friendly dark themed web UI for monitoring and control
- **OTA Updates**: Update firmware wirelessly through the web interface
- **Type2 Charging**: Supports IEC 61851 charging protocol and CP/PP signals
- **Multi-Flatpack Control**: Controls up to 3 Eltek Flatpack2 rectifiers via isolated CAN buses
- **Manual CC/CV Charging**: Configurable voltage and current settings
- **Series/Parallel Configuration**: Supports both series and parallel flatpack arrangements
- **Persistent Settings**: Stores configuration in SPIFFS filesystem

## Hardware

This firmware is designed for a custom ESP32-S3 board with the following features:
- ESP32-S3 microcontroller
- 3x isolated CAN buses using MCP2515 transceivers on VSPI bus
- MCP23017 I²C GPIO expander for power control and status LEDs
- Type2 Control Pilot (CP) input for EV charging

### Pin Configuration

- I²C: SCL=GPIO8, SDA=GPIO9
- SPI: MOSI=GPIO11, MISO=GPIO13, SCK=GPIO12
- CAN chip select pins: CS1=GPIO14, CS2=GPIO17, CS3=GPIO18
- Control Pilot input: GPIO2
- Heartbeat LED: GPIO48

## Building and Flashing

This project uses PlatformIO for development. To build and flash the firmware:

1. **Install PlatformIO**:
   - Install [Visual Studio Code](https://code.visualstudio.com/)
   - Install the [PlatformIO extension](https://platformio.org/install/ide?install=vscode)

2. **Clone the repository**:
   ```bash
   git clone https://github.com/yourusername/esp-flatcharge-fw.git
   cd esp-flatcharge-fw
   ```

3. **Build the project**:
   ```bash
   pio run
   ```

4. **Upload firmware to ESP32-S3**:
   ```bash
   pio run --target upload
   ```

5. **Upload filesystem (SPIFFS) with web interface files**:
   ```bash
   pio run --target uploadfs
   ```

## Usage

1. **Connect to the WiFi access point**:
   - SSID: `FLATCHARGE`
   - Password: `FLATCHARGE!`

2. **Access web interface**:
   - Open a web browser and navigate to `http://192.168.4.1` (default AP IP)

3. **Configure settings**:
   - Set charging parameters (voltage, current)
   - Choose series or parallel flatpack configuration
   - Enable/disable charging

4. **OTA Updates**:
   - Go to `http://192.168.4.1/firmware`
   - Upload new firmware (.bin file)
   
## CAN Communication

The firmware communicates with Eltek Flatpack2 rectifiers via CAN bus at 125kbps. It implements:

1. **Flatpack autodiscovery**: Listen for Flatpack serial broadcasts
2. **Login sequence**: Required to control output
3. **Voltage and current control**: Full CC/CV charging control

## Type2 Charging

The firmware interprets the Control Pilot (CP) PWM signal according to IEC 61851:
- Detects EV connection states (A-E)
- Reads allowed current from PWM duty cycle
- Adjusts charger output accordingly

# ESP-Flatcharge-FW

ESP32S3-based controller firmware for Eltek Flatpack2 power supplies used in battery charging applications.

## Overview

This firmware is designed for an ESP32S3 board that manages up to three isolated MCP2515 CAN buses and one TWAI CAN bus to control Eltek Flatpack2 power supply units (PSUs). The system is used for battery charging control with support for multiple battery chemistries (LFP, LTO, NMC) and configurable charging profiles.

### Key Features

- Initialization and configuration of all CAN buses at 125k baud
- Auto-detection of Flatpack PSUs via CAN messages
- Management of PSU login and periodic re-login
- Aggregation of PSU data for coordinated battery charging control
- Support for multiple battery chemistries with configurable charging curves
- **Web server with WiFi connectivity for remote monitoring and control**
- **WiFi Access Point mode (SSID: FLATCHARGE, password: flatcharge!)**
- **WiFi Client mode with persistent credential storage**
- **Real-time web dashboard for monitoring flatpack and battery status**
- **REST API endpoints for system integration**
- Robust modular architecture with clear abstraction layers

## Hardware Configuration

The system uses the following hardware components:

- **ESP32S3** microcontroller using the Arduino framework via PlatformIO
- **Three MCP2515 CAN controllers** connected via SPI, each with its own CS and INT pins
- **MCP23017 GPIO expander** at I2C address 0x26 for controlling power and reset lines
- **CAN bus configuration**: 125 kbps baud rate, 8 MHz crystal for MCP2515

Refer to `pins.md` for detailed hardware pin mapping.

## Software Architecture

The firmware is organized into several manager classes, each responsible for a specific subsystem:

### CANManager

Handles communication with the MCP2515 CAN controllers, including:
- Initialization and configuration of CAN controllers
- Message filtering
- Message sending and receiving with callback support

### HardwareManager

Manages hardware peripherals and control lines:
- MCP23017 GPIO expander initialization and control
- CAN bus power control
- Reset signals for MCP2515 controllers
- EVSE mode indicator LEDs

### FlatpackManager

Manages Eltek Flatpack2 PSU communication and control:
- PSU detection via CAN hello messages
- Login and authentication using PSU serial numbers
- Status and alert message processing
- Command transmission for voltage/current setting
- Data aggregation from multiple PSUs

### BatteryManager

Manages battery charging logic and profiles:
- Support for different battery chemistries (LFP, LTO, NMC)
- Configurable charging curves and parameters
- CC/CV mode transitions
- Temperature monitoring and safety limits
- State of charge estimation

### WebServerManager

Provides web-based monitoring and control interface:
- WiFi Access Point mode (SSID: FLATCHARGE, password: flatcharge!)
- WiFi Client mode with persistent credential storage
- Dual mode operation (AP + Client simultaneously)
- Async web server with REST API endpoints
- Real-time monitoring dashboard
- Configuration interface for WiFi and charging parameters
- JSON API for system integration

## Installation and Setup

### Prerequisites

- PlatformIO
- ESP-IDF (compatible with Arduino framework)
- Access to required libraries (included in project)

### Building and Uploading

1. Clone the repository
2. Open the project in PlatformIO
3. Configure settings in `platformio.ini` as needed
4. Build and upload the firmware:

```bash
pio run -t upload
```

### Configuration

The firmware can be configured by modifying constants in `main.cpp`:

```cpp
// Battery configuration
#define BATTERY_CHEMISTRY BatteryChemistry::LFP  // LFP, LTO, or NMC
#define BATTERY_CELL_COUNT 16                    // Number of cells in series
#define BATTERY_CAPACITY 100.0f                  // Capacity in Ah

// Feature flags
#define ENABLE_BATTERY_CHARGING false  // Enable/disable charging logic
#define ENABLE_VOLTAGE_SWEEP true      // Enable/disable voltage sweep mode
#define ENABLE_WEB_SERVER true         // Enable/disable web server and WiFi
```

Battery chemistry parameters can be adjusted in `BatteryManager.cpp` by modifying the default parameter structures:

```cpp
const BatteryParameters BatteryManager::defaultLFP = {
    .chemistry = BatteryChemistry::LFP,
    .cellCount = 16,  // Default for 48V nominal (16 * 3.2V = 51.2V)
    .cellVoltageMin = 2500,     // 2.5V
    .cellVoltageNominal = 3200, // 3.2V
    .cellVoltageMax = 3650,     // 3.65V
    .cellVoltageFloat = 3400,   // 3.4V
    // ... other parameters ...
};
```

## Flatpack2 CAN Protocol

The firmware implements the Eltek Flatpack2 CAN protocol, which includes:

- **Hello messages**: Used for PSU detection (CAN ID: 0x0500XXXX)
- **Login messages**: Required to establish communication using PSU serial number
- **Status messages**: Provide voltage, current, and temperature information
- **Alert messages**: Indicate PSU alarms and warnings
- **Command messages**: Used to set output voltage, current, and protection limits

Login sessions must be refreshed every 5 seconds to maintain connection.

Refer to `flatpack.md` and `fp-Protocol.md` for detailed protocol information.

## Usage

1. Flash the firmware to your ESP32S3 board
2. Connect the hardware as described in `pins.md`
3. Power on the system - it will initialize all CAN buses and start the web server
4. Connect Flatpack PSUs to any of the CAN buses (CAN1, CAN2, or CAN3)
5. The system will automatically detect PSUs and begin login procedures
6. Monitor status via:
   - Serial console at 115200 baud
   - Web dashboard at http://192.168.4.1 (when connected to FLATCHARGE WiFi AP)
   - REST API endpoints for system integration
7. Configure battery parameters and charging setpoints via web interface or API
8. Optionally connect to your WiFi network via the web interface for remote access

## Debugging

## Web Server and WiFi Features

The firmware includes a comprehensive web server for remote monitoring and control of the flatpack charging system.

### WiFi Modes

#### Access Point Mode (Default)
- **SSID**: FLATCHARGE
- **Password**: flatcharge!
- **IP Address**: 192.168.4.1
- **Web Interface**: http://192.168.4.1

#### Client Mode
- Connect to existing WiFi networks
- Persistent credential storage
- Automatic reconnection on power cycle
- Configure via web interface

#### Dual Mode
- Operates as both AP and client simultaneously
- Provides local access via AP while connected to network
- Seamless switching between access methods

### Web Dashboard

The web dashboard provides real-time monitoring and control:

- **Flatpack Status**: Number of detected PSUs, total current, average voltage
- **Battery Status**: Pack voltage, current, temperature, charging mode
- **WiFi Status**: Connection info, IP addresses, signal strength
- **Controls**: Set charging parameters, configure WiFi networks
- **Auto-refresh**: Updates every 2 seconds for real-time monitoring

### REST API Endpoints

#### System Status
- `GET /api/status` - Complete system status (flatpacks, battery, WiFi)
- `GET /api/system` - System information (chip model, memory, uptime)

#### Flatpack Management
- `GET /api/flatpacks` - Detailed flatpack PSU information
- Individual PSU data includes: serial, voltage, current, temperature, status, CAN bus

#### Battery Management
- `GET /api/battery` - Battery status and parameters
- `POST /api/battery` - Update battery parameters (chemistry, cell count, limits)

#### Charging Control
- `POST /api/charging` - Set charging parameters
  ```json
  {
    "voltage": 5440,  // Voltage in centivolts (54.40V)
    "current": 100,   // Current in deciamps (10.0A)
    "ovp": 5984       // Over-voltage protection in centivolts (59.84V)
  }
  ```

#### WiFi Configuration
- `GET /api/wifi` - Current WiFi configuration and status
- `POST /api/wifi` - Connect to WiFi network
  ```json
  {
    "ssid": "YourNetwork",
    "password": "YourPassword"
  }
  ```

#### System Control
- `POST /api/restart` - Restart the system

### Configuration

Web server features can be controlled via compile-time flags in `main.cpp`:

```cpp
#define ENABLE_WEB_SERVER true  // Enable/disable web server and WiFi
```

### Security Considerations

- Default AP password should be changed for production use
- Web interface has no authentication by default
- Consider implementing HTTPS for sensitive deployments
- API endpoints accept JSON payloads for configuration changes

## Debugging

Debug output is available via the serial console at 115200 baud. Additional debug options can be enabled by uncommenting debug defines in `main.cpp`:

```cpp
#define DEBUG_CAN_MESSAGES   // Enable to print all CAN messages
#define DEBUG_LOGIN_MESSAGES // Enable to print login refresh messages
```

## License

This project is proprietary and intended for specific use cases only. All rights reserved.

## Further Documentation

- `flatpack.md`: Details about the Flatpack2 CAN protocol
- `fp-Protocol.md`: Extended protocol documentation with message formats
- `pins.md`: Hardware pin assignments and configuration

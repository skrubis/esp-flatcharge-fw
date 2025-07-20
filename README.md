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

Handles battery charging logic:
- Support for multiple battery chemistries (LFP, LTO, NMC)
- CC/CV/Float charging mode transitions
- Temperature monitoring and protection
- Charging parameter calculation based on battery state

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

The main configuration parameters are defined at the top of `main.cpp`:

```cpp
#define SERIAL_BAUD 115200
#define STATUS_UPDATE_INTERVAL 15000   // Status display interval (15 sec)
#define LOGIN_REFRESH_INTERVAL 5000    // Login refresh interval (5 sec)
#define CHARGING_UPDATE_INTERVAL 1000  // Charging control update interval (1 sec)
#define BATTERY_CHEMISTRY BatteryChemistry::LFP  // Default battery chemistry
#define BATTERY_CELL_COUNT 16          // Default cell count (16S LFP = 51.2V nominal)
#define BATTERY_CAPACITY 100.0f        // Default capacity in Ah
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

1. Connect the ESP32S3 board to the Flatpack2 PSUs via CAN buses
2. Power on the system
3. The firmware will automatically:
   - Initialize hardware and CAN buses
   - Detect connected Flatpack2 PSUs
   - Log into detected PSUs
   - Begin charging based on configured battery parameters
   - Maintain login sessions with periodic refreshes
   - Monitor and display system status every 15 seconds

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

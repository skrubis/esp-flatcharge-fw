# ESP Flatcharge Firmware

This is the main firmware for the ESP32-S3 based Flatpack charger controller.

## What's New (2025-09)

- Gree 36S LTO BMS support over TWAI. Decodes per-cell voltages, two temperatures, and SOC using extended IDs `0x1811F401..0x1819F401`, `0x1808F401`, `0x1807F401`.
- ADS1220 24-bit current sensor integration (SPI) with NVS-persisted zero and scale calibration, exposed via REST API.
- New REST endpoints: `GET /api/gree` for LTO data, `GET/POST /api/ads1220` for current sensor status and calibration.
- Platform targets in PlatformIO: `vx1` (Vectrix Li-ion, 3 PSUs) and `gree_tractor` (LTO 36S, 2 PSUs).
- NMC/LG MEB safety: per-cell voltage clamped to 4.16V max; manual mode and AC presets supported via API.

## Current sensing

- Preferred: ADS1220 (24-bit ADC)
  - Hardware defaults: CS=GPIO39, DRDY=GPIO40 (configurable in code).
  - API: `GET /api/ads1220` returns `{"currentA", "valid", "zeroV", "apv"}`.
  - API: `POST /api/ads1220` accepts `{ "calibrateZero": <uint16 samples> }` or `{ "setScaleApv": <float A_per_V> }`.
  - Calibration is stored in NVS and restored at boot.

- Legacy: Analog Hall (CurrentSensorManager)
  - Reads an ADC channel and can optionally transmit TWAI frames on `0x1850F401` (disabled by default).
  - Kept for backward compatibility; no dedicated web UI. Prefer ADS1220 for higher precision and stability.

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
- Gree/Cree LTO BMS decode (36S) over TWAI with per-cell insight
- ADS1220 high-resolution current sensing with on-device calibration persistence
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

### Platform targets and building

Two dedicated environments are provided in `platformio.ini`:

- `env:vx1` — Vectrix VX1 (Li-ion/NMC), 3 PSUs in series
- `env:gree_tractor` — Gree/Cree LTO (36S), 2 PSUs in series

Build and upload examples:

```bash
# Vectrix VX1 build
pio run -e vx1
pio run -e vx1 -t upload

# Gree LTO build
pio run -e gree_tractor
pio run -e gree_tractor -t upload
```

Compile-time platform selection is in `include/BuildConfig.h` via
`BATTERY_PLATFORM_VX1` or `BATTERY_PLATFORM_GREE_TRACTOR` and `SERIES_PSU_COUNT`.
Per-PSU absolute voltage limits are also defined there.

### Configuration

- Feature flags in `src/main.cpp`:

```cpp
#define ENABLE_BATTERY_CHARGING true
#define ENABLE_VOLTAGE_SWEEP false
#define ENABLE_WEB_SERVER true
```

- Runtime battery and safety settings are exposed via REST (see API below). Defaults per chemistry are in `lib/BatteryManager/BatteryManager.cpp` and are applied by platform selection.

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
- `POST /api/battery` - Update parameters and safety flags
  - Body fields (send as needed):
    - `operatingMode`: "manual" | "bms"
    - `manualCurrentLimit`: float Amps (per PSU when in series)
    - `voltageDropCompensation`: float Volts
    - `voltageCalibrationOffset`: float Volts (displayed pack voltage)
    - `disableCurrentLimit`: bool (dangerous, bypasses firmware soft-start/clamps)
    - `defaultPerPsuVoltage`: float Volts (programmed into PSU NVRAM for fallback)
    - `acPreset`: uint8 (0=NONE, 1=1P 7A shared, 2=1P 15A shared, 3=3P 16A/PSU)
    - `maxCellVoltage`: float Volts per cell (chemistry-clamped; NMC hard cap 4.16V)

#### Charging Control
- `POST /api/charging` - Set charging parameters
  ```json
  {
    "voltage": 5440,  // Voltage in centivolts (54.40V)
    "current": 100,   // Current in deciamps (10.0A)
    "ovp": 5984       // Over-voltage protection in centivolts (59.84V)
  }
  ```
  Or enable/disable charging:
  ```json
  { "enabled": true }
  ```

#### Gree/Cree LTO (platform-specific)
- `GET /api/gree` - Returns per-cell array, pack voltage, temps, SOC

#### ADS1220 Current Sensor
- `GET /api/ads1220` - `{ "currentA": float, "valid": bool, "zeroV": float, "apv": float }`
- `POST /api/ads1220`
  ```json
  { "calibrateZero": 64 }
  // or
  { "setScaleApv": 149.25 }
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

## Safety and best practices (NMC / LG MEB)

- **Max per-cell voltage** for NMC must not exceed **4.16V**. Use `POST /api/battery { "maxCellVoltage": 4.16 }`.
- Prefer **manual operating mode** during bench tests: `POST /api/battery { "operatingMode": "manual" }`.
- **AC presets** help derive safe per-PSU DC current:
  - Single-phase adapters: keep around **5A or less per PSU**.
  - Three-phase: up to **~13A per phase**; set manual current accordingly.
- When testing with no battery (multimeter only), start with low current limits and ensure OVP headroom.

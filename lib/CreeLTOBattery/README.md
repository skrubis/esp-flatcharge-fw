# Cree LTO Battery Library for ESP32

A comprehensive ESP32 library for communicating with Cree 80V Lithium-Titanate (LTO) battery packs via TWAI (Two-Wire Automotive Interface) CAN bus. This library is specifically designed for the ESP Flatcharge project and provides real-time monitoring of battery parameters.

## Features

- **Real-time Battery Monitoring**: Monitor all 36 cell voltages, temperatures, and state of charge
- **Thread-Safe Operation**: Uses FreeRTOS mutexes for safe multi-threaded access
- **Callback System**: Register callbacks for battery data updates, raw CAN frames, and errors
- **Comprehensive Statistics**: Track frame counts, decode errors, and communication health
- **Data Validation**: Automatic validation of received data with age checking
- **Easy Integration**: Simple API designed for seamless integration into existing projects

## Protocol Details

Based on reverse engineering of the Cree 80V LTO battery CAN protocol:

| Parameter | CAN ID | Data Format | Engineering Formula |
|-----------|--------|-------------|-------------------|
| Cell Voltages 1-36 | 0x1811F401 - 0x1819F401 | 4 cells per frame, little-endian | V = uint16_le / 1000 |
| Temperatures | 0x1808F401 | T1: bytes 2-3 (LE), T2: bytes 4-5 (BE) | T = uint16 / 32 °C |
| State of Charge | 0x1807F401 | Byte 6 | SOC = byte6 / 2.55 % |

## Hardware Requirements

- **ESP32-S3** (or compatible ESP32 with TWAI support)
- **CAN Transceiver** (e.g., MCP2551, TJA1050)
- **Cree 80V LTO Battery Pack** with active BMS

### Default Pin Configuration

- **TX Pin**: GPIO 21
- **RX Pin**: GPIO 22
- **CAN Speed**: 500 kbps

## Installation

1. Copy the `CreeLTOBattery` folder to your PlatformIO `lib` directory
2. Include the library in your project:

```cpp
#include <CreeLTOBattery.h>
```

## Quick Start

### Basic Usage

```cpp
#include <CreeLTOBattery.h>

CreeLTOBattery ltoBattery;

void setup() {
    Serial.begin(115200);
    
    // Initialize with default pins (TX: GPIO21, RX: GPIO22)
    if (ltoBattery.begin()) {
        Serial.println("Battery monitoring started!");
    } else {
        Serial.println("Failed to initialize!");
    }
}

void loop() {
    if (ltoBattery.hasRecentData()) {
        Serial.printf("Pack: %.2fV, SOC: %.1f%%\n",
                     ltoBattery.getPackVoltage(),
                     ltoBattery.getSOC());
    }
    delay(1000);
}
```

### Advanced Usage with Callbacks

```cpp
#include <CreeLTOBattery.h>

CreeLTOBattery ltoBattery;

void setup() {
    Serial.begin(115200);
    
    // Set up callbacks before initialization
    ltoBattery.onBatteryData([](const CreeLTOBatteryData& data) {
        Serial.printf("Battery Update: %.2fV, %.1f%%\n",
                     data.pack_voltage, data.soc_percent);
    });
    
    ltoBattery.onError([](const char* error_msg, uint32_t error_code) {
        Serial.printf("Error: %s (Code: %lu)\n", error_msg, error_code);
    });
    
    // Initialize with custom pins
    if (ltoBattery.begin(GPIO_NUM_5, GPIO_NUM_4)) {
        Serial.println("Advanced monitoring started!");
    }
}

void loop() {
    // Callbacks handle real-time updates
    delay(100);
}
```

## API Reference

### Initialization

#### `bool begin(gpio_num_t tx_pin, gpio_num_t rx_pin, twai_timing_config_t timing)`

Initialize the library with specified pins and timing configuration.

**Parameters:**
- `tx_pin`: CAN TX pin (default: GPIO_NUM_21)
- `rx_pin`: CAN RX pin (default: GPIO_NUM_22)  
- `timing`: TWAI timing configuration (default: 500kbps)

**Returns:** `true` if initialization successful, `false` otherwise

#### `void end()`

Stop the library and free all resources.

### Callback Registration

#### `void onBatteryData(BatteryDataCallback callback)`

Register callback for battery data updates.

```cpp
ltoBattery.onBatteryData([](const CreeLTOBatteryData& data) {
    // Handle battery data update
});
```

#### `void onRawFrame(RawFrameCallback callback)`

Register callback for raw CAN frame monitoring.

```cpp
ltoBattery.onRawFrame([](uint32_t can_id, const uint8_t* data, uint8_t length) {
    // Handle raw CAN frame
});
```

#### `void onError(ErrorCallback callback)`

Register callback for error notifications.

```cpp
ltoBattery.onError([](const char* error_msg, uint32_t error_code) {
    // Handle error
});
```

### Data Access Methods

#### Individual Parameter Access

- `float getPackVoltage()` - Get total pack voltage (V)
- `float getCellVoltage(uint8_t cell_number)` - Get individual cell voltage (1-36)
- `float getMinCellVoltage()` - Get minimum cell voltage (V)
- `float getMaxCellVoltage()` - Get maximum cell voltage (V)
- `float getCellVoltageDelta()` - Get cell voltage spread (V)
- `float getTemperature1()` - Get temperature probe 1 (°C)
- `float getTemperature2()` - Get temperature probe 2 (°C)
- `float getSOC()` - Get state of charge (%)

#### Bulk Data Access

#### `CreeLTOBatteryData getBatteryData()`

Get complete battery data structure with all parameters and validity flags.

#### `CreeLTOStatistics getStatistics()`

Get communication statistics including frame counts and error counters.

### Status Methods

#### `bool isConnected()`

Check if library is initialized and running.

#### `bool hasRecentData(uint32_t max_age_ms = 5000)`

Check if recent battery data has been received within specified time.

#### `bool isDataValid(uint32_t max_age_ms = 5000)`

Alias for `hasRecentData()`.

#### `uint32_t getLastUpdateTime()`

Get timestamp of most recent data update.

### Utility Methods

#### `void printBatteryData()`

Print formatted battery data to Serial.

#### `void printStatistics()`

Print communication statistics to Serial.

#### `void resetStatistics()`

Reset all statistics counters.

## Data Structures

### CreeLTOBatteryData

Complete battery data structure:

```cpp
struct CreeLTOBatteryData {
    float cell_voltages[36];    // Individual cell voltages (V)
    float pack_voltage;         // Total pack voltage (V)
    float min_cell_voltage;     // Minimum cell voltage (V)
    float max_cell_voltage;     // Maximum cell voltage (V)
    float cell_voltage_delta;   // Cell voltage spread (V)
    float temperature_1;        // Temperature probe 1 (°C)
    float temperature_2;        // Temperature probe 2 (°C)
    float soc_percent;          // State of charge (%)
    
    // Data validity flags
    bool cells_valid;
    bool temperature_valid;
    bool soc_valid;
    
    // Update timestamps
    uint32_t last_cell_update;
    uint32_t last_temperature_update;
    uint32_t last_soc_update;
};
```

### CreeLTOStatistics

Communication statistics:

```cpp
struct CreeLTOStatistics {
    uint32_t frames_received;           // Total frames received
    uint32_t cell_frames_received;      // Cell voltage frames
    uint32_t temperature_frames_received; // Temperature frames
    uint32_t soc_frames_received;       // SOC frames
    uint32_t unknown_frames_received;   // Unknown frames
    uint32_t decode_errors;             // Decode error count
    uint32_t bus_errors;                // CAN bus errors
    uint32_t last_error_time;           // Last error timestamp
};
```

## Examples

The library includes comprehensive examples:

### Basic Usage (`examples/BasicUsage/BasicUsage.ino`)

Demonstrates:
- Library initialization
- Reading battery parameters
- Basic data callbacks
- Periodic status printing

### Advanced Usage (`examples/AdvancedUsage/AdvancedUsage.ino`)

Demonstrates:
- Raw CAN frame monitoring
- Battery health monitoring
- Data trend analysis
- Error handling and recovery
- Historical data logging

## Troubleshooting

### Common Issues

**No data received:**
- Check CAN transceiver wiring
- Verify battery is powered and BMS is active
- Confirm CAN bus termination (120Ω resistors)
- Check baud rate (should be 500kbps)

**Decode errors:**
- Verify CAN ID filters are correct
- Check for electrical noise on CAN lines
- Ensure proper grounding

**High frame loss:**
- Reduce other CAN traffic
- Check for CAN bus overload
- Verify ESP32 task priorities

### Debug Output

Enable debug output by calling:

```cpp
ltoBattery.onRawFrame([](uint32_t can_id, const uint8_t* data, uint8_t length) {
    Serial.printf("CAN: ID=0x%08X, Data=", can_id);
    for (int i = 0; i < length; i++) {
        Serial.printf("%02X ", data[i]);
    }
    Serial.println();
});
```

### Performance Considerations

- The library uses a dedicated FreeRTOS task for CAN reception
- Callbacks are executed in the context of the receive task
- Keep callback functions lightweight to avoid blocking reception
- Use `hasRecentData()` to check data validity before processing

## Technical Details

### CAN Frame Processing

The library automatically processes the following CAN frames:

1. **Cell Voltages** (0x1811F401 - 0x1819F401): 9 frames containing 4 cell voltages each
2. **Pack Current** (0x1804F401): Single frame with current in bytes 2-3
3. **Temperatures** (0x1808F401): Single frame with two temperature probes
4. **State of Charge** (0x1807F401): Single frame with SOC in byte 6

### Thread Safety

All public methods are thread-safe using FreeRTOS mutexes. The library can be safely called from multiple tasks simultaneously.

### Memory Usage

- **RAM**: ~2KB for data structures and task stack
- **Flash**: ~15KB for library code
- **Task Stack**: 4KB (configurable via `CREE_LTO_TASK_STACK_SIZE`)

## Integration with ESP Flatcharge Project

This library is designed to integrate seamlessly with the existing ESP Flatcharge firmware:

```cpp
// In your main application
#include <CreeLTOBattery.h>

CreeLTOBattery ltoBattery;

void setup() {
    // Initialize other managers first
    // ...
    
    // Initialize LTO battery monitoring
    if (ltoBattery.begin(GPIO_NUM_5, GPIO_NUM_4)) { // Use available pins
        ltoBattery.onBatteryData([](const CreeLTOBatteryData& data) {
            // Integrate with existing battery management logic
            updateBatteryStatus(data);
        });
    }
}
```

## License

This library is part of the ESP Flatcharge project. See project license for details.

## Contributing

This library was generated based on analysis of the gree-lto-logger4 project. Contributions and improvements are welcome.

## Version History

- **v1.0.0**: Initial release with full Cree LTO battery support

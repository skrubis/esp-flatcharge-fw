/**
 * @file CreeLTOBattery.h
 * @brief Cree 80V LTO Battery CAN Communication Library for ESP32 TWAI
 * @author Generated for ESP Flatcharge Project
 * @version 1.0.0
 * 
 * This library provides communication interface for Cree 80V Lithium-Titanate (LTO) 
 * battery packs via ESP32 TWAI CAN bus. Based on protocol analysis from gree-lto-logger4.
 * 
 * Protocol Details:
 * - Cell voltages: IDs 0x1811F401 - 0x1819F401 (36 cells, 4 per frame)
 * - Pack current: ID 0x1804F401 (bytes 2-3, big-endian, scale 0.005A)
 * - Temperatures: ID 0x1808F401 (2 probes, bytes 2-5)
 * - State of Charge: ID 0x1807F401 (byte 6)
 */

#ifndef CREE_LTO_BATTERY_H
#define CREE_LTO_BATTERY_H

#include <Arduino.h>
#include <driver/twai.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <functional>

// Library version
#define CREE_LTO_BATTERY_VERSION "1.0.0"

// Cree LTO Battery CAN IDs (29-bit extended format)
#define CREE_LTO_CELL_VOLTAGE_BASE  0x1811F401  // Cell voltages 1-4
#define CREE_LTO_CELL_VOLTAGE_END   0x1819F401  // Cell voltages 33-36
#define CREE_LTO_TEMPERATURE_ID     0x1808F401  // Temperature probes
#define CREE_LTO_SOC_ID             0x1807F401  // State of charge

// Battery specifications
#define CREE_LTO_CELL_COUNT         36
#define CREE_LTO_NOMINAL_VOLTAGE    80.0f
#define CREE_LTO_CELLS_PER_FRAME    4
#define CREE_LTO_VOLTAGE_FRAMES     9

// Default TWAI configuration
#define CREE_LTO_DEFAULT_TX_PIN     GPIO_NUM_21
#define CREE_LTO_DEFAULT_RX_PIN     GPIO_NUM_22
#define CREE_LTO_DEFAULT_BITRATE    TWAI_TIMING_CONFIG_500KBITS
#define CREE_LTO_QUEUE_SIZE         20
#define CREE_LTO_TASK_STACK_SIZE    4096
#define CREE_LTO_TASK_PRIORITY      5

// Battery data structure
struct CreeLTOBatteryData {
    // Cell voltages (V)
    float cell_voltages[CREE_LTO_CELL_COUNT];
    float pack_voltage;
    float min_cell_voltage;
    float max_cell_voltage;
    float cell_voltage_delta;
    
    // Temperatures (°C)
    float temperature_1;
    float temperature_2;
    
    // State of charge (%)
    float soc_percent;
    
    // Data validity flags
    bool cells_valid;
    bool temperature_valid;
    bool soc_valid;
    
    // Timestamps
    uint32_t last_cell_update;
    uint32_t last_temperature_update;
    uint32_t last_soc_update;
    
    CreeLTOBatteryData() {
        memset(cell_voltages, 0, sizeof(cell_voltages));
        pack_voltage = 0.0f;
        min_cell_voltage = 0.0f;
        max_cell_voltage = 0.0f;
        cell_voltage_delta = 0.0f;
        temperature_1 = 0.0f;
        temperature_2 = 0.0f;
        soc_percent = 0.0f;
        cells_valid = false;
        temperature_valid = false;
        soc_valid = false;
        last_cell_update = 0;
        last_temperature_update = 0;
        last_soc_update = 0;
    }
};

// Statistics structure
struct CreeLTOStatistics {
    uint32_t frames_received;
    uint32_t cell_frames_received;
    uint32_t temperature_frames_received;
    uint32_t soc_frames_received;
    uint32_t unknown_frames_received;
    uint32_t decode_errors;
    uint32_t bus_errors;
    uint32_t last_error_time;
    
    CreeLTOStatistics() {
        memset(this, 0, sizeof(CreeLTOStatistics));
    }
};

// Callback function types
typedef std::function<void(const CreeLTOBatteryData&)> BatteryDataCallback;
typedef std::function<void(uint32_t can_id, const uint8_t* data, uint8_t length)> RawFrameCallback;
typedef std::function<void(const char* error_msg, uint32_t error_code)> ErrorCallback;

class CreeLTOBattery {
private:
    // TWAI configuration
    gpio_num_t tx_pin;
    gpio_num_t rx_pin;
    twai_timing_config_t timing_config;
    twai_general_config_t general_config;
    twai_filter_config_t filter_config;
    
    // Task handles
    TaskHandle_t receive_task_handle;
    QueueHandle_t message_queue;
    
    // Battery data
    CreeLTOBatteryData battery_data;
    CreeLTOStatistics statistics;
    
    // Callbacks
    BatteryDataCallback data_callback;
    RawFrameCallback raw_frame_callback;
    ErrorCallback error_callback;
    
    // Internal state
    bool initialized;
    bool running;
    SemaphoreHandle_t data_mutex;
    
    // Private methods
    static void receiveTask(void* parameter);
    void processCANFrame(const twai_message_t& message);
    bool decodeCellVoltages(uint32_t can_id, const uint8_t* data);
    bool decodeTemperatures(const uint8_t* data);
    bool decodeSOC(const uint8_t* data);
    void updatePackStatistics();
    void handleError(const char* error_msg, uint32_t error_code = 0);
    
    // Utility functions (inspired by gree-lto-logger4)
    static uint16_t extractUint16LE(const uint8_t* data, uint8_t offset);
    static uint16_t extractUint16BE(const uint8_t* data, uint8_t offset);
    
public:
    CreeLTOBattery();
    ~CreeLTOBattery();
    
    // Configuration methods
    bool begin(gpio_num_t tx_pin = CREE_LTO_DEFAULT_TX_PIN, 
               gpio_num_t rx_pin = CREE_LTO_DEFAULT_RX_PIN,
               twai_timing_config_t timing = CREE_LTO_DEFAULT_BITRATE);
    void end();
    
    // Callback registration
    void onBatteryData(BatteryDataCallback callback);
    void onRawFrame(RawFrameCallback callback);
    void onError(ErrorCallback callback);
    
    // Data access methods
    CreeLTOBatteryData getBatteryData();
    CreeLTOStatistics getStatistics();
    bool isDataValid(uint32_t max_age_ms = 5000);
    
    // Individual data getters
    float getPackVoltage();
    float getCellVoltage(uint8_t cell_number); // 1-36
    float getMinCellVoltage();
    float getMaxCellVoltage();
    float getCellVoltageDelta();
    float getTemperature1();
    float getTemperature2();
    float getSOC();
    
    // Status methods
    bool isConnected();
    bool hasRecentData(uint32_t max_age_ms = 5000);
    uint32_t getLastUpdateTime();
    
    // Utility methods
    void printBatteryData();
    void printStatistics();
    void resetStatistics();
    
    // Advanced methods
    void setCustomFilter(uint32_t filter_id, uint32_t filter_mask);
    void enableRawFrameLogging(bool enable);
};

#endif // CREE_LTO_BATTERY_H

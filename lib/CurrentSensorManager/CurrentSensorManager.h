/**
 * @file CurrentSensorManager.h
 * @brief External Hall Current Sensor Manager for ESP Flatcharge
 * 
 * This library manages external hall effect current sensors connected to ADC channels.
 * Provides calibrated current readings and transmits data via TWAI CAN bus for 
 * external monitoring and logging systems.
 * 
 * Features:
 * - Configurable ADC channel selection
 * - User-adjustable offset and scaling calibration
 * - Periodic CAN message transmission
 * - Web interface integration for settings
 * - Compatible with all charging modes
 * 
 * Copyright (C) 2025 ESP Flatcharge Project
 */

#pragma once

#include <Arduino.h>
#include <driver/twai.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <functional>

/**
 * @brief Current sensor configuration structure
 */
struct CurrentSensorConfig {
    uint8_t adcChannel;         // ADC channel (0-7 for ESP32-S3)
    float offsetVoltage;        // Zero current voltage offset (V)
    float scaleFactor;          // Amps per volt (A/V)
    uint16_t transmitInterval;  // CAN transmission interval (ms)
    bool enabled;               // Enable/disable sensor
};

/**
 * @brief Current sensor data structure
 */
struct CurrentSensorData {
    float rawVoltage;           // Raw ADC voltage (V)
    float calibratedCurrent;    // Calibrated current (A)
    uint32_t timestamp;         // Measurement timestamp (ms)
    bool dataValid;             // Data validity flag
};

/**
 * @brief Current sensor callback function types
 */
using CurrentDataCallback = std::function<void(const CurrentSensorData&)>;

/**
 * @brief Current Sensor Manager Class
 */
class CurrentSensorManager {
public:
    /**
     * @brief Constructor
     */
    CurrentSensorManager();
    
    /**
     * @brief Destructor
     */
    ~CurrentSensorManager();
    
    /**
     * @brief Initialize current sensor manager
     * @param config Initial sensor configuration
     * @return true if initialization successful
     */
    bool initialize(const CurrentSensorConfig& config);
    
    /**
     * @brief Update sensor readings and transmit CAN messages
     * Call this periodically from main loop
     */
    void update();
    
    /**
     * @brief Get current sensor reading
     * @return Current sensor data structure
     */
    CurrentSensorData getCurrentReading();
    
    /**
     * @brief Update sensor configuration
     * @param config New configuration settings
     */
    void updateConfiguration(const CurrentSensorConfig& config);
    
    /**
     * @brief Get current configuration
     * @return Current sensor configuration
     */
    CurrentSensorConfig getConfiguration() const;
    
    /**
     * @brief Set data callback for real-time updates
     * @param callback Function to call when new data is available
     */
    void setDataCallback(CurrentDataCallback callback);
    
    /**
     * @brief Calibrate sensor with known current
     * @param knownCurrent Known current value (A)
     * @param measuredVoltage Measured ADC voltage (V)
     */
    void calibrateWithKnownCurrent(float knownCurrent, float measuredVoltage);
    
    /**
     * @brief Set zero current offset
     * Call this when no current is flowing
     */
    void setZeroCurrentOffset();
    
    /**
     * @brief Print sensor status and readings
     */
    void printStatus() const;

private:
    // Configuration
    CurrentSensorConfig config;
    
    // Data storage
    CurrentSensorData sensorData;
    
    // Timing
    uint32_t lastReadingTime;
    uint32_t lastTransmitTime;
    
    // Thread safety
    SemaphoreHandle_t dataMutex;
    
    // Callback
    CurrentDataCallback dataCallback;
    
    // Constants
    static constexpr uint16_t ADC_RESOLUTION = 4095;    // 12-bit ADC
    static constexpr float ADC_VREF = 3.3f;             // Reference voltage
    static constexpr uint8_t ADC_SAMPLES = 16;          // Averaging samples
    static constexpr uint32_t CAN_ID_CURRENT = 0x1850F401; // CAN ID for current data
    
    // Default configuration values
    static constexpr uint8_t DEFAULT_ADC_CHANNEL = 0;
    static constexpr float DEFAULT_OFFSET_VOLTAGE = 1.65f;  // Typical for 3.3V/2 offset
    static constexpr float DEFAULT_SCALE_FACTOR = 20.0f;    // 20A/V for typical hall sensor
    static constexpr uint16_t DEFAULT_TRANSMIT_INTERVAL = 100; // 100ms = 10Hz
    
    /**
     * @brief Read ADC channel with averaging
     * @return Average ADC voltage in volts
     */
    float readADCVoltage();
    
    /**
     * @brief Convert ADC voltage to calibrated current
     * @param voltage ADC voltage reading
     * @return Calibrated current in amps
     */
    float voltageToCurrent(float voltage);
    
    /**
     * @brief Transmit current data via CAN
     */
    void transmitCANMessage();
    
    /**
     * @brief Validate ADC channel number
     * @param channel ADC channel to validate
     * @return true if channel is valid
     */
    bool isValidADCChannel(uint8_t channel) const;
};

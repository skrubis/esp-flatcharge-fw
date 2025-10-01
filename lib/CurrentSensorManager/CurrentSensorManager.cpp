/**
 * @file CurrentSensorManager.cpp
 * @brief Implementation of External Hall Current Sensor Manager
 * 
 * Copyright (C) 2025 ESP Flatcharge Project
 */

#include "CurrentSensorManager.h"

/**
 * @brief Constructor
 */
CurrentSensorManager::CurrentSensorManager() :
    lastReadingTime(0),
    lastTransmitTime(0),
    dataMutex(nullptr),
    dataCallback(nullptr)
{
    // Initialize default configuration
    config.adcChannel = DEFAULT_ADC_CHANNEL;
    config.offsetVoltage = DEFAULT_OFFSET_VOLTAGE;
    config.scaleFactor = DEFAULT_SCALE_FACTOR;
    config.transmitInterval = DEFAULT_TRANSMIT_INTERVAL;
    // Default disabled until initialize() is called with a valid configuration
    config.enabled = false;
    
    // Initialize sensor data
    sensorData.rawVoltage = 0.0f;
    sensorData.calibratedCurrent = 0.0f;
    sensorData.timestamp = 0;
    sensorData.dataValid = false;
}

/**
 * @brief Destructor
 */
CurrentSensorManager::~CurrentSensorManager()
{
    if (dataMutex) {
        vSemaphoreDelete(dataMutex);
        dataMutex = nullptr;
    }
}

/**
 * @brief Initialize current sensor manager
 */
bool CurrentSensorManager::initialize(const CurrentSensorConfig& initialConfig)
{
    Serial.println("[CurrentSensor] Initializing current sensor manager...");
    
    // Validate configuration
    if (!isValidADCChannel(initialConfig.adcChannel)) {
        Serial.printf("[CurrentSensor] ERROR: Invalid ADC channel %d\n", initialConfig.adcChannel);
        return false;
    }
    
    // Create mutex for thread safety
    dataMutex = xSemaphoreCreateMutex();
    if (!dataMutex) {
        Serial.println("[CurrentSensor] ERROR: Failed to create data mutex");
        return false;
    }
    
    // Update configuration
    config = initialConfig;
    
    // Initialize ADC
    analogReadResolution(12);  // Set 12-bit resolution
    analogSetAttenuation(ADC_11db);  // Set attenuation for 3.3V range
    
    Serial.printf("[CurrentSensor] Initialized on ADC channel %d\n", config.adcChannel);
    Serial.printf("[CurrentSensor] Offset: %.3fV, Scale: %.1fA/V, Interval: %dms\n", 
                  config.offsetVoltage, config.scaleFactor, config.transmitInterval);
    
    return true;
}

/**
 * @brief Update sensor readings and transmit CAN messages
 */
void CurrentSensorManager::update()
{
    // Not initialized or explicitly disabled: do nothing
    if (!config.enabled || dataMutex == nullptr) {
        return;
    }
    
    uint32_t currentTime = millis();
    
    // Read sensor every 10ms for smooth readings
    if (currentTime - lastReadingTime >= 10) {
        lastReadingTime = currentTime;
        
        // Read ADC voltage
        float voltage = readADCVoltage();
        float current = voltageToCurrent(voltage);
        
        // Update sensor data with thread safety
        if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            sensorData.rawVoltage = voltage;
            sensorData.calibratedCurrent = current;
            sensorData.timestamp = currentTime;
            sensorData.dataValid = true;
            xSemaphoreGive(dataMutex);
            
            // Call callback if set
            if (dataCallback) {
                dataCallback(sensorData);
            }
        }
    }
    
    // Transmit CAN message at configured interval
    if (currentTime - lastTransmitTime >= config.transmitInterval) {
        lastTransmitTime = currentTime;
        transmitCANMessage();
    }
}

/**
 * @brief Get current sensor reading
 */
CurrentSensorData CurrentSensorManager::getCurrentReading()
{
    CurrentSensorData data;
    
    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        data = sensorData;
        xSemaphoreGive(dataMutex);
    } else {
        // Return invalid data if mutex timeout
        data.dataValid = false;
    }
    
    return data;
}

/**
 * @brief Update sensor configuration
 */
void CurrentSensorManager::updateConfiguration(const CurrentSensorConfig& newConfig)
{
    if (!isValidADCChannel(newConfig.adcChannel)) {
        Serial.printf("[CurrentSensor] ERROR: Invalid ADC channel %d\n", newConfig.adcChannel);
        return;
    }
    
    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        config = newConfig;
        xSemaphoreGive(dataMutex);
        
        Serial.printf("[CurrentSensor] Configuration updated: CH%d, Offset=%.3fV, Scale=%.1fA/V\n",
                      config.adcChannel, config.offsetVoltage, config.scaleFactor);
    }
}

/**
 * @brief Get current configuration
 */
CurrentSensorConfig CurrentSensorManager::getConfiguration() const
{
    CurrentSensorConfig cfg;
    
    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        cfg = config;
        xSemaphoreGive(dataMutex);
    }
    
    return cfg;
}

/**
 * @brief Set data callback
 */
void CurrentSensorManager::setDataCallback(CurrentDataCallback callback)
{
    dataCallback = callback;
}

/**
 * @brief Calibrate sensor with known current
 */
void CurrentSensorManager::calibrateWithKnownCurrent(float knownCurrent, float measuredVoltage)
{
    if (abs(knownCurrent) < 0.1f) {
        Serial.println("[CurrentSensor] WARNING: Use setZeroCurrentOffset() for zero current calibration");
        return;
    }
    
    // Calculate new scale factor: A/V = known_current / (measured_voltage - offset)
    float voltageAboveOffset = measuredVoltage - config.offsetVoltage;
    if (abs(voltageAboveOffset) < 0.01f) {
        Serial.println("[CurrentSensor] ERROR: Voltage too close to offset for calibration");
        return;
    }
    
    float newScaleFactor = knownCurrent / voltageAboveOffset;
    
    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        config.scaleFactor = newScaleFactor;
        xSemaphoreGive(dataMutex);
        
        Serial.printf("[CurrentSensor] Calibrated: %.1fA at %.3fV -> Scale factor: %.1fA/V\n",
                      knownCurrent, measuredVoltage, newScaleFactor);
    }
}

/**
 * @brief Set zero current offset
 */
void CurrentSensorManager::setZeroCurrentOffset()
{
    float currentVoltage = readADCVoltage();
    
    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        config.offsetVoltage = currentVoltage;
        xSemaphoreGive(dataMutex);
        
        Serial.printf("[CurrentSensor] Zero offset set to %.3fV\n", currentVoltage);
    }
}

/**
 * @brief Print sensor status
 */
void CurrentSensorManager::printStatus() const
{
    CurrentSensorData data = const_cast<CurrentSensorManager*>(this)->getCurrentReading();
    
    Serial.println("\n=== CURRENT SENSOR STATUS ===");
    Serial.printf("ADC Channel: %d\n", config.adcChannel);
    Serial.printf("Enabled: %s\n", config.enabled ? "YES" : "NO");
    Serial.printf("Offset Voltage: %.3f V\n", config.offsetVoltage);
    Serial.printf("Scale Factor: %.1f A/V\n", config.scaleFactor);
    Serial.printf("Transmit Interval: %d ms\n", config.transmitInterval);
    
    if (data.dataValid) {
        Serial.printf("Raw Voltage: %.3f V\n", data.rawVoltage);
        Serial.printf("Calibrated Current: %.2f A\n", data.calibratedCurrent);
        Serial.printf("Data Age: %lu ms\n", millis() - data.timestamp);
    } else {
        Serial.println("Data: INVALID");
    }
    Serial.println("=============================");
}

/**
 * @brief Read ADC channel with averaging
 */
float CurrentSensorManager::readADCVoltage()
{
    uint32_t adcSum = 0;
    
    // Take multiple samples for averaging
    for (uint8_t i = 0; i < ADC_SAMPLES; i++) {
        adcSum += analogRead(config.adcChannel);
        delayMicroseconds(100);  // Small delay between samples
    }
    
    // Calculate average and convert to voltage
    float adcAverage = (float)adcSum / ADC_SAMPLES;
    float voltage = (adcAverage / ADC_RESOLUTION) * ADC_VREF;
    
    return voltage;
}

/**
 * @brief Convert ADC voltage to calibrated current
 */
float CurrentSensorManager::voltageToCurrent(float voltage)
{
    // Current = (Voltage - Offset) * Scale Factor
    return (voltage - config.offsetVoltage) * config.scaleFactor;
}

/**
 * @brief Transmit current data via CAN
 */
void CurrentSensorManager::transmitCANMessage()
{
    if (!sensorData.dataValid) {
        return;
    }
    
    // Prepare CAN message
    twai_message_t message;
    message.identifier = CAN_ID_CURRENT;
    message.flags = TWAI_MSG_FLAG_EXTD;  // Extended ID
    message.data_length_code = 8;
    
    // Pack data into CAN frame
    // Bytes 0-3: Current in mA (signed 32-bit, little endian)
    int32_t currentMilliAmps = (int32_t)(sensorData.calibratedCurrent * 1000.0f);
    message.data[0] = currentMilliAmps & 0xFF;
    message.data[1] = (currentMilliAmps >> 8) & 0xFF;
    message.data[2] = (currentMilliAmps >> 16) & 0xFF;
    message.data[3] = (currentMilliAmps >> 24) & 0xFF;
    
    // Bytes 4-5: Raw voltage in mV (unsigned 16-bit, little endian)
    uint16_t voltageMilliVolts = (uint16_t)(sensorData.rawVoltage * 1000.0f);
    message.data[4] = voltageMilliVolts & 0xFF;
    message.data[5] = (voltageMilliVolts >> 8) & 0xFF;
    
    // Bytes 6-7: Timestamp low 16 bits (little endian)
    uint16_t timestampLow = sensorData.timestamp & 0xFFFF;
    message.data[6] = timestampLow & 0xFF;
    message.data[7] = (timestampLow >> 8) & 0xFF;
    
    // Transmit message
    esp_err_t result = twai_transmit(&message, pdMS_TO_TICKS(10));
    if (result != ESP_OK) {
        Serial.printf("[CurrentSensor] CAN transmit error: %d\n", result);
    }
}

/**
 * @brief Validate ADC channel number
 */
bool CurrentSensorManager::isValidADCChannel(uint8_t channel) const
{
    // ESP32-S3 has ADC channels 0-9 on ADC1 and 0-9 on ADC2
    // For simplicity, we'll use ADC1 channels 0-7
    return (channel <= 7);
}

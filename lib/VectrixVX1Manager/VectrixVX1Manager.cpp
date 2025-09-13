/*
 * VectrixVX1Manager.cpp - Vectrix VX1 BMS CAN Message Handler Implementation
 * 
 * This library handles CAN messages from Vectrix VX1 BMS systems,
 * specifically decoding FEF3 messages for battery monitoring and charging control.
 * 
 * Copyright (C) 2025 ESP Flatcharge Project
 */

#include "VectrixVX1Manager.h"

/**
 * @brief Constructor
 */
VectrixVX1Manager::VectrixVX1Manager() : 
    cellCount(36),
    targetCellVoltage(CELL_VOLTAGE_DEFAULT_TARGET),
    lastTempMax(-127),
    lastTempMin(127),
    lastTempUpdateTime(0),
    dataMutex(nullptr),
    dataCallback(nullptr),
    chargingCallback(nullptr)
{
}

/**
 * @brief Destructor
 */
VectrixVX1Manager::~VectrixVX1Manager()
{
    if (dataMutex) {
        vSemaphoreDelete(dataMutex);
        dataMutex = nullptr;
    }
}

/**
 * @brief Initialize the VX1 manager
 */
bool VectrixVX1Manager::initialize(uint8_t cellCount)
{
    this->cellCount = cellCount;
    
    // Create mutex for thread-safe data access
    dataMutex = xSemaphoreCreateMutex();
    if (!dataMutex) {
        Serial.println("[VX1Manager] ERROR: Failed to create mutex");
        return false;
    }
    
    // Initialize BMS data
    bmsData = VX1BmsData();
    chargingParams = VX1ChargingParams();
    
    Serial.printf("[VX1Manager] Initialized for %dS Li-ion pack\n", cellCount);
    return true;
}

/**
 * @brief Process incoming CAN message
 */
bool VectrixVX1Manager::processCanMessage(uint32_t canId, const uint8_t* data, uint8_t dlc)
{
    if (!data || dlc != 8) {
        return false;
    }
    
    uint8_t sourceAddress;
    if (!isVX1FEF3Message(canId, sourceAddress)) {
        return false;
    }
    
    // Decode the FEF3 message
    if (decodeFEF3Message(data, sourceAddress)) {
        // Update charging parameters based on new data
        updateChargingParameters();
        
        // Call callbacks if set
        if (dataCallback) {
            dataCallback(bmsData);
        }
        if (chargingCallback) {
            chargingCallback(chargingParams);
        }
        
        return true;
    }
    
    return false;
}

/**
 * @brief Decode FEF3 message data
 */
bool VectrixVX1Manager::decodeFEF3Message(const uint8_t* data, uint8_t sourceAddress)
{
    if (!dataMutex) return false;
    
    // Lock mutex for thread-safe access
    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        
        // Decode FEF3 message according to VX1 BMS format
        // Byte 0: Low temperature (°C, no offset)
        bmsData.tempMin = static_cast<int8_t>(data[0]);
        
        // Byte 1: High temperature (°C, no offset)  
        bmsData.tempMax = static_cast<int8_t>(data[1]);
        
        // Byte 2: Ambient temperature (°C, no offset)
        bmsData.tempAmbient = static_cast<int8_t>(data[2]);
        
        // Bytes 3-4: Cell high voltage (12-bit, 1.5mV/bit) + highest cell number (4-bit)
        uint16_t cellHighRaw = data[3] | ((data[4] & 0x0F) << 8);
        bmsData.cellVoltageMax = cellHighRaw * VOLTAGE_SCALE;  // Convert to mV
        bmsData.cellIndexMax = (data[4] >> 4) & 0x0F;
        
        // Bytes 5-6: Cell low voltage (12-bit, 1.5mV/bit) + lowest cell number (4-bit)
        uint16_t cellLowRaw = data[5] | ((data[6] & 0x0F) << 8);
        bmsData.cellVoltageMin = cellLowRaw * VOLTAGE_SCALE;   // Convert to mV
        bmsData.cellIndexMin = (data[6] >> 4) & 0x0F;
        
        // Byte 7: Battery module number (4-bit) + thermal switch status (4-bit)
        // NOTE: Module number is unreliable garbage from master node - ignore it
        bmsData.moduleNumber = 0;  // Ignore unreliable module number
        bmsData.thermalSwitch = data[7] & 0x0F;
        
        // Calculate derived values
        bmsData.voltageDelta = bmsData.cellVoltageMax - bmsData.cellVoltageMin;
        
        // Estimate pack voltage (assuming all cells are at average voltage)
        float avgCellVoltage = (bmsData.cellVoltageMax + bmsData.cellVoltageMin) / 2.0f;
        bmsData.packVoltage = (avgCellVoltage / 1000.0f) * cellCount;  // Convert mV to V and multiply by cell count
        
        // Update metadata
        bmsData.lastUpdateTime = millis();
        bmsData.sourceAddress = sourceAddress;
        bmsData.dataValid = true;
        
        xSemaphoreGive(dataMutex);
        
        Serial.printf("[VX1Manager] FEF3 decoded: Vmin=%.1fmV Vmax=%.1fmV Tmin=%d°C Tmax=%d°C Module=%d\n",
                      bmsData.cellVoltageMin, bmsData.cellVoltageMax, 
                      bmsData.tempMin, bmsData.tempMax, bmsData.moduleNumber);
        
        return true;
    } else {
        Serial.println("[VX1Manager] WARNING: Could not acquire mutex in decodeFEF3Message()");
        return false;
    }
}

/**
 * @brief Update charging parameters based on current BMS data
 */
void VectrixVX1Manager::updateChargingParameters()
{
    if (!dataMutex) return;
    
    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        calculateChargingParameters();
        xSemaphoreGive(dataMutex);
    }
}

/**
 * @brief Calculate charging parameters based on current BMS data
 */
void VectrixVX1Manager::calculateChargingParameters()
{
    // Check if data is valid and recent
    if (!bmsData.dataValid || !isDataValid()) {
        chargingParams.chargingAllowed = false;
        return;
    }
    
    // Temperature safety checks with anomaly detection
    bool tempOk = (bmsData.tempMax <= TEMP_MAX_CHARGING) && 
                  (bmsData.tempMin >= TEMP_MIN_CHARGING) &&
                  checkTemperatureAnomalies();
    
    // Voltage safety checks
    bool voltageOk = (bmsData.cellVoltageMax <= CELL_VOLTAGE_ABSOLUTE_MAX * 1000) &&  // Convert V to mV
                     (bmsData.cellVoltageMin >= CELL_VOLTAGE_MIN * 1000) &&
                     (bmsData.voltageDelta <= VOLTAGE_DELTA_MAX);
    
    // Overall charging permission
    chargingParams.chargingAllowed = tempOk && voltageOk;
    
    // Calculate target voltage (CV phase voltage)
    // Use user-configurable target voltage instead of fixed 4.2V
    chargingParams.targetVoltage = cellCount * targetCellVoltage;
    
    // Calculate maximum charging current based on conditions
    if (chargingParams.chargingAllowed) {
        // Base current limit (can be adjusted based on requirements)
        float baseCurrent = 10.0f;  // 10A base current for VX1
        
        // Progressive current derating based on temperature
        if (bmsData.tempMax > TEMP_NORMAL_MAX) {
            // Progressive derating from 35°C to 45°C
            float tempRatio = (float)(bmsData.tempMax - TEMP_NORMAL_MAX) / (TEMP_MAX_CHARGING - TEMP_NORMAL_MAX);
            baseCurrent *= (1.0f - tempRatio * 0.5f);  // Reduce up to 50% at max temp
        }
        
        // Reduce current if voltage delta is high (balancing needed)
        if (bmsData.voltageDelta > 100.0f) {  // > 100mV delta
            baseCurrent *= 0.5f;  // Reduce to 50% for balancing
            chargingParams.balancingNeeded = true;
        } else {
            chargingParams.balancingNeeded = false;
        }
        
        // Reduce current as we approach full charge
        float maxCellVoltage = bmsData.cellVoltageMax / 1000.0f;  // Convert to V
        if (maxCellVoltage > 4.0f) {
            float voltageRatio = (CELL_VOLTAGE_ABSOLUTE_MAX - maxCellVoltage) / (CELL_VOLTAGE_ABSOLUTE_MAX - 4.0f);
            baseCurrent *= voltageRatio;
        }
        
        chargingParams.maxCurrent = baseCurrent;
    } else {
        chargingParams.maxCurrent = 0.0f;
        chargingParams.balancingNeeded = false;
    }
    
    // Set safety limits
    chargingParams.maxCellVoltage = CELL_VOLTAGE_ABSOLUTE_MAX * 1000;  // Convert to mV
    chargingParams.minCellVoltage = CELL_VOLTAGE_MIN * 1000;
    chargingParams.maxTemperature = TEMP_MAX_CHARGING;
}

/**
 * @brief Check if CAN ID matches VX1 FEF3 message
 */
bool VectrixVX1Manager::isVX1FEF3Message(uint32_t canId, uint8_t& sourceAddress) const
{
    // Extract PGN from CAN ID (bits 8-25)
    uint32_t pgn = (canId >> 8) & 0x3FFFF;
    
    // Extract source address (bits 0-7)
    sourceAddress = canId & 0xFF;
    
    // Check if this is a FEF3 message from expected source address
    return (pgn == VX1_FEF3_PGN) && (sourceAddress == VX1_SOURCE_ADDRESS);
}

/**
 * @brief Get current BMS data
 */
VX1BmsData VectrixVX1Manager::getBmsData() const
{
    if (!dataMutex) return VX1BmsData();
    
    VX1BmsData data;
    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        data = bmsData;
        xSemaphoreGive(dataMutex);
    }
    return data;
}

/**
 * @brief Get current charging parameters
 */
VX1ChargingParams VectrixVX1Manager::getChargingParameters() const
{
    if (!dataMutex) return VX1ChargingParams();
    
    VX1ChargingParams params;
    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        params = chargingParams;
        xSemaphoreGive(dataMutex);
    }
    return params;
}

/**
 * @brief Get recommended charging voltage limit
 */
float VectrixVX1Manager::getChargingVoltageLimit() const
{
    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        float limit = chargingParams.targetVoltage;
        xSemaphoreGive(dataMutex);
        return limit;
    }
    return 0.0f;
}

/**
 * @brief Set target cell voltage
 */
void VectrixVX1Manager::setTargetCellVoltage(float targetVoltage)
{
    // Validate voltage range
    if (targetVoltage < CELL_VOLTAGE_MIN || targetVoltage > CELL_VOLTAGE_ABSOLUTE_MAX) {
        Serial.printf("[VX1Manager] WARNING: Target voltage %.2fV out of range (%.1f-%.1fV)\n", 
                      targetVoltage, CELL_VOLTAGE_MIN, CELL_VOLTAGE_ABSOLUTE_MAX);
        return;
    }
    
    // Warn about high voltage
    if (targetVoltage > 4.15f) {
        Serial.printf("[VX1Manager] WARNING: High target voltage %.2fV - use with caution!\n", targetVoltage);
        if (targetVoltage >= CELL_VOLTAGE_ABSOLUTE_MAX) {
            Serial.println("[VX1Manager] DANGER: 4.2V charging is risky - monitor closely!");
        }
    }
    
    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        this->targetCellVoltage = targetVoltage;
        // Recalculate charging parameters with new target
        calculateChargingParameters();
        xSemaphoreGive(dataMutex);
        
        Serial.printf("[VX1Manager] Target cell voltage set to %.2fV (pack: %.1fV)\n", 
                      targetVoltage, targetVoltage * cellCount);
    }
}

/**
 * @brief Get target cell voltage
 */
float VectrixVX1Manager::getTargetCellVoltage() const
{
    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        float target = targetCellVoltage;
        xSemaphoreGive(dataMutex);
        return target;
    }
    return CELL_VOLTAGE_DEFAULT_TARGET;
}

/**
 * @brief Check for temperature anomalies and deviations
 */
bool VectrixVX1Manager::checkTemperatureAnomalies()
{
    uint32_t currentTime = millis();
    int8_t tempDelta = bmsData.tempMax - bmsData.tempMin;
    
    // Check temperature spread between sensors
    if (tempDelta > TEMP_DELTA_CRITICAL) {
        Serial.printf("[VX1Manager] CRITICAL: Temperature delta %d°C exceeds limit (%d°C)\n", 
                      tempDelta, TEMP_DELTA_CRITICAL);
        Serial.println("[VX1Manager] Possible thermal runaway or sensor failure!");
        return false;
    }
    
    if (tempDelta > TEMP_DELTA_WARNING) {
        Serial.printf("[VX1Manager] WARNING: High temperature delta %d°C (limit: %d°C)\n", 
                      tempDelta, TEMP_DELTA_WARNING);
        Serial.println("[VX1Manager] Check for hot spots or cooling issues");
    }
    
    // Check absolute temperature warnings
    if (bmsData.tempMax >= TEMP_ABSOLUTE_WARNING) {
        Serial.printf("[VX1Manager] WARNING: High temperature %d°C approaching limit (%d°C)\n", 
                      bmsData.tempMax, TEMP_MAX_CHARGING);
    }
    
    // Check temperature rise rate (if we have previous data)
    if (lastTempUpdateTime > 0 && currentTime > lastTempUpdateTime) {
        uint32_t timeDelta = currentTime - lastTempUpdateTime;
        if (timeDelta >= 60000) {  // At least 1 minute between readings
            int8_t tempRise = bmsData.tempMax - lastTempMax;
            float riseRate = (float)tempRise / (timeDelta / 60000.0f);  // °C per minute
            
            if (riseRate > TEMP_RATE_WARNING) {
                Serial.printf("[VX1Manager] WARNING: Rapid temperature rise %.1f°C/min (limit: %d°C/min)\n", 
                              riseRate, TEMP_RATE_WARNING);
                Serial.println("[VX1Manager] Possible thermal issue - reduce charging current");
            }
        }
    }
    
    // Update temperature history
    lastTempMax = bmsData.tempMax;
    lastTempMin = bmsData.tempMin;
    lastTempUpdateTime = currentTime;
    
    // Return false only for critical conditions
    return (tempDelta <= TEMP_DELTA_CRITICAL);
}

/**
 * @brief Check if BMS data is valid and recent
 */
bool VectrixVX1Manager::isDataValid(uint32_t maxAgeMs) const
{
    if (!bmsData.dataValid) return false;
    
    uint32_t currentTime = millis();
    uint32_t age = currentTime - bmsData.lastUpdateTime;
    
    // Handle millis() rollover
    if (currentTime < bmsData.lastUpdateTime) {
        age = (UINT32_MAX - bmsData.lastUpdateTime) + currentTime + 1;
    }
    
    return age <= maxAgeMs;
}

/**
 * @brief Set callback for BMS data updates
 */
void VectrixVX1Manager::setDataCallback(VX1DataCallback callback)
{
    dataCallback = callback;
}

/**
 * @brief Set callback for charging parameter updates
 */
void VectrixVX1Manager::setChargingCallback(VX1ChargingCallback callback)
{
    chargingCallback = callback;
}

/**
 * @brief Print current status to serial console
 */
void VectrixVX1Manager::printStatus() const
{
    VX1BmsData data = getBmsData();
    VX1ChargingParams params = getChargingParameters();
    
    Serial.println("\n=== Vectrix VX1 BMS Status ===");
    
    if (!data.dataValid) {
        Serial.println("Status: NO DATA");
        return;
    }
    
    uint32_t age = millis() - data.lastUpdateTime;
    Serial.printf("Data Age: %lu ms\n", age);
    Serial.printf("Source: 0x%02X, Module: %d\n", data.sourceAddress, data.moduleNumber);
    
    Serial.printf("Cell Voltages: Min=%.1fmV (cell %d), Max=%.1fmV (cell %d)\n",
                  data.cellVoltageMin, data.cellIndexMin,
                  data.cellVoltageMax, data.cellIndexMax);
    Serial.printf("Voltage Delta: %.1fmV\n", data.voltageDelta);
    Serial.printf("Pack Voltage: %.1fV\n", data.packVoltage);
    
    Serial.printf("Temperatures: Min=%d°C, Max=%d°C, Ambient=%d°C\n",
                  data.tempMin, data.tempMax, data.tempAmbient);
    
    Serial.printf("Charging: %s", params.chargingAllowed ? "ALLOWED" : "BLOCKED");
    if (params.chargingAllowed) {
        Serial.printf(" (%.1fA max, %.1fV target)", params.maxCurrent, params.targetVoltage);
    }
    Serial.println();
    
    if (params.balancingNeeded) {
        Serial.println("WARNING: Cell balancing needed (high voltage delta)");
    }
    
    Serial.println();
}

/**
 * @brief Get pack voltage estimate
 */
float VectrixVX1Manager::getPackVoltage() const
{
    return getBmsData().packVoltage;
}

/**
 * @brief Get cell voltage delta
 */
float VectrixVX1Manager::getVoltageDelta() const
{
    return getBmsData().voltageDelta;
}

/**
 * @brief Get temperature range
 */
void VectrixVX1Manager::getTemperatureRange(int8_t& minTemp, int8_t& maxTemp) const
{
    VX1BmsData data = getBmsData();
    minTemp = data.tempMin;
    maxTemp = data.tempMax;
}

/**
 * @brief Check if charging should be allowed based on current conditions
 */
bool VectrixVX1Manager::isChargingAllowed() const
{
    return getChargingParameters().chargingAllowed;
}

/**
 * @brief Get recommended charging current limit
 */
float VectrixVX1Manager::getChargingCurrentLimit() const
{
    return getChargingParameters().maxCurrent;
}

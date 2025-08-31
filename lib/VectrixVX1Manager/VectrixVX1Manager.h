/*
 * VectrixVX1Manager.h - Vectrix VX1 BMS CAN Message Handler
 * 
 * This library handles CAN messages from Vectrix VX1 BMS systems,
 * specifically decoding FEF3 messages for battery monitoring and charging control.
 * 
 * Copyright (C) 2025 ESP Flatcharge Project
 */

#ifndef VECTRIX_VX1_MANAGER_H
#define VECTRIX_VX1_MANAGER_H

#include <Arduino.h>
#include <vector>
#include <functional>

// FreeRTOS includes for thread safety
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

/**
 * @brief Vectrix VX1 BMS data structure
 */
struct VX1BmsData {
    // Temperature data (°C)
    int8_t tempMin;
    int8_t tempMax;
    int8_t tempAmbient;
    
    // Cell voltage data
    float cellVoltageMin;      // Minimum cell voltage in mV
    float cellVoltageMax;      // Maximum cell voltage in mV
    uint8_t cellIndexMin;      // Index of minimum voltage cell (0-35 for 36S)
    uint8_t cellIndexMax;      // Index of maximum voltage cell (0-35 for 36S)
    
    // Module and status data
    uint8_t moduleNumber;      // Battery module number (0-3)
    uint8_t thermalSwitch;     // Thermal switch status
    
    // Calculated values
    float packVoltage;         // Estimated pack voltage (36 * average cell voltage)
    float voltageDelta;        // Voltage difference between max and min cells
    
    // Message metadata
    uint32_t lastUpdateTime;   // Last time this data was updated (millis())
    uint8_t sourceAddress;     // CAN source address (typically 0x40)
    bool dataValid;            // True if data is valid and recent
    
    // Constructor
    VX1BmsData() : 
        tempMin(0), tempMax(0), tempAmbient(0),
        cellVoltageMin(0), cellVoltageMax(0),
        cellIndexMin(0), cellIndexMax(0),
        moduleNumber(0), thermalSwitch(0),
        packVoltage(0), voltageDelta(0),
        lastUpdateTime(0), sourceAddress(0x40),
        dataValid(false) {}
};

/**
 * @brief Charging parameters derived from VX1 BMS data
 */
struct VX1ChargingParams {
    float targetVoltage;       // Target charging voltage in V
    float maxCurrent;          // Maximum charging current in A
    bool chargingAllowed;      // True if charging is safe
    bool balancingNeeded;      // True if cell balancing is needed
    
    // Safety limits
    float maxCellVoltage;      // Maximum safe cell voltage in mV
    float minCellVoltage;      // Minimum safe cell voltage in mV
    int8_t maxTemperature;     // Maximum safe temperature in °C
    
    VX1ChargingParams() :
        targetVoltage(0), maxCurrent(0),
        chargingAllowed(false), balancingNeeded(false),
        maxCellVoltage(4200), minCellVoltage(3000),
        maxTemperature(45) {}
};

/**
 * @brief Callback function types for VX1 events
 */
typedef std::function<void(const VX1BmsData&)> VX1DataCallback;
typedef std::function<void(const VX1ChargingParams&)> VX1ChargingCallback;

/**
 * @brief Vectrix VX1 BMS Manager Class
 */
class VectrixVX1Manager {
public:
    /**
     * @brief Constructor
     */
    VectrixVX1Manager();
    
    /**
     * @brief Destructor
     */
    ~VectrixVX1Manager();
    
    /**
     * @brief Initialize the VX1 manager
     * @param cellCount Number of cells in series (user-configurable)
     * @return true if initialization successful
     */
    bool initialize(uint8_t cellCount);
    
    /**
     * @brief Process incoming CAN message
     * @param canId CAN message ID
     * @param data Message data (8 bytes)
     * @param dlc Data length code
     * @return true if message was processed
     */
    bool processCanMessage(uint32_t canId, const uint8_t* data, uint8_t dlc);
    
    /**
     * @brief Update charging parameters based on current BMS data
     * Called periodically to recalculate charging parameters
     */
    void updateChargingParameters();
    
    /**
     * @brief Get current BMS data
     * @return Current VX1 BMS data structure
     */
    VX1BmsData getBmsData() const;
    
    /**
     * @brief Get current charging parameters
     * @return Current charging parameters
     */
    VX1ChargingParams getChargingParameters() const;
    
    /**
     * @brief Check if BMS data is valid and recent
     * @param maxAgeMs Maximum age in milliseconds (default 5000ms)
     * @return true if data is valid and within age limit
     */
    bool isDataValid(uint32_t maxAgeMs = 5000) const;
    
    /**
     * @brief Set callback for BMS data updates
     * @param callback Function to call when new BMS data is received
     */
    void setDataCallback(VX1DataCallback callback);
    
    /**
     * @brief Set callback for charging parameter updates
     * @param callback Function to call when charging parameters change
     */
    void setChargingCallback(VX1ChargingCallback callback);
    
    /**
     * @brief Print current status to serial console
     */
    void printStatus() const;
    
    /**
     * @brief Get pack voltage estimate
     * @return Estimated pack voltage in V
     */
    float getPackVoltage() const;
    
    /**
     * @brief Get cell voltage delta
     * @return Voltage difference between highest and lowest cells in mV
     */
    float getVoltageDelta() const;
    
    /**
     * @brief Get temperature range
     * @param minTemp Reference to store minimum temperature
     * @param maxTemp Reference to store maximum temperature
     */
    void getTemperatureRange(int8_t& minTemp, int8_t& maxTemp) const;
    
    /**
     * @brief Check if charging should be allowed based on current conditions
     * @return true if charging is safe
     */
    bool isChargingAllowed() const;
    
    /**
     * @brief Get recommended charging current limit
     * @return Maximum safe charging current in A
     */
    float getChargingCurrentLimit() const;
    
    /**
     * @brief Get recommended charging voltage limit
     * @return Maximum safe charging voltage in V
     */
    float getChargingVoltageLimit() const;
    
    /**
     * @brief Set target cell voltage for charging
     * @param targetVoltage Target voltage per cell in V (3.0V to 4.2V)
     */
    void setTargetCellVoltage(float targetVoltage);
    
    /**
     * @brief Get current target cell voltage
     * @return Target voltage per cell in V
     */
    float getTargetCellVoltage() const;

private:
    // Configuration
    uint8_t cellCount;
    float targetCellVoltage;      // User-configurable target cell voltage
    
    // BMS data storage
    VX1BmsData bmsData;
    VX1ChargingParams chargingParams;
    
    // Temperature monitoring history
    int8_t lastTempMax;
    int8_t lastTempMin;
    uint32_t lastTempUpdateTime;
    
    // Thread safety
    SemaphoreHandle_t dataMutex;
    
    // Callbacks
    VX1DataCallback dataCallback;
    VX1ChargingCallback chargingCallback;
    
    // Constants for VX1 BMS
    static constexpr uint32_t VX1_FEF3_PGN = 0x00FEF3;
    static constexpr float VOLTAGE_SCALE = 1.5f;  // mV per bit
    static constexpr uint8_t VX1_SOURCE_ADDRESS = 0x40;
    static constexpr uint32_t DATA_TIMEOUT_MS = 5000;
    
    // Li-ion charging parameters (user-configurable)
    static constexpr float CELL_VOLTAGE_ABSOLUTE_MAX = 4.2f;  // Absolute maximum (use with caution)
    static constexpr float CELL_VOLTAGE_MIN = 3.0f;          // Minimum cell voltage (V)
    static constexpr float CELL_VOLTAGE_NOMINAL = 3.7f;      // Nominal cell voltage (V)
    static constexpr float CELL_VOLTAGE_DEFAULT_TARGET = 4.0f; // Safe default target
    static constexpr int8_t TEMP_MAX_CHARGING = 45;      // Maximum charging temperature (°C)
    static constexpr int8_t TEMP_MIN_CHARGING = 0;       // Minimum charging temperature (°C)
    static constexpr float VOLTAGE_DELTA_MAX = 200.0f;   // Maximum allowed cell voltage delta (mV)
    
    // Temperature deviation monitoring
    static constexpr int8_t TEMP_DELTA_WARNING = 10;     // Warning threshold for temp difference (°C)
    static constexpr int8_t TEMP_DELTA_CRITICAL = 15;    // Critical threshold for temp difference (°C)
    static constexpr int8_t TEMP_RATE_WARNING = 5;       // Warning for temp rise rate (°C/min)
    static constexpr int8_t TEMP_ABSOLUTE_WARNING = 40;  // Warning threshold (°C)
    static constexpr int8_t TEMP_NORMAL_MAX = 35;        // Normal operating maximum (°C)
    
    /**
     * @brief Decode FEF3 message data
     * @param data Message data (8 bytes)
     * @param sourceAddress CAN source address
     * @return true if decoding successful
     */
    bool decodeFEF3Message(const uint8_t* data, uint8_t sourceAddress);
    
    /**
     * @brief Calculate charging parameters based on current BMS data
     */
    void calculateChargingParameters();
    
    /**
     * @brief Check for temperature anomalies and deviations
     * @return true if temperatures are within normal range
     */
    bool checkTemperatureAnomalies();
    
    /**
     * @brief Check if CAN ID matches VX1 FEF3 message
     * @param canId CAN message ID
     * @param sourceAddress Reference to store extracted source address
     * @return true if this is a VX1 FEF3 message
     */
    bool isVX1FEF3Message(uint32_t canId, uint8_t& sourceAddress) const;
};

#endif // VECTRIX_VX1_MANAGER_H

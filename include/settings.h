#pragma once

#include <Arduino.h>
#include <ArduinoJson.h>
#include <SPIFFS.h>
#include "config.h"

// Enum for the BMS mode
enum class BMSMode {
    OPENINVERTER,
    VX1,
    TC_CHARGER,
    MANUAL
};

// Enum for the Flatpack configuration
enum class FlatpackConfig {
    SERIES,
    PARALLEL
};

// Structure to hold all settings
struct ChargingSettings {
    // General settings
    bool chargingEnabled = false;
    float maxCurrentLimit = 16.0;  // In Amperes
    
    // Battery settings
    BMSMode bmsMode = BMSMode::MANUAL;
    float targetVoltage = 57.0;    // In Volts
    float targetCurrent = 16.0;    // In Amperes
    uint8_t cellCount = 14;        // Number of cells
    float cellTargetVoltage = 4.15; // Target voltage per cell
    
    // Flatpack configuration
    FlatpackConfig flatpackConfig = FlatpackConfig::SERIES;
    bool useThreePhase = true;     // Three phase or single phase
};

class Settings {
public:
    Settings();
    
    /**
     * Initialize the settings
     * Loads settings from SPIFFS if available
     */
    bool begin();
    
    /**
     * Save current settings to SPIFFS
     */
    bool save();
    
    /**
     * Load settings from SPIFFS
     */
    bool load();
    
    /**
     * Reset settings to defaults
     */
    void resetToDefaults();
    
    /**
     * Get the current settings
     */
    ChargingSettings& getSettings();
    
    /**
     * Calculate the appropriate target voltage
     * based on cell count and target cell voltage
     */
    float calculateTargetVoltage();
    
    /**
     * Calculate the maximum allowable current
     * based on EVSE limits and flatpack configuration
     */
    float calculateMaxCurrent(float evseCurrentLimit);
    
private:
    ChargingSettings _settings;
    bool _initialized = false;
    
    /**
     * Create the config.json file with default settings if it doesn't exist
     */
    bool _createDefaultConfig();
};

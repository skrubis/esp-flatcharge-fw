#include "settings.h"

Settings::Settings() {}

bool Settings::begin() {
    // Check if SPIFFS is mounted
    if (!SPIFFS.begin(true)) {
        Serial.println("Failed to mount SPIFFS");
        return false;
    }
    
    // Try to load settings
    if (!load()) {
        // If loading fails, create default settings
        Serial.println("Creating default settings");
        if (!_createDefaultConfig()) {
            Serial.println("Failed to create default settings");
            return false;
        }
    }
    
    _initialized = true;
    return true;
}

bool Settings::save() {
    if (!_initialized) {
        return false;
    }
    
    // Create JSON document
    DynamicJsonDocument doc(1024);
    
    // Add general settings
    doc["chargingEnabled"] = _settings.chargingEnabled;
    doc["maxCurrentLimit"] = _settings.maxCurrentLimit;
    
    // Add BMS settings
    doc["bmsMode"] = static_cast<int>(_settings.bmsMode);
    doc["targetVoltage"] = _settings.targetVoltage;
    doc["targetCurrent"] = _settings.targetCurrent;
    doc["cellCount"] = _settings.cellCount;
    doc["cellTargetVoltage"] = _settings.cellTargetVoltage;
    
    // Add Flatpack configuration
    doc["flatpackConfig"] = static_cast<int>(_settings.flatpackConfig);
    doc["useThreePhase"] = _settings.useThreePhase;
    
    // Open file for writing
    File file = SPIFFS.open(CONFIG_FILE, "w");
    if (!file) {
        Serial.println("Failed to open config file for writing");
        return false;
    }
    
    // Serialize JSON to file
    if (serializeJson(doc, file) == 0) {
        Serial.println("Failed to write config file");
        file.close();
        return false;
    }
    
    file.close();
    Serial.println("Config saved successfully");
    return true;
}

bool Settings::load() {
    // Check if configuration file exists
    if (!SPIFFS.exists(CONFIG_FILE)) {
        Serial.println("Config file does not exist");
        return false;
    }
    
    // Open file for reading
    File file = SPIFFS.open(CONFIG_FILE, "r");
    if (!file) {
        Serial.println("Failed to open config file for reading");
        return false;
    }
    
    // Parse JSON
    DynamicJsonDocument doc(1024);
    DeserializationError error = deserializeJson(doc, file);
    file.close();
    
    if (error) {
        Serial.print("Failed to parse config file: ");
        Serial.println(error.c_str());
        return false;
    }
    
    // Read general settings
    _settings.chargingEnabled = doc["chargingEnabled"] | false;
    _settings.maxCurrentLimit = doc["maxCurrentLimit"] | 16.0;
    
    // Read BMS settings
    _settings.bmsMode = static_cast<BMSMode>(doc["bmsMode"] | static_cast<int>(BMSMode::MANUAL));
    _settings.targetVoltage = doc["targetVoltage"] | 57.0;
    _settings.targetCurrent = doc["targetCurrent"] | 16.0;
    _settings.cellCount = doc["cellCount"] | 14;
    _settings.cellTargetVoltage = doc["cellTargetVoltage"] | 4.15;
    
    // Read Flatpack configuration
    _settings.flatpackConfig = static_cast<FlatpackConfig>(doc["flatpackConfig"] | static_cast<int>(FlatpackConfig::SERIES));
    _settings.useThreePhase = doc["useThreePhase"] | true;
    
    Serial.println("Config loaded successfully");
    return true;
}

void Settings::resetToDefaults() {
    _settings.chargingEnabled = false;
    _settings.maxCurrentLimit = 16.0;
    
    _settings.bmsMode = BMSMode::MANUAL;
    _settings.targetVoltage = 57.0;
    _settings.targetCurrent = 16.0;
    _settings.cellCount = 14;
    _settings.cellTargetVoltage = 4.15;
    
    _settings.flatpackConfig = FlatpackConfig::SERIES;
    _settings.useThreePhase = true;
    
    // Save the default settings
    save();
}

ChargingSettings& Settings::getSettings() {
    return _settings;
}

float Settings::calculateTargetVoltage() {
    if (_settings.cellCount > 0) {
        float voltage = _settings.cellCount * _settings.cellTargetVoltage;
        
        // Clamp to valid range
        if (voltage < FP_MIN_VOLTAGE) voltage = FP_MIN_VOLTAGE;
        if (voltage > FP_MAX_VOLTAGE) voltage = FP_MAX_VOLTAGE;
        
        return voltage;
    }
    
    return _settings.targetVoltage;
}

float Settings::calculateMaxCurrent(float evseCurrentLimit) {
    // Start with the user-defined limit
    float maxCurrent = _settings.targetCurrent;
    
    // Limit to hardware maximum
    if (maxCurrent > _settings.maxCurrentLimit) {
        maxCurrent = _settings.maxCurrentLimit;
    }
    
    // Limit based on EVSE
    if (maxCurrent > evseCurrentLimit) {
        maxCurrent = evseCurrentLimit;
    }
    
    // For Three-phase mode, divide current by 3 for each FP
    if (_settings.useThreePhase) {
        // In three phase mode, each FP can draw up to MAX_CURRENT_THREE_PHASE
        if (maxCurrent > MAX_CURRENT_THREE_PHASE) {
            maxCurrent = MAX_CURRENT_THREE_PHASE;
        }
    } else {
        // In single phase mode, total draw is limited to MAX_CURRENT_SINGLE_PHASE
        if (maxCurrent > MAX_CURRENT_SINGLE_PHASE) {
            maxCurrent = MAX_CURRENT_SINGLE_PHASE;
        }
    }
    
    return maxCurrent;
}

bool Settings::_createDefaultConfig() {
    // Reset settings to defaults
    resetToDefaults();
    
    // Save to file
    return save();
}

#include "BatteryManager.h"

// Default parameters for LFP (Lithium Iron Phosphate) batteries
const BatteryParameters BatteryManager::defaultLFP = {
    .chemistry = BatteryChemistry::LFP,
    .cellCount = 16,  // Default for 48V nominal (16 * 3.2V = 51.2V)
    .cellVoltageMin = 2500,     // 2.5V
    .cellVoltageNominal = 3200, // 3.2V
    .cellVoltageMax = 3650,     // 3.65V
    .cellVoltageFloat = 3400,   // 3.4V
    .currentMax = 50,           // 0.5C by default (50% of capacity)
    .currentTaper = 10,         // 0.1C (10% of capacity)
    .currentFloat = 2,          // 0.02C (2% of capacity)
    .tempMin = 0,               // 0°C
    .tempMax = 45,              // 45°C
    .maxChargingTime = 720,     // 12 hours
    .maxCCTime = 480,           // 8 hours
    .maxCVTime = 240,           // 4 hours
    .maxFloatTime = 240         // 4 hours
};

// Default parameters for LTO (Lithium Titanate Oxide) batteries
const BatteryParameters BatteryManager::defaultLTO = {
    .chemistry = BatteryChemistry::LTO,
    .cellCount = 24,  // Default for 48V nominal (24 * 2.3V = 55.2V)
    .cellVoltageMin = 1800,     // 1.8V
    .cellVoltageNominal = 2300, // 2.3V
    .cellVoltageMax = 2800,     // 2.8V
    .cellVoltageFloat = 2500,   // 2.5V
    .currentMax = 100,          // 1.0C by default (100% of capacity)
    .currentTaper = 15,         // 0.15C (15% of capacity)
    .currentFloat = 3,          // 0.03C (3% of capacity)
    .tempMin = -10,             // -10°C
    .tempMax = 55,              // 55°C
    .maxChargingTime = 720,     // 12 hours
    .maxCCTime = 360,           // 6 hours
    .maxCVTime = 240,           // 4 hours
    .maxFloatTime = 240         // 4 hours
};

// Default parameters for NMC (Lithium Nickel Manganese Cobalt) batteries
const BatteryParameters BatteryManager::defaultNMC = {
    .chemistry = BatteryChemistry::NMC,
    .cellCount = 13,  // Default for 48V nominal (13 * 3.7V = 48.1V)
    .cellVoltageMin = 3000,     // 3.0V
    .cellVoltageNominal = 3700, // 3.7V
    .cellVoltageMax = 4200,     // 4.2V
    .cellVoltageFloat = 4000,   // 4.0V
    .currentMax = 70,           // 0.7C by default (70% of capacity)
    .currentTaper = 15,         // 0.15C (15% of capacity)
    .currentFloat = 2,          // 0.02C (2% of capacity)
    .tempMin = 5,               // 5°C
    .tempMax = 40,              // 40°C
    .maxChargingTime = 720,     // 12 hours
    .maxCCTime = 480,           // 8 hours
    .maxCVTime = 240,           // 4 hours
    .maxFloatTime = 240         // 4 hours
};

BatteryManager::BatteryManager() : 
    capacityAh(0.0f), 
    lastUpdateTime(0), 
    initialized(false) {
    
    // Initialize status
    status.packVoltage = 0.0f;
    status.packCurrent = 0.0f;
    status.cellVoltageAvg = 0.0f;
    status.temperature = 25; // Default room temperature
    status.mode = ChargingMode::OFF;
    status.stateOfCharge = 0;
    status.chargingTimeMin = 0;
    status.isCharging = false;
    status.isError = false;
    status.errorFlags = 0;
}

BatteryManager::~BatteryManager() {
}

bool BatteryManager::initialize(BatteryChemistry chemistry, uint16_t cellCount, float capacityAh) {
    this->capacityAh = capacityAh;
    
    // Select default parameters based on chemistry
    switch (chemistry) {
        case BatteryChemistry::LFP:
            params = defaultLFP;
            break;
        case BatteryChemistry::LTO:
            params = defaultLTO;
            break;
        case BatteryChemistry::NMC:
            params = defaultNMC;
            break;
        default:
            return false;
    }
    
    // Override cell count if specified
    if (cellCount > 0) {
        params.cellCount = cellCount;
    }
    
    // Reset charging stats
    resetChargingStats();
    
    Serial.printf("[BatteryManager] Initialized with %s chemistry, %d cells, %.1f Ah capacity\n",
                 chemistry == BatteryChemistry::LFP ? "LFP" : 
                 chemistry == BatteryChemistry::LTO ? "LTO" : "NMC",
                 params.cellCount, capacityAh);
    
    initialized = true;
    return true;
}

void BatteryManager::setParameters(const BatteryParameters& params) {
    this->params = params;
    
    Serial.println("[BatteryManager] Battery parameters updated");
}

const BatteryParameters& BatteryManager::getParameters() const {
    return params;
}

void BatteryManager::updateStatus(float voltage, float current, int8_t temperature) {
    unsigned long currentTime = millis();
    
    // Calculate time elapsed since last update
    if (lastUpdateTime > 0 && status.isCharging) {
        unsigned long elapsedMs = currentTime - lastUpdateTime;
        status.chargingTimeMin += (elapsedMs / 60000); // Convert to minutes
    }
    
    lastUpdateTime = currentTime;
    
    // Update battery status
    status.packVoltage = voltage;
    status.packCurrent = current;
    status.temperature = temperature;
    
    if (params.cellCount > 0) {
        status.cellVoltageAvg = voltage / params.cellCount;
    } else {
        status.cellVoltageAvg = 0.0f;
    }
    
    // Simple SoC estimation based on voltage
    // This is a very rough estimate and should be improved with proper algorithms
    float cellVoltage = status.cellVoltageAvg;
    float vMin = params.cellVoltageMin / 1000.0f;
    float vMax = params.cellVoltageMax / 1000.0f;
    
    if (cellVoltage <= vMin) {
        status.stateOfCharge = 0;
    } else if (cellVoltage >= vMax) {
        status.stateOfCharge = 100;
    } else {
        status.stateOfCharge = (uint8_t)((cellVoltage - vMin) / (vMax - vMin) * 100.0f);
    }
    
    // Check for error conditions
    updateChargingMode();
    
    // Check for temperature limits
    if (status.temperature < params.tempMin || status.temperature > params.tempMax) {
        status.isError = true;
        status.errorFlags |= 0x01; // Temperature error flag
    }
    
    // Check for charging time limits
    if (status.chargingTimeMin > params.maxChargingTime) {
        status.isError = true;
        status.errorFlags |= 0x02; // Time limit error flag
    }
}

const BatteryStatus& BatteryManager::getStatus() const {
    return status;
}

bool BatteryManager::startCharging() {
    if (!initialized) {
        Serial.println("[BatteryManager] ERROR: Battery not initialized");
        return false;
    }
    
    if (status.isError) {
        Serial.println("[BatteryManager] ERROR: Cannot start charging due to error condition");
        return false;
    }
    
    status.isCharging = true;
    updateChargingMode();
    
    Serial.println("[BatteryManager] Charging started");
    return true;
}

void BatteryManager::stopCharging() {
    status.isCharging = false;
    status.mode = ChargingMode::OFF;
    
    Serial.println("[BatteryManager] Charging stopped");
}

void BatteryManager::update() {
    if (!initialized || !status.isCharging) {
        return;
    }
    
    // Update charging mode based on current battery state
    updateChargingMode();
    
    // Check for error conditions
    if (status.isError) {
        stopCharging();
        Serial.println("[BatteryManager] Charging stopped due to error");
    }
}

void BatteryManager::getChargingSetpoints(uint16_t& voltage, uint16_t& current, uint16_t& ovp) {
    if (!initialized || !status.isCharging) {
        voltage = 0;
        current = 0;
        ovp = 0;
        return;
    }
    
    // Calculate charging voltage based on mode and parameters
    float targetVoltage = calculateChargingVoltage();
    
    // Calculate charging current based on mode and parameters
    float targetCurrent = calculateChargingCurrent();
    
    // Convert to PSU units (centivolts and deciamps)
    voltage = static_cast<uint16_t>(targetVoltage * 100.0f);
    current = static_cast<uint16_t>(targetCurrent * 10.0f);
    
    // Set OVP to 5% above target voltage
    ovp = static_cast<uint16_t>(targetVoltage * 105.0f / 100.0f);
    
    // Sanity check - ensure OVP is at least 1V above target
    if (ovp - voltage < 100) {
        ovp = voltage + 100;
    }
}

void BatteryManager::resetChargingStats() {
    status.chargingTimeMin = 0;
    status.isCharging = false;
    status.isError = false;
    status.errorFlags = 0;
    status.mode = ChargingMode::OFF;
    lastUpdateTime = 0;
}

void BatteryManager::printStatus() const {
    Serial.println("=== Battery Status ===");
    Serial.printf("Chemistry: %s, Cell Count: %d\n",
                 params.chemistry == BatteryChemistry::LFP ? "LFP" : 
                 params.chemistry == BatteryChemistry::LTO ? "LTO" : "NMC",
                 params.cellCount);
    Serial.printf("Voltage: %.2fV (%.3fV/cell), Current: %.2fA, Temp: %dC\n",
                 status.packVoltage, status.cellVoltageAvg, status.packCurrent, status.temperature);
    Serial.printf("Charging Mode: %s, SoC: %d%%\n",
                 status.mode == ChargingMode::OFF ? "OFF" : 
                 status.mode == ChargingMode::CONSTANT_CURRENT ? "CC" : 
                 status.mode == ChargingMode::CONSTANT_VOLTAGE ? "CV" : 
                 status.mode == ChargingMode::FLOAT ? "Float" : "Error",
                 status.stateOfCharge);
    Serial.printf("Charging Time: %d minutes\n", status.chargingTimeMin);
    
    if (status.isError) {
        Serial.print("ERROR FLAGS: ");
        if (status.errorFlags & 0x01) Serial.print("TEMPERATURE ");
        if (status.errorFlags & 0x02) Serial.print("TIME_LIMIT ");
        if (status.errorFlags & 0x04) Serial.print("VOLTAGE_HIGH ");
        if (status.errorFlags & 0x08) Serial.print("VOLTAGE_LOW ");
        Serial.println();
    }
}

void BatteryManager::updateChargingMode() {
    if (!status.isCharging) {
        status.mode = ChargingMode::OFF;
        return;
    }
    
    // Get cell voltage in volts
    float cellV = status.cellVoltageAvg;
    float cellVMax = params.cellVoltageMax / 1000.0f;
    float cellVFloat = params.cellVoltageFloat / 1000.0f;
    
    // Get charging current as C-rate (proportion of capacity)
    float cRate = 0.0f;
    if (capacityAh > 0) {
        cRate = (status.packCurrent / capacityAh) * 100.0f; // in percent of capacity
    }
    
    ChargingMode previousMode = status.mode;
    
    // Determine charging mode based on voltage and current
    if (cellV < cellVMax && (status.mode != ChargingMode::CONSTANT_VOLTAGE)) {
        // Below max voltage - use CC mode
        status.mode = ChargingMode::CONSTANT_CURRENT;
    } 
    else if (cellV >= cellVMax) {
        // At or above max voltage - use CV mode
        status.mode = ChargingMode::CONSTANT_VOLTAGE;
        
        // If current has tapered down sufficiently, switch to float
        if (cRate <= params.currentFloat) {
            status.mode = ChargingMode::FLOAT;
        }
    }
    
    // Log mode transitions
    if (previousMode != status.mode) {
        Serial.printf("[BatteryManager] Charging mode changed: %s -> %s\n",
                     previousMode == ChargingMode::OFF ? "OFF" : 
                     previousMode == ChargingMode::CONSTANT_CURRENT ? "CC" : 
                     previousMode == ChargingMode::CONSTANT_VOLTAGE ? "CV" : 
                     previousMode == ChargingMode::FLOAT ? "Float" : "Error",
                     status.mode == ChargingMode::OFF ? "OFF" : 
                     status.mode == ChargingMode::CONSTANT_CURRENT ? "CC" : 
                     status.mode == ChargingMode::CONSTANT_VOLTAGE ? "CV" : 
                     status.mode == ChargingMode::FLOAT ? "Float" : "Error");
    }
}

float BatteryManager::calculateChargingCurrent() const {
    if (!status.isCharging) return 0.0f;
    
    float maxCurrent = (capacityAh * params.currentMax) / 100.0f; // Convert from percentage to A
    
    switch (status.mode) {
        case ChargingMode::CONSTANT_CURRENT:
            return maxCurrent;
            
        case ChargingMode::CONSTANT_VOLTAGE:
            // In CV mode, current is determined by the charger itself based on the voltage setpoint
            return maxCurrent; // Return max current to allow charger to regulate
            
        case ChargingMode::FLOAT:
            return (capacityAh * params.currentFloat) / 100.0f;
            
        default:
            return 0.0f;
    }
}

float BatteryManager::calculateChargingVoltage() const {
    if (!status.isCharging) return 0.0f;
    
    float voltage = 0.0f;
    
    switch (status.mode) {
        case ChargingMode::CONSTANT_CURRENT:
        case ChargingMode::CONSTANT_VOLTAGE:
            // Use maximum cell voltage for both CC and CV modes
            voltage = (params.cellVoltageMax / 1000.0f) * params.cellCount;
            break;
            
        case ChargingMode::FLOAT:
            // Use float voltage
            voltage = (params.cellVoltageFloat / 1000.0f) * params.cellCount;
            break;
            
        default:
            voltage = 0.0f;
            break;
    }
    
    return voltage;
}

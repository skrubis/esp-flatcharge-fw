#include "BatteryManager.h"
#include "VectrixVX1Manager.h"
#ifdef __has_include
#  if __has_include("BuildConfig.h")
#    include "BuildConfig.h"
#  endif
#endif
#ifndef FLATPACK_VOLT_MIN
#define FLATPACK_VOLT_MIN 43.5f
#endif
#ifndef FLATPACK_VOLT_MAX
#define FLATPACK_VOLT_MAX 57.5f
#endif

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
    .cellVoltageMax = 4160,     // 4.16V (safety cap for NMC)
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

// Default parameters for Li-ion (Generic Li-ion for VX1) batteries - SAFE NMC VOLTAGES
const BatteryParameters BatteryManager::defaultLIION = {
    .chemistry = BatteryChemistry::LIION,
    .cellCount = 36,  // VX1 uses 36S Li-ion pack
    .cellVoltageMin = 3000,     // 3.0V
    .cellVoltageNominal = 3700, // 3.7V
    .cellVoltageMax = 4050,     // 4.05V - SAFE DEFAULT for failsafe (was 4.16V)
    .cellVoltageFloat = 4000,   // 4.0V
    .currentMax = 100,          // 1.0C - 40A per PSU for 157Ah (was 15%)
    .currentTaper = 25,         // 0.25C (25% of capacity)
    .currentFloat = 5,          // 0.05C (5% of capacity)
    .tempMin = 0,               // 0°C
    .tempMax = 45,              // 45°C
    .maxChargingTime = 720,     // 12 hours
    .maxCCTime = 480,           // 8 hours
    .maxCVTime = 240,           // 4 hours
    .maxFloatTime = 240         // 4 hours
};

BatteryManager::BatteryManager() : 
    capacityAh(0.0f), 
    lastUpdateTime(0), 
    initialized(false),
    currentSource(BatterySource::MANUAL) {
    
    // Initialize status
    status.packVoltage = 0.0f;
    status.packCurrent = 0.0f;
    status.cellVoltageAvg = 0.0f;
    status.cellVoltageMin = 0.0f;
    status.cellVoltageMax = 0.0f;
    status.voltageDelta = 0.0f;
    status.temperature = 25; // Default room temperature
    status.temperatureMin = 25;
    status.temperatureMax = 25;
    status.mode = ChargingMode::OFF;
    status.stateOfCharge = 0;
    status.chargingTimeMin = 0;
    status.isCharging = false;
    status.isError = false;
    status.errorFlags = 0;
    status.dataSource = BatterySource::MANUAL;
    status.bmsDataValid = false;
    status.operatingMode = OperatingMode::MANUAL_CONTROLLED; // Default to manual for VX1
    status.manualCurrentLimit = 40.0f; // Default 40A per PSU for 157Ah modules
    status.voltageDropCompensation = 0.0f; // No compensation by default
    status.disableCurrentLimit = false; // By default, current limiting enabled
    status.defaultPerPsuVoltage = 43.7f; // Safe default fallback per PSU (V)
    status.acPresetId = static_cast<uint8_t>(acPreset);

    // Initialize current ramp
    rampCurrentA = 0.0f;
    rampTargetA = 0.0f;
    rampLastUpdate = 0;

    // Calibration
    voltageCalibrationOffsetV = 0.0f;
    // Initialize last commanded per-PSU voltage tracker
    lastPerPsuVoltageCmdV = 0.0f;
}

bool BatteryManager::setMaxCellVoltage(float cellV) {
    // Clamp to a safe range (3.80V .. 4.16V)
    if (cellV < 3.80f) cellV = 3.80f;
    if (cellV > 4.16f) cellV = 4.16f;
    uint16_t mv = static_cast<uint16_t>(lroundf(cellV * 1000.0f));
    params.cellVoltageMax = mv;
    // Ensure float voltage does not exceed max (keep at least 50mV below if higher)
    if (params.cellVoltageFloat >= params.cellVoltageMax) {
        params.cellVoltageFloat = (params.cellVoltageMax > 50) ? (params.cellVoltageMax - 50) : params.cellVoltageMax;
    }
    Serial.printf("[BatteryManager] Max cell voltage set to: %.3fV (float=%u mV)\n", cellV, params.cellVoltageFloat);
    return true;
}

BatteryManager::~BatteryManager() {
}

bool BatteryManager::initialize(BatteryChemistry chemistry, uint16_t cellCount, float capacityAh, BatterySource source) {
    this->capacityAh = capacityAh;
    this->currentSource = source;
    
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
        case BatteryChemistry::LIION:
            params = defaultLIION;
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
    
    const char* chemName = "UNKNOWN";
    switch (chemistry) {
        case BatteryChemistry::LFP: chemName = "LFP"; break;
        case BatteryChemistry::LTO: chemName = "LTO"; break;
        case BatteryChemistry::NMC: chemName = "NMC"; break;
        case BatteryChemistry::LIION: chemName = "Li-ion"; break;
    }
    
    const char* sourceName = "UNKNOWN";
    switch (source) {
        case BatterySource::MANUAL: sourceName = "Manual"; break;
        case BatterySource::CREE_LTO: sourceName = "Cree LTO"; break;
        case BatterySource::VECTRIX_VX1: sourceName = "Vectrix VX1"; break;
    }
    
    Serial.printf("[BatteryManager] Initialized with %s chemistry, %d cells, %.1f Ah capacity, %s source\n",
                 chemName, params.cellCount, capacityAh, sourceName);
    
    status.dataSource = source;
    
    initialized = true;
    return true;
}

bool BatteryManager::setDisableCurrentLimit(bool disabled) {
    status.disableCurrentLimit = disabled;
    Serial.printf("[BatteryManager] Disable current limit: %s\n", disabled ? "ON" : "OFF");
    return true;
}

bool BatteryManager::setDefaultPerPsuVoltage(float volts) {
    // Clamp to Flatpack allowable range 43.5V..57.5V
    if (volts < 43.5f) volts = 43.5f;
    if (volts > 57.5f) volts = 57.5f;
    status.defaultPerPsuVoltage = volts;
    Serial.printf("[BatteryManager] Default per-PSU fallback voltage set to: %.2fV\n", volts);
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
    
    // Update battery status (apply calibration offset)
    status.packVoltage = voltage + voltageCalibrationOffsetV;
    status.packCurrent = current;
    status.temperature = temperature;
    
    if (params.cellCount > 0) {
        status.cellVoltageAvg = status.packVoltage / params.cellCount;
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

    // Reset and initialize current ramp
    rampCurrentA = 0.0f;
    if (status.operatingMode == OperatingMode::MANUAL_CONTROLLED) {
        rampTargetA = status.manualCurrentLimit;
    } else {
        rampTargetA = calculateChargingCurrent();
    }
    rampLastUpdate = millis();
    
    Serial.println("[BatteryManager] Charging started");
    return true;
}

void BatteryManager::stopCharging() {
    status.isCharging = false;
    status.mode = ChargingMode::OFF;
    // Reset LV tracking accumulator
    lastPerPsuVoltageCmdV = 0.0f;
    
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
    getChargingSetpoints(voltage, current, ovp, 1); // Default to 1 PSU for backward compatibility
}

void BatteryManager::getChargingSetpoints(uint16_t& voltage, uint16_t& current, uint16_t& ovp, uint8_t psuCount) {
    if (!initialized || !status.isCharging) {
        voltage = 0;
        current = 0;
        ovp = 0;
        return;
    }
    
    // Calculate total pack voltage based on battery parameters and clamp to safety cap
    float totalPackVoltage = calculateChargingVoltage();
    // Chemistry-based hard cap derived from per-cell max (e.g., 4.16V) and cell count
    const float PACK_CAP_BY_CELLS_V = (params.cellVoltageMax / 1000.0f) * static_cast<float>(params.cellCount);
    const float ABS_MAX_PACK_V_V = 150.0f; // absolute upper bound, secondary
    const float MAX_PACK_VOLTAGE_V = (PACK_CAP_BY_CELLS_V < ABS_MAX_PACK_V_V) ? PACK_CAP_BY_CELLS_V : ABS_MAX_PACK_V_V;
    if (totalPackVoltage > MAX_PACK_VOLTAGE_V) {
        totalPackVoltage = MAX_PACK_VOLTAGE_V;
    }
    
    // Divide voltage by number of PSUs in series and add compensation
    float targetVoltagePerPsu = (totalPackVoltage + status.voltageDropCompensation) / psuCount;
    
    // Convert to centivolts (0.01V units)
    voltage = static_cast<uint16_t>(targetVoltagePerPsu * 100.0f);
    
    // If current limiting is disabled, bypass ramping and LV safe-start. Command target voltage
    // and allow per-PSU current up to device limit. OVP: clamp to chemistry cap (<= 4.16V/cell).
    if (status.disableCurrentLimit) {
        const float PER_PSU_CURRENT_LIMIT_A = 41.7f; // 2kW FP max at ~48V
        current = static_cast<uint16_t>(PER_PSU_CURRENT_LIMIT_A * 10.0f); // deciamps
        // Per-PSU OVP cap from per-cell max
        const float packOvCapV = (params.cellVoltageMax / 1000.0f) * static_cast<float>(params.cellCount);
        const float ovpCapPerPsuV = packOvCapV / static_cast<float>(psuCount);
        // Try to keep ~1.0V headroom over commanded voltage but never exceed cap
        float ovpVoltagePerPsu = targetVoltagePerPsu + 1.0f;
        if (ovpVoltagePerPsu > ovpCapPerPsuV) ovpVoltagePerPsu = ovpCapPerPsuV;
        ovp = static_cast<uint16_t>(ovpVoltagePerPsu * 100.0f);
        return;
    }
    
    // Calculate target charging current (A)
    float desiredCurrentA;
    if (status.operatingMode == OperatingMode::MANUAL_CONTROLLED) {
        desiredCurrentA = status.manualCurrentLimit;
    } else {
        desiredCurrentA = calculateChargingCurrent();
        // Safety cap: never exceed manual limit even in BMS-controlled mode
        if (desiredCurrentA > status.manualCurrentLimit) {
            desiredCurrentA = status.manualCurrentLimit;
        }
    }

    // Enforce per-PSU current limit for 2kW Flatpacks
    const float PER_PSU_CURRENT_LIMIT_A = 41.7f; // 2kW FP max at ~48V
    if (desiredCurrentA > PER_PSU_CURRENT_LIMIT_A) {
        desiredCurrentA = PER_PSU_CURRENT_LIMIT_A;
    }

    // Update ramp target
    rampTargetA = desiredCurrentA;

    // Deep-LV behavior: For single-phase AC presets, duty-cycle the output below 43.5V/PSU
    // to limit average AC power while still raising pack voltage. For other presets, enforce
    // a hard lockout (0A) until >= 43.5V/PSU.
    {
        float measuredPerPsuV_pre = 0.0f;
        if (psuCount > 0) measuredPerPsuV_pre = status.packVoltage / static_cast<float>(psuCount);
        const bool isSinglePhase = (acPreset == AcPreset::SINGLE_PHASE_7A) || (acPreset == AcPreset::SINGLE_PHASE_15A);
        const float PRECHARGE_LOCK_V = FLATPACK_VOLT_MIN - 0.5f;   // e.g., 43.0V with min 43.5V
        const float PRECHARGE_RELEASE_V = FLATPACK_VOLT_MIN;       // release when >= min

        // Common OVP cap computation used by both paths
        float absCellMaxV = params.cellVoltageMax / 1000.0f;
        if (params.chemistry == BatteryChemistry::LIION || params.chemistry == BatteryChemistry::NMC) {
            absCellMaxV = 4.16f;
        }
        float packOvCapV = absCellMaxV * static_cast<float>(params.cellCount);
        float ovpCapPerPsuV = packOvCapV / static_cast<float>(psuCount);

        if (measuredPerPsuV_pre > 0.0f && measuredPerPsuV_pre < PRECHARGE_RELEASE_V) {
            if (isSinglePhase) {
                // Duty-cycled deep-LV precharge to limit average AC draw
                // Compute duty from requested per-PSU manual current vs an estimated on-peak current
                // Keep instantaneous per-PSU peak low to avoid tripping 1P fuses. Use preset-based caps.
                float I_ON_PEAK_A = (acPreset == AcPreset::SINGLE_PHASE_7A) ? 5.0f : 8.0f; // A per PSU (safety-aligned)
                float iAvg = status.manualCurrentLimit;     // pack current == per-PSU current in series
                if (iAvg < 0.1f) iAvg = 0.1f;
                // Duty represents the requested average as a fraction of ON-peak capability
                float duty = iAvg / I_ON_PEAK_A;            // 0..>
                if (duty > 1.0f) duty = 1.0f;               // cannot exceed 100%
                if (duty < 0.05f) duty = 0.05f;             // avoid vanishingly small pulses
                if (duty > 0.85f) duty = 0.85f;             // keep some OFF time for thermal margin

                // Use a coarse 10-second window with 1-second ticks (matches update cadence)
                static uint8_t windowTick = 0;
                static const uint8_t WINDOW_TICKS = 10;
                uint8_t onTicks = (uint8_t)roundf(duty * WINDOW_TICKS);
                if (onTicks < 1) onTicks = 1;
                if (onTicks > WINDOW_TICKS - 1) onTicks = WINDOW_TICKS - 1; // ensure some OFF time
                bool onPhase = (windowTick < onTicks);

                // Advance window position
                windowTick = (windowTick + 1) % WINDOW_TICKS;

                float ovpPerPsuV = FLATPACK_VOLT_MIN + 1.0f; // keep ~1V headroom over commanded
                if (ovpPerPsuV > ovpCapPerPsuV) ovpPerPsuV = ovpCapPerPsuV;

                if (onPhase) {
                    // Short ON burst: command a low stable voltage near min and allow higher current
                    float onV = 43.70f; // low stable point noted in field reports
                    if (onV < FLATPACK_VOLT_MIN) onV = FLATPACK_VOLT_MIN;
                    if (onV > FLATPACK_VOLT_MAX) onV = FLATPACK_VOLT_MAX;
                    uint16_t vCv = static_cast<uint16_t>(onV * 100.0f);
                    uint16_t i_dA = static_cast<uint16_t>(I_ON_PEAK_A * 10.0f); // per-PSU
                    uint16_t ovpCv = static_cast<uint16_t>(ovpPerPsuV * 100.0f);
                    voltage = vCv; current = i_dA; ovp = ovpCv;
                    #ifdef DEBUG_SETPOINTS
                    Serial.printf("[DEEP-LV DUTY] ON %u/%u (duty=%.0f%%): V=%.2f, Ipk=%.1fA/PSU, OVP=%.2f (meas=%.2f, preset=%u)\n",
                                  onTicks, WINDOW_TICKS, duty*100.0f, onV, I_ON_PEAK_A, ovpPerPsuV, measuredPerPsuV_pre, (unsigned)acPreset);
                    #endif
                    return;
                } else {
                    // OFF phase: force output off (0A) at minimum voltage to reduce AC draw
                    float offV = FLATPACK_VOLT_MIN;
                    uint16_t vCv = static_cast<uint16_t>(offV * 100.0f);
                    uint16_t i_dA = 0; // 0.0A disables output
                    uint16_t ovpCv = static_cast<uint16_t>(ovpPerPsuV * 100.0f);
                    voltage = vCv; current = i_dA; ovp = ovpCv;
                    #ifdef DEBUG_SETPOINTS
                    Serial.printf("[DEEP-LV DUTY] OFF %u/%u (duty=%.0f%%): V=%.2f, I=0A, OVP=%.2f (meas=%.2f, preset=%u)\n",
                                  onTicks, WINDOW_TICKS, duty*100.0f, offV, ovpPerPsuV, measuredPerPsuV_pre, (unsigned)acPreset);
                    #endif
                    return;
                }
            } else if (measuredPerPsuV_pre < PRECHARGE_LOCK_V) {
                // Non-single-phase: hold outputs OFF until >= 43.5V/PSU
                float ovpPerPsuV = FLATPACK_VOLT_MIN + 1.0f; // keep ~1V headroom over commanded
                if (ovpPerPsuV > ovpCapPerPsuV) ovpPerPsuV = ovpCapPerPsuV;
                voltage = static_cast<uint16_t>(FLATPACK_VOLT_MIN * 100.0f);
                current = 0; // 0.0A per PSU disables output
                ovp = static_cast<uint16_t>(ovpPerPsuV * 100.0f);
                #ifdef DEBUG_SETPOINTS
                Serial.printf("[SAFE-START] Precharge lockout: V/PSU=%.2fV < %.2fV => OFF (V=%.2f, OVP=%.2f)\n",
                              measuredPerPsuV_pre, PRECHARGE_LOCK_V, FLATPACK_VOLT_MIN, ovpPerPsuV);
                #endif
                return;
            }
        }
    }

    // Low-voltage current control for ALL modes: below ~47V/PSU the FPs don't regulate CC well.
    // Adjust the per-PSU voltage around the measured value to track the desired current.
    // Once above the LV threshold, command the chemistry target directly.
    {
        const float LV_CC_THRESHOLD_V = 47.0f;
        float measuredPerPsuV = 0.0f;
        if (psuCount > 0) measuredPerPsuV = status.packVoltage / static_cast<float>(psuCount);

        if (measuredPerPsuV > 0.0f && measuredPerPsuV < LV_CC_THRESHOLD_V) {
            // Initialize accumulator on first use
            if (lastPerPsuVoltageCmdV <= 0.0f) lastPerPsuVoltageCmdV = measuredPerPsuV;

            // Controller uses desired target current (from rampTargetA) vs measured pack current
            const float desiredI = rampTargetA;            // A (series stack => pack current == per-PSU current)
            const float measuredI = status.packCurrent;     // A
            // Smooth the measured current to avoid reacting to jitter/noise
            static bool lvFiltInit = false;
            static float lvFiltI = 0.0f;
            if (!lvFiltInit) { lvFiltI = measuredI; lvFiltInit = true; }
            else { lvFiltI = 0.8f * lvFiltI + 0.2f * measuredI; }
            const float errorI = desiredI - lvFiltI;        // positive if we want more current

            // Proportional control: ~5 mV per amp. Keep upward deltas small.
            float deltaV = 0.005f * errorI;                 // V
            if (deltaV > 0.01f) deltaV = 0.01f;             // step up at most +0.01V/update
            if (deltaV < -0.20f) deltaV = -0.20f;           // step down at most -0.20V/update

            // Base adjustments strictly on the CURRENT measured voltage in LV region
            // Using last-commanded here caused overshoot and 40A surges when connecting the load.
            float baseV = measuredPerPsuV;
            float cmdV = baseV + deltaV;

            // Immediate overcurrent clamp: soften response to avoid 4<->11A swings.
            // Trigger a bit earlier and drop by a small adaptive amount with a short hold.
            const float OC_MARGIN_A = 0.6f; // start clamping when >0.6A over target
            static unsigned long lvOcHoldUntilMs = 0; // cooldown to block step-ups after OC
            if (measuredI > (desiredI + OC_MARGIN_A)) {
                float overshootA = measuredI - desiredI;
                float dropV = 0.03f * overshootA; // 30 mV per amp overshoot
                if (dropV < 0.08f) dropV = 0.08f; // minimum 80 mV drop
                if (dropV > 0.15f) dropV = 0.15f; // cap at 150 mV drop
                cmdV = measuredPerPsuV - dropV;
                unsigned long holdMs = (overshootA > 1.5f) ? 2000UL : 1200UL; // shorter hold
                lvOcHoldUntilMs = millis() + holdMs;
                #ifdef DEBUG_SETPOINTS
                Serial.printf("[LV-CTRL] OC clamp: over=%.1fA drop=%.2fV hold=%lums, cmd->%.2fV (m=%.2fV)\n",
                              overshootA, dropV, (unsigned long)holdMs, cmdV, measuredPerPsuV);
                #endif
            }

            // Keep commanded voltage near measured in LV region, but allow BELOW measured when over-current.
            // Allow only a TINY positive headroom; Flatpacks can surge with larger +dV near 44V.
            const float BELOW_LIMIT = 1.00f;   // allow up to 1.0V below measured when shedding current fast
            const float ABOVE_LIMIT = 0.01f;   // cap at +0.01V above measured to gently probe upward
            // Only enforce a tiny positive headroom when we're well under target (>2.5A under)
            const float I_MIN_HEADROOM_TRIGGER_A = 2.5f;

            float lowerBound = measuredPerPsuV - BELOW_LIMIT;   // allow dipping below measured
            float upperBound = measuredPerPsuV + ABOVE_LIMIT;   // cap headroom above measured
            // Apply a small deadband to reduce chatter
            const float I_DEADBAND_A = 2.5f; // don't react inside +/-2.5A
            static int underTargetStreak = 0; // consecutive cycles under target beyond deadband
            if (errorI > I_DEADBAND_A) {
                underTargetStreak++;
                // We want more current: allow slightly above measured, but never beyond +ABOVE_LIMIT
                // Only enable tiny positive headroom after a short streak to avoid ping-pong
                bool allowHeadroom = (errorI > I_MIN_HEADROOM_TRIGGER_A) && (underTargetStreak >= 2);
                float minAboveUnder = allowHeadroom ? 0.01f : 0.00f; // 10mV only when far under and sustained
                float minAbove = measuredPerPsuV + minAboveUnder;
                if (cmdV < minAbove) cmdV = minAbove;
            } else if (errorI < -I_DEADBAND_A) {
                underTargetStreak = 0;
                // Over current: allow below measured
                if (cmdV < lowerBound) cmdV = lowerBound;
            } else {
                // Inside deadband: do not accumulate under-target streak
                underTargetStreak = 0;
            }
            // Absolute slew-rate limiting relative to last commanded setpoint to prevent large upward jumps
            if (lastPerPsuVoltageCmdV > 0.0f) {
                const float MAX_STEP_UP_V = 0.01f;   // allow 10mV upward steps (matches centivolt resolution)
                const float MAX_STEP_DOWN_V = 0.20f; // gentle shedding per update
                float maxUp = lastPerPsuVoltageCmdV + MAX_STEP_UP_V;
                float maxDown = lastPerPsuVoltageCmdV - MAX_STEP_DOWN_V;
                if (cmdV > maxUp) cmdV = maxUp;
                if (cmdV < maxDown) cmdV = maxDown;
            }

            // During OC cooldown, block any upward moves and cap at measured
            {
                unsigned long nowMs = millis();
                if (lvOcHoldUntilMs > nowMs) {
                    if (cmdV > measuredPerPsuV) cmdV = measuredPerPsuV; // never above measured during cooldown
                    if (lastPerPsuVoltageCmdV > 0.0f && cmdV > lastPerPsuVoltageCmdV) cmdV = lastPerPsuVoltageCmdV; // freeze up-steps
                }
            }

            if (cmdV > upperBound) cmdV = upperBound;

            // Bound by chemistry target on the high side and hardware minimum on the low side
            if (cmdV > targetVoltagePerPsu) cmdV = targetVoltagePerPsu;
            if (cmdV < FLATPACK_VOLT_MIN) cmdV = FLATPACK_VOLT_MIN;

            voltage = static_cast<uint16_t>(cmdV * 100.0f);
            lastPerPsuVoltageCmdV = cmdV;

            // Debug when deviation is notable
            if (fabsf(errorI) > 2.0f) {
                Serial.printf("[LV-CTRL] m=%.2fV -> cmd=%.2fV (dV=%.2f), bounds=[%.2f..%.2f], I=%.1f/%.1fA, tgt=%.2fV\n",
                              measuredPerPsuV, cmdV, deltaV, lowerBound, upperBound,
                              measuredI, desiredI, targetVoltagePerPsu);
            }
        } else {
            // Outside LV region, command directly to chemistry target and reset accumulator
            lastPerPsuVoltageCmdV = targetVoltagePerPsu;
            // 'voltage' is already set above to the chemistry target
        }
    }

    // Apply 5-minute linear ramp from 0A to target
    unsigned long now = millis();
    if (rampLastUpdate == 0) {
        rampLastUpdate = now;
    }
    float dt = (now - rampLastUpdate) / 1000.0f; // seconds
    // Faster ramp in manual mode for more responsive control
    const bool acPresetActive = (acPreset != AcPreset::NONE);
    const float RAMP_DURATION_S = acPresetActive
                                  ? 30.0f
                                  : ((status.operatingMode == OperatingMode::MANUAL_CONTROLLED) ? 60.0f : 300.0f);
    float rampRateAperS = (rampTargetA > 0.0f) ? (rampTargetA / RAMP_DURATION_S) : 0.0f;
    rampCurrentA += rampRateAperS * dt;
    if (rampCurrentA > rampTargetA) rampCurrentA = rampTargetA;
    if (rampCurrentA < 0.0f) rampCurrentA = 0.0f;
    rampLastUpdate = now;

    // Convert to deciamps (0.1A units)
    current = static_cast<uint16_t>(rampCurrentA * 10.0f);
    // Ensure we send at least 0.1A when ramping up to a non-zero target
    if (current == 0 && rampTargetA > 0.0f) {
        current = 1; // 0.1A
    }
    
    // Set OVP with chemistry-safe absolute cap:
    // - For LIION/NMC use 4.16 V/cell as an absolute safety ceiling regardless of the user CV target
    // - For other chemistries (e.g., LFP/LTO) keep params.cellVoltageMax as the absolute cap
    // We always try to MAINTAIN headroom by RAISING OVP first. Only if that is not possible (cap reached),
    // we will reduce the commanded set voltage to keep a minimum headroom.
    {
        float absCellMaxV = params.cellVoltageMax / 1000.0f;
        if (params.chemistry == BatteryChemistry::LIION || params.chemistry == BatteryChemistry::NMC) {
            // Absolute safety ceiling for Li-ion/NMC
            absCellMaxV = 4.16f;
        }
        const float packOvCapV = absCellMaxV * static_cast<float>(params.cellCount);
        const float ovpCapPerPsuV = packOvCapV / static_cast<float>(psuCount);

        // Current commanded per-PSU voltage from 'voltage' (centivolts)
        float cmdPerPsuV = voltage / 100.0f;

        // Desired OVP headroom (pack protection margin)
        const float desiredHeadroomV = 1.0f; // soft target
        // Dynamic minimum headroom: larger in CC, smaller in CV to avoid lowering the CV target
        const bool isCV = (status.mode == ChargingMode::CONSTANT_VOLTAGE || status.mode == ChargingMode::FLOAT);
        const float minHeadroomV = isCV ? 0.05f : 0.30f;

        float ovpVoltagePerPsu = cmdPerPsuV + desiredHeadroomV;
        if (ovpVoltagePerPsu > ovpCapPerPsuV) ovpVoltagePerPsu = ovpCapPerPsuV;

        // Ensure minimum headroom between OVP and commanded voltage
        if ((ovpVoltagePerPsu - cmdPerPsuV) < minHeadroomV) {
            // First try to raise OVP up to the cap to satisfy minimum headroom
            float candidate = cmdPerPsuV + minHeadroomV;
            if (candidate <= ovpCapPerPsuV) {
                ovpVoltagePerPsu = candidate; // Prefer raising OVP
            } else {
                if (isCV) {
                    // In CV, do NOT lower commanded target; accept smaller headroom at the cap
                    ovpVoltagePerPsu = ovpCapPerPsuV;
                    #ifdef DEBUG_SETPOINTS
                    Serial.printf("[SETPOINT] CV at cap: keeping V/PSU %.2fV, OVP %.2fV (reduced headroom)\n",
                                  cmdPerPsuV, ovpVoltagePerPsu);
                    #endif
                } else {
                    // In CC, keep minimum headroom by slightly reducing the commanded voltage
                    float newCmdV = ovpCapPerPsuV - minHeadroomV;
                    if (newCmdV < FLATPACK_VOLT_MIN) newCmdV = FLATPACK_VOLT_MIN;
                    if (newCmdV < cmdPerPsuV) {
                        voltage = static_cast<uint16_t>(newCmdV * 100.0f);
                        #ifdef DEBUG_SETPOINTS
                        Serial.printf("[SETPOINT] Lowering V/PSU to %.2fV (CC) to keep %.2fV headroom (OVP %.2fV, cap cell=%.2fV)\n",
                                      newCmdV, minHeadroomV, ovpCapPerPsuV, absCellMaxV);
                        #endif
                    }
                    cmdPerPsuV = voltage / 100.0f; // refresh after clamp
                }
            }
        }

        ovp = static_cast<uint16_t>(ovpVoltagePerPsu * 100.0f);
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

void BatteryManager::setBatterySource(BatterySource source) {
    currentSource = source;
    status.dataSource = source;
    Serial.printf("[BatteryManager] Battery source changed to %s\n", 
                 source == BatterySource::MANUAL ? "Manual" :
                 source == BatterySource::CREE_LTO ? "Cree LTO" : "Vectrix VX1");
}

BatterySource BatteryManager::getBatterySource() const {
    return currentSource;
}

void BatteryManager::updateFromVX1Data(const VX1BmsData& vx1Data) {
    if (!vx1Data.dataValid) {
        status.bmsDataValid = false;
        return;
    }
    
    // Update battery status from VX1 BMS data
    // Do NOT override packVoltage here; we rely on actual PSU measurements for pack voltage.
    // status.packVoltage remains as last measured from PSUs (updated via updateStatus()).
    status.cellVoltageMin = vx1Data.cellVoltageMin / 1000.0f;  // Convert mV to V
    status.cellVoltageMax = vx1Data.cellVoltageMax / 1000.0f;  // Convert mV to V
    status.cellVoltageAvg = (status.cellVoltageMin + status.cellVoltageMax) / 2.0f;
    status.voltageDelta = vx1Data.voltageDelta;
    status.temperatureMin = vx1Data.tempMin;
    status.temperatureMax = vx1Data.tempMax;
    status.temperature = vx1Data.tempMax;  // Use max temp for safety
    status.bmsDataValid = true;
    
    // Update timing
    lastUpdateTime = millis();
    
    // Simple SoC estimation based on cell voltage
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
    
    // Check for error conditions based on VX1 data
    status.isError = false;
    status.errorFlags = 0;
    
    // Temperature checks
    if (status.temperatureMax > params.tempMax || status.temperatureMin < params.tempMin) {
        status.isError = true;
        status.errorFlags |= 0x01; // Temperature error
    }
    
    // Voltage checks
    // Do NOT error on crossing the user CV target; only error if exceeding an absolute safety ceiling
    // with a small tolerance for measurement noise.
    {
        const float tolHighV = 0.02f; // 20 mV tolerance on high side
        const float tolLowV  = 0.02f; // 20 mV tolerance on low side

        // Absolute ceiling per chemistry (independent of user slider for LIION/NMC)
        float absCellMaxV = params.cellVoltageMax / 1000.0f;
        if (params.chemistry == BatteryChemistry::LIION || params.chemistry == BatteryChemistry::NMC) {
            absCellMaxV = 4.16f; // safety cap for Li-ion/NMC per requirements
        }

        const float absCellMinV = params.cellVoltageMin / 1000.0f;

        bool overMax = status.cellVoltageMax > (absCellMaxV + tolHighV);
        bool underMin = status.cellVoltageMin < (absCellMinV - tolLowV);
        if (overMax || underMin) {
            status.isError = true;
            status.errorFlags |= 0x04; // Voltage error
            #ifdef DEBUG_STATUS_MESSAGES
            Serial.printf("[BatteryManager] Voltage error: cellMax=%.3fV (absMax=%.3fV), cellMin=%.3fV (absMin=%.3fV)\n",
                          status.cellVoltageMax, absCellMaxV, status.cellVoltageMin, absCellMinV);
            #endif
        }
    }
    
    // High voltage delta check
    if (status.voltageDelta > 200.0f) {  // > 200mV delta
        status.errorFlags |= 0x08; // Balancing needed warning
    }
    
    updateChargingMode();
}

void BatteryManager::updateFromCreeLTO(float voltage, float current, int8_t temperature) {
    // Update battery status from Cree LTO BMS data
    status.packVoltage = voltage;
    status.packCurrent = current;
    status.temperature = temperature;
    status.bmsDataValid = true;
    
    if (params.cellCount > 0) {
        status.cellVoltageAvg = voltage / params.cellCount;
    }
    
    lastUpdateTime = millis();
    
    // Simple SoC estimation
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
    
    updateChargingMode();
}

void BatteryManager::printStatus() const {
    Serial.println("\n=== Battery Status ===");
    
    const char* chemName = "UNKNOWN";
    switch (params.chemistry) {
        case BatteryChemistry::LFP: chemName = "LFP"; break;
        case BatteryChemistry::LTO: chemName = "LTO"; break;
        case BatteryChemistry::NMC: chemName = "NMC"; break;
        case BatteryChemistry::LIION: chemName = "Li-ion"; break;
    }
    
    const char* sourceName = "UNKNOWN";
    switch (status.dataSource) {
        case BatterySource::MANUAL: sourceName = "Manual"; break;
        case BatterySource::CREE_LTO: sourceName = "Cree LTO"; break;
        case BatterySource::VECTRIX_VX1: sourceName = "Vectrix VX1"; break;
    }
    
    Serial.printf("Chemistry: %s\n", chemName);
    Serial.printf("Data Source: %s\n", sourceName);
    Serial.printf("Cell Count: %d\n", params.cellCount);
    Serial.printf("Capacity: %.1f Ah\n", capacityAh);
    Serial.printf("Pack Voltage: %.2f V\n", status.packVoltage);
    Serial.printf("Pack Current: %.2f A\n", status.packCurrent);
    
    if (status.dataSource == BatterySource::VECTRIX_VX1 && status.bmsDataValid) {
        Serial.printf("Cell Voltages: Min=%.3fV, Avg=%.3fV, Max=%.3fV\n", 
                     status.cellVoltageMin, status.cellVoltageAvg, status.cellVoltageMax);
        Serial.printf("Voltage Delta: %.1fmV\n", status.voltageDelta);
        Serial.printf("Temperatures: Min=%d°C, Max=%d°C\n", 
                     status.temperatureMin, status.temperatureMax);
    } else {
        Serial.printf("Cell Voltage Avg: %.3f V\n", status.cellVoltageAvg);
        Serial.printf("Temperature: %d°C\n", status.temperature);
    }
    
    Serial.printf("State of Charge: %d%%\n", status.stateOfCharge);
    Serial.printf("Charging Mode: %s\n", 
                 status.mode == ChargingMode::OFF ? "OFF" :
                 status.mode == ChargingMode::CONSTANT_CURRENT ? "CC" :
                 status.mode == ChargingMode::CONSTANT_VOLTAGE ? "CV" :
                 status.mode == ChargingMode::FLOAT ? "FLOAT" : "ERROR");
    Serial.printf("Charging Time: %d min\n", status.chargingTimeMin);
    Serial.printf("Is Charging: %s\n", status.isCharging ? "Yes" : "No");
    Serial.printf("BMS Data Valid: %s\n", status.bmsDataValid ? "Yes" : "No");
    Serial.printf("Error State: %s\n", status.isError ? "Yes" : "No");
    if (status.isError) {
        Serial.printf("Error Flags: 0x%08X\n", status.errorFlags);
    }
    Serial.println();
}

void BatteryManager::updateChargingMode() {
    if (!status.isCharging) {
        status.mode = ChargingMode::OFF;
        return;
    }
    
    // Get representative cell voltage in volts
    // In VX1 (BMS) mode, use the HIGHEST cell voltage to drive mode switching and termination.
    float cellV = (status.dataSource == BatterySource::VECTRIX_VX1)
                  ? status.cellVoltageMax
                  : status.cellVoltageAvg;
    float cellVMax = params.cellVoltageMax / 1000.0f;
    float cellVFloat = params.cellVoltageFloat / 1000.0f;
    
    // Get charging current as C-rate (proportion of capacity)
    float cRate = 0.0f;
    if (capacityAh > 0) {
        cRate = (status.packCurrent / capacityAh) * 100.0f; // in percent of capacity
    }
    
    ChargingMode previousMode = status.mode;
    
    // Add hysteresis to prevent rapid mode switching
    const float VOLTAGE_HYSTERESIS = 0.05f; // 50mV hysteresis per cell
    
    // Determine charging mode based on voltage and current with hysteresis
    if (status.mode == ChargingMode::FLOAT) {
        // In Float mode - only switch back to CC if voltage drops significantly below target
        if (cellV < (cellVFloat - VOLTAGE_HYSTERESIS)) {
            status.mode = ChargingMode::CONSTANT_CURRENT;
        }
    }
    else if (status.mode == ChargingMode::CONSTANT_VOLTAGE) {
        // Hold CV for a minimum time and use a stricter float threshold for Li-ion/NMC
        const unsigned long CV_HOLD_MIN_MS = 600000UL; // 10 minutes
        float floatThreshold = params.currentFloat;    // percent of capacity
        if (params.chemistry == BatteryChemistry::LIION || params.chemistry == BatteryChemistry::NMC) {
            if (floatThreshold > 2.0f) floatThreshold = 2.0f; // Max 0.02C for Li-ion/NMC
        }

        unsigned long now = millis();
        unsigned long timeInCv = (cvEntryTimeMs > 0 && now >= cvEntryTimeMs) ? (now - cvEntryTimeMs) : 0;

        // In CV mode - switch to Float only if current is low enough AND we've held CV long enough
        if ((cRate <= floatThreshold) && (timeInCv >= CV_HOLD_MIN_MS)) {
            status.mode = ChargingMode::FLOAT;
        }
        // Switch back to CC if voltage drops significantly
        else if (cellV < (cellVMax - VOLTAGE_HYSTERESIS)) {
            status.mode = ChargingMode::CONSTANT_CURRENT;
        }
    }
    else {
        // In CC mode or starting up
        if (cellV < (cellVMax - VOLTAGE_HYSTERESIS)) {
            // Below max voltage with hysteresis - use CC mode
            status.mode = ChargingMode::CONSTANT_CURRENT;
        } 
        else if (cellV >= cellVMax) {
            // At or above max voltage - use CV mode
            status.mode = ChargingMode::CONSTANT_VOLTAGE;
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
        // Track CV entry time to enforce hold period before FLOAT
        if (status.mode == ChargingMode::CONSTANT_VOLTAGE) {
            cvEntryTimeMs = millis();
        }
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

void BatteryManager::setOperatingMode(OperatingMode mode) {
    status.operatingMode = mode;
    Serial.printf("[BatteryManager] Operating mode changed to: %s\n", 
                 mode == OperatingMode::MANUAL_CONTROLLED ? "Manual" : "BMS Controlled");
}

OperatingMode BatteryManager::getOperatingMode() const {
    return status.operatingMode;
}

bool BatteryManager::setManualCurrentLimit(float currentLimit) {
    // Clamp current limit to reasonable range (0.1A to 41.7A per Flatpack2 module)
    if (currentLimit < 0.1f) {
        currentLimit = 0.1f;
    } else if (currentLimit > 41.7f) {
        currentLimit = 41.7f;
    }
    
    status.manualCurrentLimit = currentLimit;
    Serial.printf("[BatteryManager] Manual current limit set to: %.1fA\n", currentLimit);
    return true;
}

float BatteryManager::getManualCurrentLimit() const {
    return status.manualCurrentLimit;
}

bool BatteryManager::setVoltageDropCompensation(float compensation) {
    // Clamp compensation to reasonable range (0-5V)
    if (compensation < 0.0f) {
        compensation = 0.0f;
    } else if (compensation > 5.0f) {
        compensation = 5.0f;
    }
    
    status.voltageDropCompensation = compensation;
    Serial.printf("[BatteryManager] Voltage drop compensation set to: %.2fV\n", compensation);
    return true;
}

float BatteryManager::getVoltageDropCompensation() const {
    return status.voltageDropCompensation;
}

bool BatteryManager::setVoltageCalibrationOffset(float offsetV) {
    // Clamp to a safe range (-5V .. +5V)
    if (offsetV > 5.0f) offsetV = 5.0f;
    if (offsetV < -5.0f) offsetV = -5.0f;
    voltageCalibrationOffsetV = offsetV;
    Serial.printf("[BatteryManager] Voltage calibration offset set to: %.2fV\n", offsetV);
    return true;
}

float BatteryManager::getVoltageCalibrationOffset() const {
    return voltageCalibrationOffsetV;
}

bool BatteryManager::setChargingEnabled(bool enabled) {
    status.isCharging = enabled;
    Serial.printf("[BatteryManager] Charging %s\n", enabled ? "enabled" : "disabled");
    return true;
}

bool BatteryManager::saveSettings() {
    if (!preferences.begin("battery", false)) {
        Serial.println("[BatteryManager] Failed to open NVS namespace");
        return false;
    }
    
    // Save operating mode
    preferences.putUChar("opMode", static_cast<uint8_t>(status.operatingMode));
    
    // Save manual current limit
    preferences.putFloat("manualCurrent", status.manualCurrentLimit);
    
    // Save voltage drop compensation
    preferences.putFloat("voltageComp", status.voltageDropCompensation);
    // Save voltage calibration offset
    preferences.putFloat("voltCal", voltageCalibrationOffsetV);
    // Save disable current limit flag
    preferences.putBool("disableCL", status.disableCurrentLimit);
    // Save default per-PSU fallback voltage
    preferences.putFloat("defaultPSUV", status.defaultPerPsuVoltage);
    // Save AC preset selection
    preferences.putUChar("acPreset", static_cast<uint8_t>(acPreset));
    
    // Save battery parameters
    preferences.putUShort("cellVoltMax", params.cellVoltageMax);
    preferences.putUShort("cellVoltFloat", params.cellVoltageFloat);
    preferences.putUShort("cellVoltMin", params.cellVoltageMin);
    preferences.putUShort("currentMax", params.currentMax);
    preferences.putUShort("currentFloat", params.currentFloat);
    preferences.putUChar("cellCount", params.cellCount);
    preferences.putUChar("chemistry", static_cast<uint8_t>(params.chemistry));
    
    preferences.end();
    Serial.println("[BatteryManager] Settings saved to NVS");
    return true;
}

bool BatteryManager::loadSettings() {
    if (!preferences.begin("battery", true)) {
        Serial.println("[BatteryManager] Failed to open NVS namespace for reading");
        return false;
    }
    
    // Load operating mode
    uint8_t opMode = preferences.getUChar("opMode", static_cast<uint8_t>(OperatingMode::MANUAL_CONTROLLED));
    status.operatingMode = static_cast<OperatingMode>(opMode);
    
    // Load manual current limit
    status.manualCurrentLimit = preferences.getFloat("manualCurrent", 40.0f);
    
    // Load voltage drop compensation
    status.voltageDropCompensation = preferences.getFloat("voltageComp", 0.0f);
    // Load voltage calibration offset
    voltageCalibrationOffsetV = preferences.getFloat("voltCal", 0.0f);
    // Load disable current limit flag
    status.disableCurrentLimit = preferences.getBool("disableCL", false);
    // Load default per-PSU fallback voltage (clamped later on setter)
    status.defaultPerPsuVoltage = preferences.getFloat("defaultPSUV", 43.7f);
    // Load AC preset selection
    acPreset = static_cast<AcPreset>(preferences.getUChar("acPreset", static_cast<uint8_t>(AcPreset::NONE)));
    status.acPresetId = static_cast<uint8_t>(acPreset);
    
    // Load battery parameters if they exist
    if (preferences.isKey("cellVoltMax")) {
        params.cellVoltageMax = preferences.getUShort("cellVoltMax", 4160);
        params.cellVoltageFloat = preferences.getUShort("cellVoltFloat", 4050);
        params.cellVoltageMin = preferences.getUShort("cellVoltMin", 3000);
        params.currentMax = preferences.getUShort("currentMax", 4000);
        params.currentFloat = preferences.getUShort("currentFloat", 200);
        params.cellCount = preferences.getUChar("cellCount", 36);
        uint8_t chemistry = preferences.getUChar("chemistry", static_cast<uint8_t>(BatteryChemistry::NMC));
        params.chemistry = static_cast<BatteryChemistry>(chemistry);
        
        Serial.println("[BatteryManager] Settings loaded from NVS");
        Serial.printf("  Operating Mode: %s\n", status.operatingMode == OperatingMode::MANUAL_CONTROLLED ? "Manual" : "BMS");
        Serial.printf("  Manual Current: %.1fA\n", status.manualCurrentLimit);
        Serial.printf("  Voltage Compensation: %.2fV\n", status.voltageDropCompensation);
        Serial.printf("  Max Cell Voltage: %.3fV\n", params.cellVoltageMax / 1000.0f);
    } else {
        Serial.println("[BatteryManager] No saved settings found, using defaults");
    }
    
    preferences.end();
    return true;
}

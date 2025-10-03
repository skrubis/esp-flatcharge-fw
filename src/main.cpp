#include <Arduino.h>
#include "FlatpackManager.h"
#include "BatteryManager.h"
#include "BatteryManager.h"
#include "FlatpackManager.h"
#include "CANManager.h"
#include "HardwareManager.h"
#include "WebServerManager.h"
#include "VectrixVX1Manager.h"
#include "GreeLTOManager.h"
#include "ADS1220Manager.h"
#include "MetricsManager.h"
#include "charging_profiles.h"
#include <driver/twai.h>

// Build-time configuration (e.g., series PSU count)
#include "BuildConfig.h"

// Global configuration
#define SERIAL_BAUD 115200
#define STATUS_UPDATE_INTERVAL 15000   // Status display interval (15 sec)
#define LOGIN_REFRESH_INTERVAL 5000    // Login refresh interval (5 sec)
#define CHARGING_UPDATE_INTERVAL 1000  // Charging control update interval (1 sec)
#define VOLTAGE_SWEEP_INTERVAL 3000    // Voltage sweep step interval (3 sec)
// Battery configuration options
#define BATTERY_SOURCE_MANUAL 0        // Manual configuration/preset values
#define BATTERY_SOURCE_CREE_LTO 1      // Cree LTO battery with CAN communication  
#define BATTERY_SOURCE_VECTRIX_VX1 2   // Vectrix VX1 BMS via CAN (FEF3 messages)

// Default configuration - can be changed via serial commands or web interface
#define DEFAULT_BATTERY_SOURCE BATTERY_SOURCE_MANUAL
#define DEFAULT_BATTERY_CHEMISTRY BatteryChemistry::LFP
#define DEFAULT_BATTERY_CELL_COUNT 16
#define DEFAULT_BATTERY_CAPACITY 100.0f

// VX1 specific configuration
#define VX1_BATTERY_CHEMISTRY BatteryChemistry::LIION
#define VX1_BATTERY_CAPACITY 157.0f    // Actual VX1 pack capacity (Ah)

// User-configurable battery settings
int userCellCount = 36;                // User-configurable cell count (default 36S for VX1)
float userTargetCellVoltage = 4.15f;   // User-configurable target voltage (4.15V for 36S Li-ion)
static const uint8_t EXPECTED_SERIES_PSU_COUNT = 3; // Number of PSUs in series for fallback programming


// Charging profile presets
#define PROFILE_CITY_VOLTAGE 4.00f     // Conservative for daily city riding
#define PROFILE_TRAVEL_VOLTAGE 4.15f   // Higher capacity for long trips
#define PROFILE_MAX_VOLTAGE 4.20f      // Maximum (use with extreme caution)

// Feature flags - change these to enable/disable features
#define ENABLE_BATTERY_CHARGING true   // Set to true to enable battery charging logic
#define ENABLE_VOLTAGE_SWEEP false      // Set to true to enable voltage sweep mode
#define ENABLE_WEB_SERVER true         // Set to true to enable web server and WiFi

// Configuration validation - ensure mutually exclusive modes
#if ENABLE_BATTERY_CHARGING && ENABLE_VOLTAGE_SWEEP
#error "ENABLE_BATTERY_CHARGING and ENABLE_VOLTAGE_SWEEP cannot both be true - they are mutually exclusive"
#endif

// Global manager instances
FlatpackManager flatpackManager;
BatteryManager batteryManager;
CANManager canManager;
HardwareManager hardwareManager;
WebServerManager webServerManager;
VectrixVX1Manager vx1Manager;
GreeLTOManager greeManager;
ADS1220Manager ads1220;
MetricsManager metrics;

// Mismatch guard: if detected series PSUs != configured, we hold normal updates
static bool gPsuMismatchActive = false;

// TWAI configuration for VX1 mode (250kbps)
// Per pins.md: RX=GPIO5, TX=GPIO6. Macro expects (TX, RX) order.
twai_general_config_t twai_g_config = TWAI_GENERAL_CONFIG_DEFAULT(GPIO_NUM_6, GPIO_NUM_5, TWAI_MODE_NORMAL);
twai_timing_config_t twai_t_config = TWAI_TIMING_CONFIG_250KBITS();
// Keep hardware filter accept-all for now (cross-IDF mapping for extended ID filter is brittle).
// We apply a fast software filter in processTWAIMessages() to reduce CPU load when bike is ON.
twai_filter_config_t twai_f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

// Current battery source configuration (compile-time selection)
#if defined(BATTERY_PLATFORM_VX1)
int currentBatterySource = BATTERY_SOURCE_VECTRIX_VX1;
#elif defined(BATTERY_PLATFORM_GREE_TRACTOR)
int currentBatterySource = BATTERY_SOURCE_CREE_LTO;
#else
int currentBatterySource = BATTERY_SOURCE_VECTRIX_VX1;
#endif

// Timing variables
unsigned long lastStatusPrint = 0;
unsigned long lastLoginRefresh = 0;
unsigned long lastChargingUpdate = 0;
unsigned long lastVoltageSweepUpdate = 0;
unsigned long lastWebServerUpdate = 0;

// TWAI RX statistics
static volatile uint32_t g_twaiRxTotal = 0;
static float g_twaiRxRate = 0.0f;
static unsigned long g_twaiRateTs = 0;
static uint32_t g_twaiRxLast = 0;

extern "C" float getTwaiRxRate() { return g_twaiRxRate; }
extern "C" uint32_t getTwaiRxCount() { return g_twaiRxTotal; }

// Set maximum cell voltage (per cell, Volts). Persists to NVS.
bool setMaxCellVoltageCb(float cellV) {
    Serial.printf("[Main] Setting max cell voltage to %.3fV\n", cellV);
    bool result = batteryManager.setMaxCellVoltage(cellV);
    if (result) result = batteryManager.saveSettings();
    return result;
}

// Apply AC preset: persist selection and compute a safe per-PSU DC current limit
// Preset IDs: 0=NONE, 1=Single-phase 7A (shared), 2=Single-phase 15A (shared), 3=Three-phase 16A (per PSU)
bool setAcPresetCb(uint8_t presetId) {
    Serial.printf("[Main] Setting AC preset to %u\n", presetId);
    // Persist preset selection on device
    batteryManager.setAcPreset(static_cast<AcPreset>(presetId));

    // Gather live data
    const auto& fps = flatpackManager.getFlatpacks();
    std::vector<FlatpackData> online;
    online.reserve(fps.size());
    for (const auto& p : fps) if (p.loggedIn) online.push_back(p);
    const uint8_t psuCount = online.empty() ? SERIES_PSU_COUNT : static_cast<uint8_t>(online.size());

    // Average VAC across online PSUs
    float vacAvg = 230.0f;
    if (!online.empty()) {
        float sumVac = 0.0f; for (const auto& p : online) sumVac += p.inputVoltage; // already in VAC
        vacAvg = sumVac / online.size();
    }

    // Determine per-PSU DC voltage from battery status or PSU readings
    float vpsu = 48.0f;
    const BatteryStatus& bs = batteryManager.getStatus();
    if (bs.packVoltage > 1.0f && psuCount > 0) {
        vpsu = bs.packVoltage / static_cast<float>(psuCount);
    } else if (!online.empty()) {
        float sumV = 0.0f; for (const auto& p : online) sumV += (p.outputVoltage / 100.0f); // cV -> V
        vpsu = sumV / online.size();
    }
    if (vpsu < 43.5f) vpsu = 43.5f; if (vpsu > 57.5f) vpsu = 57.5f;

    // Map preset to per-PSU AC current (A)
    float iac_per_psu = 0.0f;
    if (presetId == static_cast<uint8_t>(AcPreset::SINGLE_PHASE_7A)) {
        iac_per_psu = 7.0f / static_cast<float>(psuCount);
    } else if (presetId == static_cast<uint8_t>(AcPreset::SINGLE_PHASE_15A)) {
        iac_per_psu = 15.0f / static_cast<float>(psuCount);
    } else if (presetId == static_cast<uint8_t>(AcPreset::THREE_PHASE_16A)) {
        iac_per_psu = 16.0f; // each PSU on its own phase
    } else {
        // None/unknown preset: do not change manual limit, just persist preset
        return batteryManager.saveSettings();
    }

    // Convert AC to DC per PSU
    const float eta = 0.93f;   // efficiency estimate
    const float margin = 0.90f; // safety margin
    float idc_per_psu = (vacAvg * iac_per_psu * eta * margin) / (vpsu > 1.0f ? vpsu : 48.0f);
    if (idc_per_psu < 0.1f) idc_per_psu = 0.1f;
    if (idc_per_psu > 41.7f) idc_per_psu = 41.7f;

    Serial.printf("[Main] AC preset computed per-PSU current: %.1fA (VAC=%.0f, V/PSU=%.1f, PSUs=%u)\n",
                  idc_per_psu, vacAvg, vpsu, psuCount);

    // Apply and persist manual current limit
    bool ok = batteryManager.setManualCurrentLimit(idc_per_psu);
    if (ok) ok = batteryManager.saveSettings();
    return ok;
}

// Voltage sweep parameters (in centivolts)
#define VOLTAGE_SWEEP_MIN 4500   // 45.00V
#define VOLTAGE_SWEEP_MAX 5840   // 58.40V
#define VOLTAGE_SWEEP_STEP 100   // 1.00V
#define VOLTAGE_SWEEP_CURRENT 500 // 50.0A fixed current during sweep
#define VOLTAGE_SWEEP_OVP_OFFSET 100 // 1.0V above setpoint for OVP
uint16_t currentSweepVoltage = VOLTAGE_SWEEP_MIN;
bool sweepDirectionUp = true;

/**
 * @brief CAN send callback for FlatpackManager
 * This function is called when FlatpackManager needs to send a CAN message
 * @param frame CAN frame to send
 * @param canBusId ID of the CAN bus to send on (1-3)
 * @return true if message was sent successfully
 */
bool onCanSend(const struct can_frame& frame, uint8_t canBusId) {
    // Validate CAN bus ID (1-3 are valid)
    if (canBusId < 1 || canBusId > 3) {
        Serial.printf("[ERROR] Invalid CAN bus ID for sending: %d\n", canBusId);
        return false;
    }
    
    // Send the message on the specified bus
    bool result = canManager.sendMessage(canBusId, frame);
    
    // Debug output
    if (result) {
        Serial.printf("[CAN%d] >>> ID: 0x%08lX, DLC: %d, Data[0]: 0x%02X\n",
                    canBusId, frame.can_id, frame.can_dlc, frame.data[0]);
    } else {
        Serial.printf("[ERROR] Failed to send CAN message on CAN%d\n", canBusId);
    }
    
    return result;
}

// Constants for battery management
#define DEFAULT_ROOM_TEMPERATURE 25  // Default room temperature in Celsius
#define LOGIN_REFRESH_TIMEOUT 5000   // PSU login refresh timeout (5 seconds)
#define COMMUNICATION_TIMEOUT 10000  // PSU communication timeout (10 seconds)

// Battery state tracking
float totalPackVoltage = 0.0f;
float totalPackCurrent = 0.0f;
int batteryTemperature = DEFAULT_ROOM_TEMPERATURE;
bool chargingEnabled = false;

// Web server callback functions
const std::vector<FlatpackData>& getFlatpackData() {
    return flatpackManager.getFlatpacks();
}

const BatteryStatus& getBatteryStatus() {
    return batteryManager.getStatus();
}

const BatteryParameters& getBatteryParameters() {
    return batteryManager.getParameters();
}

bool setChargingParameters(uint16_t voltage, uint16_t current, uint16_t ovp) {
    return flatpackManager.setOutputParameters(voltage, current, ovp);
}

bool setBatteryParameters(const BatteryParameters& params) {
    batteryManager.setParameters(params);
    return true;
}

bool setOperatingMode(OperatingMode mode) {
    batteryManager.setOperatingMode(mode);
    batteryManager.saveSettings();
    return true;
}

bool setManualCurrentLimit(float current) {
    Serial.printf("[Main] Setting manual current limit to %.1fA\n", current);
    bool result = batteryManager.setManualCurrentLimit(current);
    if (result) batteryManager.saveSettings();
    return result;
}

bool setVoltageDropCompensation(float compensation) {
    Serial.printf("[Main] Setting voltage drop compensation to %.1fV\n", compensation);
    bool result = batteryManager.setVoltageDropCompensation(compensation);
    if (result) batteryManager.saveSettings();
    return result;
}

bool setVoltageCalibrationOffsetCb(float offsetV) {
    Serial.printf("[Main] Setting voltage calibration offset to %.2fV\n", offsetV);
    bool result = batteryManager.setVoltageCalibrationOffset(offsetV);
    if (result) batteryManager.saveSettings();
    return result;
}

// Disable current limiting (dangerous). When enabled, firmware will bypass soft-start and current clamps.
bool setDisableCurrentLimitCb(bool disabled) {
    Serial.printf("[Main] Disable current limit: %s\n", disabled ? "ON" : "OFF");
    bool result = batteryManager.setDisableCurrentLimit(disabled);
    if (result) batteryManager.saveSettings();
    return result;
}

// Set default per-PSU voltage used as NVRAM fallback inside each rectifier
bool setDefaultPerPsuVoltageCb(float volts) {
    Serial.printf("[Main] Setting default per-PSU fallback voltage to %.2fV\n", volts);
    bool result = batteryManager.setDefaultPerPsuVoltage(volts);
    if (result) {
        batteryManager.saveSettings();
        // Program into all detected PSUs now so it's effective on logout/power cycle
        float perPsuV = batteryManager.getDefaultPerPsuVoltage();
        if (perPsuV < 43.5f) perPsuV = 43.5f;
        if (perPsuV > 57.5f) perPsuV = 57.5f;
        uint16_t perPsuCv = static_cast<uint16_t>(perPsuV * 100.0f);
        if (flatpackManager.setDefaultVoltage(perPsuCv, 0)) {
            Serial.printf("[FLATPACK] Programmed default fallback voltage: %.2fV per PSU\n", perPsuV);
        }
    }
    return result;
}

bool setChargingEnabled(bool enabled) {
    Serial.printf("[Main] %s charging\n", enabled ? "Starting" : "Stopping");
    if (enabled) {
        // Start charging sequence to initialize ramp and mode
        return batteryManager.startCharging();
    } else {
        // Stop charging cleanly
        batteryManager.stopCharging();
        // After stopping, push a safe low-output setpoint to quickly reduce current
        // Compute a per-PSU target below current measured voltage to stop current flow
        uint16_t totalCurrent_dA = 0, avgVoltage_cV = 0; uint8_t active = 0;
        float perPsuMeasuredV = 44.0f; // reasonable fallback
        if (flatpackManager.getAggregatedData(totalCurrent_dA, avgVoltage_cV, active) && active > 0) {
            perPsuMeasuredV = avgVoltage_cV / 100.0f;
        }
        float targetPerPsuV = perPsuMeasuredV - 1.0f; // 1V below measured to stop current
        if (targetPerPsuV < FLATPACK_VOLT_MIN) targetPerPsuV = FLATPACK_VOLT_MIN;
        if (targetPerPsuV > FLATPACK_VOLT_MAX) targetPerPsuV = FLATPACK_VOLT_MAX;
        // Chemistry-safe OVP cap
        const BatteryParameters& bp = batteryManager.getParameters();
        float packOvCapV = (bp.cellVoltageMax / 1000.0f) * bp.cellCount;
        float ovpCapPerPsuV = packOvCapV / static_cast<float>(SERIES_PSU_COUNT);
        float ovpPerPsuV = targetPerPsuV + 1.0f; if (ovpPerPsuV > ovpCapPerPsuV) ovpPerPsuV = ovpCapPerPsuV;
        uint16_t vCv = static_cast<uint16_t>(targetPerPsuV * 100.0f);
        uint16_t i_dA = 0; // 0.0A per PSU to force output off
        uint16_t ovpCv = static_cast<uint16_t>(ovpPerPsuV * 100.0f);
        if (flatpackManager.setOutputParameters(vCv, i_dA, ovpCv, 0)) {
            Serial.printf("[SAFE] Post-stop downshift applied: %.2fV/PSU, 0.1A, OVP=%.2fV\n", targetPerPsuV, ovpPerPsuV);
        }
        return true;
    }
}

/**
 * @brief Callback for when a new Flatpack PSU is detected
 * 
 * @param serial Serial number of the detected PSU
 */
void onFlatpackDetected(uint64_t serial) {
    Serial.printf("=== New Flatpack Detected: %012llX ===\n", serial);

    // Immediately send login message to the detected PSU
    // Our canSendCallback will handle sending it to the correct bus
    struct can_frame loginMsg;
    if (flatpackManager.sendLoginMessage(serial, loginMsg)) {
        Serial.printf("[FLATPACK] Login sent for %012llX\n", serial);
        
        // Program a safer low fallback voltage to PSU NVRAM so that if comms are lost or on power cycle,
        // each rectifier falls back to a conservative per-PSU voltage that reduces surge at low pack voltage.
        // Use configured default per-PSU fallback voltage (clamped in setter), within 43.5..57.5V range.
        float perPsuV = batteryManager.getDefaultPerPsuVoltage();
        if (perPsuV < 43.5f) perPsuV = 43.5f;
        if (perPsuV > 57.5f) perPsuV = 57.5f;
        uint16_t perPsuCv = static_cast<uint16_t>(perPsuV * 100.0f);
        if (flatpackManager.setDefaultVoltage(perPsuCv, 0)) { // program all detected PSUs
            Serial.printf("[FLATPACK] Programmed default fallback voltage: %.2fV per PSU\n", perPsuV);
        } else {
            Serial.println("[FLATPACK] WARNING: Failed to program default fallback voltage");
        }

        // Set initial output parameters for the new PSU
        // Our canSendCallback will handle sending to the correct bus
        uint16_t voltage, current, ovp;
        
        // Use configured series PSU count, warn if detection disagrees
        uint8_t detectedCount = flatpackManager.getFlatpacks().size();
        if (detectedCount != SERIES_PSU_COUNT) {
            Serial.printf("[WARN] Detected %u PSUs, but build is configured for %u in series. Using configured value.\n",
                          detectedCount, SERIES_PSU_COUNT);
        }
        batteryManager.getChargingSetpoints(voltage, current, ovp, SERIES_PSU_COUNT);
        
        if (flatpackManager.setOutputParameters(voltage, current, ovp, serial)) {
            Serial.printf("[FLATPACK] Set initial parameters for PSU %012llX\n", serial);
            // Parameters are sent via the canSendCallback
        }

        // Auto-start charging if enabled in settings
        if (batteryManager.getAutoStartCharging()) {
            setChargingEnabled(true);
        }
    } else {
        Serial.printf("[ERROR] Failed to build login message for flatpack %012llX\n", serial);
    }
}

/**
 * @brief Callback for when a Flatpack PSU status update is received
 * 
 * @param flatpack Data structure containing PSU status
 */
void onFlatpackStatus(const FlatpackData& flatpack) {
    // Update only for successful login
    if (flatpack.loggedIn) {
        Serial.printf("[STATUS] Flatpack %012llX: %d.%02dV @ %d.%dA, Status: %d\n",
                    flatpack.serial, 
                    flatpack.outputVoltage/100, flatpack.outputVoltage%100,
                    flatpack.outputCurrent/10, flatpack.outputCurrent%10,
                    flatpack.status);
                    
        // Get aggregated data from all PSUs
        uint16_t totalCurrent, avgVoltage;
        uint8_t activePsuCount;
        
        if (flatpackManager.getAggregatedData(totalCurrent, avgVoltage, activePsuCount)) {
            // Convert from fixed-point to float
            // Series stack assumption: Pack voltage is sum (avg * count). Pack current is the same through each PSU,
            // so use the average (not sum) of PSU currents.
            totalPackVoltage = (avgVoltage / 100.0f) * activePsuCount;       // cV -> V, then sum
            totalPackCurrent = (activePsuCount > 0) ? (totalCurrent / 10.0f) / activePsuCount : 0.0f; // dA -> A, average

            // Note: Do not override control current with ADS1220; keep Flatpack-derived for control
            
            // Update battery manager with latest measurements
            batteryManager.updateStatus(totalPackVoltage, totalPackCurrent, batteryTemperature);
        }
    }
}

/**
 * @brief CAN message processing callback
 * @param msg CAN frame received
 * @param source Source identifier (e.g., "CAN1", "CAN2", etc.)
 */
void onCanMessage(const struct can_frame& msg, const char* source) {
    // Forward message to flatpack manager for processing
    flatpackManager.processCanMessage(msg, source);
    
    // Forward message to VX1 manager if VX1 source is selected
    if (currentBatterySource == BATTERY_SOURCE_VECTRIX_VX1) {
        vx1Manager.processCanMessage(msg.can_id, msg.data, msg.can_dlc);
    }
}

/**
 * @brief TWAI message processing for VX1 mode
 * This function processes messages from the TWAI bus (250kbps for VX1)
 */
void processTWAIMessages() {
    twai_message_t message;
    while (twai_receive(&message, 0) == ESP_OK) {
        // Count all frames from the vehicle bus
        g_twaiRxTotal++;

        const bool isExtended = (message.flags & TWAI_MSG_FLAG_EXTD);
        if (!isExtended) {
            continue;
        }

        if (currentBatterySource == BATTERY_SOURCE_VECTRIX_VX1) {
            // VX1 software filter: only pass Extended FEF3 from SA 0x40 to the VX1 manager
            uint32_t id = message.identifier;
            uint32_t pgn = (id >> 8) & 0x3FFFF; // 18-bit PGN
            uint8_t sa = id & 0xFF;            // Source Address
            if (pgn == 0xFEF3 && sa == 0x40) {
                vx1Manager.processCanMessage(message.identifier, message.data, message.data_length_code);
            }
        } else if (currentBatterySource == BATTERY_SOURCE_CREE_LTO) {
            // Gree/Cree LTO: forward relevant IDs to manager
            uint32_t id = message.identifier;
            // IDs: 0x1807F401, 0x1808F401, 0x1811F401..0x1819F401
            bool inRange = (id == 0x1807F401u) || (id == 0x1808F401u) ||
                           (id >= 0x1811F401u && id <= 0x1819F401u);
            if (inRange) {
                greeManager.processCanMessage(id, message.data, message.data_length_code);
            }
        }
    }

    // Update RX rate once per second
    unsigned long now = millis();
    if (now - g_twaiRateTs >= 1000) {
        float dt = (now - g_twaiRateTs) / 1000.0f;
        if (dt <= 0) dt = 1.0f;
        g_twaiRxRate = (g_twaiRxTotal - g_twaiRxLast) / dt;
        g_twaiRxLast = g_twaiRxTotal;
        g_twaiRateTs = now;
    }
}

/**
 * @brief Update charging parameters on all detected PSUs
 * 
 * Gets charging setpoints from battery manager and sends them to all PSUs
 */
void updateChargingParameters() {
    // Get charging parameters from battery manager
    uint16_t voltage, current, ovp;
    
    // Evaluate runtime stack health (count online vs expected)
    const auto& fps = flatpackManager.getFlatpacks();
    uint8_t online = 0; for (const auto& p : fps) { if (p.loggedIn) ++online; }
    if (online != SERIES_PSU_COUNT) {
        if (!gPsuMismatchActive) {
            Serial.printf("[ERROR] PSU series count mismatch: online=%u, expected=%u. Entering mismatch lockout.\n", online, SERIES_PSU_COUNT);
        }
        gPsuMismatchActive = true;
        // Apply conservative downshift continuously while mismatched
        uint16_t totalCurrent_dA = 0, avgVoltage_cV = 0; uint8_t active = 0;
        float perPsuMeasuredV = 44.0f; // fallback
        if (flatpackManager.getAggregatedData(totalCurrent_dA, avgVoltage_cV, active) && active > 0) {
            perPsuMeasuredV = avgVoltage_cV / 100.0f;
        }
        float targetPerPsuV = perPsuMeasuredV - 1.0f; // 1V below measured to stop current
        if (targetPerPsuV < FLATPACK_VOLT_MIN) targetPerPsuV = FLATPACK_VOLT_MIN;
        if (targetPerPsuV > FLATPACK_VOLT_MAX) targetPerPsuV = FLATPACK_VOLT_MAX;
        const BatteryParameters& bp = batteryManager.getParameters();
        float packOvCapV = (bp.cellVoltageMax / 1000.0f) * bp.cellCount;
        float ovpCapPerPsuV = packOvCapV / static_cast<float>(SERIES_PSU_COUNT);
        float ovpPerPsuV = targetPerPsuV + 1.0f; if (ovpPerPsuV > ovpCapPerPsuV) ovpPerPsuV = ovpCapPerPsuV;
        uint16_t vCv = static_cast<uint16_t>(targetPerPsuV * 100.0f);
        uint16_t i_dA = 1; // 0.1A per PSU
        uint16_t ovpCv = static_cast<uint16_t>(ovpPerPsuV * 100.0f);
        if (flatpackManager.setOutputParameters(vCv, i_dA, ovpCv, 0)) {
            Serial.printf("[SAFE] Mismatch downshift applied: %.2fV/PSU, 0.1A, OVP=%.2fV\n", targetPerPsuV, ovpPerPsuV);
        }
        return;
    } else if (gPsuMismatchActive) {
        Serial.println("[INFO] PSU mismatch resolved. Resuming normal charging updates.");
        gPsuMismatchActive = false;
    }

    // Deep-LV precharge is handled inside BatteryManager to keep the series chain coherent.
    // Avoid per-PSU staggering here; all series PSUs must conduct simultaneously.

    // Use configured series PSU count; warn if runtime detection also disagreed earlier
    uint8_t detectedCount = fps.size();
    if (detectedCount != SERIES_PSU_COUNT) {
        Serial.printf("[WARN] Detected %u PSUs in registry, but using configured %u.\n", detectedCount, SERIES_PSU_COUNT);
    }
    batteryManager.getChargingSetpoints(voltage, current, ovp, SERIES_PSU_COUNT);
    
    // Only update if we have valid setpoints
    if (voltage > 0 && current > 0 && ovp > 0) {
        struct can_frame setpointMsg;
        if (flatpackManager.setOutputParameters(voltage, current, ovp, 0)) { // 0 = all PSUs
            // Parameters are sent via the canSendCallback
            Serial.printf("[CHARGING] Updated setpoints: V=%d.%02dV, I=%d.%dA, OVP=%d.%02dV\n",
                         voltage/100, voltage%100, current/10, current%10, ovp/100, ovp%100);
        }
    }
}

/**
 * @brief Perform a voltage sweep to test PSUs
 * 
 * Incrementally changes the output voltage between min and max values
 * to verify PSU control and operation
 */
void updateVoltageSweep() {
    // Check if it's time to update the voltage
    if (millis() - lastVoltageSweepUpdate < VOLTAGE_SWEEP_INTERVAL) {
        return;
    }
    
    lastVoltageSweepUpdate = millis();
    
    // Set fixed current limit and OVP
    uint16_t current = VOLTAGE_SWEEP_CURRENT;  // 50.0A
    uint16_t ovp = currentSweepVoltage + VOLTAGE_SWEEP_OVP_OFFSET;  // OVP is 1V higher than setpoint
    
    // Update the voltage based on sweep direction
    if (sweepDirectionUp) {
        currentSweepVoltage += VOLTAGE_SWEEP_STEP;
        if (currentSweepVoltage >= VOLTAGE_SWEEP_MAX) {
            currentSweepVoltage = VOLTAGE_SWEEP_MAX;
            sweepDirectionUp = false;
        }
    } else {
        currentSweepVoltage -= VOLTAGE_SWEEP_STEP;
        if (currentSweepVoltage <= VOLTAGE_SWEEP_MIN) {
            currentSweepVoltage = VOLTAGE_SWEEP_MIN;
            sweepDirectionUp = true;
        }
    }
    
    // Apply the voltage to all PSUs
    if (flatpackManager.setOutputParameters(currentSweepVoltage, current, ovp, 0)) {
        Serial.printf("[VOLTAGE SWEEP] Set voltage to %d.%02dV (Current: %d.%dA, OVP: %d.%02dV)\n",
                     currentSweepVoltage/100, currentSweepVoltage%100, 
                     current/10, current%10, 
                     ovp/100, ovp%100);
    }
}

/**
 * @brief Arduino setup function - initializes all components
 */
void setup() {
    // Basic output on UART0 just in case CDC doesn't work
    Serial0.begin(SERIAL_BAUD);
    Serial0.println("Starting up - Basic UART0 output");
    
    // Initialize USB CDC serial
    Serial.begin(SERIAL_BAUD);
    Serial.setTxTimeoutMs(0);  // Disable TX timeout
    
    // Force some output to USB CDC
    Serial.println();
    Serial.println("=== Flatpack CAN Controller Starting ===");
    Serial.println("Hardware: ESP32-S3 + 3x MCP2515 CAN controllers");
    Serial.println("Protocol: Eltek Flatpack2 CAN @ 125kbit/s");
    Serial.flush();
    
    // Additional logging to UART0
    Serial0.println("Initialized USB CDC Serial");
    
    // Wait longer for USB to be ready
    delay(3000);
    
    // Initialize hardware (MCP23017 GPIO expander)
    if (!hardwareManager.initialize()) {
        Serial.println("FATAL: Hardware initialization failed!");
        Serial.println("Check I2C connections to MCP23017 GPIO expander");
        while (1) { 
            Serial.println("System halted - hardware initialization required");
            delay(5000); 
        }
    }
    Serial.println("[INIT] Hardware manager initialized successfully");
    
    // Initialize TWAI for selected platform (both VX1 and Gree use 250kbps)
    if (currentBatterySource == BATTERY_SOURCE_VECTRIX_VX1 ||
        currentBatterySource == BATTERY_SOURCE_CREE_LTO) {
        Serial.println("[INIT] Configuring TWAI (250kbps)");

        if (twai_driver_install(&twai_g_config, &twai_t_config, &twai_f_config) == ESP_OK) {
            Serial.println("[INIT] TWAI driver installed successfully");
            if (twai_start() == ESP_OK) {
                Serial.println("[INIT] TWAI started successfully at 250kbps");
            } else {
                Serial.println("WARNING: TWAI start failed");
            }
        } else {
            Serial.println("WARNING: TWAI driver installation failed");
        }
    }
    
    // Initialize CAN manager (MCP2515 controllers)
    if (!canManager.initialize()) {
        Serial.println("FATAL: CAN initialization failed!");
        Serial.println("Check SPI connections to MCP2515 controllers");
        Serial.println("Verify CAN controller power and reset signals");
        while (1) { 
            Serial.println("System halted - CAN initialization required");
            delay(5000); 
        }
    }
    Serial.println("[INIT] CAN manager initialized successfully");
    
    // Initialize battery manager based on selected source
    BatteryChemistry chemistry;
    uint16_t cellCount;
    float capacity;
    BatterySource source;
    
    switch (currentBatterySource) {
        case BATTERY_SOURCE_VECTRIX_VX1:
            chemistry = VX1_BATTERY_CHEMISTRY;
            cellCount = userCellCount;  // Use user-configurable cell count
            capacity = VX1_BATTERY_CAPACITY;
            source = BatterySource::VECTRIX_VX1;
            Serial.printf("[INIT] Configuring for Vectrix VX1 BMS (%dS, %.2fV target)\n", 
                         userCellCount, userTargetCellVoltage);
            break;
        case BATTERY_SOURCE_CREE_LTO:
            chemistry = BatteryChemistry::LTO;
            cellCount = 36;  // Gree LTO pack (36S)
            capacity = 100.0f;
            source = BatterySource::CREE_LTO;
            Serial.println("[INIT] Configuring for Gree/Cree LTO battery (36S)");
            break;
        case BATTERY_SOURCE_MANUAL:
        default:
            chemistry = DEFAULT_BATTERY_CHEMISTRY;
            cellCount = DEFAULT_BATTERY_CELL_COUNT;
            capacity = DEFAULT_BATTERY_CAPACITY;
            source = BatterySource::MANUAL;
            Serial.println("[INIT] Configuring for manual battery control");
            break;
    }
    
    if (!batteryManager.initialize(chemistry, cellCount, capacity, source)) {
        Serial.println("FATAL: Battery manager initialization failed!");
        Serial.println("Check battery chemistry and parameter configuration");
        while (1) { 
            Serial.println("System halted - battery manager initialization required");
            delay(5000); 
        }
    }
    Serial.println("[INIT] Battery manager initialized successfully");
    
    // Load saved settings after initialization so they override defaults
    batteryManager.loadSettings();
    
    // Initialize VX1 manager if VX1 source is selected
    if (currentBatterySource == BATTERY_SOURCE_VECTRIX_VX1) {
        if (!vx1Manager.initialize(userCellCount)) {
            Serial.println("WARNING: VX1 manager initialization failed!");
        } else {
            Serial.println("[INIT] VX1 manager initialized successfully");
            
            // Set user-configurable target voltage
            vx1Manager.setTargetCellVoltage(userTargetCellVoltage);
            
            // Set up VX1 callbacks to update battery manager
            vx1Manager.setDataCallback([](const VX1BmsData& data) {
                batteryManager.updateFromVX1Data(data);
            });
        }
    }

    // Initialize Gree LTO manager if selected
    if (currentBatterySource == BATTERY_SOURCE_CREE_LTO) {
        if (!greeManager.initialize(36)) {
            Serial.println("WARNING: Gree LTO manager initialization failed!");
        } else {
            Serial.println("[INIT] Gree LTO manager initialized successfully");
            greeManager.setDataCallback([](const GreeLtoData& gd) {
                // Use ADS1220 SPI current sensor for pack current when available
                float iA = NAN;
                if (ads1220.isValid()) {
                    iA = ads1220.getCurrentA();
                } else {
                    // Fallback: sum PSU currents (deciamps -> amps) if ADS1220 not ready
                    const auto& fps = flatpackManager.getFlatpacks();
                    uint16_t sum_dA = 0; for (const auto& p : fps) sum_dA += p.outputCurrent;
                    iA = sum_dA / 10.0f;
                }
                batteryManager.updateFromGreeLTODetailed(
                    gd.pack_voltage,
                    iA,
                    gd.temperature_1,
                    gd.temperature_2,
                    gd.min_cell_voltage,
                    gd.max_cell_voltage,
                    gd.cell_voltage_delta * 1000.0f, // mV
                    gd.soc_percent
                );
            });
        }
    }
    
    // Legacy ESP32 ADC current sensor removed; using ADS1220 exclusively
    Serial.println("[INIT] Using ADS1220 current sensor (ESP32 ADC path removed)");

    // Initialize ADS1220 current sensor (preferred for analysis/display)
    {
        ADS1220Manager::Config cfg; cfg.csPin = 39; cfg.drdyPin = -1; // poll without DRDY to ensure updates
        cfg.sampleRateSPS = 90; // faster response
        if (ads1220.begin(cfg)) {
            Serial.println("[INIT] ADS1220 current sensor initialized");
            ads1220.loadCalibration();
        } else {
            Serial.println("WARNING: ADS1220 current sensor init failed");
        }
    }
    
    // Set up callbacks
    canManager.setMessageCallback(onCanMessage);
    flatpackManager.setDetectionCallback(onFlatpackDetected);
    flatpackManager.setStatusCallback(onFlatpackStatus);
    flatpackManager.setCanSendCallback(onCanSend);  // Register our new CAN send callback
    Serial.println("[INIT] Callback handlers registered");
    
    // Initialize web server if enabled
    if (ENABLE_WEB_SERVER) {
        if (webServerManager.initialize(80)) {
            // Set up web server callbacks
            webServerManager.setFlatpackDataCallback(getFlatpackData);
            webServerManager.setBatteryStatusCallback(getBatteryStatus);
            webServerManager.setBatteryParametersCallback(getBatteryParameters);
            webServerManager.setChargingParametersCallback(setChargingParameters);
            webServerManager.setBatteryParametersCallback(setBatteryParameters);
            webServerManager.setOperatingModeCallback(setOperatingMode);
            webServerManager.setManualCurrentCallback(setManualCurrentLimit);
            webServerManager.setVoltageCompensationCallback(setVoltageDropCompensation);
            webServerManager.setChargingEnabledCallback(setChargingEnabled);
            webServerManager.setVoltageCalibrationCallback(setVoltageCalibrationOffsetCb);
            webServerManager.setDisableCurrentLimitCallback(setDisableCurrentLimitCb);
            webServerManager.setDefaultPerPsuVoltageCallback(setDefaultPerPsuVoltageCb);
            webServerManager.setAcPresetCallback(setAcPresetCb);
            webServerManager.setMaxCellVoltageCallback(setMaxCellVoltageCb);
            // Auto-start charging toggle
            webServerManager.setAutoStartChargingCallback([](bool enable){
                bool ok = batteryManager.setAutoStartCharging(enable);
                if (ok) ok = batteryManager.saveSettings();
                return ok;
            });
            // Metrics (InfluxDB) config endpoints
            webServerManager.setMetricsGetCallback([](){
                return metrics.getConfigJSON();
            });
            webServerManager.setMetricsSetCallback([](const String& body, String& message){
                return metrics.setConfigFromJSON(body, message);
            });
            // ADS1220 endpoints
            webServerManager.setAdsGetCallback([](float& currentA, bool& valid, float& zeroV, float& apv, float& rawV){
                // Return the latest current even if "valid" is borderline; use a longer validity window
                currentA = ads1220.getCurrentA();
                valid = ads1220.isValid(3000);
                zeroV = ads1220.getZeroOffsetVolts();
                apv = ads1220.getScaleAmpsPerVolt();
                rawV = ads1220.getLastDiffVolts();
            });
            webServerManager.setAdsCalZeroCallback([](uint16_t avgSamples){
                return ads1220.calibrateZero(avgSamples);
            });
            webServerManager.setAdsSetScaleCallback([](float apv){
                return ads1220.setScaleAmpsPerVolt(apv);
            });
            // Gree JSON provider (cells, temps, SOC)
            webServerManager.setGreeJsonCallback([]() -> String {
                GreeLtoData gd = greeManager.getData();
                bool valid = greeManager.isDataValid(2000);
                String s; s.reserve(2048);
                s += "{";
                s += "\"valid\":"; s += (valid?"true":"false"); s += ",";
                s += "\"packVoltage\":"; s += String(gd.pack_voltage, 3); s += ",";
                s += "\"minCell\":"; s += String(gd.min_cell_voltage, 3); s += ",";
                s += "\"maxCell\":"; s += String(gd.max_cell_voltage, 3); s += ",";
                s += "\"delta\":"; s += String(gd.cell_voltage_delta, 3); s += ",";
                s += "\"soc\":"; s += String((int)gd.soc_percent); s += ",";
                s += "\"t1\":"; s += String((int)gd.temperature_1); s += ",";
                s += "\"t2\":"; s += String((int)gd.temperature_2); s += ",";
                s += "\"cells\":[";
                for (int i=0;i<36;i++) { s += String(gd.cell_voltages[i],3); if (i!=35) s += ","; }
                s += "]";
                s += "}";
                return s;
            });
            
            // Start Access Point mode
            if (webServerManager.startAccessPoint("FLATCHARGE", "flatcharge!")) {
                Serial.println("[INIT] Web server and WiFi AP started successfully");
                Serial.printf("[INIT] Web interface: %s\n", webServerManager.getWebServerURL().c_str());
            } else {
                Serial.println("[ERROR] Failed to start WiFi Access Point");
            }
        } else {
            Serial.println("[ERROR] Failed to initialize web server");
        }
    } else {
        Serial.println("[INIT] Web server disabled by configuration flag");
    }
    
    Serial.println("=== System Ready ===");
    Serial.println("Waiting for Flatpack detection...");
    Serial.println("Make sure flatpack has AC power and is connected to CAN1, CAN2 or CAN3");
    
    // Runtime configuration validation
    if (ENABLE_BATTERY_CHARGING && ENABLE_VOLTAGE_SWEEP) {
        Serial.println("FATAL: Both battery charging and voltage sweep modes are enabled!");
        Serial.println("These modes are mutually exclusive. Please disable one in main.cpp");
        while (1) { delay(1000); }
    }
    
    // Handle battery charging and voltage sweep based on configuration flags
    if (ENABLE_BATTERY_CHARGING) {
        chargingEnabled = true;
        batteryManager.startCharging();
        Serial.println("[INIT] Battery charging enabled");
    } else {
        chargingEnabled = false;
        Serial.println("[INIT] Battery charging disabled by configuration flag");
    }
    
    if (ENABLE_VOLTAGE_SWEEP) {
        Serial.printf("[INIT] Voltage sweep mode enabled (Range: %.1fV to %.1fV, Step: %.1fV, Interval: %dms)\n",
                    VOLTAGE_SWEEP_MIN/100.0f, VOLTAGE_SWEEP_MAX/100.0f, VOLTAGE_SWEEP_STEP/100.0f, VOLTAGE_SWEEP_INTERVAL);
    }
    
    if (!ENABLE_BATTERY_CHARGING && !ENABLE_VOLTAGE_SWEEP) {
        Serial.println("[INIT] WARNING: Neither battery charging nor voltage sweep is enabled");
        Serial.println("[INIT] PSUs will be detected but no charging control will be performed");
    }

    // Metrics providers and start
    metrics.setBatteryProvider([](){ return batteryManager.getStatus(); });
    metrics.setGreeProvider([](GreeLtoData& out){ out = greeManager.getData(); return greeManager.isDataValid(2000); });
    metrics.begin();
}

/**
 * @brief Arduino main loop function
 */
void loop() {
    // Process CAN messages
    canManager.processMessages();
    
    // Process TWAI messages for VX1 mode
    processTWAIMessages();
    
    // Update managers
    flatpackManager.update();
    batteryManager.update();
    ads1220.update();
    metrics.update();
    
    // Update web server if enabled
    if (ENABLE_WEB_SERVER) {
        webServerManager.update();
    }
    
    // Handle charging updates
    unsigned long currentTime = millis();
    
    // Refresh flatpack logins periodically
    if (currentTime - lastLoginRefresh > LOGIN_REFRESH_INTERVAL) {
        lastLoginRefresh = currentTime;

        bool anyLoginSent = false;
        // Re-login to all detected flatpacks to keep them logged in
        auto flatpacks = flatpackManager.getFlatpacks();
        for (const auto& fp : flatpacks) {
            if (fp.detected) {
                struct can_frame loginMsg;
                if (flatpackManager.sendLoginMessage(fp.serial, loginMsg)) {
                    anyLoginSent = true;
                    #ifdef DEBUG_LOGIN_MESSAGES
                    Serial.printf("[FLATPACK] Login refresh for %012llX initiated\n", fp.serial);
                    #endif
                }
            }
        }

        // Immediately re-apply charging setpoints after any login refresh to ensure
        // current limit and voltage are enforced even if a PSU had logged out.
        if (anyLoginSent) {
            updateChargingParameters();
        }
    }
    
    // Either run battery charging or voltage sweep based on configuration
    if (ENABLE_BATTERY_CHARGING && chargingEnabled) {
        // Update charging parameters periodically
        if (currentTime - lastChargingUpdate > CHARGING_UPDATE_INTERVAL) {
            lastChargingUpdate = currentTime;
            updateChargingParameters();
        }
    } else if (ENABLE_VOLTAGE_SWEEP) {
        // Run voltage sweep mode
        updateVoltageSweep();
    }
    
    // Print status periodically
    if (currentTime - lastStatusPrint > STATUS_UPDATE_INTERVAL) {
        lastStatusPrint = currentTime;
        
        Serial.println("\n=== SYSTEM STATUS ===");
        
        // Print flatpack status
        flatpackManager.printStatus();
        
        // Print battery status
        batteryManager.printStatus();
        
        // Print VX1 status if VX1 source is selected
        if (currentBatterySource == BATTERY_SOURCE_VECTRIX_VX1) {
            vx1Manager.printStatus();
        }
        
        // Print ADS1220 current sensor status
        Serial.println("\n=== ADS1220 STATUS ===");
        Serial.printf("Valid: %s\n", ads1220.isValid() ? "YES" : "NO");
        Serial.printf("Current: %.3f A\n", ads1220.getCurrentA());
        Serial.printf("Zero Offset: %.3f V\n", ads1220.getZeroOffsetVolts());
        Serial.printf("Scale: %.3f A/V\n", ads1220.getScaleAmpsPerVolt());
        
        // Print web server status if enabled
        if (ENABLE_WEB_SERVER) {
            webServerManager.printStatus();
        }
        
        // Check if no PSUs detected and provide troubleshooting info
        auto flatpacks = flatpackManager.getFlatpacks();
        if (flatpacks.empty()) {
            Serial.println("\n=== NO FLATPACK PSUs DETECTED ===");
            Serial.println("Troubleshooting steps:");
            Serial.println("1. Verify Flatpack has AC power (LED indicators should be on)");
            Serial.println("2. Check CAN bus connections (CAN-H, CAN-L, GND)");
            Serial.println("3. Verify CAN bus termination resistors (120 ohm)");
            Serial.println("4. Check CAN controller power and reset signals");
            Serial.println("5. Monitor CAN traffic with oscilloscope if available");
        } else {
            // Check for communication issues with detected PSUs
            bool hasCommIssues = false;
            for (const auto& fp : flatpacks) {
                if (fp.detected && !fp.loggedIn) {
                    if (!hasCommIssues) {
                        Serial.println("\n=== COMMUNICATION ISSUES DETECTED ===");
                        hasCommIssues = true;
                    }
                    Serial.printf("PSU %s: Detected but not logged in\n", fp.serialStr);
                }
            }
        }
        Serial.println();
    }
    
    // Small delay to prevent overwhelming the system
    delay(10);
}

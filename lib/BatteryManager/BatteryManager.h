#pragma once

#include <Arduino.h>
#include <vector>
#include <functional>
#include <Preferences.h>

// Forward declarations
struct VX1BmsData;

/**
 * @brief Battery chemistry types
 */
enum class BatteryChemistry {
    LFP,  // Lithium Iron Phosphate
    LTO,  // Lithium Titanate Oxide
    NMC,  // Lithium Nickel Manganese Cobalt
    LIION // Generic Li-ion (for VX1)
};

/**
 * @brief Battery data source
 */
enum class BatterySource {
    MANUAL,     // Manual configuration
    CREE_LTO,   // Cree LTO battery with CAN communication
    VECTRIX_VX1 // Vectrix VX1 BMS via CAN (FEF3 messages)
};

/**
 * @brief Operating mode for charging control
 */
enum class OperatingMode {
    BMS_CONTROLLED,  // BMS controls charging parameters
    MANUAL_CONTROLLED // Manual control via web interface
};

/**
 * @brief AC input presets used to compute a safe per-PSU DC current
 */
enum class AcPreset : uint8_t {
    NONE = 0,
    SINGLE_PHASE_7A = 1,   // Shared across all PSUs
    SINGLE_PHASE_15A = 2,  // Shared across all PSUs
    THREE_PHASE_16A = 3    // Per phase / per PSU
};

/**
 * @brief Charging mode
 */
enum class ChargingMode {
    OFF,        // Charging disabled
    CONSTANT_CURRENT,  // Constant current mode
    CONSTANT_VOLTAGE,  // Constant voltage mode
    FLOAT,      // Float/maintenance charging
    ERROR       // Error state
};

/**
 * @brief Battery charging parameters
 * All voltage values are per-cell voltages in millivolts
 */
struct BatteryParameters {
    BatteryChemistry chemistry;
    uint16_t cellCount;
    
    // Voltage limits per cell (in millivolts)
    uint16_t cellVoltageMin;      // Minimum cell voltage
    uint16_t cellVoltageNominal;  // Nominal cell voltage
    uint16_t cellVoltageMax;      // Maximum cell voltage
    uint16_t cellVoltageFloat;    // Float/maintenance voltage
    
    // Current limits (in percent of capacity, 0-100)
    uint8_t currentMax;           // Maximum charging current (0-100% of capacity)
    uint8_t currentTaper;         // Current at which to begin CV mode (0-100% of capacity)
    uint8_t currentFloat;         // Current for float charging (0-100% of capacity)
    
    // Temperature limits (in °C)
    int8_t tempMin;               // Minimum charging temperature
    int8_t tempMax;               // Maximum charging temperature
    
    // Charging timers (in minutes)
    uint16_t maxChargingTime;     // Maximum total charging time
    uint16_t maxCCTime;           // Maximum time in CC mode
    uint16_t maxCVTime;           // Maximum time in CV mode
    uint16_t maxFloatTime;        // Maximum time in float mode
};

/**
 * @brief Battery status information
 */
struct BatteryStatus {
    float packVoltage;            // Total pack voltage (V)
    float packCurrent;            // Total pack current (A)
    float cellVoltageAvg;         // Average cell voltage (V)
    float cellVoltageMin;         // Minimum cell voltage (V) - from BMS
    float cellVoltageMax;         // Maximum cell voltage (V) - from BMS
    float voltageDelta;           // Voltage difference between max and min cells (mV)
    int8_t temperature;           // Battery temperature (°C)
    int8_t temperatureMin;        // Minimum temperature (°C) - from BMS
    int8_t temperatureMax;        // Maximum temperature (°C) - from BMS
    ChargingMode mode;            // Current charging mode
    uint8_t stateOfCharge;        // Estimated state of charge (0-100%)
    uint16_t chargingTimeMin;     // Time spent charging (minutes)
    bool isCharging;              // Whether actively charging
    bool isError;                 // Whether error condition exists
    uint32_t errorFlags;          // Error condition flags
    BatterySource dataSource;     // Current data source
    bool bmsDataValid;            // Whether BMS data is valid and recent
    OperatingMode operatingMode;  // Manual or BMS controlled
    float manualCurrentLimit;     // Manual current limit (A) when in manual mode
    float voltageDropCompensation; // Voltage drop compensation (V)
    bool disableCurrentLimit;      // If true, bypass firmware current limiting (dangerous)
    float defaultPerPsuVoltage;    // Default fallback voltage per PSU (V) for logout
    uint8_t acPresetId;            // Persisted AC preset selection (see AcPreset)
};

/**
 * @brief Manager for battery charging
 * 
 * Handles:
 * - Different battery chemistries (LFP, LTO, NMC)
 * - Charging profiles and curves
 * - CC/CV mode transitions
 * - Cell voltage limits
 */
class BatteryManager {
public:
    BatteryManager();
    ~BatteryManager();
    
    /**
     * @brief Initialize battery parameters
     * 
     * @param chemistry Battery chemistry type
     * @param cellCount Number of cells in series
     * @param capacityAh Battery capacity in Amp-hours
     * @param source Battery data source type
     * @return true if initialization successful
     */
    bool initialize(BatteryChemistry chemistry, uint16_t cellCount, float capacityAh, BatterySource source = BatterySource::MANUAL);
    
    /**
     * @brief Configure battery parameters
     * 
     * @param params Battery parameters
     */
    void setParameters(const BatteryParameters& params);
    
    /**
     * @brief Get current battery parameters
     * 
     * @return BatteryParameters structure
     */
    const BatteryParameters& getParameters() const;
    
    /**
     * @brief Update battery status with new measurements
     * 
     * @param voltage Current battery voltage (V)
     * @param current Current charging current (A)
     * @param temperature Battery temperature (°C)
     */
    void updateStatus(float voltage, float current, int8_t temperature);
    
    /**
     * @brief Get current battery status
     * 
     * @return BatteryStatus structure
     */
    const BatteryStatus& getStatus() const;
    
    /**
     * @brief Start charging process
     * 
     * @return true if charging started successfully
     */
    bool startCharging();
    
    /**
     * @brief Stop charging process
     */
    void stopCharging();
    
    /**
     * @brief Update charging logic (call in main loop)
     * 
     * @return Charging setpoints for PSUs
     */
    void update();
    
    /**
     * @brief Get charging setpoints
     * 
     * @param voltage Output voltage setpoint in centivolts
     * @param current Output current setpoint in deciamps
     * @param ovp Over-voltage protection setpoint in centivolts
     */
    void getChargingSetpoints(uint16_t& voltage, uint16_t& current, uint16_t& ovp);
    
    /**
     * @brief Get charging setpoints for series-connected PSUs
     * 
     * @param voltage Output voltage setpoint in centivolts (per PSU)
     * @param current Output current setpoint in deciamps
     * @param ovp Over-voltage protection setpoint in centivolts (per PSU)
     * @param psuCount Number of PSUs in series (voltage will be divided by this)
     */
    void getChargingSetpoints(uint16_t& voltage, uint16_t& current, uint16_t& ovp, uint8_t psuCount);
    
    /**
     * @brief Reset charging statistics and timers
     */
    void resetChargingStats();
    
    /**
     * @brief Set battery data source
     * 
     * @param source Battery data source type
     */
    void setBatterySource(BatterySource source);
    
    /**
     * @brief Get current battery data source
     * 
     * @return Current battery data source
     */
    BatterySource getBatterySource() const;
    
    /**
     * @brief Update battery status from VX1 BMS data
     * 
     * @param vx1Data VX1 BMS data structure
     */
    void updateFromVX1Data(const struct VX1BmsData& vx1Data);
    
    /**
     * @brief Set operating mode (BMS controlled vs Manual)
     * 
     * @param mode Operating mode to set
     */
    void setOperatingMode(OperatingMode mode);
    
    /**
     * @brief Get current operating mode
     * 
     * @return Current operating mode
     */
    OperatingMode getOperatingMode() const;
    
    /**
     * @brief Set manual current limit
     * 
     * @param currentLimit Current limit in Amperes (0.1A to 40A)
     * @return true if successful
     */
    bool setManualCurrentLimit(float currentLimit);
    
    /**
     * @brief Get manual current limit
     * 
     * @return Manual current limit in Amperes
     */
    float getManualCurrentLimit() const;
    
    /**
     * @brief Set voltage drop compensation
     * 
     * @param compensation Voltage compensation in volts
     * @return true if successful
     */
    bool setVoltageDropCompensation(float compensation);
    
    /**
     * @brief Enable or disable charging
     * 
     * @param enabled True to enable charging, false to disable
     * @return true if successful
     */
    bool setChargingEnabled(bool enabled);

    /**
     * @brief Save current settings to NVS
     * 
     * @return true if successful
     */
    bool saveSettings();

    /**
     * @brief Load settings from NVS
     * 
     * @return true if successful
     */
    bool loadSettings();
    
    /**
     * @brief Get voltage drop compensation
     * 
     * @return Voltage drop compensation in Volts
     */
    float getVoltageDropCompensation() const;

    /**
     * @brief Set displayed pack voltage calibration offset (adds to measured voltage)
     *
     * @param offsetV Offset in Volts (positive to increase displayed voltage)
     * @return true if successful
     */
    bool setVoltageCalibrationOffset(float offsetV);

    /**
     * @brief Get displayed pack voltage calibration offset in Volts
     */
    float getVoltageCalibrationOffset() const;
    
    // Advanced control flags
    bool setDisableCurrentLimit(bool disabled);
    bool getDisableCurrentLimit() const { return status.disableCurrentLimit; }
    bool setDefaultPerPsuVoltage(float volts);
    float getDefaultPerPsuVoltage() const { return status.defaultPerPsuVoltage; }
    // AC preset selection (persisted)
    void setAcPreset(AcPreset preset) { acPreset = preset; status.acPresetId = static_cast<uint8_t>(preset); }
    AcPreset getAcPreset() const { return acPreset; }

    /**
     * @brief Set maximum cell voltage (per cell) in Volts
     * Clamped to a chemistry-safe range; persisted via saveSettings().
     */
    bool setMaxCellVoltage(float cellV);
    
    /**
     * @brief Update battery status from Cree LTO BMS data
     * 
     * @param voltage Pack voltage (V)
     * @param current Pack current (A)
     * @param temperature Temperature (°C)
     */
    void updateFromCreeLTO(float voltage, float current, int8_t temperature);

    /**
     * @brief Update battery status from Gree LTO (detailed) data
     * Includes per-pack min/max/delta volts (delta in mV), two temps, and SOC.
     */
    void updateFromGreeLTODetailed(float packV, float currentA, int8_t temp1, int8_t temp2,
                                   float cellMinV, float cellMaxV, float delta_mV, uint8_t socPercent);
    
    /**
     * @brief Print battery status information to Serial
     */
    void printStatus() const;
    
private:
    BatteryParameters params;
    BatteryStatus status;
    
    float capacityAh;             // Battery capacity in Ah
    unsigned long lastUpdateTime; // Last update time
    bool initialized;             // Whether battery is initialized
    BatterySource currentSource;  // Current battery data source
    
    // Default parameters for different chemistries
    static const BatteryParameters defaultLFP;
    static const BatteryParameters defaultLTO;
    static const BatteryParameters defaultNMC;
    static const BatteryParameters defaultLIION;
    
    // Internal methods
    void updateChargingMode();
    void updateBatteryHealth();
    void updateStateOfCharge();
    void validateParameters();

    // NVS storage
    Preferences preferences;

    /**
     * @brief Calculate optimal charging current
     * 
     * @return Optimal charging current in Amps
     */
    float calculateChargingCurrent() const;
    
    /**
     * @brief Calculate optimal charging voltage
     * 
     * @return Optimal charging voltage in Volts
     */
    float calculateChargingVoltage() const;

    // Current ramp control (to avoid inrush)
    float rampCurrentA;           // Current applied to chargers (ramps towards target)
    float rampTargetA;            // Target current limit (A)
    unsigned long rampLastUpdate; // Last time ramp was updated

    // Calibration
    float voltageCalibrationOffsetV; // Applied to displayed/measured pack voltage

    // AC preset persistence
    AcPreset acPreset = AcPreset::NONE;

    // Track last commanded per-PSU voltage to allow LV current tracking to step up
    // beyond measured+step each update (prevents getting stuck at measured+0.5V)
    float lastPerPsuVoltageCmdV;

    // Timestamp when we entered CV mode; used to hold CV for a minimum time before FLOAT
    unsigned long cvEntryTimeMs = 0;
};

#pragma once

#include <Arduino.h>
#include <vector>

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
 * @brief Battery data source types
 */
enum class BatterySource {
    MANUAL,     // Manual configuration/preset values
    CREE_LTO,   // Cree LTO battery with CAN communication
    VECTRIX_VX1 // Vectrix VX1 BMS via CAN (FEF3 messages)
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
     * @brief Update battery status from Cree LTO BMS data
     * 
     * @param voltage Pack voltage (V)
     * @param current Pack current (A)
     * @param temperature Temperature (°C)
     */
    void updateFromCreeLTO(float voltage, float current, int8_t temperature);
    
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
    
    /**
     * @brief Update charging mode based on battery state
     */
    void updateChargingMode();
    
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
};

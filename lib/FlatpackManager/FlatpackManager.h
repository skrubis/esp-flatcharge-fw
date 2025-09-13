#pragma once

#include <Arduino.h>
#include <mcp2515.h>
#include <vector>
#include <functional>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

/**
 * @brief Data structure for Flatpack PSU
 * Contains all data related to a single Flatpack2 power supply unit
 */
struct FlatpackData {
    uint64_t serial;                // Serial number as decimal value
    uint8_t serialBytes[6];         // Raw serial number bytes
    char serialStr[13];             // Serial as string for display
    bool detected;                  // Whether the PSU has been detected
    bool loggedIn;                  // Whether we're currently logged in
    uint8_t canBusId;               // Which CAN bus this PSU is on (1-3)
    uint8_t psuId;                  // PSU ID for CAN addressing (1-63)
    unsigned long lastStatusTime;   // Last time status was received
    unsigned long lastLoginTime;    // Last time login was sent
    
    // Status information
    uint8_t status;                 // Status code
    uint8_t intakeTemp;             // Intake temperature in °C
    uint8_t exhaustTemp;            // Exhaust temperature in °C
    uint16_t inputVoltage;          // Input voltage (AC)
    uint16_t outputVoltage;         // Output voltage in centivolts (V*100)
    uint16_t outputCurrent;         // Output current in deciamps (A*10)
    
    // Command settings
    uint16_t setVoltage;            // Commanded voltage setpoint
    uint16_t setCurrent;            // Commanded current setpoint
    uint16_t setOvp;                // Commanded OVP setpoint
    
    // Alert information
    uint8_t alertFlags[2];          // Alert flag bytes
    bool hasAlerts;                 // Whether alerts are active
};

/**
 * @brief Manager for Eltek Flatpack2 power supplies
 * 
 * Handles:
 * - Auto-detection of Flatpacks on CAN buses
 * - Login/authentication
 * - Status monitoring
 * - Command transmission
 * - Alert handling
 * - Multiple PSU aggregation
 */
class FlatpackManager {
public:
    // Callback function types
    using FlatpackDetectionCallback = std::function<void(uint64_t)>;
    using FlatpackStatusCallback = std::function<void(const FlatpackData&)>;
    using FlatpackCanSendCallback = std::function<bool(const struct can_frame&, uint8_t)>;
    
    FlatpackManager();
    ~FlatpackManager();
    
    /**
     * @brief Process an incoming CAN message for Flatpack communication
     * 
     * @param msg CAN frame to process
     * @param canName Name of CAN bus (e.g., "CAN1")
     */
    void processCanMessage(const struct can_frame& msg, const char* canName);
    
    /**
     * @brief Set callback for Flatpack detection events
     * 
     * @param callback Function to call when a new Flatpack is detected
     */
    void setDetectionCallback(FlatpackDetectionCallback callback);
    
    /**
     * @brief Set callback for Flatpack status updates
     * 
     * @param callback Function to call when status is received
     */
    void setStatusCallback(FlatpackStatusCallback callback);
    
    /**
     * @brief Set the callback function for sending CAN messages
     * 
     * This callback is used when the FlatpackManager needs to send CAN messages
     * (like login, commands, etc). The main application must implement this callback
     * to route messages to the appropriate CAN bus hardware.
     * 
     * @param callback Function to call to send CAN messages (returns success/failure)
     */
    void setCanSendCallback(FlatpackCanSendCallback callback);
    
    /**
     * @brief Periodic update function (call in loop)
     * Handles login timeouts and scheduled commands
     */
    void update();
    
    /**
     * @brief Build a login message for a specific Flatpack
     * 
     * @param serial Serial number of the Flatpack
     * @param loginMsg Reference to CAN frame where login message will be stored
     * @return true if successful
     */
    bool sendLoginMessage(uint64_t serial, struct can_frame& loginMsg);
    
    /**
     * @brief Set output parameters for a specific Flatpack or all PSUs
     * 
     * @param voltage Voltage setpoint in centivolts (V * 100)
     * @param current Current setpoint in deciamps (A * 10)
     * @param ovp Over-voltage protection setpoint in centivolts
     * @param serial Optional: specific PSU serial number, 0 for all PSUs
     * @return true if command was sent successfully
     */
    bool setOutputParameters(uint16_t voltage, uint16_t current, uint16_t ovp, uint64_t serial = 0);
    
    /**
     * @brief Set default output voltage for when PSU loses communication
     * 
     * @param voltage Default voltage in centivolts
     * @param serial Optional: specific PSU serial number, 0 for all PSUs
     * @return true if command was sent successfully
     */
    bool setDefaultVoltage(uint16_t voltage, uint64_t serial = 0);
    
    /**
     * @brief Get list of all detected Flatpacks
     * 
     * @return Vector of FlatpackData structures
     */
    const std::vector<FlatpackData>& getFlatpacks() const;
    
    /**
     * @brief Print status information for all PSUs to Serial
     */
    void printStatus() const;
    
    /**
     * @brief Get aggregated data from all active PSUs
     * 
     * @param totalCurrent Total output current in deciamps
     * @param avgVoltage Average output voltage in centivolts
     * @param activePsuCount Number of active PSUs
     * @return true if data was aggregated successfully
     */
    bool getAggregatedData(uint16_t& totalCurrent, uint16_t& avgVoltage, uint8_t& activePsuCount) const;

private:
    std::vector<FlatpackData> flatpacks;
    FlatpackDetectionCallback detectionCallback;
    FlatpackStatusCallback statusCallback;
    FlatpackCanSendCallback canSendCallback;
    
    // Thread safety
    SemaphoreHandle_t flatpacksMutex;
    
    // Track which buses have already detected a PSU (only one per bus)
    static const uint8_t MAX_CAN_BUSES = 4; // 1 TWAI + 3 MCP2515
    bool _psuDetectedOnBus[MAX_CAN_BUSES];
    uint64_t _psuSerialOnBus[MAX_CAN_BUSES]; // Track which PSU serial is on each bus
    
    // Constants for Flatpack2 protocol (match Flatpack2-main implementation)
    static constexpr uint32_t CAN_ID_HELLO = 0x05000000;
    static constexpr uint32_t CAN_ID_LOGIN = 0x05004804;
    static constexpr uint32_t CAN_ID_STATUS = 0x05000000;
    static constexpr uint32_t CAN_ID_SET_OUTPUT = 0x05FF4004;
    static constexpr uint32_t CAN_ID_SET_DEFAULT = 0x05009C00;
    static constexpr uint32_t CAN_ID_ALERT_REQUEST = 0x05000000;
    
    // Status types
    static constexpr uint8_t STATUS_NORMAL = 0x04;
    static constexpr uint8_t STATUS_CURRENT_LIMIT = 0x08;
    static constexpr uint8_t STATUS_ALARM = 0x0C;
    static constexpr uint8_t STATUS_WALK_IN = 0x10;
    
    // Helper functions
    void processHelloMessage(const struct can_frame& msg, uint8_t canBusId);
    void processStatusMessage(const struct can_frame& msg, uint8_t canBusId);
    uint8_t canNameToId(const char* canName) const;
    void serialToString(uint64_t serial, char* buffer) const;
    uint64_t bytesToSerial(const uint8_t* bytes) const;
    void bytesToHexString(const uint8_t* bytes, int len, char* buffer) const;
};

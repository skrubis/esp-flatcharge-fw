#include <Arduino.h>
#include "CANManager.h"
#include "HardwareManager.h"
#include "FlatpackManager.h"
#include "BatteryManager.h"

// Global configuration
#define SERIAL_BAUD 115200
#define STATUS_UPDATE_INTERVAL 15000   // Status display interval (15 sec)
#define LOGIN_REFRESH_INTERVAL 5000    // Login refresh interval (5 sec)
#define CHARGING_UPDATE_INTERVAL 1000  // Charging control update interval (1 sec)
#define VOLTAGE_SWEEP_INTERVAL 3000    // Voltage sweep step interval (3 sec)
#define BATTERY_CHEMISTRY BatteryChemistry::LFP  // Default battery chemistry
#define BATTERY_CELL_COUNT 16          // Default cell count (16S LFP = 51.2V nominal)
#define BATTERY_CAPACITY 100.0f        // Default capacity in Ah

// Feature flags - change these to enable/disable features
#define ENABLE_BATTERY_CHARGING false  // Set to false to disable battery charging logic
#define ENABLE_VOLTAGE_SWEEP true      // Set to true to enable voltage sweep mode

// Global managers
CANManager canManager;
HardwareManager hardwareManager;
FlatpackManager flatpackManager;
BatteryManager batteryManager;

// Timing variables
unsigned long lastStatusPrint = 0;
unsigned long lastLoginRefresh = 0;
unsigned long lastChargingUpdate = 0;
unsigned long lastVoltageSweepUpdate = 0;

// Voltage sweep parameters
#define VOLTAGE_SWEEP_MIN 4500   // 45.00V
#define VOLTAGE_SWEEP_MAX 5840   // 58.40V
#define VOLTAGE_SWEEP_STEP 100   // 1.00V
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

// Battery state tracking
float totalPackVoltage = 0.0f;
float totalPackCurrent = 0.0f;
int batteryTemperature = 25;  // Default room temperature
bool chargingEnabled = false;

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
        
        // Set default voltage parameters for the new PSU
        // Our canSendCallback will handle sending to the correct bus
        uint16_t voltage, current, ovp;
        batteryManager.getChargingSetpoints(voltage, current, ovp);
        
        if (flatpackManager.setOutputParameters(voltage, current, ovp, serial)) {
            Serial.printf("[FLATPACK] Set initial parameters for PSU %012llX\n", serial);
            // Parameters are sent via the canSendCallback
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
            totalPackVoltage = avgVoltage / 100.0f;
            totalPackCurrent = totalCurrent / 10.0f;
            
            // Update battery manager with latest measurements
            batteryManager.updateStatus(totalPackVoltage, totalPackCurrent, batteryTemperature);
        }
    }
}

/**
 * @brief CAN message processing callback
 * 
 * @param msg CAN frame received
 * @param canName String identifier of the CAN bus (e.g., "CAN1")
 */
void onCanMessage(const struct can_frame& msg, const char* canName) {
    // Determine CAN ID from name
    int canId = 1;
    if (strcmp(canName, "CAN2") == 0) canId = 2;
    else if (strcmp(canName, "CAN3") == 0) canId = 3;
    
    #ifdef DEBUG_CAN_MESSAGES
    // Print all CAN messages for debugging
    Serial.printf("[%s] ID:0x%08X DLC:%d Data:", canName, msg.can_id, msg.can_dlc);
    for (int i = 0; i < msg.can_dlc; i++) {
        Serial.printf(" %02X", msg.data[i]);
    }
    Serial.println();
    #endif
    
    // Process flatpack messages
    flatpackManager.processCanMessage(msg, canName);
}

/**
 * @brief Update charging parameters on all detected PSUs
 * 
 * Gets charging setpoints from battery manager and sends them to all PSUs
 */
void updateChargingParameters() {
    // Get charging parameters from battery manager
    uint16_t voltage, current, ovp;
    batteryManager.getChargingSetpoints(voltage, current, ovp);
    
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
    uint16_t current = 500;  // 50.0A
    uint16_t ovp = currentSweepVoltage + 100;  // OVP is 1V higher than setpoint
    
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
    Serial.begin(SERIAL_BAUD);
    Serial.println();
    Serial.println("=== Flatpack CAN Controller Starting ===");
    Serial.println("Hardware: ESP32-S3 + 3x MCP2515 CAN controllers");
    Serial.println("Protocol: Eltek Flatpack2 CAN @ 125kbit/s");
    Serial.println();
    
    // Initialize hardware (MCP23017 GPIO expander)
    if (!hardwareManager.initialize()) {
        Serial.println("FATAL: Hardware initialization failed!");
        while (1) { delay(1000); }
    }
    Serial.println("[INIT] Hardware manager initialized successfully");
    
    // Initialize CAN controllers
    if (!canManager.initialize()) {
        Serial.println("FATAL: CAN initialization failed!");
        while (1) { delay(1000); }
    }
    Serial.println("[INIT] CAN manager initialized successfully");
    
    // Initialize battery manager with default values
    if (!batteryManager.initialize(BATTERY_CHEMISTRY, BATTERY_CELL_COUNT, BATTERY_CAPACITY)) {
        Serial.println("FATAL: Battery manager initialization failed!");
        while (1) { delay(1000); }
    }
    Serial.println("[INIT] Battery manager initialized successfully");
    
    // Set up callbacks
    canManager.setMessageCallback(onCanMessage);
    flatpackManager.setDetectionCallback(onFlatpackDetected);
    flatpackManager.setStatusCallback(onFlatpackStatus);
    flatpackManager.setCanSendCallback(onCanSend);  // Register our new CAN send callback
    Serial.println("[INIT] Callback handlers registered");
    
    Serial.println("=== System Ready ===");
    Serial.println("Waiting for Flatpack detection...");
    Serial.println("Make sure flatpack has AC power and is connected to CAN1, CAN2 or CAN3");
    
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
}

/**
 * @brief Arduino main loop function
 */
void loop() {
    // Process incoming CAN messages
    canManager.processMessages();
    
    // Update managers
    flatpackManager.update();
    batteryManager.update();
    
    // Handle charging updates
    unsigned long currentTime = millis();
    
    // Refresh flatpack logins periodically
    if (currentTime - lastLoginRefresh > LOGIN_REFRESH_INTERVAL) {
        lastLoginRefresh = currentTime;

        // Re-login to all detected flatpacks to keep them logged in
        auto flatpacks = flatpackManager.getFlatpacks();
        for (const auto& fp : flatpacks) {
            if (fp.detected) {
                struct can_frame loginMsg;
                if (flatpackManager.sendLoginMessage(fp.serial, loginMsg)) {
                    #ifdef DEBUG_LOGIN_MESSAGES
                    Serial.printf("[FLATPACK] Login refresh for %012llX initiated\n", fp.serial);
                    #endif
                    // The canSendCallback will handle the actual sending of the login message
                    // on the appropriate CAN bus where the PSU is located
                }
            }
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
        
        // Check if no PSUs detected
        auto flatpacks = flatpackManager.getFlatpacks();
        if (flatpacks.empty()) {
            Serial.println("\nWaiting for flatpack detection...");
            Serial.println("Ensure flatpack has AC power and CAN connection.");
        }
        Serial.println();
    }
    
    // Small delay to prevent overwhelming the system
    delay(10);
}
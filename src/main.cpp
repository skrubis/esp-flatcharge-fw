#include <Arduino.h>
#include "FlatpackManager.h"
#include "BatteryManager.h"
#include "BatteryManager.h"
#include "FlatpackManager.h"
#include "CANManager.h"
#include "HardwareManager.h"
#include "WebServerManager.h"
#include "VectrixVX1Manager.h"
#include "CurrentSensorManager.h"
#include "charging_profiles.h"
#include <driver/twai.h>

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
#define VX1_BATTERY_CAPACITY 20.0f     // Typical VX1 capacity

// User-configurable battery settings
int userCellCount = 36;                // User-configurable cell count (default 36S for VX1)
float userTargetCellVoltage = 4.00f;   // User-configurable target voltage (default 4.00V for city riding)

// Current sensor configuration
CurrentSensorConfig currentSensorConfig = {
    .adcChannel = 0,           // ADC channel 0 (GPIO1 on ESP32-S3)
    .offsetVoltage = 1.65f,    // 3.3V/2 for bipolar hall sensor
    .scaleFactor = 20.0f,      // 20A/V (adjust based on your sensor)
    .transmitInterval = 100,   // 100ms = 10Hz transmission rate
    .enabled = true            // Enable current monitoring
};

// Charging profile presets
#define PROFILE_CITY_VOLTAGE 4.00f     // Conservative for daily city riding
#define PROFILE_TRAVEL_VOLTAGE 4.15f   // Higher capacity for long trips
#define PROFILE_MAX_VOLTAGE 4.20f      // Maximum (use with extreme caution)

// Feature flags - change these to enable/disable features
#define ENABLE_BATTERY_CHARGING false  // Set to false to disable battery charging logic
#define ENABLE_VOLTAGE_SWEEP true      // Set to true to enable voltage sweep mode
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
CurrentSensorManager currentSensorManager;

// TWAI configuration for VX1 mode (250kbps)
twai_general_config_t twai_g_config = TWAI_GENERAL_CONFIG_DEFAULT(GPIO_NUM_21, GPIO_NUM_22, TWAI_MODE_NORMAL);
twai_timing_config_t twai_t_config = TWAI_TIMING_CONFIG_250KBITS();
twai_filter_config_t twai_f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

// Current battery source configuration
int currentBatterySource = DEFAULT_BATTERY_SOURCE;

// Timing variables
unsigned long lastStatusPrint = 0;
unsigned long lastLoginRefresh = 0;
unsigned long lastChargingUpdate = 0;
unsigned long lastVoltageSweepUpdate = 0;
unsigned long lastWebServerUpdate = 0;

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
    if (currentBatterySource != BATTERY_SOURCE_VECTRIX_VX1) {
        return;
    }
    
    twai_message_t message;
    while (twai_receive(&message, 0) == ESP_OK) {
        // Forward VX1 messages to VX1 manager
        vx1Manager.processCanMessage(message.identifier, message.data, message.data_length_code);
        
        // Debug output
        Serial.printf("[TWAI] ID:0x%08lX DLC:%d Data:", message.identifier, message.data_length_code);
        for (uint8_t i = 0; i < message.data_length_code; ++i) {
            Serial.printf(" %02X", message.data[i]);
        }
        Serial.println();
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
    Serial.begin(SERIAL_BAUD);
    Serial.println();
    Serial.println("=== Flatpack CAN Controller Starting ===");
    Serial.println("Hardware: ESP32-S3 + 3x MCP2515 CAN controllers");
    Serial.println("Protocol: Eltek Flatpack2 CAN @ 125kbit/s");
    Serial.println();
    
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
    
    // Initialize TWAI for VX1 mode if selected
    if (currentBatterySource == BATTERY_SOURCE_VECTRIX_VX1) {
        Serial.println("[INIT] Configuring TWAI for VX1 mode (250kbps)");
        
        // Install TWAI driver
        if (twai_driver_install(&twai_g_config, &twai_t_config, &twai_f_config) == ESP_OK) {
            Serial.println("[INIT] TWAI driver installed successfully");
            
            // Start TWAI driver
            if (twai_start() == ESP_OK) {
                Serial.println("[INIT] TWAI started successfully at 250kbps for VX1");
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
            cellCount = 24;  // Typical LTO configuration
            capacity = 100.0f;
            source = BatterySource::CREE_LTO;
            Serial.println("[INIT] Configuring for Cree LTO battery");
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
    
    // Initialize current sensor manager
    if (!currentSensorManager.initialize(currentSensorConfig)) {
        Serial.println("WARNING: Current sensor initialization failed!");
    } else {
        Serial.println("[INIT] Current sensor manager initialized successfully");
        
        // Set up current sensor callback for real-time monitoring
        currentSensorManager.setDataCallback([](const CurrentSensorData& data) {
            // Optional: Log current readings or trigger alerts
            if (abs(data.calibratedCurrent) > 50.0f) {  // Alert for high current
                Serial.printf("[CurrentSensor] HIGH CURRENT: %.1fA\n", data.calibratedCurrent);
            }
        });
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
    currentSensorManager.update();
    
    // Update web server if enabled
    if (ENABLE_WEB_SERVER) {
        webServerManager.update();
    }
    
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
        
        // Print VX1 status if VX1 source is selected
        if (currentBatterySource == BATTERY_SOURCE_VECTRIX_VX1) {
            vx1Manager.printStatus();
        }
        
        // Print current sensor status
        currentSensorManager.printStatus();
        
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
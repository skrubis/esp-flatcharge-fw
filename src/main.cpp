#include <Arduino.h>
#include <WiFi.h>
#include <Wire.h>
#include <SPI.h>
#include <SPIFFS.h>

#include "config.h"
#include "gpio_expander.h"
#include "flatpack.h"
#include "type2.h"
#include "settings.h"
#include "webserver.h"
#include "ota.h"

// Global objects
GPIOExpander gpioExpander;
Settings settings;
Type2Controller type2Controller(CONTROL_PILOT_IN);
WebServerManager webServer;
OTAManager otaManager;

// Flatpack controllers
FlatpackController flatpack1(SPI_CAN1_CS, GPIOEXP_SPI_CAN1_POWER, GPIOEXP_SPI_CAN1_INT, GPIOEXP_SPI_CAN1_RST);
FlatpackController flatpack2(SPI_CAN2_CS, GPIOEXP_SPI_CAN2_POWER, GPIOEXP_SPI_CAN2_INT, GPIOEXP_SPI_CAN2_RST);
FlatpackController flatpack3(SPI_CAN3_CS, GPIOEXP_SPI_CAN3_POWER, GPIOEXP_SPI_CAN3_INT, GPIOEXP_SPI_CAN3_RST);

// Array of Flatpack controllers for easier management
FlatpackController* flatpackControllers[3] = {&flatpack1, &flatpack2, &flatpack3};

// Status LED
bool ledState = false;
unsigned long lastLedToggle = 0;
unsigned long ledInterval = 1000;  // 1 second heartbeat

// Function declarations
bool initializeHardware();
bool initializeFilesystem();
bool setupWiFi();
void updateHeartbeat();
void updateCharging();
bool scanI2C();

// Setup routine
void setup() {
    // Start serial
    Serial.begin(115200);
    Serial.println("\nFlatCharge v0.1 initializing...");
    
    // Initialize hardware
    if (!initializeHardware()) {
        Serial.println("Hardware initialization failed");
        return;
    }
    
    // Initialize filesystem
    if (!initializeFilesystem()) {
        Serial.println("Filesystem initialization failed");
        return;
    }
    
    // Setup WiFi
    if (!setupWiFi()) {
        Serial.println("WiFi setup failed");
        return;
    }
    
    // Initialize settings from SPIFFS
    if (!settings.begin()) {
        Serial.println("Settings initialization failed");
        return;
    }
    
    // Set pointers to controllers for the web server
    webServer.setSettingsManager(&settings);
    webServer.setFlatpackControllers(flatpackControllers, 3);
    webServer.setType2Controller(&type2Controller);
    
    // Initialize web server
    if (!webServer.begin()) {
        Serial.println("Web server initialization failed");
        return;
    }
    
    Serial.println("Initialization complete");
}

void loop() {
    // Update heartbeat LED
    updateHeartbeat();
    
    // Update modules
    type2Controller.update();
    flatpack1.update();
    flatpack2.update();
    flatpack3.update();
    webServer.update();
    
    // Update charging logic
    updateCharging();
    
    // Small delay to prevent watchdog resets
    delay(1);
}

bool initializeHardware() {
    Serial.println("Initializing hardware...");
    
    // Setup GPIO for LED
    pinMode(HEARTBEAT_LED, OUTPUT);
    digitalWrite(HEARTBEAT_LED, HIGH);  // Start with LED on
    
    // Setup I2C
    Wire.begin(I2C_SDA, I2C_SCL);
    
    // Scan I2C bus
    if (!scanI2C()) {
        Serial.println("I2C scan failed to find expected devices");
        return false;
    }
    
    // Initialize GPIO expander
    if (!gpioExpander.begin()) {
        Serial.println("GPIO expander initialization failed");
        return false;
    }
    
    // Initialize SPI
    SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);
    
    // Initialize Flatpacks by first enabling power through GPIO expander
    gpioExpander.enableSPICAN(1, true);
    if (!flatpack1.begin()) {
        Serial.println("Flatpack 1 initialization failed");
        // Continue anyway, not critical
    }
    
    gpioExpander.enableSPICAN(2, true);
    if (!flatpack2.begin()) {
        Serial.println("Flatpack 2 initialization failed");
        // Continue anyway, not critical
    }
    
    gpioExpander.enableSPICAN(3, true);
    if (!flatpack3.begin()) {
        Serial.println("Flatpack 3 initialization failed");
        // Continue anyway, not critical
    }
    
    // Initialize Type2 controller
    if (!type2Controller.begin()) {
        Serial.println("Type2 controller initialization failed");
        return false;
    }
    
    return true;
}

bool initializeFilesystem() {
    Serial.println("Initializing filesystem...");
    
    if (!SPIFFS.begin(true)) {  // Format on failure
        Serial.println("SPIFFS mount failed");
        return false;
    }
    
    Serial.println("SPIFFS mounted successfully");
    return true;
}

bool setupWiFi() {
    Serial.println("Setting up WiFi Access Point...");
    
    WiFi.mode(WIFI_AP);
    bool result = WiFi.softAP(WIFI_SSID, WIFI_PASSWORD, WIFI_CHANNEL, 0, WIFI_MAX_CONNECTIONS);
    
    if (result) {
        IPAddress IP = WiFi.softAPIP();
        Serial.print("AP IP address: ");
        Serial.println(IP);
        return true;
    } else {
        Serial.println("AP setup failed");
        return false;
    }
}

void updateHeartbeat() {
    unsigned long currentMillis = millis();
    
    // Toggle LED based on interval
    if (currentMillis - lastLedToggle >= ledInterval) {
        lastLedToggle = currentMillis;
        ledState = !ledState;
        digitalWrite(HEARTBEAT_LED, ledState);
    }
}

void updateCharging() {
    // Get current settings
    ChargingSettings &config = settings.getSettings();
    
    // Check if charging is enabled
    if (!config.chargingEnabled) {
        // If charging is disabled, set all flatpacks to 0 current
        flatpack1.setVoltageAndCurrent(FP_MAX_VOLTAGE, 0);  // Use max voltage as default when idle
        flatpack2.setVoltageAndCurrent(FP_MAX_VOLTAGE, 0);
        flatpack3.setVoltageAndCurrent(FP_MAX_VOLTAGE, 0);
        return;
    }
    
    // Get the maximum allowed current from Type2
    float evseAllowedCurrent = type2Controller.getMaxAllowedCurrent(config.useThreePhase);
    
    // Calculate per-flatpack current based on configuration
    float targetVoltage = config.targetVoltage;
    float targetCurrent = config.targetCurrent;
    
    // Limit current based on EVSE
    if (targetCurrent > evseAllowedCurrent) {
        targetCurrent = evseAllowedCurrent;
    }
    
    if (config.bmsMode == BMSMode::MANUAL) {
        // For manual mode, calculate target voltage from cell count if needed
        if (config.cellCount > 0) {
            targetVoltage = config.cellCount * config.cellTargetVoltage;
            // Clamp voltage to valid range
            if (targetVoltage < FP_MIN_VOLTAGE) targetVoltage = FP_MIN_VOLTAGE;
            if (targetVoltage > FP_MAX_VOLTAGE) targetVoltage = FP_MAX_VOLTAGE;
        }
        
        if (config.flatpackConfig == FlatpackConfig::SERIES) {
            // In series, voltage is divided across flatpacks
            float fpVoltage = targetVoltage / 3.0;
            
            // Current is the same through all flatpacks
            flatpack1.setVoltageAndCurrent(fpVoltage, targetCurrent);
            flatpack2.setVoltageAndCurrent(fpVoltage, targetCurrent);
            flatpack3.setVoltageAndCurrent(fpVoltage, targetCurrent);
        }
        else if (config.flatpackConfig == FlatpackConfig::PARALLEL) {
            // In parallel, voltage is the same for all flatpacks
            // Current is divided among flatpacks
            float fpCurrent = targetCurrent / 3.0;
            
            flatpack1.setVoltageAndCurrent(targetVoltage, fpCurrent);
            flatpack2.setVoltageAndCurrent(targetVoltage, fpCurrent);
            flatpack3.setVoltageAndCurrent(targetVoltage, fpCurrent);
        }
    }
    // BMS modes to be implemented later
}

bool scanI2C() {
    Serial.println("Scanning I2C bus...");
    bool foundMCP23017 = false;
    
    for (byte address = 1; address < 127; address++) {
        Wire.beginTransmission(address);
        byte error = Wire.endTransmission();
        
        if (error == 0) {
            Serial.print("I2C device found at address 0x");
            if (address < 16) {
                Serial.print("0");
            }
            Serial.print(address, HEX);
            Serial.println();
            
            if (address == MCP23017_ADDRESS) {
                foundMCP23017 = true;
                Serial.println("MCP23017 GPIO expander found!");
            }
        }
    }
    
    return foundMCP23017;
}

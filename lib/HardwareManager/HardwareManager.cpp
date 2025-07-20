#include "HardwareManager.h"

HardwareManager::HardwareManager() : mcpInitialized(false) {
    // Constructor
}

HardwareManager::~HardwareManager() {
    // Destructor
}

bool HardwareManager::initialize() {
    Serial.println("[HardwareManager] Initializing hardware...");
    
    // Set up the interrupt pin for MCP23017
    pinMode(PIN_MCP_INTERRUPT, INPUT_PULLUP);
    
    // Initialize I2C
    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
    
    // Initialize MCP23017
    if (!mcp.begin_I2C(MCP_ADDR, &Wire)) {
        Serial.println("[HardwareManager] Failed to initialize MCP23017 GPIO expander!");
        return false;
    }
    mcpInitialized = true;
    
    // Configure pins for LED output
    mcp.pinMode(PIN_EVSE_IDLE, OUTPUT);
    mcp.pinMode(PIN_EVSE_CHARGE_NO_VENT, OUTPUT);
    mcp.pinMode(PIN_EVSE_CHARGE_VENT, OUTPUT);
    mcp.pinMode(PIN_EVSE_FAULT, OUTPUT);
    
    // Configure pins for CAN power control
    mcp.pinMode(PIN_ENABLE_TWAI, OUTPUT);
    mcp.pinMode(PIN_ENABLE_CAN1, OUTPUT);
    mcp.pinMode(PIN_ENABLE_CAN2, OUTPUT);
    mcp.pinMode(PIN_ENABLE_CAN3, OUTPUT);
    
    // Configure pins for CAN reset
    mcp.pinMode(PIN_CAN1_RST, OUTPUT);
    mcp.pinMode(PIN_CAN2_RST, OUTPUT);
    mcp.pinMode(PIN_CAN3_RST, OUTPUT);
    
    // Initialize all pins to OFF state
    mcp.digitalWrite(PIN_EVSE_IDLE, HIGH);        // IDLE mode active
    mcp.digitalWrite(PIN_EVSE_CHARGE_NO_VENT, LOW);
    mcp.digitalWrite(PIN_EVSE_CHARGE_VENT, LOW);
    mcp.digitalWrite(PIN_EVSE_FAULT, LOW);
    
    mcp.digitalWrite(PIN_ENABLE_TWAI, LOW);      // All CAN buses OFF
    mcp.digitalWrite(PIN_ENABLE_CAN1, LOW);
    mcp.digitalWrite(PIN_ENABLE_CAN2, LOW);
    mcp.digitalWrite(PIN_ENABLE_CAN3, LOW);
    
    mcp.digitalWrite(PIN_CAN1_RST, HIGH);        // Reset pins inactive
    mcp.digitalWrite(PIN_CAN2_RST, HIGH);
    mcp.digitalWrite(PIN_CAN3_RST, HIGH);
    
    Serial.println("[HardwareManager] Hardware initialization complete");
    
    // Enable all CAN buses by default
    enableCanBusPower(0, true); // TWAI
    enableCanBusPower(1, true); // CAN1
    enableCanBusPower(2, true); // CAN2
    enableCanBusPower(3, true); // CAN3
    
    return true;
}

bool HardwareManager::enableCanBusPower(uint8_t busId, bool enable) {
    if (!mcpInitialized) return false;
    
    uint8_t pin;
    switch (busId) {
        case 0: pin = PIN_ENABLE_TWAI; break; // TWAI CAN
        case 1: pin = PIN_ENABLE_CAN1; break;
        case 2: pin = PIN_ENABLE_CAN2; break;
        case 3: pin = PIN_ENABLE_CAN3; break;
        default: return false;
    }
    
    mcp.digitalWrite(pin, enable ? HIGH : LOW);
    Serial.printf("[HardwareManager] CAN bus %d power set to %s\n", 
                  busId, enable ? "ON" : "OFF");
    
    return true;
}

bool HardwareManager::resetCanController(uint8_t busId) {
    if (!mcpInitialized) return false;
    
    uint8_t pin;
    switch (busId) {
        case 1: pin = PIN_CAN1_RST; break;
        case 2: pin = PIN_CAN2_RST; break;
        case 3: pin = PIN_CAN3_RST; break;
        default: return false;
    }
    
    // Perform reset sequence (active low)
    mcp.digitalWrite(pin, LOW);
    delay(10);
    mcp.digitalWrite(pin, HIGH);
    delay(50); // Allow time for controller to initialize
    
    Serial.printf("[HardwareManager] Reset CAN controller %d\n", busId);
    
    return true;
}

bool HardwareManager::setEvseMode(uint8_t mode) {
    if (!mcpInitialized) return false;
    
    // Turn off all mode LEDs first
    mcp.digitalWrite(PIN_EVSE_IDLE, LOW);
    mcp.digitalWrite(PIN_EVSE_CHARGE_NO_VENT, LOW);
    mcp.digitalWrite(PIN_EVSE_CHARGE_VENT, LOW);
    mcp.digitalWrite(PIN_EVSE_FAULT, LOW);
    
    // Turn on appropriate LED based on mode
    switch (mode) {
        case 0: // IDLE
            mcp.digitalWrite(PIN_EVSE_IDLE, HIGH);
            break;
        case 1: // CHARGE_NO_VENT
            mcp.digitalWrite(PIN_EVSE_CHARGE_NO_VENT, HIGH);
            break;
        case 2: // CHARGE_VENT_REQ
            mcp.digitalWrite(PIN_EVSE_CHARGE_VENT, HIGH);
            break;
        case 3: // FAULT
            mcp.digitalWrite(PIN_EVSE_FAULT, HIGH);
            break;
        default:
            return false;
    }
    
    return true;
}

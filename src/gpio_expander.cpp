#include "gpio_expander.h"

GPIOExpander::GPIOExpander() {}

bool GPIOExpander::begin() {
    // Initialize the MCP23017
    if (!_mcp.begin_I2C(MCP23017_ADDRESS)) {
        Serial.println("Error initializing MCP23017");
        return false;
    }
    
    // Configure pins for I/O direction
    // Port A pins (0-7): handle SPI CAN INT and RST pins - all as outputs except INT pins as inputs
    _mcp.pinMode(GPIOEXP_SPI_CAN1_RST, OUTPUT);
    _mcp.pinMode(GPIOEXP_SPI_CAN1_INT, INPUT);
    _mcp.pinMode(GPIOEXP_SPI_CAN2_RST, OUTPUT);
    _mcp.pinMode(GPIOEXP_SPI_CAN2_INT, INPUT);
    _mcp.pinMode(GPIOEXP_SPI_CAN3_RST, OUTPUT);
    _mcp.pinMode(GPIOEXP_SPI_CAN3_INT, INPUT);
    
    // Port B pins (8-15): all outputs for power control and EVSE mode indicators
    _mcp.pinMode(GPIOEXP_EVSE_MODE_IDLE + 8, OUTPUT);
    _mcp.pinMode(GPIOEXP_EVSE_MODE_CHARGE + 8, OUTPUT);
    _mcp.pinMode(GPIOEXP_EVSE_MODE_CHARGE_VENT + 8, OUTPUT);
    _mcp.pinMode(GPIOEXP_EVSE_MODE_FAULT + 8, OUTPUT);
    _mcp.pinMode(GPIOEXP_TWAI_CAN_POWER + 8, OUTPUT);
    _mcp.pinMode(GPIOEXP_SPI_CAN3_POWER + 8, OUTPUT);
    _mcp.pinMode(GPIOEXP_SPI_CAN2_POWER + 8, OUTPUT);
    _mcp.pinMode(GPIOEXP_SPI_CAN1_POWER + 8, OUTPUT);
    
    // Start with all power off
    _mcp.digitalWrite(GPIOEXP_SPI_CAN1_POWER + 8, LOW);
    _mcp.digitalWrite(GPIOEXP_SPI_CAN2_POWER + 8, LOW);
    _mcp.digitalWrite(GPIOEXP_SPI_CAN3_POWER + 8, LOW);
    _mcp.digitalWrite(GPIOEXP_TWAI_CAN_POWER + 8, LOW);
    
    // Reset all CAN controllers
    _mcp.digitalWrite(GPIOEXP_SPI_CAN1_RST, LOW);
    _mcp.digitalWrite(GPIOEXP_SPI_CAN2_RST, LOW);
    _mcp.digitalWrite(GPIOEXP_SPI_CAN3_RST, LOW);
    
    delay(10);  // Brief delay for reset to take effect
    
    _mcp.digitalWrite(GPIOEXP_SPI_CAN1_RST, HIGH);
    _mcp.digitalWrite(GPIOEXP_SPI_CAN2_RST, HIGH);
    _mcp.digitalWrite(GPIOEXP_SPI_CAN3_RST, HIGH);
    
    // Initialize all EVSE mode indicators as off
    _mcp.digitalWrite(GPIOEXP_EVSE_MODE_IDLE + 8, LOW);
    _mcp.digitalWrite(GPIOEXP_EVSE_MODE_CHARGE + 8, LOW);
    _mcp.digitalWrite(GPIOEXP_EVSE_MODE_CHARGE_VENT + 8, LOW);
    _mcp.digitalWrite(GPIOEXP_EVSE_MODE_FAULT + 8, LOW);
    
    _initialized = true;
    return true;
}

bool GPIOExpander::enableSPICAN(uint8_t canBus, bool enable) {
    if (!_initialized) return false;
    
    uint8_t powerPin = _mapCANBusToPowerPin(canBus) + 8;  // Port B pins start at 8
    _mcp.digitalWrite(powerPin, enable ? HIGH : LOW);
    
    return true;
}

bool GPIOExpander::enableTWAICAN(bool enable) {
    if (!_initialized) return false;
    
    _mcp.digitalWrite(GPIOEXP_TWAI_CAN_POWER + 8, enable ? HIGH : LOW);
    
    return true;
}

void GPIOExpander::setEVSEModeIndicator(uint8_t mode) {
    if (!_initialized) return;
    
    // Turn off all indicators first
    _mcp.digitalWrite(GPIOEXP_EVSE_MODE_IDLE + 8, LOW);
    _mcp.digitalWrite(GPIOEXP_EVSE_MODE_CHARGE + 8, LOW);
    _mcp.digitalWrite(GPIOEXP_EVSE_MODE_CHARGE_VENT + 8, LOW);
    _mcp.digitalWrite(GPIOEXP_EVSE_MODE_FAULT + 8, LOW);
    
    // Set the requested mode
    switch (mode) {
        case 0:  // IDLE
            _mcp.digitalWrite(GPIOEXP_EVSE_MODE_IDLE + 8, HIGH);
            break;
        case 1:  // CHARGING
            _mcp.digitalWrite(GPIOEXP_EVSE_MODE_CHARGE + 8, HIGH);
            break;
        case 2:  // CHARGING with VENT
            _mcp.digitalWrite(GPIOEXP_EVSE_MODE_CHARGE_VENT + 8, HIGH);
            break;
        case 3:  // FAULT
            _mcp.digitalWrite(GPIOEXP_EVSE_MODE_FAULT + 8, HIGH);
            break;
    }
}

bool GPIOExpander::readSPICANInt(uint8_t canBus) {
    if (!_initialized) return false;
    
    uint8_t intPin = _mapCANBusToIntPin(canBus);
    return _mcp.digitalRead(intPin) == LOW;  // INT pins are active LOW
}

void GPIOExpander::resetSPICAN(uint8_t canBus) {
    if (!_initialized) return;
    
    uint8_t rstPin = _mapCANBusToRstPin(canBus);
    _mcp.digitalWrite(rstPin, LOW);
    delay(10);  // Brief delay for reset
    _mcp.digitalWrite(rstPin, HIGH);
}

bool GPIOExpander::scanForDevices() {
    byte error, address;
    bool foundExpander = false;
    
    for (address = 1; address < 127; address++) {
        Wire.beginTransmission(address);
        error = Wire.endTransmission();
        
        if (error == 0) {
            if (address == MCP23017_ADDRESS) {
                foundExpander = true;
                Serial.println("MCP23017 GPIO expander found!");
            }
        }
    }
    
    return foundExpander;
}

uint8_t GPIOExpander::_mapCANBusToPowerPin(uint8_t canBus) {
    switch (canBus) {
        case 1: return GPIOEXP_SPI_CAN1_POWER;
        case 2: return GPIOEXP_SPI_CAN2_POWER;
        case 3: return GPIOEXP_SPI_CAN3_POWER;
        default: return GPIOEXP_SPI_CAN1_POWER;  // Default to CAN1
    }
}

uint8_t GPIOExpander::_mapCANBusToIntPin(uint8_t canBus) {
    switch (canBus) {
        case 1: return GPIOEXP_SPI_CAN1_INT;
        case 2: return GPIOEXP_SPI_CAN2_INT;
        case 3: return GPIOEXP_SPI_CAN3_INT;
        default: return GPIOEXP_SPI_CAN1_INT;  // Default to CAN1
    }
}

uint8_t GPIOExpander::_mapCANBusToRstPin(uint8_t canBus) {
    switch (canBus) {
        case 1: return GPIOEXP_SPI_CAN1_RST;
        case 2: return GPIOEXP_SPI_CAN2_RST;
        case 3: return GPIOEXP_SPI_CAN3_RST;
        default: return GPIOEXP_SPI_CAN1_RST;  // Default to CAN1
    }
}

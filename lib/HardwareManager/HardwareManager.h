#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MCP23X17.h>

/**
 * @brief Hardware Manager for ESP32-S3 Flatpack Controller
 * 
 * This class manages the hardware interfaces including:
 * - MCP23017 GPIO Expander (ADDRESS 0x26)
 * - CAN Bus power control
 * - Reset signal handling
 */
class HardwareManager {
public:
    HardwareManager();
    ~HardwareManager();
    
    /**
     * @brief Initialize the hardware manager
     * 
     * @return true if initialization successful
     * @return false if any component failed to initialize
     */
    bool initialize();
    
    /**
     * @brief Enable power to specified CAN bus
     * 
     * @param busId Bus ID (0=TWAI CAN, 1-3=SPI CAN1-3)
     * @param enable True to enable, false to disable
     * @return true if successful
     */
    bool enableCanBusPower(uint8_t busId, bool enable);
    
    /**
     * @brief Reset a specific CAN controller
     * 
     * @param busId Bus ID (1-3=SPI CAN1-3)
     * @return true if successful
     */
    bool resetCanController(uint8_t busId);
    
    /**
     * @brief Set EVSE mode indicator LEDs
     * 
     * @param mode 0=IDLE, 1=CHARGE_NO_VENT, 2=CHARGE_VENT_REQ, 3=FAULT
     * @return true if successful
     */
    bool setEvseMode(uint8_t mode);
    
private:
    Adafruit_MCP23X17 mcp;
    bool mcpInitialized;
    
    // MCP23017 pin definitions (from pins.md)
    static constexpr uint8_t MCP_ADDR = 0x26;
    
    // GPB pins
    static constexpr uint8_t PIN_EVSE_IDLE = 8;       // GPB0
    static constexpr uint8_t PIN_EVSE_CHARGE_NO_VENT = 9;  // GPB1
    static constexpr uint8_t PIN_EVSE_CHARGE_VENT = 10;    // GPB2
    static constexpr uint8_t PIN_EVSE_FAULT = 11;     // GPB3
    static constexpr uint8_t PIN_ENABLE_TWAI = 12;    // GPB4
    static constexpr uint8_t PIN_ENABLE_CAN3 = 13;    // GPB5
    static constexpr uint8_t PIN_ENABLE_CAN2 = 14;    // GPB6
    static constexpr uint8_t PIN_ENABLE_CAN1 = 15;    // GPB7
    
    // GPA pins
    static constexpr uint8_t PIN_CAN2_RST = 0;        // GPA0
    static constexpr uint8_t PIN_CAN3_RST = 3;        // GPA3
    static constexpr uint8_t PIN_CAN1_RST = 5;        // GPA5
    
    // ESP32-S3 pins
    static constexpr uint8_t PIN_MCP_INTERRUPT = 1;
    static constexpr uint8_t PIN_I2C_SCL = 9;
    static constexpr uint8_t PIN_I2C_SDA = 8;
};

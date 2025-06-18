#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MCP23X17.h>
#include "config.h"

class GPIOExpander {
public:
    GPIOExpander();
    
    /**
     * Initialize the MCP23017 GPIO expander
     */
    bool begin();
    
    /**
     * Enable/disable SPI CAN bus power
     * @param canBus 1-3 to select which CAN bus
     * @param enable true to enable, false to disable
     */
    bool enableSPICAN(uint8_t canBus, bool enable);
    
    /**
     * Enable/disable TWAI CAN bus power
     */
    bool enableTWAICAN(bool enable);
    
    /**
     * Set the EVSE mode indicator LEDs
     */
    void setEVSEModeIndicator(uint8_t mode);
    
    /**
     * Read INT pin from a specific SPI CAN
     * @param canBus 1-3 to select which CAN bus
     */
    bool readSPICANInt(uint8_t canBus);
    
    /**
     * Toggle reset pin for a specific SPI CAN
     * @param canBus 1-3 to select which CAN bus
     */
    void resetSPICAN(uint8_t canBus);
    
    /**
     * Scan I2C bus for devices and return true if MCP23017 is found
     */
    bool scanForDevices();
    
private:
    Adafruit_MCP23X17 _mcp;
    bool _initialized = false;
    
    /**
     * Map CAN bus number to corresponding power pin on MCP23017
     */
    uint8_t _mapCANBusToPowerPin(uint8_t canBus);
    
    /**
     * Map CAN bus number to corresponding INT pin on MCP23017
     */
    uint8_t _mapCANBusToIntPin(uint8_t canBus);
    
    /**
     * Map CAN bus number to corresponding RST pin on MCP23017
     */
    uint8_t _mapCANBusToRstPin(uint8_t canBus);
};

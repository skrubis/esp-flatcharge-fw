#pragma once

#include <Arduino.h>
#include <mcp_can.h>
#include <SPI.h>
#include "config.h"

// Flatpack CAN IDs
#define FP_LOGIN_ID 0x05004804
#define FP_REQUEST_ID 0x05FF4004
#define FP_DEFAULT_VOLTAGE_ID 0x05009C00
#define FP_STATUS_ID_1 0x05014004  // normal voltage reached
#define FP_STATUS_ID_2 0x05014008  // current-limiting active
#define FP_STATUS_ID_3 0x0501400C  // walkin and below 43 volts (error) and during walk-out
#define FP_STATUS_ID_4 0x05014010  // walkin busy
#define FP_SERIAL_ID 0x05014400    // serial number broadcast

// Operation modes
enum class FlatpackMode {
    IDLE,
    SINGLE_PHASE,
    THREE_PHASE,
    AUTO
};

// Structure to hold Flatpack status
struct FlatpackStatus {
    bool connected = false;
    bool loggedIn = false;
    uint8_t serialNumber[8] = {0};
    float currentOutput = 0;     // Amperes
    float voltageOutput = 0;     // Volts
    float temperature1 = 0;      // Celsius
    float temperature2 = 0;      // Celsius
    uint8_t inputVoltage = 0;    // Volts
    uint32_t lastMessageTime = 0;
    bool isCurrentLimiting = false;
    bool isWalkingIn = false;
    bool isError = false;
};

class FlatpackController {
public:
    FlatpackController(uint8_t cs_pin, uint8_t power_pin, uint8_t int_pin, uint8_t rst_pin);
    
    // Initialization
    bool begin();
    bool enablePower(bool enable);
    
    // Discovery and connection
    bool checkForMessages();
    bool hasSerialNumber();
    
    // Communication with Flatpack
    bool login();
    bool setVoltageAndCurrent(float voltage, float current);
    bool setDefaultVoltage(float voltage);
    
    // Status
    FlatpackStatus getStatus();
    
    // Periodic tasks
    void update();
    
private:
    MCP_CAN _canBus;
    uint8_t _cs_pin;
    uint8_t _power_pin;
    uint8_t _int_pin;
    uint8_t _rst_pin;
    bool _initialized = false;
    FlatpackStatus _status;
    
    // Internal methods
    void processStatusMessage(long id, uint8_t len, uint8_t* data);
    void processSerialMessage(uint8_t len, uint8_t* data);
    void convertToHex(float value, uint16_t multiplier, uint8_t* lowByte, uint8_t* highByte);
};

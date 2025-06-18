#include "flatpack.h"

FlatpackController::FlatpackController(uint8_t cs_pin, uint8_t power_pin, uint8_t int_pin, uint8_t rst_pin) 
    : _canBus(cs_pin), _cs_pin(cs_pin), _power_pin(power_pin), _int_pin(int_pin), _rst_pin(rst_pin) {
}

bool FlatpackController::begin() {
    // Initialize CAN bus at 125 kbps
    if (_canBus.begin(MCP_ANY, CAN_SPEED, MCP_16MHZ) != CAN_OK) {
        Serial.print("Error initializing MCP2515 CAN bus for CS pin ");
        Serial.println(_cs_pin);
        return false;
    }
    
    // Set to normal mode
    _canBus.setMode(MCP_NORMAL);
    
    Serial.print("MCP2515 CAN bus initialized on CS pin ");
    Serial.println(_cs_pin);
    
    _initialized = true;
    return true;
}

bool FlatpackController::enablePower(bool enable) {
    // This would use the GPIO expander, but we leave that to the main application
    // which has access to the GPIO expander object
    return true;
}

bool FlatpackController::checkForMessages() {
    if (!_initialized) return false;
    
    // Check if there are any messages on the CAN bus
    byte len = 0;
    byte buf[8];
    unsigned long canId = 0;
    
    if (_canBus.checkReceive() == CAN_MSGAVAIL) {
        // Read data
        if (_canBus.readMsgBuf(&canId, &len, buf) == CAN_OK) {
            // Process the message
            if (canId == FP_SERIAL_ID) {
                processSerialMessage(len, buf);
                return true;
            } else if ((canId & 0xFFFFFFF0) == 0x05014000) {
                // Status message
                processStatusMessage(canId, len, buf);
                return true;
            }
        }
    }
    
    return false;
}

bool FlatpackController::hasSerialNumber() {
    // Check if we have received a serial number from the Flatpack
    bool hasSerial = false;
    for (int i = 0; i < 6; i++) {
        if (_status.serialNumber[i] != 0) {
            hasSerial = true;
            break;
        }
    }
    return hasSerial;
}

bool FlatpackController::login() {
    if (!_initialized || !hasSerialNumber()) return false;
    
    // Create login message with serial number
    byte loginMsg[8];
    memcpy(loginMsg, _status.serialNumber, 6);
    loginMsg[6] = 0;
    loginMsg[7] = 0;
    
    // Send login message
    if (_canBus.sendMsgBuf(FP_LOGIN_ID, 1, 8, loginMsg) == CAN_OK) {
        _status.loggedIn = true;
        Serial.println("Flatpack login sent successfully");
        return true;
    }
    
    Serial.println("Failed to send Flatpack login message");
    return false;
}

bool FlatpackController::setVoltageAndCurrent(float voltage, float current) {
    if (!_initialized || !_status.loggedIn) return false;
    
    // Create message for setting voltage and current
    byte setMsg[8];
    
    // Convert current from Amperes to deci-Amperes and to hex
    uint16_t currentValue = static_cast<uint16_t>(current * 10.0);
    convertToHex(current, 10, &setMsg[0], &setMsg[1]);
    
    // Convert voltage from Volts to centi-Volts and to hex
    uint16_t voltageValue = static_cast<uint16_t>(voltage * 100.0);
    convertToHex(voltage, 100, &setMsg[2], &setMsg[3]);
    
    // Set the same voltage for both values
    setMsg[4] = setMsg[2];
    setMsg[5] = setMsg[3];
    
    // Set over-voltage protection to 2V higher
    float ovp = voltage + 2.0;
    if (ovp > FP_MAX_VOLTAGE) ovp = FP_MAX_VOLTAGE;
    convertToHex(ovp, 100, &setMsg[6], &setMsg[7]);
    
    // Send the message
    if (_canBus.sendMsgBuf(FP_REQUEST_ID, 1, 8, setMsg) == CAN_OK) {
        Serial.print("Set Flatpack to ");
        Serial.print(voltage);
        Serial.print("V, ");
        Serial.print(current);
        Serial.println("A");
        return true;
    }
    
    Serial.println("Failed to send Flatpack settings");
    return false;
}

bool FlatpackController::setDefaultVoltage(float voltage) {
    if (!_initialized || !_status.loggedIn) return false;
    
    // Create message for setting default voltage
    byte setMsg[5];
    setMsg[0] = 0x29;
    setMsg[1] = 0x15;
    setMsg[2] = 0x00;
    
    // Convert voltage from Volts to centi-Volts and to hex
    convertToHex(voltage, 100, &setMsg[3], &setMsg[4]);
    
    // Send the message
    if (_canBus.sendMsgBuf(FP_DEFAULT_VOLTAGE_ID, 1, 5, setMsg) == CAN_OK) {
        Serial.print("Set Flatpack default voltage to ");
        Serial.print(voltage);
        Serial.println("V");
        return true;
    }
    
    Serial.println("Failed to send Flatpack default voltage");
    return false;
}

FlatpackStatus FlatpackController::getStatus() {
    return _status;
}

void FlatpackController::update() {
    if (!_initialized) return;
    
    // Check for new messages
    checkForMessages();
    
    // If we have a serial number but haven't logged in, attempt to login
    if (hasSerialNumber() && !_status.loggedIn) {
        login();
    }
    
    // Check if connection is still active based on message timestamp
    unsigned long currentTime = millis();
    if (currentTime - _status.lastMessageTime > 10000) {
        // No messages for 10 seconds, consider disconnected
        _status.connected = false;
    }
}

void FlatpackController::processStatusMessage(long id, uint8_t len, uint8_t* data) {
    if (len < 8) return;
    
    // Update connection status
    _status.connected = true;
    _status.lastMessageTime = millis();
    
    // Process data based on message type
    switch (id) {
        case FP_STATUS_ID_1:  // normal voltage reached
            _status.isCurrentLimiting = false;
            _status.isWalkingIn = false;
            _status.isError = false;
            break;
        
        case FP_STATUS_ID_2:  // current-limiting active
            _status.isCurrentLimiting = true;
            _status.isWalkingIn = false;
            _status.isError = false;
            break;
            
        case FP_STATUS_ID_3:  // walkin/error/walkout
            _status.isCurrentLimiting = false;
            _status.isWalkingIn = false;
            _status.isError = true;
            break;
            
        case FP_STATUS_ID_4:  // walkin busy
            _status.isCurrentLimiting = false;
            _status.isWalkingIn = true;
            _status.isError = false;
            break;
    }
    
    // Parse temperature 1
    _status.temperature1 = data[0];
    
    // Parse current output (dA)
    uint16_t currentOutput = (static_cast<uint16_t>(data[2]) << 8) | data[1];
    _status.currentOutput = currentOutput / 10.0;  // Convert to Amperes
    
    // Parse voltage output (cV)
    uint16_t voltageOutput = (static_cast<uint16_t>(data[4]) << 8) | data[3];
    _status.voltageOutput = voltageOutput / 100.0;  // Convert to Volts
    
    // Parse input voltage
    _status.inputVoltage = data[5];
    
    // Parse temperature 2
    _status.temperature2 = data[7];
    
    Serial.print("Flatpack status: ");
    Serial.print(_status.voltageOutput);
    Serial.print("V, ");
    Serial.print(_status.currentOutput);
    Serial.println("A");
}

void FlatpackController::processSerialMessage(uint8_t len, uint8_t* data) {
    if (len < 8) return;
    
    // Update connection status
    _status.connected = true;
    _status.lastMessageTime = millis();
    
    // Copy serial number
    memcpy(_status.serialNumber, data, 6);  // First 6 bytes contain the serial number
    
    Serial.print("Flatpack serial number: ");
    for (int i = 0; i < 6; i++) {
        Serial.print(_status.serialNumber[i], HEX);
    }
    Serial.println();
}

void FlatpackController::convertToHex(float value, uint16_t multiplier, uint8_t* lowByte, uint8_t* highByte) {
    // Convert the value to a uint16_t with the specified multiplier
    uint16_t convertedValue = static_cast<uint16_t>(value * multiplier);
    
    // Split into high and low bytes (little endian)
    *lowByte = convertedValue & 0xFF;
    *highByte = (convertedValue >> 8) & 0xFF;
}

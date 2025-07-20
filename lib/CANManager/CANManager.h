#pragma once

#include <Arduino.h>
#include <mcp2515.h>
#include <functional>

class CANManager {
public:
    // Callback function type for CAN message processing
    using MessageCallback = std::function<void(const struct can_frame&, const char*)>;
    
    CANManager();
    ~CANManager();
    
    // Initialize all CAN controllers
    bool initialize();
    
    // Set callback for processing received messages
    void setMessageCallback(MessageCallback callback);
    
    // Process all CAN controllers for incoming messages
    void processMessages();
    
    // Send message on specific CAN controller
    bool sendMessage(int canId, const struct can_frame& msg);
    
    // Get reference to specific CAN controller
    MCP2515* getController(int canId);
    
private:
    MCP2515* can1;
    MCP2515* can2; 
    MCP2515* can3;
    
    MessageCallback messageCallback;
    
    // Pin definitions
    static constexpr uint8_t PIN_CAN1_CS = 14;
    static constexpr uint8_t PIN_CAN2_CS = 17;
    static constexpr uint8_t PIN_CAN3_CS = 18;
    static constexpr uint8_t PIN_CAN1_INT = 47;
    static constexpr uint8_t PIN_CAN2_INT = 21;
    static constexpr uint8_t PIN_CAN3_INT = 10;
    static constexpr uint8_t PIN_SCK = 12;
    static constexpr uint8_t PIN_MISO = 13;
    static constexpr uint8_t PIN_MOSI = 11;
    
    // Initialize single MCP2515 controller
    bool initController(MCP2515& controller);
    
    // Process messages from single controller
    void processController(MCP2515& controller, const char* name, uint8_t intPin);
};

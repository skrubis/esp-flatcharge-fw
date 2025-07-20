#include "CANManager.h"
#include <SPI.h>

CANManager::CANManager() : messageCallback(nullptr) {
    can1 = new MCP2515(PIN_CAN1_CS);
    can2 = new MCP2515(PIN_CAN2_CS);
    can3 = new MCP2515(PIN_CAN3_CS);
}

CANManager::~CANManager() {
    delete can1;
    delete can2;
    delete can3;
}

bool CANManager::initialize() {
    Serial.println("Initializing CAN controllers...");
    
    // Initialize SPI
    SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI);
    
    // Set interrupt pins as inputs with pullup
    pinMode(PIN_CAN1_INT, INPUT_PULLUP);
    pinMode(PIN_CAN2_INT, INPUT_PULLUP);
    pinMode(PIN_CAN3_INT, INPUT_PULLUP);
    
    // Initialize all controllers
    bool success = true;
    success &= initController(*can1);
    success &= initController(*can2);
    success &= initController(*can3);
    
    if (success) {
        Serial.println("All CAN controllers initialized successfully.");
    } else {
        Serial.println("Failed to initialize one or more CAN controllers.");
    }
    
    return success;
}

bool CANManager::initController(MCP2515& controller) {
    if (controller.reset() != MCP2515::ERROR_OK) {
        Serial.println("CAN controller reset failed!");
        return false;
    }
    
    if (controller.setBitrate(CAN_125KBPS, MCP_8MHZ) != MCP2515::ERROR_OK) {
        Serial.println("CAN controller bitrate setup failed!");
        return false;
    }
    
    // Accept all messages
    controller.setFilterMask(MCP2515::MASK0, true, 0);
    controller.setFilter(MCP2515::RXF0, true, 0);
    controller.setFilter(MCP2515::RXF1, true, 0);

    controller.setFilterMask(MCP2515::MASK1, true, 0);
    controller.setFilter(MCP2515::RXF2, true, 0);
    controller.setFilter(MCP2515::RXF3, true, 0);
    controller.setFilter(MCP2515::RXF4, true, 0);
    controller.setFilter(MCP2515::RXF5, true, 0);

    if (controller.setNormalMode() != MCP2515::ERROR_OK) {
        Serial.println("CAN controller normal mode failed!");
        return false;
    }
    
    return true;
}

void CANManager::setMessageCallback(MessageCallback callback) {
    messageCallback = callback;
}

void CANManager::processMessages() {
    processController(*can1, "CAN1", PIN_CAN1_INT);
    processController(*can2, "CAN2", PIN_CAN2_INT);
    processController(*can3, "CAN3", PIN_CAN3_INT);
}

void CANManager::processController(MCP2515& controller, const char* name, uint8_t intPin) {
    if (!digitalRead(intPin)) {
        struct can_frame msg;
        if (controller.readMessage(&msg) == MCP2515::ERROR_OK) {
            // Print debug info
            Serial.printf("[%s] ID:0x%08lX DLC:%d Data:", name, msg.can_id, msg.can_dlc);
            for (uint8_t i = 0; i < msg.can_dlc; ++i) {
                Serial.printf(" %02X", msg.data[i]);
            }
            Serial.println();
            
            // Call callback if set
            if (messageCallback) {
                messageCallback(msg, name);
            }
        }
    }
}

bool CANManager::sendMessage(int canId, const struct can_frame& msg) {
    MCP2515* controller = nullptr;
    
    switch (canId) {
        case 1: controller = can1; break;
        case 2: controller = can2; break;
        case 3: controller = can3; break;
        default: return false;
    }
    
    // Debug output to verify what's being sent
    Serial.printf("[CANManager] Sending on CAN%d: ID=0x%08lX, Extended=%d, DLC=%d\n", 
                 canId, 
                 msg.can_id & CAN_EFF_MASK, 
                 (msg.can_id & CAN_EFF_FLAG) ? 1 : 0,
                 msg.can_dlc);
    
    // Send the message
    MCP2515::ERROR result = controller->sendMessage(&msg);
    
    // Check result
    if (result != MCP2515::ERROR_OK) {
        Serial.printf("[CANManager] Send error on CAN%d: %d\n", canId, (int)result);
    }
    
    return result == MCP2515::ERROR_OK;
}

MCP2515* CANManager::getController(int canId) {
    switch (canId) {
        case 1: return can1;
        case 2: return can2;
        case 3: return can3;
        default: return nullptr;
    }
}

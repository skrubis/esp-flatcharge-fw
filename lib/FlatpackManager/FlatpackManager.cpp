/**
 * @file FlatpackManager.cpp
 * @brief Implementation of FlatpackManager for controlling Eltek Flatpack2 PSUs via CAN
 * 
 * This implementation handles the Flatpack2 CAN protocol including:
 * - PSU auto-detection via hello messages
 * - Login authentication with PSU serial numbers
 * - Periodic login refresh (required every 5 seconds)
 * - Status message processing
 * - Voltage and current setpoint commands
 * - Data aggregation from multiple PSUs
 * 
 * Protocol details based on flatpack.md and fp-Protocol.md documentation.
 * 
 * @see FlatpackManager.h
 */

#include "FlatpackManager.h"

/**
 * @brief Construct a new FlatpackManager object
 * 
 * Initializes the manager with empty detection and status callbacks.
 */
FlatpackManager::FlatpackManager() : 
    detectionCallback(nullptr), 
    statusCallback(nullptr),
    canSendCallback(nullptr),
    flatpacksMutex(nullptr) {
    // Initialize detected PSU flags for each bus
    for (int i = 0; i < MAX_CAN_BUSES; i++) {
        _psuDetectedOnBus[i] = false;
        _psuSerialOnBus[i] = 0;
    }
    
    // Create mutex for thread safety
    flatpacksMutex = xSemaphoreCreateMutex();
    if (!flatpacksMutex) {
        Serial.println("[FlatpackManager] FATAL: Failed to create mutex");
    }
}

/**
 * @brief Destroy the FlatpackManager object
 */
FlatpackManager::~FlatpackManager() {
    // Clean up mutex
    if (flatpacksMutex) {
        vSemaphoreDelete(flatpacksMutex);
        flatpacksMutex = nullptr;
    }
}

/**
 * @brief Process incoming CAN messages for Flatpack protocol
 * 
 * This method identifies message types based on their CAN IDs and
 * delegates processing to specialized handlers. Currently supports:
 * - Hello messages: Used for PSU detection (0x0500XXXX with first byte 0x1B)
 * - Status messages: Provide voltage/current data (0x05XX40YY)
 * 
 * @param msg CAN frame to process
 * @param canName Name of the CAN bus where message was received (e.g., "CAN1")
 */
void FlatpackManager::processCanMessage(const struct can_frame& msg, const char* canName) {
    // Convert CAN bus name to numeric ID for internal tracking
    uint8_t canBusId = canNameToId(canName);
    
    // Extract the CAN ID for debugging
    Serial.printf("[FlatpackManager] Processing CAN message: ID=0x%08X, DLC=%d, Data[0]=0x%02X on %s\n",
                  msg.can_id, msg.can_dlc, msg.data[0], canName);
    
    // Process CAN Hello packet - hello messages start with 0x1B in first data byte
    // Format: [0x1B][6-byte serial][0x00]
    // Note: Actual CAN ID from hardware may be 0x85XXXXXX instead of 0x05XXXXXX
    if (msg.can_dlc == 8 && msg.data[0] == 0x1B) {
        processHelloMessage(msg, canBusId);
        return;
    }
    
    // Process Status message
    // Format: [intake temp][current LSB][current MSB][voltage LSB][voltage MSB]
    //         [input V LSB][input V MSB][exhaust temp]
    // Note: The CAN ID format may vary, so we look for patterns in the message structure
    //       and use data[0] != 0x1B to differentiate from hello messages
    if (msg.can_dlc == 8 && (msg.can_id & 0x0000FF00) == 0x00004000 && msg.data[0] != 0x1B) {
        processStatusMessage(msg, canBusId);
        return;
    }
}

/**
 * @brief Set the callback function for PSU detection events
 * 
 * This callback is triggered whenever a new PSU is detected via hello messages.
 * The callback receives the serial number of the detected PSU.
 * 
 * @param callback Function to call when a new PSU is detected
 */
void FlatpackManager::setDetectionCallback(FlatpackDetectionCallback callback) {
    detectionCallback = callback;
}

/**
 * @brief Set the callback function for PSU status updates
 * 
 * This callback is triggered whenever a status message is received from a PSU.
 * The callback receives a reference to the FlatpackData structure with current PSU state.
 * 
 * @param callback Function to call when a status update is received
 */
void FlatpackManager::setStatusCallback(FlatpackStatusCallback callback) {
    statusCallback = callback;
}

/**
 * @brief Set the callback function for sending CAN messages
 */
void FlatpackManager::setCanSendCallback(FlatpackCanSendCallback callback) {
    canSendCallback = callback;
}

/**
 * @brief Periodic update function for FlatpackManager
 * 
 * This method should be called regularly from the main loop.
 * It performs these operations:
 * - Refreshes logins for PSUs to maintain their session (every 5 seconds)
 * - Checks for PSU communication timeouts (10 seconds without status update)
 */
void FlatpackManager::update() {
    if (!flatpacksMutex) return;
    
    unsigned long currentTime = millis();
    
    // Lock mutex for thread-safe access to flatpacks vector
    if (xSemaphoreTake(flatpacksMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        // Check login timeouts and refresh as needed
        for (auto& fp : flatpacks) {
            if (fp.detected && fp.loggedIn) {
                // Check if login timeout is approaching (refresh at 5 seconds)
                // Flatpack2 protocol requires login refresh every 5 seconds to maintain session
                static constexpr unsigned long LOGIN_REFRESH_TIMEOUT = 5000;
                if (currentTime - fp.lastLoginTime > LOGIN_REFRESH_TIMEOUT) {
                    struct can_frame loginMsg;
                    if (sendLoginMessage(fp.serial, loginMsg)) {
                        fp.lastLoginTime = currentTime;
                        // Log is handled by the caller after receiving the frame
                    }
                }
                
                // Check if we've lost communication with this PSU
                // If no status updates for 10 seconds, consider communication lost
                static constexpr unsigned long COMMUNICATION_TIMEOUT = 10000;
                if (currentTime - fp.lastStatusTime > COMMUNICATION_TIMEOUT) {
                    Serial.printf("[FlatpackManager] Lost communication with PSU %s on CAN%d\n", 
                                  fp.serialStr, fp.canBusId);
                    fp.loggedIn = false;
                }
            }
        }
        xSemaphoreGive(flatpacksMutex);
    } else {
        Serial.println("[FlatpackManager] WARNING: Could not acquire mutex in update()");
    }
}

bool FlatpackManager::sendLoginMessage(uint64_t serial, struct can_frame& loginMsg) {
    if (!flatpacksMutex) return false;
    
    // Find the PSU in our list to get its assigned ID
    uint8_t psuId = 0;
    uint8_t canBusId = 0;
    uint8_t serialBytes[6];
    char serialStr[13];
    
    // Lock mutex for thread-safe access to flatpacks vector
    if (xSemaphoreTake(flatpacksMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        // Look up the PSU details
        for (const auto& fp : flatpacks) {
            if (fp.serial == serial && fp.detected) {
                psuId = fp.psuId;
                canBusId = fp.canBusId;
                memcpy(serialBytes, fp.serialBytes, 6);
                strcpy(serialStr, fp.serialStr);
                break;
            }
        }
        xSemaphoreGive(flatpacksMutex);
    } else {
        Serial.println("[FlatpackManager] WARNING: Could not acquire mutex in sendLoginMessage()");
        return false;
    }
    
    // If not found, assign a new ID
    if (psuId == 0) {
        Serial.printf("[FlatpackManager] WARNING: Login requested for unknown PSU %llx\n", serial);
        return false;
    }
    
    // Build the login message (format in flatpack.md)
    loginMsg.can_id = CAN_ID_LOGIN;  // 0x05004800
    loginMsg.can_id |= CAN_EFF_FLAG; // Extended frame format
    loginMsg.can_dlc = 8;            // 8 bytes
    
    // Copy the 6-byte PSU serial into bytes 0-5
    memcpy(loginMsg.data, serialBytes, 6);
    
    // Login command in bytes 6-7 (always 0x0008)
    loginMsg.data[6] = 0x00;
    loginMsg.data[7] = 0x08;
    
    // Log the login message
    Serial.printf("[FLATPACK] Built login for %s (ID: 0x%08X). Sending on CAN%d.\n", 
                 serialStr, loginMsg.can_id, canBusId);
    
    // Actually transmit the message if we have a callback registered
    if (canSendCallback) {
        bool result = canSendCallback(loginMsg, canBusId);
        if (!result) {
            Serial.printf("[FLATPACK] ERROR: Failed to send login message on CAN%d\n", canBusId);
            return false;
        }
        return true;
    } else {
        Serial.println("[FLATPACK] ERROR: No CAN send callback registered!");
        return false;
    }
}

bool FlatpackManager::setOutputParameters(uint16_t voltage, uint16_t current, uint16_t ovp, uint64_t serial) {
    struct can_frame frame;
    
    // Build setpoint message (always to broadcast address 0xFF for now)
    frame.can_id = CAN_ID_SET_OUTPUT;
    frame.can_id |= CAN_EFF_FLAG;
    frame.can_dlc = 8;
    
    // Populate data bytes in little-endian format
    // Current bytes (deciAmps)
    frame.data[0] = current & 0xFF;
    frame.data[1] = (current >> 8) & 0xFF;
    
    // Voltage bytes (centiVolts) - duplicated in two fields
    frame.data[2] = voltage & 0xFF;
    frame.data[3] = (voltage >> 8) & 0xFF;
    frame.data[4] = voltage & 0xFF;
    frame.data[5] = (voltage >> 8) & 0xFF;
    
    // OVP bytes (centiVolts)
    frame.data[6] = ovp & 0xFF;
    frame.data[7] = (ovp >> 8) & 0xFF;
    
    // Store the requested values in all detected PSUs or just the specified one
    bool success = false;
    
    for (auto& fp : flatpacks) {
        if (fp.detected && (serial == 0 || fp.serial == serial)) {
            // Store the commanded values
            fp.setVoltage = voltage;
            fp.setCurrent = current;
            fp.setOvp = ovp;
            
            // Send the command to the PSU's bus
            if (canSendCallback) {
                if (canSendCallback(frame, fp.canBusId)) {
                    success = true;
                    Serial.printf("[FLATPACK] Sent output parameters to PSU %s on CAN%d: %d.%02dV, %d.%01dA, OVP %d.%02dV\n",
                                fp.serialStr, fp.canBusId,
                                voltage/100, voltage%100,
                                current/10, current%10,
                                ovp/100, ovp%100);
                } else {
                    Serial.printf("[FLATPACK] ERROR: Failed to send output parameters to CAN%d\n", fp.canBusId);
                }
            } else {
                Serial.println("[FLATPACK] ERROR: No CAN send callback registered!");
                return false;
            }
        }
    }
    
    return success;
}

bool FlatpackManager::setDefaultVoltage(uint16_t voltage, uint64_t serial) {
    struct can_frame frame;
    
    // Build default voltage message
    frame.can_id = CAN_ID_SET_DEFAULT;
    frame.can_id |= CAN_EFF_FLAG;
    frame.can_dlc = 5;
    
    // Fixed prefix bytes
    frame.data[0] = 0x29;
    frame.data[1] = 0x15;
    frame.data[2] = 0x00;
    
    // Voltage bytes (little-endian)
    frame.data[3] = voltage & 0xFF;
    frame.data[4] = (voltage >> 8) & 0xFF;
    
    // Send to all detected PSUs or just the specified one
    bool success = false;
    
    for (auto& fp : flatpacks) {
        if (fp.detected && (serial == 0 || fp.serial == serial)) {
            // Send the command to the PSU's bus
            if (canSendCallback) {
                if (canSendCallback(frame, fp.canBusId)) {
                    success = true;
                    Serial.printf("[FLATPACK] Sent default voltage to PSU %s on CAN%d: %d.%02dV\n",
                                fp.serialStr, fp.canBusId, voltage/100, voltage%100);
                } else {
                    Serial.printf("[FLATPACK] ERROR: Failed to send default voltage to CAN%d\n", fp.canBusId);
                }
            } else {
                Serial.println("[FLATPACK] ERROR: No CAN send callback registered!");
                return false;
            }
        }
    }
    
    return success;
}

const std::vector<FlatpackData>& FlatpackManager::getFlatpacks() const {
    return flatpacks;
}

void FlatpackManager::printStatus() const {
    Serial.println("=== Flatpack Status Summary ===");
    
    if (flatpacks.empty()) {
        Serial.println("No Flatpack PSUs detected.");
        return;
    }
    
    for (const auto& fp : flatpacks) {
        Serial.printf("PSU %d (S/N: %s) on CAN%d: ", fp.psuId, fp.serialStr, fp.canBusId);
        
        if (!fp.loggedIn) {
            Serial.println("NOT LOGGED IN");
            continue;
        }
        
        // Print status
        String statusStr;
        switch (fp.status) {
            case STATUS_NORMAL:       statusStr = "NORMAL"; break;
            case STATUS_CURRENT_LIMIT: statusStr = "CURRENT LIMIT"; break;
            case STATUS_ALARM:        statusStr = "ALARM"; break;
            case STATUS_WALK_IN:      statusStr = "WALK-IN"; break;
            default:                  statusStr = "UNKNOWN"; break;
        }
        
        Serial.printf("%d.%02dV @ %d.%dA, Status: %s, Temp: %d°C/%d°C\n",
                     fp.outputVoltage/100, fp.outputVoltage%100,
                     fp.outputCurrent/10, fp.outputCurrent%10,
                     statusStr.c_str(), fp.intakeTemp, fp.exhaustTemp);
    }
    
    // Print aggregated data
    uint16_t totalCurrent, avgVoltage;
    uint8_t activePsuCount;
    
    if (getAggregatedData(totalCurrent, avgVoltage, activePsuCount)) {
        Serial.printf("AGGREGATED DATA: %d PSUs active, Total Current: %d.%dA, Voltage: %d.%02dV\n",
                     activePsuCount,
                     totalCurrent/10, totalCurrent%10,
                     avgVoltage/100, avgVoltage%100);
    }
}

bool FlatpackManager::getAggregatedData(uint16_t& totalCurrent, uint16_t& avgVoltage, uint8_t& activePsuCount) const {
    totalCurrent = 0;
    uint32_t voltageSum = 0;
    activePsuCount = 0;
    
    for (const auto& fp : flatpacks) {
        if (fp.detected && fp.loggedIn) {
            totalCurrent += fp.outputCurrent;
            voltageSum += fp.outputVoltage;
            activePsuCount++;
        }
    }
    
    if (activePsuCount > 0) {
        avgVoltage = voltageSum / activePsuCount;
        return true;
    }
    
    return false;
}

/**
 * @brief Process a Flatpack2 hello message
 * 
 * Hello messages are used for PSU auto-detection. They contain the PSU's serial number
 * and are sent periodically by each PSU. When a new PSU is detected, it's added to the 
 * flatpacks vector and the detection callback is triggered.
 * 
 * @param msg CAN frame containing hello message (format: [0x1B][6-byte serial][0x00])
 * @param canBusId Numeric ID of the CAN bus where message was received
 */
void FlatpackManager::processHelloMessage(const struct can_frame& msg, uint8_t canBusId) {
    // Validate canBusId range
    if (canBusId >= MAX_CAN_BUSES || canBusId == 0) {
        Serial.printf("[FlatpackManager] Invalid CAN bus ID: %d\n", canBusId);
        return;
    }
    
    // Extract serial number from message data
    uint8_t serialBytes[6];
    memcpy(serialBytes, &msg.data[1], 6);
    uint64_t serial = bytesToSerial(serialBytes);
    
    // Log hello message
    char serialStr[13];
    bytesToHexString(serialBytes, 6, serialStr);
    Serial.printf("[FlatpackManager] Hello message received: Serial=%s on CAN%d\n", 
                  serialStr, canBusId);
    
    if (!flatpacksMutex) return;
    
    // Lock mutex for thread-safe access to flatpacks vector
    if (xSemaphoreTake(flatpacksMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        // Check if we already know about this PSU
        for (auto& fp : flatpacks) {
            if (fp.serial == serial) {
                if (fp.canBusId == canBusId) {
                    // We already know about this PSU on this CAN bus - this is normal
                    xSemaphoreGive(flatpacksMutex);
                    return;
                } else {
                    // Same PSU detected on different bus - this is an error condition
                    Serial.printf("[FlatpackManager] ERROR: PSU %s detected on CAN%d but already registered on CAN%d\n", 
                                serialStr, canBusId, fp.canBusId);
                    xSemaphoreGive(flatpacksMutex);
                    return;
                }
            }
        }
        
        // Check if we already have a DIFFERENT PSU on this bus
        if (_psuDetectedOnBus[canBusId] && _psuSerialOnBus[canBusId] != serial) {
            // We already have a different PSU on this bus - enforce one PSU per bus rule
            Serial.printf("[FlatpackManager] WARNING: Multiple PSUs detected on CAN%d. Ignoring new PSU %s (existing: %012llX)\n", 
                        canBusId, serialStr, _psuSerialOnBus[canBusId]);
            xSemaphoreGive(flatpacksMutex);
            return;
        }
        
        // This is a new PSU on a previously unused bus - add it to our list
        FlatpackData newPsu;
        newPsu.serial = serial;
        memcpy(newPsu.serialBytes, serialBytes, 6);
        serialToString(serial, newPsu.serialStr);
        newPsu.detected = true;
        newPsu.loggedIn = false;
        newPsu.canBusId = canBusId;
        newPsu.psuId = flatpacks.size() + 1;  // Assign sequential ID starting from 1
        newPsu.lastStatusTime = 0;
        newPsu.lastLoginTime = 0;
        newPsu.status = 0;
        newPsu.intakeTemp = 0;
        newPsu.exhaustTemp = 0;
        newPsu.inputVoltage = 0;
        newPsu.outputVoltage = 0;
        newPsu.outputCurrent = 0;
        newPsu.setVoltage = 0;
        newPsu.setCurrent = 0;
        newPsu.setOvp = 0;
        newPsu.alertFlags[0] = 0;
        newPsu.alertFlags[1] = 0;
        newPsu.hasAlerts = false;
        
        // Add to our list of flatpacks
        flatpacks.push_back(newPsu);
        
        // Mark this bus as having a detected PSU
        _psuDetectedOnBus[canBusId] = true;
        _psuSerialOnBus[canBusId] = serial;
        
        Serial.printf("[FlatpackManager] New PSU detected: %s on CAN%d (assigned ID: %d)\n", 
                     newPsu.serialStr, canBusId, newPsu.psuId);
        
        xSemaphoreGive(flatpacksMutex);
        
        // Initiate login right away (outside mutex to avoid deadlock)
        struct can_frame loginMsg;
        if (sendLoginMessage(serial, loginMsg)) {
            Serial.printf("[FlatpackManager] Initial login sent to PSU %s\n", newPsu.serialStr);
        }
        
        // Notify via callback
        if (detectionCallback) {
            detectionCallback(serial);
        }
    } else {
        Serial.println("[FlatpackManager] WARNING: Could not acquire mutex in processHelloMessage()");
    }
}

void FlatpackManager::processStatusMessage(const struct can_frame& msg, uint8_t canBusId) {
    if (!flatpacksMutex) return;
    
    uint8_t statusCode = (msg.can_id & 0x000000FF);
    
    // Lock mutex for thread-safe access to flatpacks vector
    if (xSemaphoreTake(flatpacksMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        // Look for PSUs on this CAN bus
        for (auto& fp : flatpacks) {
            // Match based on CAN bus ID - we only have one PSU per bus
            if (fp.canBusId == canBusId) {
            // Update PSU status
            fp.loggedIn = true;
            fp.lastStatusTime = millis();
            fp.status = statusCode;
            
            // Parse status data
            fp.intakeTemp = msg.data[0];
            fp.exhaustTemp = msg.data[7];
            fp.inputVoltage = (msg.data[6] << 8) | msg.data[5];
            fp.outputCurrent = (msg.data[2] << 8) | msg.data[1];
            fp.outputVoltage = (msg.data[4] << 8) | msg.data[3];
            
            // Debug output for status updates
            #ifdef DEBUG_STATUS_MESSAGES
            Serial.printf("[FlatpackManager] Status update for PSU %s on CAN%d: %d.%02dV @ %d.%dA\n", 
                fp.serialStr, fp.canBusId, 
                fp.outputVoltage/100, fp.outputVoltage%100,
                fp.outputCurrent/10, fp.outputCurrent%10);
            #endif
            
                // Notify via callback
                if (statusCallback) {
                    statusCallback(fp);
                }
                
                xSemaphoreGive(flatpacksMutex);
                return;
            }
        }
        
        xSemaphoreGive(flatpacksMutex);
        
        // If we get here, we received status for a PSU that wasn't in our list
        // This is normal during startup before PSUs are detected
        #ifdef DEBUG_STATUS_MESSAGES
        Serial.printf("[FlatpackManager] Received status message on CAN%d with no matching PSU\n", canBusId);
        #endif
    } else {
        Serial.println("[FlatpackManager] WARNING: Could not acquire mutex in processStatusMessage()");
    }
}

uint8_t FlatpackManager::canNameToId(const char* canName) const {
    if (strcmp(canName, "CAN1") == 0) return 1;
    if (strcmp(canName, "CAN2") == 0) return 2;
    if (strcmp(canName, "CAN3") == 0) return 3;
    return 0; // Unknown
}

void FlatpackManager::serialToString(uint64_t serial, char* buffer) const {
    // Convert serial number to string of 12 hex digits
    sprintf(buffer, "%012llX", serial);
}

uint64_t FlatpackManager::bytesToSerial(const uint8_t* bytes) const {
    uint64_t serial = 0;
    for (int i = 0; i < 6; i++) {
        serial = (serial << 8) | bytes[i];
    }
    return serial;
}

void FlatpackManager::bytesToHexString(const uint8_t* bytes, int len, char* buffer) const {
    for (int i = 0; i < len; i++) {
        sprintf(buffer + (i * 2), "%02X", bytes[i]);
    }
    buffer[len * 2] = '\0';
}

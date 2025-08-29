/**
 * @file CreeLTOBattery.cpp
 * @brief Implementation of Cree 80V LTO Battery CAN Communication Library
 */

#include "CreeLTOBattery.h"

CreeLTOBattery::CreeLTOBattery() 
    : tx_pin(CREE_LTO_DEFAULT_TX_PIN)
    , rx_pin(CREE_LTO_DEFAULT_RX_PIN)
    , receive_task_handle(nullptr)
    , message_queue(nullptr)
    , initialized(false)
    , running(false)
    , data_mutex(nullptr)
{
    // Initialize timing config to default
    timing_config = CREE_LTO_DEFAULT_BITRATE;
}

CreeLTOBattery::~CreeLTOBattery() {
    end();
}

bool CreeLTOBattery::begin(gpio_num_t tx_pin, gpio_num_t rx_pin, twai_timing_config_t timing) {
    if (initialized) {
        handleError("Already initialized");
        return false;
    }
    
    this->tx_pin = tx_pin;
    this->rx_pin = rx_pin;
    this->timing_config = timing;
    
    // Create mutex for thread-safe data access
    data_mutex = xSemaphoreCreateMutex();
    if (data_mutex == nullptr) {
        handleError("Failed to create data mutex");
        return false;
    }
    
    // Configure TWAI general settings
    general_config = TWAI_GENERAL_CONFIG_DEFAULT(tx_pin, rx_pin, TWAI_MODE_NORMAL);
    general_config.tx_queue_len = CREE_LTO_QUEUE_SIZE;
    general_config.rx_queue_len = CREE_LTO_QUEUE_SIZE;
    
    // Configure filter to accept Cree LTO battery frames
    // Accept extended frames with IDs 0x1804F401, 0x1807F401, 0x1808F401, 0x1811F401-0x1819F401
    filter_config.acceptance_code = 0x18000000;  // Match 0x18xxxxxx
    filter_config.acceptance_mask = 0xFF000000;  // Mask first byte only
    filter_config.single_filter = true;
    
    // Install TWAI driver
    esp_err_t result = twai_driver_install(&general_config, &timing_config, &filter_config);
    if (result != ESP_OK) {
        handleError("Failed to install TWAI driver", result);
        vSemaphoreDelete(data_mutex);
        data_mutex = nullptr;
        return false;
    }
    
    // Start TWAI driver
    result = twai_start();
    if (result != ESP_OK) {
        handleError("Failed to start TWAI driver", result);
        twai_driver_uninstall();
        vSemaphoreDelete(data_mutex);
        data_mutex = nullptr;
        return false;
    }
    
    // Create message queue for internal communication
    message_queue = xQueueCreate(CREE_LTO_QUEUE_SIZE, sizeof(twai_message_t));
    if (message_queue == nullptr) {
        handleError("Failed to create message queue");
        twai_stop();
        twai_driver_uninstall();
        vSemaphoreDelete(data_mutex);
        data_mutex = nullptr;
        return false;
    }
    
    // Create receive task
    BaseType_t task_result = xTaskCreate(
        receiveTask,
        "CreeLTO_RX",
        CREE_LTO_TASK_STACK_SIZE,
        this,
        CREE_LTO_TASK_PRIORITY,
        &receive_task_handle
    );
    
    if (task_result != pdPASS) {
        handleError("Failed to create receive task");
        vQueueDelete(message_queue);
        message_queue = nullptr;
        twai_stop();
        twai_driver_uninstall();
        vSemaphoreDelete(data_mutex);
        data_mutex = nullptr;
        return false;
    }
    
    initialized = true;
    running = true;
    
    Serial.println("Cree LTO Battery library initialized successfully");
    return true;
}

void CreeLTOBattery::end() {
    if (!initialized) {
        return;
    }
    
    running = false;
    
    // Delete receive task
    if (receive_task_handle != nullptr) {
        vTaskDelete(receive_task_handle);
        receive_task_handle = nullptr;
    }
    
    // Delete message queue
    if (message_queue != nullptr) {
        vQueueDelete(message_queue);
        message_queue = nullptr;
    }
    
    // Stop and uninstall TWAI driver
    twai_stop();
    twai_driver_uninstall();
    
    // Delete mutex
    if (data_mutex != nullptr) {
        vSemaphoreDelete(data_mutex);
        data_mutex = nullptr;
    }
    
    initialized = false;
    Serial.println("Cree LTO Battery library stopped");
}

void CreeLTOBattery::receiveTask(void* parameter) {
    CreeLTOBattery* instance = static_cast<CreeLTOBattery*>(parameter);
    twai_message_t message;
    
    while (instance->running) {
        // Receive CAN message with timeout
        esp_err_t result = twai_receive(&message, pdMS_TO_TICKS(100));
        
        if (result == ESP_OK) {
            instance->processCANFrame(message);
        } else if (result == ESP_ERR_TIMEOUT) {
            // Timeout is normal, continue
            continue;
        } else {
            instance->handleError("CAN receive error", result);
            instance->statistics.bus_errors++;
        }
        
        // Small delay to prevent task from consuming too much CPU
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    
    vTaskDelete(nullptr);
}

void CreeLTOBattery::processCANFrame(const twai_message_t& message) {
    if (xSemaphoreTake(data_mutex, pdMS_TO_TICKS(10)) != pdTRUE) {
        return;
    }
    
    statistics.frames_received++;
    uint32_t timestamp = millis();
    bool decoded = false;
    
    // Call raw frame callback if registered
    if (raw_frame_callback) {
        raw_frame_callback(message.identifier, message.data, message.data_length_code);
    }
    
    // Decode based on CAN ID
    if (message.identifier >= CREE_LTO_CELL_VOLTAGE_BASE && 
        message.identifier <= CREE_LTO_CELL_VOLTAGE_END) {
        decoded = decodeCellVoltages(message.identifier, message.data);
        if (decoded) {
            statistics.cell_frames_received++;
            battery_data.last_cell_update = timestamp;
        }
    } else if (message.identifier == CREE_LTO_TEMPERATURE_ID) {
        decoded = decodeTemperatures(message.data);
        if (decoded) {
            statistics.temperature_frames_received++;
            battery_data.last_temperature_update = timestamp;
        }
    } else if (message.identifier == CREE_LTO_SOC_ID) {
        decoded = decodeSOC(message.data);
        if (decoded) {
            statistics.soc_frames_received++;
            battery_data.last_soc_update = timestamp;
        }
    } else {
        statistics.unknown_frames_received++;
    }
    
    if (!decoded && ((message.identifier >= CREE_LTO_CELL_VOLTAGE_BASE && 
                      message.identifier <= CREE_LTO_CELL_VOLTAGE_END) ||
                     message.identifier == CREE_LTO_TEMPERATURE_ID ||
                     message.identifier == CREE_LTO_SOC_ID)) {
        statistics.decode_errors++;
    }
    
    // Update pack statistics if we have cell data
    if (battery_data.cells_valid) {
        updatePackStatistics();
    }
    
    // Call data callback if registered and we have valid data
    if (data_callback && (battery_data.cells_valid || 
                         battery_data.temperature_valid || battery_data.soc_valid)) {
        data_callback(battery_data);
    }
    
    xSemaphoreGive(data_mutex);
}

bool CreeLTOBattery::decodeCellVoltages(uint32_t can_id, const uint8_t* data) {
    // Calculate which frame this is (0-8 for frames 0x1811F401 - 0x1819F401)
    uint32_t frame_offset = can_id - CREE_LTO_CELL_VOLTAGE_BASE;
    if (frame_offset % 0x10000 != 0) {
        return false; // Invalid frame ID
    }
    
    uint8_t frame_index = frame_offset / 0x10000;
    if (frame_index >= CREE_LTO_VOLTAGE_FRAMES) {
        return false;
    }
    
    // Each frame contains 4 cell voltages (little-endian, scale /1000)
    uint8_t cell_base = frame_index * CREE_LTO_CELLS_PER_FRAME;
    
    for (uint8_t i = 0; i < CREE_LTO_CELLS_PER_FRAME && (cell_base + i) < CREE_LTO_CELL_COUNT; i++) {
        uint16_t raw_voltage = extractUint16LE(data, i * 2);
        battery_data.cell_voltages[cell_base + i] = raw_voltage / 1000.0f;
    }
    
    // Check if we have received all cell voltage frames
    bool all_frames_received = true;
    uint32_t current_time = millis();
    for (uint8_t i = 0; i < CREE_LTO_VOLTAGE_FRAMES; i++) {
        // Check if this frame was updated recently (within 1 second)
        if ((current_time - battery_data.last_cell_update) > 1000) {
            all_frames_received = false;
            break;
        }
    }
    
    battery_data.cells_valid = all_frames_received;
    return true;
}

bool CreeLTOBattery::decodeTemperatures(const uint8_t* data) {
    // Temperature probe 1: bytes 2-3 (little-endian, scale /32)
    uint16_t raw_temp1 = extractUint16LE(data, 2);
    battery_data.temperature_1 = raw_temp1 / 32.0f;
    
    // Temperature probe 2: bytes 4-5 (big-endian, scale /32)
    uint16_t raw_temp2 = extractUint16BE(data, 4);
    battery_data.temperature_2 = raw_temp2 / 32.0f;
    
    battery_data.temperature_valid = true;
    return true;
}

bool CreeLTOBattery::decodeSOC(const uint8_t* data) {
    // SOC in byte 6 (scale /2.55 for percentage)
    if (data[6] == 0) {
        return false; // Invalid SOC data
    }
    
    battery_data.soc_percent = data[6] / 2.55f;
    battery_data.soc_valid = true;
    return true;
}

void CreeLTOBattery::updatePackStatistics() {
    if (!battery_data.cells_valid) {
        return;
    }
    
    float min_voltage = battery_data.cell_voltages[0];
    float max_voltage = battery_data.cell_voltages[0];
    float total_voltage = 0.0f;
    
    for (uint8_t i = 0; i < CREE_LTO_CELL_COUNT; i++) {
        float voltage = battery_data.cell_voltages[i];
        total_voltage += voltage;
        
        if (voltage < min_voltage) {
            min_voltage = voltage;
        }
        if (voltage > max_voltage) {
            max_voltage = voltage;
        }
    }
    
    battery_data.pack_voltage = total_voltage;
    battery_data.min_cell_voltage = min_voltage;
    battery_data.max_cell_voltage = max_voltage;
    battery_data.cell_voltage_delta = max_voltage - min_voltage;
}

void CreeLTOBattery::handleError(const char* error_msg, uint32_t error_code) {
    statistics.last_error_time = millis();
    
    if (error_callback) {
        error_callback(error_msg, error_code);
    } else {
        Serial.printf("Cree LTO Battery Error: %s (Code: %lu)\n", error_msg, error_code);
    }
}

uint16_t CreeLTOBattery::extractUint16LE(const uint8_t* data, uint8_t offset) {
    return data[offset] | (data[offset + 1] << 8);
}

uint16_t CreeLTOBattery::extractUint16BE(const uint8_t* data, uint8_t offset) {
    return (data[offset] << 8) | data[offset + 1];
}

// Callback registration methods
void CreeLTOBattery::onBatteryData(BatteryDataCallback callback) {
    data_callback = callback;
}

void CreeLTOBattery::onRawFrame(RawFrameCallback callback) {
    raw_frame_callback = callback;
}

void CreeLTOBattery::onError(ErrorCallback callback) {
    error_callback = callback;
}

// Data access methods
CreeLTOBatteryData CreeLTOBattery::getBatteryData() {
    CreeLTOBatteryData data_copy;
    
    if (xSemaphoreTake(data_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        data_copy = battery_data;
        xSemaphoreGive(data_mutex);
    }
    
    return data_copy;
}

CreeLTOStatistics CreeLTOBattery::getStatistics() {
    CreeLTOStatistics stats_copy;
    
    if (xSemaphoreTake(data_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        stats_copy = statistics;
        xSemaphoreGive(data_mutex);
    }
    
    return stats_copy;
}

bool CreeLTOBattery::isDataValid(uint32_t max_age_ms) {
    return hasRecentData(max_age_ms);
}

// Individual data getters
float CreeLTOBattery::getPackVoltage() {
    if (xSemaphoreTake(data_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        float voltage = battery_data.pack_voltage;
        xSemaphoreGive(data_mutex);
        return voltage;
    }
    return 0.0f;
}

float CreeLTOBattery::getCellVoltage(uint8_t cell_number) {
    if (cell_number < 1 || cell_number > CREE_LTO_CELL_COUNT) {
        return 0.0f;
    }
    
    if (xSemaphoreTake(data_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        float voltage = battery_data.cell_voltages[cell_number - 1];
        xSemaphoreGive(data_mutex);
        return voltage;
    }
    return 0.0f;
}

float CreeLTOBattery::getMinCellVoltage() {
    if (xSemaphoreTake(data_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        float voltage = battery_data.min_cell_voltage;
        xSemaphoreGive(data_mutex);
        return voltage;
    }
    return 0.0f;
}

float CreeLTOBattery::getMaxCellVoltage() {
    if (xSemaphoreTake(data_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        float voltage = battery_data.max_cell_voltage;
        xSemaphoreGive(data_mutex);
        return voltage;
    }
    return 0.0f;
}

float CreeLTOBattery::getCellVoltageDelta() {
    if (xSemaphoreTake(data_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        float delta = battery_data.cell_voltage_delta;
        xSemaphoreGive(data_mutex);
        return delta;
    }
    return 0.0f;
}

float CreeLTOBattery::getTemperature1() {
    if (xSemaphoreTake(data_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        float temp = battery_data.temperature_1;
        xSemaphoreGive(data_mutex);
        return temp;
    }
    return 0.0f;
}

float CreeLTOBattery::getTemperature2() {
    if (xSemaphoreTake(data_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        float temp = battery_data.temperature_2;
        xSemaphoreGive(data_mutex);
        return temp;
    }
    return 0.0f;
}

float CreeLTOBattery::getSOC() {
    if (xSemaphoreTake(data_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        float soc = battery_data.soc_percent;
        xSemaphoreGive(data_mutex);
        return soc;
    }
    return 0.0f;
}

// Status methods
bool CreeLTOBattery::isConnected() {
    return initialized && running;
}

bool CreeLTOBattery::hasRecentData(uint32_t max_age_ms) {
    uint32_t current_time = millis();
    
    if (xSemaphoreTake(data_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        bool recent = (current_time - battery_data.last_cell_update) < max_age_ms ||
                     (current_time - battery_data.last_temperature_update) < max_age_ms ||
                     (current_time - battery_data.last_soc_update) < max_age_ms;
        xSemaphoreGive(data_mutex);
        return recent;
    }
    return false;
}

uint32_t CreeLTOBattery::getLastUpdateTime() {
    uint32_t last_update = 0;
    
    if (xSemaphoreTake(data_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        last_update = max(battery_data.last_cell_update,
                         max(battery_data.last_temperature_update,
                            battery_data.last_soc_update));
        xSemaphoreGive(data_mutex);
    }
    
    return last_update;
}

// Utility methods
void CreeLTOBattery::printBatteryData() {
    CreeLTOBatteryData data = getBatteryData();
    
    Serial.println("=== Cree LTO Battery Data ===");
    Serial.printf("Pack Voltage: %.3f V\n", data.pack_voltage);
    Serial.printf("Min Cell: %.3f V, Max Cell: %.3f V, Delta: %.3f V\n", 
                  data.min_cell_voltage, data.max_cell_voltage, data.cell_voltage_delta);
    Serial.printf("Temperature 1: %.1f °C, Temperature 2: %.1f °C\n", 
                  data.temperature_1, data.temperature_2);
    Serial.printf("State of Charge: %.1f %%\n", data.soc_percent);
    
    Serial.println("Cell Voltages:");
    for (uint8_t i = 0; i < CREE_LTO_CELL_COUNT; i++) {
        Serial.printf("Cell %2d: %.3f V", i + 1, data.cell_voltages[i]);
        if ((i + 1) % 6 == 0) Serial.println();
        else Serial.print("  ");
    }
    if (CREE_LTO_CELL_COUNT % 6 != 0) Serial.println();
    
    Serial.printf("Data Valid - Cells: %s, Temp: %s, SOC: %s\n",
                  data.cells_valid ? "Yes" : "No",
                  data.temperature_valid ? "Yes" : "No",
                  data.soc_valid ? "Yes" : "No");
}

void CreeLTOBattery::printStatistics() {
    CreeLTOStatistics stats = getStatistics();
    
    Serial.println("=== Cree LTO Battery Statistics ===");
    Serial.printf("Total Frames: %lu\n", stats.frames_received);
    Serial.printf("Cell Frames: %lu\n", stats.cell_frames_received);
    Serial.printf("Temperature Frames: %lu\n", stats.temperature_frames_received);
    Serial.printf("SOC Frames: %lu\n", stats.soc_frames_received);
    Serial.printf("Unknown Frames: %lu\n", stats.unknown_frames_received);
    Serial.printf("Decode Errors: %lu\n", stats.decode_errors);
    Serial.printf("Bus Errors: %lu\n", stats.bus_errors);
}

void CreeLTOBattery::resetStatistics() {
    if (xSemaphoreTake(data_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        statistics = CreeLTOStatistics();
        xSemaphoreGive(data_mutex);
    }
}

void CreeLTOBattery::setCustomFilter(uint32_t filter_id, uint32_t filter_mask) {
    // This would require stopping and restarting the driver
    // Implementation left for advanced use cases
}

void CreeLTOBattery::enableRawFrameLogging(bool enable) {
    // Implementation can be added if needed for debugging
}

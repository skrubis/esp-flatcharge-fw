/**
 * @file AdvancedUsage.ino
 * @brief Advanced usage example for Cree LTO Battery library
 * 
 * This example demonstrates:
 * - Raw CAN frame monitoring
 * - Data validation and error handling
 * - Battery health monitoring
 * - Custom data processing
 */

#include <CreeLTOBattery.h>

CreeLTOBattery ltoBattery;

// Health monitoring variables
float maxCellDelta = 0.0f;
float minPackVoltage = 999.0f;
float maxPackVoltage = 0.0f;
unsigned long healthCheckInterval = 10000; // 10 seconds
unsigned long lastHealthCheck = 0;

// Data logging variables
struct BatterySnapshot {
    float pack_voltage;
    float soc;
    float max_cell_temp;
    unsigned long timestamp;
};

BatterySnapshot snapshots[10]; // Store last 10 snapshots
int snapshotIndex = 0;

void setup() {
    Serial.begin(115200);
    delay(1000);
    
    Serial.println("Cree LTO Battery Example - Advanced Usage");
    Serial.println("========================================");
    
    // Set up advanced callbacks
    ltoBattery.onBatteryData(onBatteryDataReceived);
    ltoBattery.onRawFrame(onRawFrameReceived);
    ltoBattery.onError(onErrorReceived);
    
    // Initialize with custom pins if needed
    // ltoBattery.begin(GPIO_NUM_5, GPIO_NUM_4); // Custom TX/RX pins
    
    if (ltoBattery.begin()) {
        Serial.println("Advanced monitoring started!");
        Serial.println("Features enabled:");
        Serial.println("- Real-time data callbacks");
        Serial.println("- Raw CAN frame monitoring");
        Serial.println("- Battery health tracking");
        Serial.println("- Data history logging");
    } else {
        Serial.println("Initialization failed!");
        while (1) delay(1000);
    }
}

void loop() {
    unsigned long currentTime = millis();
    
    // Perform health checks periodically
    if (currentTime - lastHealthCheck >= healthCheckInterval) {
        lastHealthCheck = currentTime;
        performHealthCheck();
        logBatterySnapshot();
    }
    
    // Check for data timeout
    if (!ltoBattery.hasRecentData(10000)) { // 10 second timeout
        Serial.println("WARNING: No battery data for 10 seconds!");
    }
    
    delay(500);
}

void onBatteryDataReceived(const CreeLTOBatteryData& data) {
    // Update health monitoring variables
    if (data.cells_valid) {
        if (data.cell_voltage_delta > maxCellDelta) {
            maxCellDelta = data.cell_voltage_delta;
        }
        
        if (data.pack_voltage > maxPackVoltage) {
            maxPackVoltage = data.pack_voltage;
        }
        
        if (data.pack_voltage < minPackVoltage && data.pack_voltage > 0) {
            minPackVoltage = data.pack_voltage;
        }
    }
    
    // Check for critical conditions
    if (data.cells_valid && data.cell_voltage_delta > 0.1f) {
        Serial.printf("WARNING: High cell voltage delta: %.3f V\n", data.cell_voltage_delta);
    }
    
    if (data.temperature_valid) {
        if (data.temperature_1 > 60.0f || data.temperature_2 > 60.0f) {
            Serial.printf("WARNING: High temperature detected: T1=%.1f°C, T2=%.1f°C\n", 
                         data.temperature_1, data.temperature_2);
        }
    }
}

void onRawFrameReceived(uint32_t can_id, const uint8_t* data, uint8_t length) {
    // Log raw frames for debugging (uncomment if needed)
    /*
    Serial.printf("Raw CAN: ID=0x%08X, Len=%d, Data=", can_id, length);
    for (int i = 0; i < length; i++) {
        Serial.printf("%02X ", data[i]);
    }
    Serial.println();
    */
}

void onErrorReceived(const char* error_msg, uint32_t error_code) {
    Serial.printf("BATTERY ERROR: %s (Code: %lu) at %lu ms\n", 
                 error_msg, error_code, millis());
    
    // Could implement error recovery logic here
    if (error_code == ESP_ERR_TIMEOUT) {
        Serial.println("Attempting to recover from timeout...");
        // Recovery logic could go here
    }
}

void performHealthCheck() {
    Serial.println("\n=== Battery Health Check ===");
    
    CreeLTOBatteryData data = ltoBattery.getBatteryData();
    CreeLTOStatistics stats = ltoBattery.getStatistics();
    
    // Overall health assessment
    bool healthy = true;
    
    if (data.cells_valid) {
        Serial.printf("Pack Voltage: %.2f V (Range: %.2f - %.2f V)\n", 
                     data.pack_voltage, minPackVoltage, maxPackVoltage);
        Serial.printf("Cell Delta: %.3f V (Max seen: %.3f V)\n", 
                     data.cell_voltage_delta, maxCellDelta);
        
        if (data.cell_voltage_delta > 0.05f) {
            Serial.println("⚠️  Cell imbalance detected");
            healthy = false;
        }
        
        // Check individual cell voltages
        int lowCells = 0, highCells = 0;
        for (int i = 0; i < CREE_LTO_CELL_COUNT; i++) {
            if (data.cell_voltages[i] < 2.0f) lowCells++;
            if (data.cell_voltages[i] > 2.8f) highCells++;
        }
        
        if (lowCells > 0) {
            Serial.printf("⚠️  %d cells below 2.0V\n", lowCells);
            healthy = false;
        }
        if (highCells > 0) {
            Serial.printf("⚠️  %d cells above 2.8V\n", highCells);
            healthy = false;
        }
    }
    
    if (data.temperature_valid) {
        float maxTemp = max(data.temperature_1, data.temperature_2);
        Serial.printf("Temperatures: %.1f°C / %.1f°C (Max: %.1f°C)\n", 
                     data.temperature_1, data.temperature_2, maxTemp);
        
        if (maxTemp > 50.0f) {
            Serial.println("⚠️  High temperature warning");
            healthy = false;
        }
    }
    
    // Communication health
    if (stats.decode_errors > 0) {
        Serial.printf("⚠️  %lu decode errors detected\n", stats.decode_errors);
        healthy = false;
    }
    
    if (stats.bus_errors > 0) {
        Serial.printf("⚠️  %lu bus errors detected\n", stats.bus_errors);
        healthy = false;
    }
    
    Serial.printf("Overall Health: %s\n", healthy ? "✅ GOOD" : "⚠️  ATTENTION NEEDED");
    
    // Frame rate analysis
    float frameRate = (float)stats.frames_received / (millis() / 1000.0f);
    Serial.printf("Frame Rate: %.1f frames/sec\n", frameRate);
    
    if (frameRate < 1.0f) {
        Serial.println("⚠️  Low frame rate - check CAN connection");
    }
}

void logBatterySnapshot() {
    CreeLTOBatteryData data = ltoBattery.getBatteryData();
    
    // Store snapshot
    snapshots[snapshotIndex].pack_voltage = data.pack_voltage;
    snapshots[snapshotIndex].soc = data.soc_percent;
    snapshots[snapshotIndex].max_cell_temp = max(data.temperature_1, data.temperature_2);
    snapshots[snapshotIndex].timestamp = millis();
    
    snapshotIndex = (snapshotIndex + 1) % 10;
    
    // Print trend analysis every 5 snapshots
    static int trendCounter = 0;
    if (++trendCounter >= 5) {
        trendCounter = 0;
        analyzeTrends();
    }
}

void analyzeTrends() {
    Serial.println("\n=== Trend Analysis (Last 10 samples) ===");
    
    float voltageSum = 0, socSum = 0, tempSum = 0;
    int validSamples = 0;
    
    for (int i = 0; i < 10; i++) {
        if (snapshots[i].timestamp > 0) {
            voltageSum += snapshots[i].pack_voltage;
            socSum += snapshots[i].soc;
            tempSum += snapshots[i].max_cell_temp;
            validSamples++;
        }
    }
    
    if (validSamples > 0) {
        Serial.printf("Average Pack Voltage: %.2f V\n", voltageSum / validSamples);
        Serial.printf("Average SOC: %.1f %%\n", socSum / validSamples);
        Serial.printf("Average Max Temperature: %.1f °C\n", tempSum / validSamples);
        
        // Simple trend detection (comparing first half vs second half)
        if (validSamples >= 6) {
            float firstHalfSOC = 0, secondHalfSOC = 0;
            for (int i = 0; i < validSamples/2; i++) {
                firstHalfSOC += snapshots[i].soc;
            }
            for (int i = validSamples/2; i < validSamples; i++) {
                secondHalfSOC += snapshots[i].soc;
            }
            
            firstHalfSOC /= (validSamples/2);
            secondHalfSOC /= (validSamples - validSamples/2);
            
            float socTrend = secondHalfSOC - firstHalfSOC;
            if (abs(socTrend) > 1.0f) {
                Serial.printf("SOC Trend: %s %.1f%%\n", 
                             socTrend > 0 ? "↗️ Increasing" : "↘️ Decreasing", 
                             abs(socTrend));
            }
        }
    }
}

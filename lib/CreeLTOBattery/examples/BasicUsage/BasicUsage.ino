/**
 * @file BasicUsage.ino
 * @brief Basic usage example for Cree LTO Battery library
 * 
 * This example demonstrates how to:
 * - Initialize the library
 * - Read battery data
 * - Set up callbacks for real-time data
 * - Monitor battery statistics
 */

#include <CreeLTOBattery.h>

// Create battery instance
CreeLTOBattery ltoBattery;

// Variables for periodic printing
unsigned long lastPrint = 0;
const unsigned long printInterval = 5000; // Print every 5 seconds

void setup() {
    Serial.begin(115200);
    delay(1000);
    
    Serial.println("Cree LTO Battery Example - Basic Usage");
    Serial.println("=====================================");
    
    // Set up callbacks before initializing
    ltoBattery.onBatteryData([](const CreeLTOBatteryData& data) {
        // This callback is called whenever new battery data is received
        Serial.printf("Battery Update - Pack: %.2fV, SOC: %.1f%%\n", 
                     data.pack_voltage, data.soc_percent);
    });
    
    ltoBattery.onError([](const char* error_msg, uint32_t error_code) {
        Serial.printf("Battery Error: %s (Code: %lu)\n", error_msg, error_code);
    });
    
    // Initialize the library with default pins (TX: GPIO21, RX: GPIO22)
    if (ltoBattery.begin()) {
        Serial.println("Cree LTO Battery initialized successfully!");
        Serial.println("Waiting for battery data...");
    } else {
        Serial.println("Failed to initialize Cree LTO Battery!");
        Serial.println("Check wiring and CAN bus connection.");
        while (1) delay(1000);
    }
}

void loop() {
    unsigned long currentTime = millis();
    
    // Print detailed battery information every 5 seconds
    if (currentTime - lastPrint >= printInterval) {
        lastPrint = currentTime;
        
        if (ltoBattery.hasRecentData()) {
            Serial.println("\n=== Battery Status ===");
            
            // Get individual values
            Serial.printf("Pack Voltage: %.3f V\n", ltoBattery.getPackVoltage());
            Serial.printf("Min Cell: %.3f V\n", ltoBattery.getMinCellVoltage());
            Serial.printf("Max Cell: %.3f V\n", ltoBattery.getMaxCellVoltage());
            Serial.printf("Cell Delta: %.3f V\n", ltoBattery.getCellVoltageDelta());
            Serial.printf("Temperature 1: %.1f °C\n", ltoBattery.getTemperature1());
            Serial.printf("Temperature 2: %.1f °C\n", ltoBattery.getTemperature2());
            Serial.printf("State of Charge: %.1f %%\n", ltoBattery.getSOC());
            
            // Print first 6 cell voltages as example
            Serial.println("Sample Cell Voltages (1-6):");
            for (int i = 1; i <= 6; i++) {
                Serial.printf("  Cell %d: %.3f V\n", i, ltoBattery.getCellVoltage(i));
            }
            
            // Print statistics
            CreeLTOStatistics stats = ltoBattery.getStatistics();
            Serial.printf("Frames Received: %lu (Cells: %lu, Temp: %lu, SOC: %lu)\n",
                         stats.frames_received, stats.cell_frames_received,
                         stats.temperature_frames_received, stats.soc_frames_received);
            
        } else {
            Serial.println("No recent battery data received!");
            Serial.println("Check CAN bus connection and battery power.");
        }
    }
    
    // Small delay to prevent overwhelming the system
    delay(100);
}

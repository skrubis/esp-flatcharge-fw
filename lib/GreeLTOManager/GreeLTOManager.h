#pragma once

#include <Arduino.h>
#include <functional>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

/**
 * GreeLTOManager - Decodes Gree/Cree 36S LTO BMS frames.
 *
 * Mirrors VectrixVX1Manager style: does NOT own TWAI. Call processCanMessage()
 * with raw TWAI frames from main.
 */

// Gree/Cree LTO CAN IDs (29-bit extended)
#define GREE_LTO_CELL_VOLTAGE_BASE  0x1811F401u  // Cell voltages 1-4
#define GREE_LTO_CELL_VOLTAGE_END   0x1819F401u  // Cell voltages 33-36
#define GREE_LTO_TEMPERATURE_ID     0x1808F401u  // Temperature probes
#define GREE_LTO_SOC_ID             0x1807F401u  // State of charge

struct GreeLtoData {
    // Per-cell voltages (V)
    float cell_voltages[36];
    // Pack summary (V)
    float pack_voltage;
    float min_cell_voltage;
    float max_cell_voltage;
    float cell_voltage_delta; // V
    // Temps (°C)
    int8_t temperature_1;
    int8_t temperature_2;
    // SOC (%)
    uint8_t soc_percent;

    // Validity
    bool cells_valid;
    bool temperature_valid;
    bool soc_valid;

    // Timestamps
    uint32_t last_cell_update;
    uint32_t last_temperature_update;
    uint32_t last_soc_update;

    GreeLtoData() {
        memset(cell_voltages, 0, sizeof(cell_voltages));
        pack_voltage = 0.0f;
        min_cell_voltage = 0.0f;
        max_cell_voltage = 0.0f;
        cell_voltage_delta = 0.0f;
        temperature_1 = 0;
        temperature_2 = 0;
        soc_percent = 0;
        cells_valid = false;
        temperature_valid = false;
        soc_valid = false;
        last_cell_update = 0;
        last_temperature_update = 0;
        last_soc_update = 0;
    }
};

using GreeDataCallback = std::function<void(const GreeLtoData&)>;

class GreeLTOManager {
public:
    GreeLTOManager();
    ~GreeLTOManager();

    bool initialize(uint8_t cellCount = 36);

    // Process a single TWAI frame (extended ID expected)
    bool processCanMessage(uint32_t canId, const uint8_t* data, uint8_t dlc);

    // Data access
    GreeLtoData getData() const;
    bool isDataValid(uint32_t maxAgeMs = 2000) const;

    // Callback
    void setDataCallback(GreeDataCallback cb);

    // Optional: provide measured pack current for downstream IR estimation (stubbed)
    void setMeasuredCurrent(float amps);

private:
    uint8_t cellCount;
    mutable SemaphoreHandle_t dataMutex;
    GreeLtoData d;
    GreeDataCallback dataCb;

    // Last current for potential IR calc
    float lastCurrentA;
    uint32_t lastDataTs;

    // Helpers
    bool decodeCellVoltages(uint32_t canId, const uint8_t* data);
    bool decodeTemperatures(const uint8_t* data);
    bool decodeSOC(const uint8_t* data);
    void updatePackSummary();

    static inline uint16_t le16(const uint8_t* p) { return (uint16_t)p[0] | ((uint16_t)p[1] << 8); }
    static inline uint16_t be16(const uint8_t* p) { return ((uint16_t)p[0] << 8) | (uint16_t)p[1]; }
};

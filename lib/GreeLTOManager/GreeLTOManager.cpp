#include "GreeLTOManager.h"

GreeLTOManager::GreeLTOManager()
    : cellCount(36), dataMutex(nullptr), dataCb(nullptr), lastCurrentA(0.0f), lastDataTs(0) {
}

GreeLTOManager::~GreeLTOManager() {
    if (dataMutex) {
        vSemaphoreDelete(dataMutex);
        dataMutex = nullptr;
    }
}

bool GreeLTOManager::initialize(uint8_t cellCount_) {
    cellCount = cellCount_;
    dataMutex = xSemaphoreCreateMutex();
    if (!dataMutex) return false;
    d = GreeLtoData();
    lastDataTs = 0;
    return true;
}

bool GreeLTOManager::processCanMessage(uint32_t canId, const uint8_t* data, uint8_t dlc) {
    if (!data || dlc != 8) return false;

    bool handled = false;
    if (canId >= GREE_LTO_CELL_VOLTAGE_BASE && canId <= GREE_LTO_CELL_VOLTAGE_END) {
        handled = decodeCellVoltages(canId, data);
    } else if (canId == GREE_LTO_TEMPERATURE_ID) {
        handled = decodeTemperatures(data);
    } else if (canId == GREE_LTO_SOC_ID) {
        handled = decodeSOC(data);
    }

    if (handled) {
        lastDataTs = millis();
        if (dataCb) {
            dataCb(getData());
        }
    }
    return handled;
}

GreeLtoData GreeLTOManager::getData() const {
    GreeLtoData out;
    if (dataMutex && xSemaphoreTake(dataMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        out = d;
        xSemaphoreGive(dataMutex);
    }
    return out;
}

bool GreeLTOManager::isDataValid(uint32_t maxAgeMs) const {
    uint32_t now = millis();
    uint32_t age = (now >= lastDataTs) ? (now - lastDataTs) : (UINT32_MAX - lastDataTs + now + 1);
    return (lastDataTs != 0) && (age <= maxAgeMs);
}

void GreeLTOManager::setDataCallback(GreeDataCallback cb) { dataCb = cb; }

void GreeLTOManager::setMeasuredCurrent(float amps) { lastCurrentA = amps; }

bool GreeLTOManager::decodeCellVoltages(uint32_t canId, const uint8_t* data) {
    uint32_t frameOffset = canId - GREE_LTO_CELL_VOLTAGE_BASE; // increments by 0x10000
    if ((frameOffset % 0x10000u) != 0u) return false;
    uint8_t frameIndex = frameOffset / 0x10000u; // 0..8
    if (frameIndex > 8) return false;

    uint8_t cellBase = frameIndex * 4;

    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(10)) != pdTRUE) return false;
    for (uint8_t i = 0; i < 4 && (cellBase + i) < 36; ++i) {
        uint16_t raw = le16(&data[i * 2]); // LE, scale /1000
        d.cell_voltages[cellBase + i] = raw / 1000.0f;
    }
    d.last_cell_update = millis();
    // After new cell data, recompute summary quickly
    updatePackSummary();
    xSemaphoreGive(dataMutex);
    return true;
}

bool GreeLTOManager::decodeTemperatures(const uint8_t* data) {
    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(10)) != pdTRUE) return false;
    // Temp1: bytes 2-3 LE /32
    uint16_t t1raw = le16(&data[2]);
    // Temp2: bytes 4-5 BE /32
    uint16_t t2raw = be16(&data[4]);
    d.temperature_1 = (int8_t)roundf(t1raw / 32.0f);
    d.temperature_2 = (int8_t)roundf(t2raw / 32.0f);
    d.temperature_valid = true;
    d.last_temperature_update = millis();
    xSemaphoreGive(dataMutex);
    return true;
}

bool GreeLTOManager::decodeSOC(const uint8_t* data) {
    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(10)) != pdTRUE) return false;
    uint8_t b = data[6];
    if (b == 0) { xSemaphoreGive(dataMutex); return false; }
    d.soc_percent = (uint8_t)roundf(b / 2.55f);
    d.soc_valid = true;
    d.last_soc_update = millis();
    xSemaphoreGive(dataMutex);
    return true;
}

void GreeLTOManager::updatePackSummary() {
    float vmin = d.cell_voltages[0];
    float vmax = d.cell_voltages[0];
    float vsum = 0.0f;
    for (uint8_t i = 0; i < 36; ++i) {
        float v = d.cell_voltages[i];
        vsum += v;
        if (v < vmin) vmin = v;
        if (v > vmax) vmax = v;
    }
    d.pack_voltage = vsum;
    d.min_cell_voltage = vmin;
    d.max_cell_voltage = vmax;
    d.cell_voltage_delta = vmax - vmin;
    // crude validity: mark cells valid if we have seen at least one full sweep recently
    d.cells_valid = (d.last_cell_update != 0);
}

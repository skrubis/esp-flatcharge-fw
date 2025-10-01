#pragma once

#include <Arduino.h>
#include <vector>
#include <functional>
#include <Preferences.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include "BatteryManager.h"
#include "GreeLTOManager.h"

class MetricsManager {
public:
    struct Config {
        bool enabled = false;
        String writeUrl;         // Full InfluxDB v2 write URL with org/bucket/precision
        String token;            // Authorization: Token <token>
        String deviceTag = "esp32s3";
        uint32_t periodMs = 5000;  // Sample/flush cadence
        uint16_t batch = 1;        // Lines per POST
        bool includeCells = true;  // Include c01..c36
        bool includeIR = true;     // Include IR fields if available
    };

    MetricsManager();
    ~MetricsManager();

    bool begin();
    void end();

    // Providers
    using GetBatteryStatusFn = std::function<BatteryStatus()>;
    using GetGreeDataFn = std::function<bool(GreeLtoData& out)>;

    void setBatteryProvider(GetBatteryStatusFn fn) { getBatteryStatus = fn; }
    void setGreeProvider(GetGreeDataFn fn) { getGreeData = fn; }

    // Call in main loop
    void update();

    // Web API helpers
    String getConfigJSON() const;
    bool setConfigFromJSON(const String& json, String& message);

private:
    // Persistence
    void loadConfig();
    void saveConfig();

    // Build line(s)
    bool buildLine(String& outLine);
    bool postBatch(const String& body);

    // IR estimation
    struct Sample {
        uint32_t tsMs = 0;
        float currentA = NAN;
        float packV = NAN;
        float cellV[36];
        bool cellsValid = false;
        Sample() { memset(cellV, 0, sizeof(cellV)); }
    };

    bool computeIRFrom(const Sample& prev, const Sample& cur);

    // State
    Preferences prefs;
    Config cfg;

    GetBatteryStatusFn getBatteryStatus;
    GetGreeDataFn getGreeData;

    // Buffers
    std::vector<String> lineBuffer;

    // Timing
    uint32_t lastSampleMs = 0;
    uint32_t lastPostMs = 0;

    // HTTP
    WiFiClientSecure tls;

    // IR results
    float irPack_mOhm = NAN;
    float irCell_mOhm[36];
    uint32_t irTsMs = 0; // when IR was computed

    // Previous sample for IR
    Sample prevSample;

    // Last post status
    int lastHttpCode = 0;
    String lastError;
};

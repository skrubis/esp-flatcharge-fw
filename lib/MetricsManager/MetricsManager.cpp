#include "MetricsManager.h"
#include <WiFi.h>

MetricsManager::MetricsManager() {
    memset(irCell_mOhm, 0, sizeof(irCell_mOhm));
}

MetricsManager::~MetricsManager() {
    end();
}

bool MetricsManager::begin() {
    loadConfig();
    tls.setInsecure();
    lineBuffer.reserve(cfg.batch > 0 ? cfg.batch : 1);
    lastSampleMs = millis();
    lastPostMs = lastSampleMs;
    return true;
}

void MetricsManager::end() {
    // nothing persistent to close here
}

void MetricsManager::loadConfig() {
    if (!prefs.begin("metrics", true)) {
        return;
    }
    cfg.enabled = prefs.getBool("enabled", false);
    cfg.writeUrl = prefs.getString("url", "");
    cfg.token = prefs.getString("token", "");
    cfg.deviceTag = prefs.getString("dev", "esp32s3");
    cfg.periodMs = prefs.getULong("period", 5000);
    cfg.batch = prefs.getUShort("batch", 1);
    cfg.includeCells = prefs.getBool("cells", true);
    cfg.includeIR = prefs.getBool("ir", true);
    prefs.end();
}

void MetricsManager::saveConfig() {
    if (!prefs.begin("metrics", false)) return;
    prefs.putBool("enabled", cfg.enabled);
    prefs.putString("url", cfg.writeUrl);
    prefs.putString("token", cfg.token);
    prefs.putString("dev", cfg.deviceTag);
    prefs.putULong("period", cfg.periodMs);
    prefs.putUShort("batch", cfg.batch);
    prefs.putBool("cells", cfg.includeCells);
    prefs.putBool("ir", cfg.includeIR);
    prefs.end();
}

String MetricsManager::getConfigJSON() const {
    DynamicJsonDocument doc(512);
    doc["enabled"] = cfg.enabled;
    doc["url"] = cfg.writeUrl;
    doc["device"] = cfg.deviceTag;
    doc["periodMs"] = cfg.periodMs;
    doc["batch"] = cfg.batch;
    doc["includeCells"] = cfg.includeCells;
    doc["includeIR"] = cfg.includeIR;
    doc["lastHttpCode"] = lastHttpCode;
    doc["lastError"] = lastError;
    String s; serializeJson(doc, s); return s;
}

bool MetricsManager::setConfigFromJSON(const String& json, String& message) {
    DynamicJsonDocument doc(1024);
    DeserializationError err = deserializeJson(doc, json);
    if (err) { message = F("Invalid JSON"); return false; }

    if (doc.containsKey("enabled")) cfg.enabled = doc["enabled"].as<bool>();
    if (doc.containsKey("url")) cfg.writeUrl = doc["url"].as<String>();
    if (doc.containsKey("token")) cfg.token = doc["token"].as<String>();
    if (doc.containsKey("device")) cfg.deviceTag = doc["device"].as<String>();
    if (doc.containsKey("periodMs")) cfg.periodMs = doc["periodMs"].as<uint32_t>();
    if (doc.containsKey("batch")) cfg.batch = doc["batch"].as<uint16_t>();
    if (doc.containsKey("includeCells")) cfg.includeCells = doc["includeCells"].as<bool>();
    if (doc.containsKey("includeIR")) cfg.includeIR = doc["includeIR"].as<bool>();

    if (cfg.periodMs < 2000) cfg.periodMs = 2000; // basic rate limit for free tier
    if (cfg.batch == 0) cfg.batch = 1;

    saveConfig();
    message = F("Metrics configuration saved");
    return true;
}

bool MetricsManager::computeIRFrom(const Sample& prev, const Sample& cur) {
    if (!prev.cellsValid || !cur.cellsValid) return false;
    float dI = cur.currentA - prev.currentA;
    if (isnan(dI) || fabsf(dI) < 0.5f) return false; // require >=0.5A step
    uint32_t dt = (cur.tsMs >= prev.tsMs) ? (cur.tsMs - prev.tsMs) : 0;
    if (dt > 10000) return false; // 10s max window

    // Pack IR (mOhm)
    if (!isnan(cur.packV) && !isnan(prev.packV)) {
        float dV = cur.packV - prev.packV;
        float r = (dV / dI) * 1000.0f; // mOhm
        irPack_mOhm = fabsf(r);
    }

    // Per-cell IR
    for (int i = 0; i < 36; ++i) {
        float dv = cur.cellV[i] - prev.cellV[i];
        float r = (dv / dI) * 1000.0f;
        irCell_mOhm[i] = fabsf(r);
    }
    irTsMs = cur.tsMs;
    return true;
}

bool MetricsManager::buildLine(String& outLine) {
    if (!getBatteryStatus) return false;

    BatteryStatus bs = getBatteryStatus();

    // Build a sample
    Sample s;
    s.tsMs = millis();
    s.currentA = bs.packCurrent;
    s.packV = bs.packVoltage;

    GreeLtoData gd;
    bool haveGree = false;
    if (getGreeData) {
        haveGree = getGreeData(gd);
        if (haveGree) {
            s.cellsValid = true;
            for (int i = 0; i < 36; ++i) s.cellV[i] = gd.cell_voltages[i];
        }
    }

    // Try to compute IR using prev sample
    if (cfg.includeIR && prevSample.tsMs != 0) {
        computeIRFrom(prevSample, s);
    }
    prevSample = s; // store for next time

    // Build line protocol
    // measurement and tags
    String m = F("pack");
    String dev = cfg.deviceTag; dev.replace(" ", "_");
    outLine.reserve(512 + (cfg.includeCells ? 36*8 : 0));
    outLine = m + ",dev=" + dev + " ";

    // fields
    bool first = true;
    auto appendField = [&](const String& k, const String& v) {
        if (!first) outLine += ","; else first = false;
        outLine += k; outLine += "="; outLine += v;
    };

    appendField(F("packV"), String(bs.packVoltage, 3));
    appendField(F("curr"), String(bs.packCurrent, 3));
    appendField(F("soc"), String((float)bs.stateOfCharge, 1));

    if (haveGree) {
        appendField(F("t1"), String((int)gd.temperature_1));
        appendField(F("t2"), String((int)gd.temperature_2));
        if (cfg.includeCells) {
            char key[8];
            for (int i = 0; i < 36; ++i) {
                snprintf(key, sizeof(key), "c%02d", i+1);
                appendField(String(key), String(gd.cell_voltages[i], 3));
            }
        }
    }

    if (cfg.includeIR && !isnan(irPack_mOhm)) {
        appendField(F("ir_pack_mOhm"), String(irPack_mOhm, 3));
        if (haveGree) {
            char key[16];
            for (int i = 0; i < 36; ++i) {
                snprintf(key, sizeof(key), "ir_c%02d_mOhm", i+1);
                appendField(String(key), String(irCell_mOhm[i], 3));
            }
        }
    }

    // timestamp (ms)
    outLine += " ";
    outLine += String((long long)millis());

    return true;
}

bool MetricsManager::postBatch(const String& body) {
    lastHttpCode = 0; lastError = String();
    if (!cfg.enabled) return true;
    if (cfg.writeUrl.length() < 10 || cfg.token.length() < 5) { lastError = F("Missing URL/token"); return false; }
    if (WiFi.status() != WL_CONNECTED) { lastError = F("WiFi not connected"); return false; }

    HTTPClient http;
    if (!http.begin(tls, cfg.writeUrl)) { lastError = F("HTTP begin failed"); return false; }
    http.addHeader("Authorization", String("Token ") + cfg.token);
    http.addHeader("Content-Type", "text/plain; charset=utf-8");

    int code = http.POST((uint8_t*)body.c_str(), body.length());
    lastHttpCode = code;
    if (code <= 0) {
        lastError = http.errorToString(code);
        http.end();
        return false;
    }
    // 204 expected
    http.end();
    return (code == 204 || code == 200 || code == 202);
}

void MetricsManager::update() {
    uint32_t now = millis();
    if (!cfg.enabled) { lastPostMs = now; lastSampleMs = now; lineBuffer.clear(); return; }

    if (now - lastSampleMs >= cfg.periodMs) {
        lastSampleMs = now;
        String line;
        if (buildLine(line)) {
            lineBuffer.push_back(line);
        }
    }

    if (!lineBuffer.empty() && ((uint32_t)lineBuffer.size() >= cfg.batch || (now - lastPostMs) >= cfg.periodMs)) {
        // Concatenate with newlines
        String body; body.reserve(lineBuffer.size() * 256);
        for (size_t i = 0; i < lineBuffer.size(); ++i) {
            body += lineBuffer[i];
            if (i + 1 != lineBuffer.size()) body += '\n';
        }
        if (postBatch(body)) {
            lineBuffer.clear();
            lastPostMs = now;
        } else {
            // Keep buffer (retry on next cycle, but cap size)
            if (lineBuffer.size() > (size_t)cfg.batch) {
                lineBuffer.erase(lineBuffer.begin(), lineBuffer.end() - cfg.batch);
            }
        }
    }
}

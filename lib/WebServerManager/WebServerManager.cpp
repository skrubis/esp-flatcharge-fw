/**
 * @file WebServerManager.cpp
 * @brief Implementation of WebServerManager for WiFi and web server functionality
 * 
 * This implementation handles:
 * - WiFi Access Point mode (SSID: FLATCHARGE, password: flatcharge!)
 * - WiFi Client mode with persistent credential storage
 * - Dual mode operation (AP + Client simultaneously)
 * - Async web server with REST API endpoints
 * - Real-time monitoring dashboard
 * - Configuration interface
 * - JSON API for flatpack and battery data
 * 
 * @see WebServerManager.h
 */

#include "WebServerManager.h"
#include <LittleFS.h>

// Externs from main.cpp for TWAI RX statistics
extern "C" float getTwaiRxRate();
extern "C" uint32_t getTwaiRxCount();

// Static constants
const char* WebServerManager::PREFS_NAMESPACE = "wifi_config";
const char* WebServerManager::PREFS_SSID_KEY = "ssid";
const char* WebServerManager::PREFS_PASSWORD_KEY = "password";
const char* WebServerManager::PREFS_ENABLED_KEY = "enabled";

/**
 * @brief Construct a new WebServerManager object
 */
WebServerManager::WebServerManager() : 
    server(nullptr),
    currentStatus(WiFiStatus::DISCONNECTED),
    lastWiFiCheck(0),
    lastReconnectAttempt(0),
    apStarted(false),
    clientConnected(false),
    serverStarted(false),
    flatpackDataCallback(nullptr),
    batteryStatusCallback(nullptr),
    batteryParametersCallback(nullptr),
    chargingParametersCallback(nullptr),
    setBatteryParamsCallback(nullptr) {
    
    // Initialize default web server config
    webConfig.port = 80;
    webConfig.enableCORS = true;
    webConfig.enableAuth = false;
    webConfig.sessionTimeout = 3600000; // 1 hour
}

/**
 * @brief Destroy the WebServerManager object
 */
WebServerManager::~WebServerManager() {
    stop();
}

/**
 * @brief Initialize the web server manager
 */
bool WebServerManager::initialize(uint16_t port) {
    Serial.println("[WebServer] Initializing WebServerManager...");
    
    webConfig.port = port;
    
    // Initialize preferences
    if (!preferences.begin(PREFS_NAMESPACE, false)) {
        Serial.println("[WebServer] Failed to initialize preferences");
        return false;
    }
    
    // Load saved WiFi configuration
    loadWiFiConfig();
    
    // Create web server instance
    server = new(std::nothrow) AsyncWebServer(webConfig.port);
    if (!server) {
        Serial.println("[WebServer] Failed to create web server");
        preferences.end();
        return false;
    }
    
    // Setup web server routes
    setupWebServer();
    
    Serial.printf("[WebServer] WebServerManager initialized on port %d\n", webConfig.port);
    return true;
}

/**
 * @brief Start WiFi Access Point mode
 */
bool WebServerManager::startAccessPoint(const String& ssid, const String& password) {
    Serial.printf("[WebServer] Starting Access Point: %s\n", ssid.c_str());
    
    // Configure AP
    WiFi.mode(WIFI_AP);
    bool success = WiFi.softAP(ssid.c_str(), password.c_str());
    
    if (success) {
        apStarted = true;
        currentStatus = WiFiStatus::AP_MODE;
        
        // Start web server if not already started
        if (!serverStarted) {
            server->begin();
            serverStarted = true;
        }
        
        // If saved client configuration exists, attempt AP+STA connection now
        if (clientConfig.enabled && clientConfig.ssid.length() > 0) {
            Serial.printf("[WebServer] Saved WiFi found (%s). Attempting AP+STA connect...\n", clientConfig.ssid.c_str());
            // Do not re-save (persistent=false)
            connectToWiFi(clientConfig.ssid, clientConfig.password, false);
        }
        
        Serial.printf("[WebServer] Access Point started successfully\n");
        Serial.printf("[WebServer] AP IP: %s\n", WiFi.softAPIP().toString().c_str());
        return true;
    } else {
        Serial.println("[WebServer] Failed to start Access Point");
        currentStatus = WiFiStatus::ERROR;
        return false;
    }
}

/**
 * @brief Connect to WiFi network as client
 */
bool WebServerManager::connectToWiFi(const String& ssid, const String& password, bool persistent) {
    Serial.printf("[WebServer] Connecting to WiFi: %s\n", ssid.c_str());
    
    // Save configuration if persistent
    if (persistent) {
        clientConfig.ssid = ssid;
        clientConfig.password = password;
        clientConfig.enabled = true;
        saveWiFiConfig();
    }
    
    // Set WiFi mode
    if (apStarted) {
        WiFi.mode(WIFI_AP_STA);
        currentStatus = WiFiStatus::DUAL_MODE;
    } else {
        WiFi.mode(WIFI_STA);
        currentStatus = WiFiStatus::CONNECTING;
    }
    
    // Begin connection
    WiFi.begin(ssid.c_str(), password.c_str());
    
    // Start web server if not already started
    if (!serverStarted) {
        server->begin();
        serverStarted = true;
    }
    
    return true;
}

/**
 * @brief Start dual mode (AP + Client)
 */
bool WebServerManager::startDualMode(const String& clientSSID, const String& clientPassword) {
    Serial.println("[WebServer] Starting dual mode (AP + Client)");
    
    // First start AP mode
    if (!startAccessPoint()) {
        return false;
    }
    
    // Then connect as client
    return connectToWiFi(clientSSID, clientPassword, true);
}

/**
 * @brief Stop WiFi and web server
 */
void WebServerManager::stop() {
    Serial.println("[WebServer] Stopping WebServerManager...");
    
    if (serverStarted && server) {
        server->end();
        serverStarted = false;
    }
    
    if (server) {
        delete server;
        server = nullptr;
    }
    
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
    
    apStarted = false;
    clientConnected = false;
    currentStatus = WiFiStatus::DISCONNECTED;
    
    preferences.end();
}

/**
 * @brief Update function (call in main loop)
 */
void WebServerManager::update() {
    unsigned long currentTime = millis();
    
    // Check WiFi status periodically
    if (currentTime - lastWiFiCheck >= WIFI_CHECK_INTERVAL) {
        lastWiFiCheck = currentTime;
        handleWiFiEvents();
        
        // Attempt reconnection if needed
        if (clientConfig.enabled && !clientConnected && 
            currentTime - lastReconnectAttempt >= RECONNECT_INTERVAL) {
            attemptReconnection();
            lastReconnectAttempt = currentTime;
        }
    }
}

/**
 * @brief Get current WiFi status
 */
WiFiStatus WebServerManager::getWiFiStatus() const {
    return currentStatus;
}

/**
 * @brief Get WiFi connection info
 */
void WebServerManager::getConnectionInfo(String& info) const {
    info = "WiFi Status: ";
    
    switch (currentStatus) {
        case WiFiStatus::DISCONNECTED:
            info += "Disconnected";
            break;
        case WiFiStatus::CONNECTING:
            info += "Connecting...";
            break;
        case WiFiStatus::CONNECTED:
            info += "Connected to " + WiFi.SSID();
            info += " (IP: " + WiFi.localIP().toString() + ")";
            break;
        case WiFiStatus::AP_MODE:
            info += "Access Point Mode";
            info += " (IP: " + WiFi.softAPIP().toString() + ")";
            break;
        case WiFiStatus::DUAL_MODE:
            info += "Dual Mode - AP: " + WiFi.softAPIP().toString();
            if (clientConnected) {
                info += ", Client: " + WiFi.localIP().toString();
            } else {
                info += ", Client: Connecting...";
            }
            break;
        case WiFiStatus::ERROR:
            info += "Error";
            break;
    }
}

/**
 * @brief Set callback for getting flatpack data
 */
void WebServerManager::setFlatpackDataCallback(FlatpackDataCallback callback) {
    flatpackDataCallback = callback;
}

/**
 * @brief Set callback for getting battery status
 */
void WebServerManager::setBatteryStatusCallback(BatteryStatusCallback callback) {
    batteryStatusCallback = callback;
}

/**
 * @brief Set callback for getting battery parameters
 */
void WebServerManager::setBatteryParametersCallback(BatteryParametersCallback callback) {
    batteryParametersCallback = callback;
}

/**
 * @brief Set callback for setting charging parameters
 */
void WebServerManager::setChargingParametersCallback(SetChargingParametersCallback callback) {
    chargingParametersCallback = callback;
}

/**
 * @brief Set callback for setting battery parameters
 */
void WebServerManager::setBatteryParametersCallback(SetBatteryParametersCallback callback) {
    setBatteryParamsCallback = callback;
}

void WebServerManager::setOperatingModeCallback(SetOperatingModeCallback callback) {
    operatingModeCallback = callback;
}

void WebServerManager::setManualCurrentCallback(SetManualCurrentCallback callback) {
    manualCurrentCallback = callback;
}

void WebServerManager::setVoltageCompensationCallback(SetVoltageCompensationCallback callback) {
    voltageCompensationCallback = callback;
}

void WebServerManager::setChargingEnabledCallback(SetChargingEnabledCallback callback) {
    chargingEnabledCallback = callback;
}

void WebServerManager::setVoltageCalibrationCallback(SetVoltageCalibrationCallback callback) {
    voltageCalibrationCallback = callback;
}

void WebServerManager::setDisableCurrentLimitCallback(SetDisableCurrentLimitCallback callback) {
    disableCurrentLimitCallback = callback;
}

void WebServerManager::setDefaultPerPsuVoltageCallback(SetDefaultPerPsuVoltageCallback callback) {
    defaultPerPsuVoltageCallback = callback;
}

void WebServerManager::setAcPresetCallback(SetAcPresetCallback callback) {
    acPresetCallback = callback;
}

void WebServerManager::setMaxCellVoltageCallback(SetMaxCellVoltageCallback callback) {
    maxCellVoltageCallback = callback;
}

/**
 * @brief Get web server URL
 */
String WebServerManager::getWebServerURL() const {
    String url = "http://";
    
    if (currentStatus == WiFiStatus::AP_MODE || currentStatus == WiFiStatus::DUAL_MODE) {
        url += WiFi.softAPIP().toString();
    } else if (currentStatus == WiFiStatus::CONNECTED) {
        url += WiFi.localIP().toString();
    } else {
        return "Not available";
    }
    
    if (webConfig.port != 80) {
        url += ":" + String(webConfig.port);
    }
    
    return url;
}

/**
 * @brief Print status information to Serial
 */
void WebServerManager::printStatus() const {
    Serial.println("=== WebServer Status ===");
    
    String info;
    getConnectionInfo(info);
    Serial.println(info);
    
    if (serverStarted) {
        Serial.printf("Web Server: Running on port %d\n", webConfig.port);
        Serial.printf("Web URL: %s\n", getWebServerURL().c_str());
    } else {
        Serial.println("Web Server: Stopped");
    }
    
    if (apStarted) {
        Serial.printf("AP Clients: %d\n", WiFi.softAPgetStationNum());
    }
    
    Serial.println("========================");
}

/**
 * @brief Load WiFi configuration from preferences
 */
void WebServerManager::loadWiFiConfig() {
    clientConfig.ssid = preferences.getString(PREFS_SSID_KEY, "");
    clientConfig.password = preferences.getString(PREFS_PASSWORD_KEY, "");
    clientConfig.enabled = preferences.getBool(PREFS_ENABLED_KEY, false);
    
    if (clientConfig.enabled && clientConfig.ssid.length() > 0) {
        Serial.printf("[WebServer] Loaded WiFi config: %s\n", clientConfig.ssid.c_str());
    }
}

/**
 * @brief Save WiFi configuration to preferences
 */
void WebServerManager::saveWiFiConfig() {
    preferences.putString(PREFS_SSID_KEY, clientConfig.ssid);
    preferences.putString(PREFS_PASSWORD_KEY, clientConfig.password);
    preferences.putBool(PREFS_ENABLED_KEY, clientConfig.enabled);
    
    Serial.printf("[WebServer] Saved WiFi config: %s\n", clientConfig.ssid.c_str());
}

/**
 * @brief Handle WiFi events and update status
 */
void WebServerManager::handleWiFiEvents() {
    wl_status_t wifiStatus = WiFi.status();
    
    switch (wifiStatus) {
        case WL_CONNECTED:
            if (!clientConnected) {
                clientConnected = true;
                Serial.printf("[WebServer] WiFi connected to %s\n", WiFi.SSID().c_str());
                Serial.printf("[WebServer] Client IP: %s\n", WiFi.localIP().toString().c_str());
                
                if (apStarted) {
                    currentStatus = WiFiStatus::DUAL_MODE;
                } else {
                    currentStatus = WiFiStatus::CONNECTED;
                }
            }
            break;
            
        case WL_DISCONNECTED:
        case WL_CONNECTION_LOST:
            if (clientConnected) {
                clientConnected = false;
                Serial.println("[WebServer] WiFi client disconnected");
                
                if (apStarted) {
                    currentStatus = WiFiStatus::AP_MODE;
                } else {
                    currentStatus = WiFiStatus::DISCONNECTED;
                }
            }
            break;
            
        case WL_CONNECT_FAILED:
        case WL_NO_SSID_AVAIL:
            Serial.println("[WebServer] WiFi connection failed");
            clientConnected = false;
            if (!apStarted) {
                currentStatus = WiFiStatus::ERROR;
            }
            break;
            
        default:
            // Connection in progress
            break;
    }
}

/**
 * @brief Attempt to reconnect to WiFi
 */
void WebServerManager::attemptReconnection() {
    if (clientConfig.ssid.length() > 0) {
        Serial.printf("[WebServer] Attempting to reconnect to %s\n", clientConfig.ssid.c_str());
        WiFi.begin(clientConfig.ssid.c_str(), clientConfig.password.c_str());
    }
}

/**
 * @brief Setup web server routes and handlers
 */
void WebServerManager::setupWebServer() {
    // Enable CORS if configured
    if (webConfig.enableCORS) {
        DefaultHeaders::Instance().addHeader("Access-Control-Allow-Origin", "*");
        DefaultHeaders::Instance().addHeader("Access-Control-Allow-Methods", "GET, POST, PUT, DELETE, OPTIONS");
        DefaultHeaders::Instance().addHeader("Access-Control-Allow-Headers", "Content-Type, Authorization");
    }
    
    // Try to mount LittleFS (for serving modern dashboard)
    if (!LittleFS.begin(true)) {
        Serial.println("[WebServer] LittleFS mount failed, falling back to embedded UI");
    } else {
        Serial.println("[WebServer] LittleFS mounted successfully");
    }

    // Setup API routes
    setupAPIRoutes();
    
    // Setup web interface
    setupWebInterface();
    
    // Handle 404
    server->onNotFound([](AsyncWebServerRequest* request) {
        request->send(404, "application/json", "{\"error\":\"Not found\"}");
    });
}

/**
 * @brief Setup API routes
 */
void WebServerManager::setupAPIRoutes() {
    // System status endpoint
    server->on("/api/status", HTTP_GET, [this](AsyncWebServerRequest* request) {
        handleGetStatus(request);
    });
    
    // Flatpack data endpoint
    server->on("/api/flatpacks", HTTP_GET, [this](AsyncWebServerRequest* request) {
        handleGetFlatpacks(request);
    });
    
    // Battery data endpoint
    server->on("/api/battery", HTTP_GET, [this](AsyncWebServerRequest* request) {
        handleGetBattery(request);
    });
    
    // Set charging parameters (use body handler to reliably parse JSON)
    server->on("/api/charging", HTTP_POST,
        [](AsyncWebServerRequest* request) {},
        NULL,
        [this](AsyncWebServerRequest* request, uint8_t* data, size_t len, size_t index, size_t total) {
            if (index == 0) {
                request->_tempObject = new String();
                ((String*)request->_tempObject)->reserve(total);
            }
            String* body = (String*)request->_tempObject;
            body->concat((const char*)data, len);
            if (index + len == total) {
                DynamicJsonDocument doc(1024);
                DeserializationError err = deserializeJson(doc, *body);
                delete body;
                request->_tempObject = nullptr;
                if (err) { sendErrorResponse(request, "Invalid JSON request"); return; }

                bool success = false;
                String message = "No action taken";

                if (doc.containsKey("enabled")) {
                    bool enabled = doc["enabled"];
                    if (chargingEnabledCallback) {
                        success = chargingEnabledCallback(enabled);
                        message = success ? (enabled ? "Charging started" : "Charging stopped") : "Failed to change charging state";
                    } else {
                        message = "Charging control not available";
                    }
                } else if (doc.containsKey("voltage") && doc.containsKey("current")) {
                    uint16_t voltage = doc["voltage"];
                    uint16_t current = doc["current"];
                    uint16_t ovp = doc.containsKey("ovp") ? doc["ovp"] : (uint16_t)(voltage * 110 / 100);
                    if (chargingParametersCallback) {
                        success = chargingParametersCallback(voltage, current, ovp);
                        message = success ? "Charging parameters updated" : "Failed to update parameters";
                    } else {
                        message = "Parameter update not available";
                    }
                } else {
                    sendErrorResponse(request, "Missing required parameters (enabled or voltage/current)");
                    return;
                }

                DynamicJsonDocument response(512);
                response["success"] = success;
                response["message"] = message;
                String resp;
                serializeJson(response, resp);
                sendJSONResponse(request, resp);
            }
        }
    );
    
    // Set battery parameters (use body handler)
    server->on("/api/battery", HTTP_POST,
        [](AsyncWebServerRequest* request) {},
        NULL,
        [this](AsyncWebServerRequest* request, uint8_t* data, size_t len, size_t index, size_t total) {
            if (index == 0) {
                request->_tempObject = new String();
                ((String*)request->_tempObject)->reserve(total);
            }
            String* body = (String*)request->_tempObject;
            body->concat((const char*)data, len);
            if (index + len == total) {
                DynamicJsonDocument doc(1024);
                DeserializationError err = deserializeJson(doc, *body);
                delete body;
                request->_tempObject = nullptr;
                if (err) { sendErrorResponse(request, "Invalid JSON request"); return; }

                bool success = true;
                String message = "Battery parameters updated";

                if (doc.containsKey("operatingMode")) {
                    String modeStr = doc["operatingMode"];
                    OperatingMode mode = (modeStr == "manual") ? OperatingMode::MANUAL_CONTROLLED : OperatingMode::BMS_CONTROLLED;
                    if (operatingModeCallback && !operatingModeCallback(mode)) {
                        success = false;
                        message = "Failed to set operating mode";
                    }
                }

                if (doc.containsKey("manualCurrentLimit")) {
                    float current = doc["manualCurrentLimit"].as<float>();
                    if (manualCurrentCallback) {
                        bool ok = manualCurrentCallback(current);
                        success = success && ok;
                        if (!ok) message = "Failed to set manual current limit";
                    }
                }

                if (doc.containsKey("voltageDropCompensation")) {
                    float compensation = doc["voltageDropCompensation"].as<float>();
                    if (voltageCompensationCallback) {
                        bool ok = voltageCompensationCallback(compensation);
                        success = success && ok;
                        if (!ok) message = "Failed to set voltage drop compensation";
                    }
                }

                if (doc.containsKey("voltageCalibrationOffset")) {
                    float offsetV = doc["voltageCalibrationOffset"].as<float>();
                    if (voltageCalibrationCallback) {
                        bool ok = voltageCalibrationCallback(offsetV);
                        success = success && ok;
                        if (!ok) message = "Failed to set voltage calibration offset";
                    }
                }

                if (doc.containsKey("disableCurrentLimit")) {
                    bool disabled = doc["disableCurrentLimit"];
                    if (disableCurrentLimitCallback) {
                        bool ok = disableCurrentLimitCallback(disabled);
                        success = success && ok;
                        if (!ok) message = "Failed to set disable current limit";
                    }
                }

                if (doc.containsKey("defaultPerPsuVoltage")) {
                    float volts = doc["defaultPerPsuVoltage"].as<float>();
                    if (defaultPerPsuVoltageCallback) {
                        bool ok = defaultPerPsuVoltageCallback(volts);
                        success = success && ok;
                        if (!ok) message = "Failed to set default per-PSU voltage";
                    }
                }

                // Handle AC preset selection (persist on device)
                if (doc.containsKey("acPreset")) {
                    uint8_t presetId = doc["acPreset"].as<uint8_t>();
                    if (acPresetCallback) {
                        bool ok = acPresetCallback(presetId);
                        success = success && ok;
                        if (!ok) message = "Failed to set AC preset";
                    }
                }

                // Handle Max Cell Voltage (Volts per cell)
                if (doc.containsKey("maxCellVoltage")) {
                    float cellV = doc["maxCellVoltage"].as<float>();
                    if (maxCellVoltageCallback) {
                        bool ok = maxCellVoltageCallback(cellV);
                        success = success && ok;
                        if (!ok) message = "Failed to set max cell voltage";
                    }
                }

                DynamicJsonDocument response(512);
                response["success"] = success;
                response["message"] = message;
                String resp;
                serializeJson(response, resp);
                sendJSONResponse(request, resp);
            }
        }
    );
    
    // WiFi configuration endpoints
    server->on("/api/wifi", HTTP_GET, [this](AsyncWebServerRequest* request) {
        handleGetWiFiConfig(request);
    });
    
    server->on("/api/wifi", HTTP_POST,
        [](AsyncWebServerRequest* request) {},
        NULL,
        [this](AsyncWebServerRequest* request, uint8_t* data, size_t len, size_t index, size_t total) {
            if (index == 0) {
                request->_tempObject = new String();
                ((String*)request->_tempObject)->reserve(total);
            }
            String* body = (String*)request->_tempObject;
            body->concat((const char*)data, len);
            if (index + len == total) {
                DynamicJsonDocument doc(1024);
                DeserializationError err = deserializeJson(doc, *body);
                delete body;
                request->_tempObject = nullptr;
                if (err) { sendErrorResponse(request, "Invalid JSON request"); return; }

                if (!doc.containsKey("ssid")) {
                    sendErrorResponse(request, "Missing SSID parameter");
                    return;
                }
                String ssid = doc["ssid"].as<String>();
                String password = doc.containsKey("password") ? doc["password"].as<String>() : "";

                bool success = connectToWiFi(ssid, password, true);
                DynamicJsonDocument response(512);
                response["success"] = success;
                response["message"] = success ? "WiFi configuration saved" : "Failed to save WiFi config";
                String resp;
                serializeJson(response, resp);
                sendJSONResponse(request, resp);
            }
        }
    );
    
    // System info endpoint
    server->on("/api/system", HTTP_GET, [this](AsyncWebServerRequest* request) {
        handleGetSystemInfo(request);
    });
    
    // Restart endpoint
    server->on("/api/restart", HTTP_POST, [this](AsyncWebServerRequest* request) {
        handleRestart(request);
    });
}

/**
 * @brief Setup web interface routes
 */
void WebServerManager::setupWebInterface() {
    // If filesystem is mounted and index.html exists, serve static site
    if (LittleFS.begin(false) && LittleFS.exists("/index.html")) {
        server->serveStatic("/", LittleFS, "/").setDefaultFile("index.html");
        Serial.println("[WebServer] Serving UI from LittleFS /index.html");
        return;
    }

    // Fallback: embedded minimal UI
    server->on("/", HTTP_GET, [](AsyncWebServerRequest* request) {
        String html = R"HTML(
<!DOCTYPE html>
<html>
<head>
    <title>FLATCHARGE Monitor</title>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <style>
        body { font-family: Arial, sans-serif; margin: 20px; background: #f0f0f0; }
        .container { max-width: 1200px; margin: 0 auto; }
        .card { background: white; padding: 20px; margin: 10px 0; border-radius: 8px; box-shadow: 0 2px 4px rgba(0,0,0,0.1); }
        .header { text-align: center; color: #333; }
        .status { display: flex; flex-wrap: wrap; gap: 20px; }
        .status-item { flex: 1; min-width: 200px; }
        .value { font-size: 24px; font-weight: bold; color: #2196F3; }
        .label { color: #666; margin-bottom: 5px; }
        .flatpack { border-left: 4px solid #4CAF50; }
        .battery { border-left: 4px solid #FF9800; }
        .wifi { border-left: 4px solid #9C27B0; }
        .controls { margin-top: 20px; }
        .btn { background: #2196F3; color: white; padding: 10px 20px; border: none; border-radius: 4px; cursor: pointer; margin: 5px; }
        .btn:hover { background: #1976D2; }
        .input-group { margin: 10px 0; }
        .input-group label { display: block; margin-bottom: 5px; }
        .input-group input { width: 100%; padding: 8px; border: 1px solid #ddd; border-radius: 4px; }
        #status { margin-top: 20px; padding: 10px; background: #e8f5e8; border-radius: 4px; }
    </style>
</head>
<body>
    <div class="container">
        <div class="card">
            <h1 class="header">🔋 FLATCHARGE Monitor</h1>
        </div>
        
        <div class="card flatpack">
            <h2>Flatpack Status</h2>
            <div class="status" id="flatpack-status">
                <div class="status-item">
                    <div class="label">Detected PSUs</div>
                    <div class="value" id="psu-count">-</div>
                </div>
                <div class="status-item">
                    <div class="label">Total Current (A)</div>
                    <div class="value" id="total-current">-</div>
                </div>
                <div class="status-item">
                    <div class="label">Average Voltage (V)</div>
                    <div class="value" id="avg-voltage">-</div>
                </div>
            </div>
        </div>
        
        <div class="card battery">
            <h2>Battery Status</h2>
            <div class="status" id="battery-status">
                <div class="status-item">
                    <div class="label">Pack Voltage (V)</div>
                    <div class="value" id="pack-voltage">-</div>
                </div>
                <div class="status-item">
                    <div class="label">Pack Current (A)</div>
                    <div class="value" id="pack-current">-</div>
                </div>
                <div class="status-item">
                    <div class="label">Temperature (°C)</div>
                    <div class="value" id="temperature">-</div>
                </div>
                <div class="status-item">
                    <div class="label">Charging Mode</div>
                    <div class="value" id="charging-mode">-</div>
                </div>
            </div>
        </div>
        
        <div class="card wifi">
            <h2>WiFi Status</h2>
            <div class="status" id="wifi-status">
                <div class="status-item">
                    <div class="label">Status</div>
                    <div class="value" id="wifi-mode">-</div>
                </div>
                <div class="status-item">
                    <div class="label">IP Address</div>
                    <div class="value" id="ip-address">-</div>
                </div>
            </div>
        </div>
        
        <div class="card">
            <h2>Controls</h2>
            <div class="controls">
                <div class="input-group">
                    <label>Charging Voltage (V):</label>
                    <input type="number" id="charge-voltage" step="0.1" min="40" max="60" value="54.4">
                </div>
                <div class="input-group">
                    <label>Charging Current (A):</label>
                    <input type="number" id="charge-current" step="0.1" min="0" max="100" value="10">
                </div>
                <button class="btn" onclick="setChargingParams()">Set Charging Parameters</button>
                
                <div class="input-group" style="margin-top: 20px;">
                    <label>WiFi SSID:</label>
                    <input type="text" id="wifi-ssid" placeholder="Enter WiFi network name">
                </div>
                <div class="input-group">
                    <label>WiFi Password:</label>
                    <input type="password" id="wifi-password" placeholder="Enter WiFi password">
                </div>
                <button class="btn" onclick="setWiFiConfig()">Connect to WiFi</button>
            </div>
            <div id="status"></div>
        </div>
    </div>

    <script>
        function updateStatus() {
            fetch('/api/status')
                .then(response => response.json())
                .then(data => {
                    // Update flatpack status
                    document.getElementById('psu-count').textContent = data.flatpacks.length;
                    document.getElementById('total-current').textContent = (data.totalCurrent / 10).toFixed(1);
                    document.getElementById('avg-voltage').textContent = (data.avgVoltage / 100).toFixed(2);
                    
                    // Update battery status
                    document.getElementById('pack-voltage').textContent = data.battery.packVoltage.toFixed(2);
                    document.getElementById('pack-current').textContent = data.battery.packCurrent.toFixed(2);
                    document.getElementById('temperature').textContent = data.battery.temperature;
                    document.getElementById('charging-mode').textContent = data.battery.mode;
                    
                    // Update WiFi status
                    document.getElementById('wifi-mode').textContent = data.wifi.status;
                    document.getElementById('ip-address').textContent = data.wifi.ip;
                })
                .catch(error => console.error('Error:', error));
        }
        
        function setChargingParams() {
            const voltage = parseFloat(document.getElementById('charge-voltage').value);
            const current = parseFloat(document.getElementById('charge-current').value);
            
            fetch('/api/charging', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({
                    voltage: Math.round(voltage * 100),
                    current: Math.round(current * 10),
                    ovp: Math.round(voltage * 110)
                })
            })
            .then(response => response.json())
            .then(data => {
                document.getElementById('status').innerHTML = 
                    data.success ? '✅ Charging parameters updated' : '❌ Failed to update parameters';
            })
            .catch(error => {
                document.getElementById('status').innerHTML = '❌ Error: ' + error;
            });
        }
        
        function setWiFiConfig() {
            const ssid = document.getElementById('wifi-ssid').value;
            const password = document.getElementById('wifi-password').value;
            
            if (!ssid) {
                document.getElementById('status').innerHTML = '❌ Please enter WiFi SSID';
                return;
            }
            
            fetch('/api/wifi', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({ ssid: ssid, password: password })
            })
            .then(response => response.json())
            .then(data => {
                document.getElementById('status').innerHTML = 
                    data.success ? '✅ WiFi configuration saved' : '❌ Failed to save WiFi config';
            })
            .catch(error => {
                document.getElementById('status').innerHTML = '❌ Error: ' + error;
            });
        }
        
        // Update status every 2 seconds
        setInterval(updateStatus, 2000);
        updateStatus(); // Initial update
    </script>
</body>
</html>
        )HTML";
        request->send(200, "text/html", html);
    });
}

/**
 * @brief Handle GET /api/status endpoint
 */
void WebServerManager::handleGetStatus(AsyncWebServerRequest* request) {
    String json = generateStatusJSON();
    sendJSONResponse(request, json);
}

/**
 * @brief Handle GET /api/flatpacks endpoint
 */
void WebServerManager::handleGetFlatpacks(AsyncWebServerRequest* request) {
    String json = generateFlatpacksJSON();
    sendJSONResponse(request, json);
}

/**
 * @brief Handle GET /api/battery endpoint
 */
void WebServerManager::handleGetBattery(AsyncWebServerRequest* request) {
    String json = generateBatteryJSON();
    sendJSONResponse(request, json);
}

/**
 * @brief Handle POST /api/charging endpoint
 */
void WebServerManager::handleSetCharging(AsyncWebServerRequest* request) {
    DynamicJsonDocument doc(1024);
    
    if (!validateJSONRequest(request, doc)) {
        sendErrorResponse(request, "Invalid JSON request");
        return;
    }
    
    bool success = false;
    String message = "No action taken";
    
    // Handle charging enable/disable
    if (doc.containsKey("enabled")) {
        bool enabled = doc["enabled"];
        if (chargingEnabledCallback) {
            success = chargingEnabledCallback(enabled);
            message = success ? (enabled ? "Charging started" : "Charging stopped") : "Failed to change charging state";
        } else {
            message = "Charging control not available";
        }
    }
    // Handle charging parameter updates
    else if (doc.containsKey("voltage") && doc.containsKey("current")) {
        uint16_t voltage = doc["voltage"];
        uint16_t current = doc["current"];
        uint16_t ovp = doc.containsKey("ovp") ? doc["ovp"] : voltage * 110 / 100;
        
        if (chargingParametersCallback) {
            success = chargingParametersCallback(voltage, current, ovp);
            message = success ? "Charging parameters updated" : "Failed to update parameters";
        } else {
            message = "Parameter update not available";
        }
    }
    else {
        sendErrorResponse(request, "Missing required parameters (enabled or voltage/current)");
        return;
    }
    
    DynamicJsonDocument response(512);
    response["success"] = success;
    response["message"] = message;
    
    String responseStr;
    serializeJson(response, responseStr);
    sendJSONResponse(request, responseStr);
}

/**
 * @brief Handle POST /api/battery endpoint
 */
void WebServerManager::handleSetBattery(AsyncWebServerRequest* request) {
    DynamicJsonDocument doc(1024);
    
    if (!validateJSONRequest(request, doc)) {
        sendErrorResponse(request, "Invalid JSON request");
        return;
    }
    
    bool success = true;
    String message = "Battery parameters updated";
    
    // Handle operating mode change
    if (doc.containsKey("operatingMode")) {
        String modeStr = doc["operatingMode"];
        OperatingMode mode = (modeStr == "manual") ? OperatingMode::MANUAL_CONTROLLED : OperatingMode::BMS_CONTROLLED;
        
        if (operatingModeCallback && !operatingModeCallback(mode)) {
            success = false;
            message = "Failed to set operating mode";
        }
    }
    
    // Handle manual current limit change (JSON body)
    if (doc.containsKey("manualCurrentLimit")) {
        float current = doc["manualCurrentLimit"].as<float>();
        if (manualCurrentCallback) {
            success = manualCurrentCallback(current);
            if (!success) message = "Failed to set manual current limit";
        }
    }

    // Handle voltage drop compensation (JSON body)
    if (doc.containsKey("voltageDropCompensation")) {
        float compensation = doc["voltageDropCompensation"].as<float>();
        if (voltageCompensationCallback) {
            success = voltageCompensationCallback(compensation);
            if (!success) message = "Failed to set voltage drop compensation";
        }
    }

    // Handle voltage calibration offset (JSON body)
    if (doc.containsKey("voltageCalibrationOffset")) {
        float offsetV = doc["voltageCalibrationOffset"].as<float>();
        if (voltageCalibrationCallback) {
            success = voltageCalibrationCallback(offsetV);
            if (!success) message = "Failed to set voltage calibration offset";
        }
    }

    // Handle disable current limit flag (dangerous)
    if (doc.containsKey("disableCurrentLimit")) {
        bool disabled = doc["disableCurrentLimit"];
        if (disableCurrentLimitCallback) {
            bool ok = disableCurrentLimitCallback(disabled);
            success = success && ok;
            if (!ok) message = "Failed to set disable current limit";
        }
    }

    // Handle default per-PSU fallback voltage
    if (doc.containsKey("defaultPerPsuVoltage")) {
        float volts = doc["defaultPerPsuVoltage"].as<float>();
        if (defaultPerPsuVoltageCallback) {
            bool ok = defaultPerPsuVoltageCallback(volts);
            success = success && ok;
            if (!ok) message = "Failed to set default per-PSU voltage";
        }
    }

    // Handle AC preset selection (persist on device)
    if (doc.containsKey("acPreset")) {
        uint8_t presetId = doc["acPreset"].as<uint8_t>();
        if (acPresetCallback) {
            bool ok = acPresetCallback(presetId);
            success = success && ok;
            if (!ok) message = "Failed to set AC preset";
        }
    }

    // Handle Max Cell Voltage (Volts per cell)
    if (doc.containsKey("maxCellVoltage")) {
        float cellV = doc["maxCellVoltage"].as<float>();
        if (maxCellVoltageCallback) {
            bool ok = maxCellVoltageCallback(cellV);
            success = success && ok;
            if (!ok) message = "Failed to set max cell voltage";
        }
    }

    DynamicJsonDocument response(512);
    response["success"] = success;
    response["message"] = message;
    
    String responseStr;
    serializeJson(response, responseStr);
    sendJSONResponse(request, responseStr);
}

/**
 * @brief Handle GET /api/wifi endpoint
 */
void WebServerManager::handleGetWiFiConfig(AsyncWebServerRequest* request) {
    DynamicJsonDocument doc(1024);
    doc["ssid"] = clientConfig.ssid;
    doc["enabled"] = clientConfig.enabled;
    doc["connected"] = clientConnected;
    doc["ap_mode"] = apStarted;
    doc["ip"] = clientConnected ? WiFi.localIP().toString() : "";
    doc["ap_ip"] = apStarted ? WiFi.softAPIP().toString() : "";
    
    String json;
    serializeJson(doc, json);
    sendJSONResponse(request, json);
}

/**
 * @brief Handle POST /api/wifi endpoint
 */
void WebServerManager::handleSetWiFiConfig(AsyncWebServerRequest* request) {
    DynamicJsonDocument doc(1024);
    
    if (!validateJSONRequest(request, doc)) {
        sendErrorResponse(request, "Invalid JSON request");
        return;
    }
    
    if (!doc.containsKey("ssid")) {
        sendErrorResponse(request, "Missing SSID parameter");
        return;
    }
    
    String ssid = doc["ssid"];
    String password = doc.containsKey("password") ? doc["password"].as<String>() : "";
    
    bool success = connectToWiFi(ssid, password, true);
    
    DynamicJsonDocument response(512);
    response["success"] = success;
    response["message"] = success ? "WiFi configuration saved" : "Failed to save WiFi config";
    
    String responseStr;
    serializeJson(response, responseStr);
    sendJSONResponse(request, responseStr);
}

/**
 * @brief Handle GET /api/system endpoint
 */
void WebServerManager::handleGetSystemInfo(AsyncWebServerRequest* request) {
    String json = generateSystemInfoJSON();
    sendJSONResponse(request, json);
}

/**
 * @brief Handle POST /api/restart endpoint
 */
void WebServerManager::handleRestart(AsyncWebServerRequest* request) {
    DynamicJsonDocument response(512);
    response["success"] = true;
    response["message"] = "System will restart in 3 seconds";
    
    String responseStr;
    serializeJson(response, responseStr);
    sendJSONResponse(request, responseStr);
    
    // Restart after a short delay
    delay(1000);
    ESP.restart();
}

/**
 * @brief Generate status JSON
 */
String WebServerManager::generateStatusJSON() const {
    DynamicJsonDocument doc(1024);
    
    // Flatpack data
    JsonArray flatpacks = doc.createNestedArray("flatpacks");
    uint16_t totalCurrent = 0;
    uint16_t avgVoltage = 0;
    uint8_t activePsuCount = 0;
    
    if (flatpackDataCallback) {
        const auto& flatpackData = flatpackDataCallback();
        for (const auto& fp : flatpackData) {
            JsonObject fpObj = flatpacks.createNestedObject();
            fpObj["serial"] = fp.serialStr;
            fpObj["loggedIn"] = fp.loggedIn;
            fpObj["status"] = fp.status;
            fpObj["can_bus"] = fp.canBusId;
            fpObj["temp_intake"] = fp.intakeTemp;
            fpObj["temp_exhaust"] = fp.exhaustTemp;

            // Raw units from device
            fpObj["voltage_cv"] = fp.outputVoltage;           // centivolts
            fpObj["current_dA"] = fp.outputCurrent;           // deciamps
            fpObj["inputVoltage"] = fp.inputVoltage;          // as reported
            fpObj["setVoltage_cv"] = fp.setVoltage;
            fpObj["setCurrent_dA"] = fp.setCurrent;
            fpObj["setOvp_cv"] = fp.setOvp;

            // Scaled values for UI
            fpObj["outputVoltage"] = fp.outputVoltage / 100.0f; // Volts
            fpObj["outputCurrent"] = fp.outputCurrent / 10.0f;  // Amps
            fpObj["temperature"] = fp.exhaustTemp;              // °C
            
            if (fp.outputCurrent > 0) {
                totalCurrent += fp.outputCurrent;
                avgVoltage += fp.outputVoltage;
                activePsuCount++;
            }
        }
    }
    
    if (activePsuCount > 0) {
        avgVoltage /= activePsuCount;
    }
    
    doc["totalCurrent"] = totalCurrent;
    doc["avgVoltage"] = avgVoltage;
    doc["activePsuCount"] = activePsuCount;
    
    // Battery data
    JsonObject battery = doc.createNestedObject("battery");
    if (batteryStatusCallback) {
        const auto& batteryStatus = batteryStatusCallback();
        battery["packVoltage"] = batteryStatus.packVoltage;
        battery["packCurrent"] = batteryStatus.packCurrent;
        battery["temperature"] = batteryStatus.temperature;
        battery["mode"] = static_cast<int>(batteryStatus.mode);
        battery["isCharging"] = batteryStatus.isCharging;
        battery["stateOfCharge"] = batteryStatus.stateOfCharge;
    }
    
    // WiFi data
    JsonObject wifi = doc.createNestedObject("wifi");
    wifi["status"] = static_cast<int>(currentStatus);
    wifi["ip"] = clientConnected ? WiFi.localIP().toString() : "";
    wifi["ap_ip"] = apStarted ? WiFi.softAPIP().toString() : "";
    wifi["ssid"] = clientConnected ? WiFi.SSID() : "";
    
    String json;
    serializeJson(doc, json);
    return json;
}

/**
 * @brief Generate flatpacks JSON
 */
String WebServerManager::generateFlatpacksJSON() const {
    DynamicJsonDocument doc(1024);
    JsonArray flatpacks = doc.createNestedArray("flatpacks");
    
    if (flatpackDataCallback) {
        const auto& flatpackData = flatpackDataCallback();
        for (const auto& fp : flatpackData) {
            JsonObject fpObj = flatpacks.createNestedObject();
            fpObj["serial"] = fp.serialStr;
            fpObj["detected"] = fp.detected;
            fpObj["loggedIn"] = fp.loggedIn;
            fpObj["canBusId"] = fp.canBusId;
            fpObj["psuId"] = fp.psuId;
            fpObj["status"] = fp.status;
            fpObj["intakeTemp"] = fp.intakeTemp;
            fpObj["exhaustTemp"] = fp.exhaustTemp;
            fpObj["inputVoltage"] = fp.inputVoltage;
            fpObj["outputVoltage"] = fp.outputVoltage;
            fpObj["outputCurrent"] = fp.outputCurrent;
            fpObj["setVoltage"] = fp.setVoltage;
            fpObj["setCurrent"] = fp.setCurrent;
            fpObj["setOvp"] = fp.setOvp;
            fpObj["hasAlerts"] = fp.hasAlerts;
            fpObj["lastStatusTime"] = fp.lastStatusTime;
        }
    }
    
    String json;
    serializeJson(doc, json);
    return json;
}

/**
 * @brief Generate battery JSON
 */
String WebServerManager::generateBatteryJSON() const {
    DynamicJsonDocument doc(1024);
    
    if (batteryStatusCallback) {
        const auto& status = batteryStatusCallback();
        doc["packVoltage"] = status.packVoltage;
        doc["packCurrent"] = status.packCurrent;
        doc["cellVoltageAvg"] = status.cellVoltageAvg;
        doc["cellVoltageMin"] = status.cellVoltageMin;
        doc["cellVoltageMax"] = status.cellVoltageMax;
        doc["voltageDelta"] = status.voltageDelta; // mV
        doc["temperature"] = status.temperature;
        doc["tempMin"] = status.temperatureMin;
        doc["tempMax"] = status.temperatureMax;
        doc["mode"] = static_cast<int>(status.mode);
        doc["stateOfCharge"] = status.stateOfCharge;
        doc["chargingTimeMin"] = status.chargingTimeMin;
        doc["isCharging"] = status.isCharging;
        doc["isError"] = status.isError;
        doc["errorFlags"] = status.errorFlags;
        doc["operatingMode"] = (status.operatingMode == OperatingMode::MANUAL_CONTROLLED) ? "manual" : "bms";
        // Data source for UI awareness (manual / Cree LTO / VX1)
        switch (status.dataSource) {
            case BatterySource::MANUAL: doc["dataSource"] = "manual"; break;
            case BatterySource::CREE_LTO: doc["dataSource"] = "cree_lto"; break;
            case BatterySource::VECTRIX_VX1: doc["dataSource"] = "vx1"; break;
        }
        doc["manualCurrentLimit"] = status.manualCurrentLimit;
        doc["voltageDropCompensation"] = status.voltageDropCompensation;
        doc["disableCurrentLimit"] = status.disableCurrentLimit;
        doc["defaultPerPsuVoltage"] = status.defaultPerPsuVoltage;
        doc["acPreset"] = status.acPresetId;
    }
    
    if (batteryParametersCallback) {
        const auto& params = batteryParametersCallback();
        JsonObject parameters = doc.createNestedObject("parameters");
        parameters["chemistry"] = static_cast<int>(params.chemistry);
        parameters["cellCount"] = params.cellCount;
        parameters["cellVoltageMin"] = params.cellVoltageMin;
        parameters["cellVoltageNominal"] = params.cellVoltageNominal;
        parameters["cellVoltageMax"] = params.cellVoltageMax;
        parameters["cellVoltageFloat"] = params.cellVoltageFloat;
        parameters["currentMax"] = params.currentMax;
        parameters["currentTaper"] = params.currentTaper;
        parameters["currentFloat"] = params.currentFloat;
        parameters["tempMin"] = params.tempMin;
        parameters["tempMax"] = params.tempMax;
    }
    
    String json;
    serializeJson(doc, json);
    return json;
}

/**
 * @brief Generate system info JSON
 */
String WebServerManager::generateSystemInfoJSON() const {
    DynamicJsonDocument doc(1024);
    
    doc["chipModel"] = ESP.getChipModel();
    doc["chipRevision"] = ESP.getChipRevision();
    doc["cpuFreqMHz"] = ESP.getCpuFreqMHz();
    doc["freeHeap"] = ESP.getFreeHeap();
    doc["totalHeap"] = ESP.getHeapSize();
    doc["uptime"] = millis();
    doc["flashSize"] = ESP.getFlashChipSize();
    doc["sketchSize"] = ESP.getSketchSize();
    doc["freeSketchSpace"] = ESP.getFreeSketchSpace();
    // WiFi info for UI
    doc["wifiStatus"] = static_cast<int>(currentStatus);
    doc["ssid"] = clientConnected ? WiFi.SSID() : "";
    doc["ip"] = clientConnected ? WiFi.localIP().toString() : "";
    doc["apIp"] = apStarted ? WiFi.softAPIP().toString() : "";
    // TWAI RX statistics for UI footer
    doc["twaiRxRate"] = getTwaiRxRate();
    doc["twaiRxCount"] = getTwaiRxCount();
    
    String json;
    serializeJson(doc, json);
    return json;
}

/**
 * @brief Send JSON response
 */
void WebServerManager::sendJSONResponse(AsyncWebServerRequest* request, const String& json, int statusCode) {
    request->send(statusCode, "application/json", json);
}

/**
 * @brief Send error response
 */
void WebServerManager::sendErrorResponse(AsyncWebServerRequest* request, const String& error, int statusCode) {
    DynamicJsonDocument doc(1024);
    doc["error"] = error;
    doc["success"] = false;
    
    String json;
    serializeJson(doc, json);
    sendJSONResponse(request, json, statusCode);
}

/**
 * @brief Validate JSON request
 */
bool WebServerManager::validateJSONRequest(AsyncWebServerRequest* request, DynamicJsonDocument& doc) {
    if (!request->hasParam("plain", true)) {
        return false;
    }
    
    String body = request->getParam("plain", true)->value();
    DeserializationError error = deserializeJson(doc, body);
    
    return error == DeserializationError::Ok;
}

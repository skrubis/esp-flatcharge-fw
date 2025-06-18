#include "webserver.h"

WebServerManager::WebServerManager() : _server(80) {}

bool WebServerManager::begin() {
    // Initialize SPIFFS to serve web files
    if (!SPIFFS.begin(true)) {
        Serial.println("Error initializing SPIFFS");
        return false;
    }
    
    // Initialize the OTA manager
    _otaManager.begin(&_server);
    
    // Setup routes for web interface
    _setupRoutes();
    
    // Start the server
    _server.begin();
    Serial.println("Web server started on port 80");
    
    return true;
}

void WebServerManager::update() {
    // Nothing to do here, as ESPAsyncWebServer handles requests asynchronously
    // This method is provided for consistency with other modules
}

void WebServerManager::setSettingsManager(Settings* settingsManager) {
    _settingsManager = settingsManager;
}

void WebServerManager::setFlatpackControllers(FlatpackController** flatpackControllers, int count) {
    _flatpackControllers = flatpackControllers;
    _flatpackCount = count;
}

void WebServerManager::setType2Controller(Type2Controller* type2Controller) {
    _type2Controller = type2Controller;
}

void WebServerManager::_setupRoutes() {
    // Setup routes for web UI
    _server.on("/", HTTP_GET, [this](AsyncWebServerRequest *request) {
        request->send(SPIFFS, "/index.html", "text/html");
    });
    
    // Route to serve CSS files
    _server.on("/style.css", HTTP_GET, [](AsyncWebServerRequest *request) {
        request->send(SPIFFS, "/style.css", "text/css");
    });
    
    // Route to serve JavaScript files
    _server.on("/script.js", HTTP_GET, [](AsyncWebServerRequest *request) {
        request->send(SPIFFS, "/script.js", "application/javascript");
    });
    
    // API route to get current status
    _server.on("/api/status", HTTP_GET, [this](AsyncWebServerRequest *request) {
        DynamicJsonDocument doc(1024);
        
        // Add charger settings
        if (_settingsManager) {
            ChargingSettings& settings = _settingsManager->getSettings();
            doc["chargingEnabled"] = settings.chargingEnabled;
            doc["maxCurrentLimit"] = settings.maxCurrentLimit;
            doc["bmsMode"] = static_cast<int>(settings.bmsMode);
            doc["targetVoltage"] = settings.targetVoltage;
            doc["targetCurrent"] = settings.targetCurrent;
            doc["flatpackConfig"] = static_cast<int>(settings.flatpackConfig);
            doc["useThreePhase"] = settings.useThreePhase;
            doc["cellCount"] = settings.cellCount;
            doc["cellTargetVoltage"] = settings.cellTargetVoltage;
        }
        
        // Add battery data
        JsonObject battery = doc.createNestedObject("battery");
        battery["voltage"] = 0.0; // Placeholder for actual battery voltage
        battery["temperatureMax"] = 0.0; // Placeholder for battery temperature
        battery["currentIn"] = 0.0; // Placeholder for current into battery
        
        // Add Flatpack data
        JsonArray flatpacks = doc.createNestedArray("flatpacks");
        for (uint8_t i = 0; i < _flatpackCount; i++) {
            if (_flatpackControllers && _flatpackControllers[i]) {
                FlatpackStatus status = _flatpackControllers[i]->getStatus();
                JsonObject fp = flatpacks.createNestedObject();
                fp["id"] = i + 1;
                fp["voltage"] = status.voltageOutput;
                fp["current"] = status.currentOutput;
                fp["temperature1"] = status.temperature1;
                fp["temperature2"] = status.temperature2;
                fp["connected"] = status.connected;
                fp["loggedIn"] = status.loggedIn;
                fp["isCurrentLimiting"] = status.isCurrentLimiting;
                fp["isWalkingIn"] = status.isWalkingIn;
                fp["isError"] = status.isError;
            }
        }
        
        // Add Type2 data
        if (_type2Controller) {
            JsonObject type2 = doc.createNestedObject("type2");
            type2["state"] = static_cast<int>(_type2Controller->readEVState());
            type2["allowedCurrent"] = _type2Controller->readAllowedCurrent();
        }
        
        // Send the JSON response
        String response;
        serializeJson(doc, response);
        request->send(200, "application/json", response);
    });
    
    // API route to update settings
    _server.on("/api/settings", HTTP_POST, [this](AsyncWebServerRequest *request) {
        // This needs to be empty, the handling happens in the body handler
        // This is a quirk of ESPAsyncWebServer
    }, NULL, [this](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total) {
        if (!_settingsManager) {
            request->send(500, "text/plain", "Settings manager not initialized");
            return;
        }
        
        // Parse the JSON body
        DynamicJsonDocument doc(1024);
        DeserializationError error = deserializeJson(doc, data, len);
        
        if (error) {
            request->send(400, "text/plain", "Invalid JSON");
            return;
        }
        
        // Update settings based on JSON
        ChargingSettings& settings = _settingsManager->getSettings();
        
        // Only update fields that are present in the request
        if (!doc["chargingEnabled"].isNull())
            settings.chargingEnabled = doc["chargingEnabled"];
        
        if (!doc["maxCurrentLimit"].isNull())
            settings.maxCurrentLimit = doc["maxCurrentLimit"];
        
        if (!doc["bmsMode"].isNull())
            settings.bmsMode = static_cast<BMSMode>(doc["bmsMode"].as<int>());
        
        if (!doc["targetVoltage"].isNull())
            settings.targetVoltage = doc["targetVoltage"];
        
        if (!doc["targetCurrent"].isNull())
            settings.targetCurrent = doc["targetCurrent"];
        
        if (!doc["flatpackConfig"].isNull())
            settings.flatpackConfig = static_cast<FlatpackConfig>(doc["flatpackConfig"].as<int>());
        
        if (!doc["useThreePhase"].isNull())
            settings.useThreePhase = doc["useThreePhase"];
        
        if (!doc["cellCount"].isNull())
            settings.cellCount = doc["cellCount"];
        
        if (!doc["cellTargetVoltage"].isNull())
            settings.cellTargetVoltage = doc["cellTargetVoltage"];
        
        // Save the updated settings
        if (_settingsManager->save()) {
            request->send(200, "text/plain", "Settings updated");
        } else {
            request->send(500, "text/plain", "Failed to save settings");
        }
    });
    
    // Setup a 404 handler
    _server.onNotFound([](AsyncWebServerRequest *request) {
        request->send(404, "text/plain", "Not found");
    });
}

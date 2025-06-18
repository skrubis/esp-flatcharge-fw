#pragma once

#include <Arduino.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
#include <ArduinoJson.h>
#include <SPIFFS.h>
#include "config.h"
#include "settings.h"
#include "flatpack.h"
#include "type2.h"
#include "ota.h"

class WebServerManager {
public:
    WebServerManager();
    
    /**
     * Initialize the web server
     * Sets up routes and starts the server
     */
    bool begin();
    
    /**
     * Update function called in the main loop
     */
    void update();
    
    /**
     * Set the settings manager
     */
    void setSettingsManager(Settings* settingsManager);
    
    /**
     * Set the type2 controller
     */
    void setType2Controller(Type2Controller* type2Controller);
    
    /**
     * Set the flatpack controllers
     */
    void setFlatpackControllers(FlatpackController** controllers, int count);
    
    /**
     * Returns the uptime in human readable format
     */
    String getUptimeString();
    
private:
    AsyncWebServer _server;
    bool _started = false;
    OTAManager _otaManager;
    
    FlatpackController** _flatpackControllers = nullptr;
    int _flatpackCount = 0;
    Type2Controller* _type2Controller = nullptr;
    Settings* _settingsManager = nullptr;
    
    void _setupRoutes();
    void _setupApiRoutes();
    
    // API Handlers
    void _handleGetStatus(AsyncWebServerRequest *request);
    void _handleSetCharging(AsyncWebServerRequest *request);
    void _handleSetTargetValues(AsyncWebServerRequest *request);
    void _handleSaveSettings(AsyncWebServerRequest *request, JsonDocument &json);
    void _handleSetDefaultVoltage(AsyncWebServerRequest *request, JsonDocument &json);
    
    // Utility functions
    String _getContentType(String filename);
};

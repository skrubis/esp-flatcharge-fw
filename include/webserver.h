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

class WebServerController {
public:
    WebServerController();
    
    /**
     * Initialize the web server
     * Sets up routes and starts the server
     */
    bool begin(FlatpackController* fp1, FlatpackController* fp2, FlatpackController* fp3, 
               Type2Controller* type2, Settings* settings);
    
    /**
     * Initialize OTA updates through the web interface
     */
    void setupOTA();
    
    /**
     * Returns the uptime in human readable format
     */
    String getUptimeString();
    
    /**
     * Process client requests and handle OTA if active
     */
    void update();
    
private:
    AsyncWebServer _server = AsyncWebServer(HTTP_PORT);
    bool _started = false;
    
    FlatpackController* _flatpack1;
    FlatpackController* _flatpack2;
    FlatpackController* _flatpack3;
    Type2Controller* _type2;
    Settings* _settings;
    
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

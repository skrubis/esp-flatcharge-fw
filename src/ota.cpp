#include "ota.h"

OTAManager::OTAManager() : _isUpdating(false) {}

bool OTAManager::begin(AsyncWebServer* server) {
    // Setup routes for OTA updates
    _setupRoutes(server);
    return true;
}

bool OTAManager::isUpdating() const {
    return _isUpdating;
}

void OTAManager::_setupRoutes(AsyncWebServer* server) {
    // Route for handling firmware upload
    server->on("/update", HTTP_POST, [this](AsyncWebServerRequest *request) {
        bool shouldReboot = !_isUpdating;
        _isUpdating = false;
        
        AsyncWebServerResponse *response = request->beginResponse(200, "text/plain", shouldReboot ? "OK" : "FAIL");
        response->addHeader("Connection", "close");
        response->addHeader("Access-Control-Allow-Origin", "*");
        request->send(response);
        
        if (shouldReboot) {
            delay(500);
            ESP.restart();
        }
    },
    [this](AsyncWebServerRequest *request, String filename, size_t index, uint8_t *data, size_t len, bool final) {
        if (!index) { // First chunk, start update
            Serial.printf("OTA Update Start: %s\n", filename.c_str());
            
            // Check if firmware update or filesystem update
            int cmd = (filename == "filesystem.bin") ? U_SPIFFS : U_FLASH;
            if (!Update.begin(UPDATE_SIZE_UNKNOWN, cmd)) {
                _isUpdating = false;
                Update.printError(Serial);
                return request->send(400, "text/plain", "OTA could not begin");
            }
            _isUpdating = true;
        }
        
        // Write chunk
        if (_isUpdating && Update.write(data, len) != len) {
            _isUpdating = false;
            Update.printError(Serial);
            return request->send(400, "text/plain", "OTA write failed");
        }
        
        if (final) { // Final chunk, end update
            if (_isUpdating && Update.end(true)) {
                Serial.println("OTA update complete");
                _isUpdating = true; // Keep true to signal we should reboot
            } else {
                _isUpdating = false;
                Update.printError(Serial);
                return request->send(400, "text/plain", "OTA end failed");
            }
        }
    });
    
    // Route to serve the OTA update form
    server->on("/firmware", HTTP_GET, [](AsyncWebServerRequest *request) {
        request->send(SPIFFS, "/firmware.html", "text/html");
    });
}

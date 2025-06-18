#ifndef OTA_H
#define OTA_H

#include <Arduino.h>
#include <Update.h>
#include <ESPAsyncWebServer.h>
#include <SPIFFS.h>

class OTAManager {
public:
    OTAManager();
    
    // Initialize the OTA manager
    bool begin(AsyncWebServer* server);
    
    // Handle OTA update status
    bool isUpdating() const;
    
private:
    bool _isUpdating;
    
    // Setup web routes for OTA updates
    void _setupRoutes(AsyncWebServer* server);
};

#endif // OTA_H

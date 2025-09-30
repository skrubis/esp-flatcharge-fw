#pragma once

#include <Arduino.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>
#include <Preferences.h>
#include <functional>
#include "FlatpackManager.h"
#include "BatteryManager.h"

/**
 * @brief WiFi connection status
 */
enum class WiFiStatus {
    DISCONNECTED,
    CONNECTING,
    CONNECTED,
    AP_MODE,
    DUAL_MODE,
    ERROR
};

/**
 * @brief WiFi configuration structure
 */
struct WiFiConfig {
    String ssid;
    String password;
    bool enabled;
    bool dhcp;
    String staticIP;
    String gateway;
    String subnet;
    String dns1;
    String dns2;
};

/**
 * @brief Web server configuration
 */
struct WebServerConfig {
    uint16_t port;
    bool enableCORS;
    bool enableAuth;
    String username;
    String password;
    uint32_t sessionTimeout;
};

/**
 * @brief Manager for WiFi connectivity and web server
 * 
 * Handles:
 * - WiFi Access Point mode (SSID: FLATCHARGE, password: flatcharge!)
 * - WiFi Client mode with persistent credential storage
 * - Dual mode operation (AP + Client simultaneously)
 * - Async web server with REST API endpoints
 * - Real-time monitoring dashboard
 * - Configuration interface
 * - JSON API for flatpack and battery data
 */
class WebServerManager {
public:
    // Callback function types
    using FlatpackDataCallback = std::function<const std::vector<FlatpackData>&()>;
    using BatteryStatusCallback = std::function<const BatteryStatus&()>;
    using BatteryParametersCallback = std::function<const BatteryParameters&()>;
    using SetChargingParametersCallback = std::function<bool(uint16_t voltage, uint16_t current, uint16_t ovp)>;
    using SetBatteryParametersCallback = std::function<bool(const BatteryParameters& params)>;
    using SetOperatingModeCallback = std::function<bool(OperatingMode mode)>;
    using SetManualCurrentCallback = std::function<bool(float current)>;
    using SetVoltageCompensationCallback = std::function<bool(float compensation)>;
    using SetChargingEnabledCallback = std::function<bool(bool enabled)>;
    using SetVoltageCalibrationCallback = std::function<bool(float offsetV)>;
    using SetDisableCurrentLimitCallback = std::function<bool(bool disabled)>;
    using SetDefaultPerPsuVoltageCallback = std::function<bool(float volts)>;
    using SetAcPresetCallback = std::function<bool(uint8_t presetId)>;
    using SetMaxCellVoltageCallback = std::function<bool(float cellV)>; // Volts per cell

    // ADS1220 current sensor callbacks
    using ADSGetCallback = std::function<void(float& currentA, bool& valid, float& zeroV, float& apv)>;
    using ADSCalZeroCallback = std::function<bool(uint16_t avgSamples)>;
    using ADSSetScaleCallback = std::function<bool(float apv)>;

    // Custom JSON providers (for platform-specific data like Gree LTO per-cell)
    using CustomJsonCallback = std::function<String()>;

    WebServerManager();
    ~WebServerManager();

    /**
     * @brief Initialize the web server manager
     * 
     * @param port Web server port (default: 80)
     * @return true if initialization successful
     */
    bool initialize(uint16_t port = 80);

    /**
     * @brief Start WiFi Access Point mode
     * 
     * @param ssid AP SSID (default: "FLATCHARGE")
     * @param password AP password (default: "flatcharge!")
     * @return true if AP started successfully
     */
    bool startAccessPoint(const String& ssid = "FLATCHARGE", const String& password = "flatcharge!");

    /**
     * @brief Connect to WiFi network as client
     * 
     * @param ssid Network SSID
     * @param password Network password
     * @param persistent Save credentials for future use
     * @return true if connection initiated successfully
     */
    bool connectToWiFi(const String& ssid, const String& password, bool persistent = true);

    /**
     * @brief Start dual mode (AP + Client)
     * 
     * @param clientSSID Client network SSID
     * @param clientPassword Client network password
     * @return true if dual mode started successfully
     */
    bool startDualMode(const String& clientSSID, const String& clientPassword);

    /**
     * @brief Stop WiFi and web server
     */
    void stop();

    /**
     * @brief Update function (call in main loop)
     * Handles WiFi reconnection and status monitoring
     */
    void update();

    /**
     * @brief Get current WiFi status
     * 
     * @return WiFiStatus enum value
     */
    WiFiStatus getWiFiStatus() const;

    /**
     * @brief Get WiFi connection info
     * 
     * @param info String to store connection information
     */
    void getConnectionInfo(String& info) const;

    /**
     * @brief Set callback for getting flatpack data
     * 
     * @param callback Function that returns flatpack data vector
     */
    void setFlatpackDataCallback(FlatpackDataCallback callback);

    /**
     * @brief Set callback for getting battery status
     * 
     * @param callback Function that returns battery status
     */
    void setBatteryStatusCallback(BatteryStatusCallback callback);

    /**
     * @brief Set callback for getting battery parameters
     * 
     * @param callback Function that returns battery parameters
     */
    void setBatteryParametersCallback(BatteryParametersCallback callback);

    /**
     * @brief Set callback for setting charging parameters
     * 
     * @param callback Function to set charging parameters
     */
    void setChargingParametersCallback(SetChargingParametersCallback callback);

    /**
     * @brief Set callback for setting battery parameters
     * 
     * @param callback Function to set battery parameters
     */
    void setBatteryParametersCallback(SetBatteryParametersCallback callback);

    /**
     * @brief Set callback for setting operating mode
     * 
     * @param callback Function to set operating mode
     */
    void setOperatingModeCallback(SetOperatingModeCallback callback);

    /**
     * @brief Set callback for setting manual current limit
     * 
     * @param callback Function to set manual current limit
     */
    void setManualCurrentCallback(SetManualCurrentCallback callback);

    /**
     * @brief Set callback for setting voltage drop compensation
     * 
     * @param callback Function to set voltage compensation
     */
    void setVoltageCompensationCallback(SetVoltageCompensationCallback callback);

    /**
     * @brief Set callback for enabling/disabling charging
     * 
     * @param callback Function to control charging state
     */
    void setChargingEnabledCallback(SetChargingEnabledCallback callback);

    /**
     * @brief Set callback for setting voltage calibration offset
     *
     * @param callback Function to set calibration offset (Volts)
     */
    void setVoltageCalibrationCallback(SetVoltageCalibrationCallback callback);
    void setDisableCurrentLimitCallback(SetDisableCurrentLimitCallback callback);
    void setDefaultPerPsuVoltageCallback(SetDefaultPerPsuVoltageCallback callback);
    void setAcPresetCallback(SetAcPresetCallback callback);
    void setMaxCellVoltageCallback(SetMaxCellVoltageCallback callback);

    // ADS1220 hooks
    void setAdsGetCallback(ADSGetCallback cb);
    void setAdsCalZeroCallback(ADSCalZeroCallback cb);
    void setAdsSetScaleCallback(ADSSetScaleCallback cb);
    void setGreeJsonCallback(CustomJsonCallback cb);

    /**
     * @brief Get web server URL
     * 
     * @return String containing the web server URL
     */
    String getWebServerURL() const;

    /**
     * @brief Print status information to Serial
     */
    void printStatus() const;

private:
    AsyncWebServer* server;
    Preferences preferences;
    WiFiConfig clientConfig;
    WebServerConfig webConfig;
    WiFiStatus currentStatus;
    unsigned long lastWiFiCheck;
    unsigned long lastReconnectAttempt;
    bool apStarted;
    bool clientConnected;
    bool serverStarted;

    // Callbacks
    FlatpackDataCallback flatpackDataCallback;
    BatteryStatusCallback batteryStatusCallback;
    BatteryParametersCallback batteryParametersCallback;
    SetChargingParametersCallback chargingParametersCallback;
    SetBatteryParametersCallback setBatteryParamsCallback;
    SetOperatingModeCallback operatingModeCallback;
    SetManualCurrentCallback manualCurrentCallback;
    SetVoltageCompensationCallback voltageCompensationCallback;
    SetChargingEnabledCallback chargingEnabledCallback;
    SetVoltageCalibrationCallback voltageCalibrationCallback;
    SetDisableCurrentLimitCallback disableCurrentLimitCallback;
    SetDefaultPerPsuVoltageCallback defaultPerPsuVoltageCallback;
    SetAcPresetCallback acPresetCallback;
    SetMaxCellVoltageCallback maxCellVoltageCallback;

    // ADS1220
    ADSGetCallback adsGetCallback;
    ADSCalZeroCallback adsCalZeroCallback;
    ADSSetScaleCallback adsSetScaleCallback;

    // Constants
    static const uint32_t WIFI_CHECK_INTERVAL = 5000;      // Check WiFi every 5 seconds
    static const uint32_t RECONNECT_INTERVAL = 30000;      // Reconnect attempt every 30 seconds
    static const uint32_t CONNECTION_TIMEOUT = 20000;      // Connection timeout 20 seconds
    static const char* PREFS_NAMESPACE;
    static const char* PREFS_SSID_KEY;
    static const char* PREFS_PASSWORD_KEY;
    static const char* PREFS_ENABLED_KEY;

    // WiFi management
    void loadWiFiConfig();
    void saveWiFiConfig();
    void handleWiFiEvents();
    void attemptReconnection();

    // Web server setup
    void setupWebServer();
    void setupAPIRoutes();
    void setupWebInterface();
    
    // API endpoint handlers
    void handleGetStatus(AsyncWebServerRequest* request);
    void handleGetFlatpacks(AsyncWebServerRequest* request);
    void handleGetBattery(AsyncWebServerRequest* request);
    void handleSetCharging(AsyncWebServerRequest* request);
    void handleSetBattery(AsyncWebServerRequest* request);
    void handleGetWiFiConfig(AsyncWebServerRequest* request);
    void handleSetWiFiConfig(AsyncWebServerRequest* request);
    void handleGetSystemInfo(AsyncWebServerRequest* request);
    void handleRestart(AsyncWebServerRequest* request);
    void handleGetADS1220(AsyncWebServerRequest* request);
    void handleGetGree(AsyncWebServerRequest* request);

    // Utility functions
    String generateStatusJSON() const;
    String generateFlatpacksJSON() const;
    String generateBatteryJSON() const;
    String generateSystemInfoJSON() const;
    String generateADS1220JSON() const;
    void sendJSONResponse(AsyncWebServerRequest* request, const String& json, int statusCode = 200);
    void sendErrorResponse(AsyncWebServerRequest* request, const String& error, int statusCode = 400);
    bool validateJSONRequest(AsyncWebServerRequest* request, DynamicJsonDocument& doc);

    // Custom JSON providers
    CustomJsonCallback greeJsonCallback;
};

#pragma once

#include <Arduino.h>
#include <SPI.h>
#include <Preferences.h>

/**
 * ADS1220Manager - Simple SPI driver for ADS1220 24-bit ADC for Hall current sensor.
 * - Shared SPI bus with MCP2515 (SCK=12, MISO=13, MOSI=11).
 * - CS pin default: GPIO39, DRDY pin optional: GPIO40.
 * - Differential AIN0-AIN1, internal 2.048V ref, PGA bypass, Gain=1.
 * - Provides bipolar current reading with zero and scale calibration.
 */
class ADS1220Manager {
    public:
    struct Config {
        int csPin = 39;      // GPIO39
        int drdyPin = 40;    // GPIO40 (optional). If <0, poll.
        float vref = 2.048f; // internal reference
        uint8_t gain = 1;    // 1x
        uint16_t sampleRateSPS = 20; // 20/45/90 typical
    };

    ADS1220Manager();
    ~ADS1220Manager();

    // Begin with explicit config; default overload provided below avoids GCC nested default-arg issue
    bool begin(const Config& cfg);
    // Convenience overload: begin with default Config()
    bool begin() { return begin(Config()); }
    void end();

    // Update periodically; reads one sample (non-blocking if DRDY high)
    void update();

    // Current in Amperes (filtered). Returns NAN if not valid yet.
    float getCurrentA() const;

    // Calibration
    bool calibrateZero(uint16_t avgSamples = 32);
    bool setScaleAmpsPerVolt(float ampsPerVolt);
    float getScaleAmpsPerVolt() const { return scaleAmpsPerVolt; }
    float getZeroOffsetVolts() const { return zeroOffsetV; }
    float getEffectiveGain() const { return effectiveGain; }
    float getLastDiffVolts() const { return lastVolts; }

    // Persistence
    bool loadCalibration();
    bool saveCalibration();
    // One-shot diagnostic: measure AIN0-AVSS and AIN1-AVSS and log absolute voltages
    void diagnosticReadInputs();

    // Validity and timestamp
    bool isValid(uint32_t maxAgeMs = 5000) const;

private:
        Config cfg;
        bool initialized;
        SPISettings spiSettings;

        // Calibration
        float zeroOffsetV;        // Volts offset at 0A
        float scaleAmpsPerVolt;   // A/V conversion
        float effectiveGain = 1.0f; // actual PGA gain (1 if bypassed)

        // Filtering
        bool filtInit;
        float filtCurrentA;
        float lastVolts = NAN;    // last input-referred differential volts (AIN0-AIN1)

        // Timestamp
        uint32_t lastUpdateMs; // for validity checks
        uint32_t lastPollUs;   // for polling cadence when DRDY not used

        // Single-shot polling state (when DRDY pin not used)
        bool convPending;        // true after START/SYNC issued, waiting for RDATA window
        uint32_t convReadyUs;    // micros() timestamp when RDATA is expected to be ready

        // Stuck-sample watchdog
        int32_t prevRaw = INT32_MIN;
        uint32_t prevRawChangeMs = 0;

        // Debug state
        int32_t lastRaw = 0;
        uint32_t lastDebugPrintMs = 0;

        // NVS
        Preferences prefs;

        // Low-level
        inline void csLow() const { digitalWrite(cfg.csPin, LOW); }
        inline void csHigh() const { digitalWrite(cfg.csPin, HIGH); }

        void spiWrite(uint8_t b);
        void spiWriteBytes(const uint8_t* d, size_t n);
        void spiReadBytes(uint8_t* d, size_t n);

        void sendReset();         // 0x06
        void sendStartSync();     // 0x08
        void writeRegisters();    // WREG 0..3 with baseline config
        bool readRegisters(uint8_t* out, uint8_t start = 0, uint8_t count = 4); // RREG helper
        void logRegsDecoded(const uint8_t* regs);
        void writeReg0(uint8_t value);

        bool readSampleRaw(int32_t& raw); // reads 24-bit signed
        float rawToVolts(int32_t raw) const;
};

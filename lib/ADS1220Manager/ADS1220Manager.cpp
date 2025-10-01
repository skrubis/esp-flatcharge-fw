#include "ADS1220Manager.h"

// ADS1220 commands
static constexpr uint8_t CMD_RESET = 0x06;
static constexpr uint8_t CMD_START_SYNC = 0x08;
static constexpr uint8_t CMD_RDATA = 0x10;
static constexpr uint8_t CMD_WREG = 0x40; // + (addr<<2) + (num-1)
static constexpr uint8_t CMD_RREG = 0x20; // + (addr<<2) + (num-1)

ADS1220Manager::ADS1220Manager()
: initialized(false), spiSettings(1000000, MSBFIRST, SPI_MODE1),
  zeroOffsetV(0.0f), scaleAmpsPerVolt(1.0f/0.0067f), // ~149.25 A/V (6.7mV/A)
  filtInit(false), filtCurrentA(NAN), lastUpdateMs(0), lastPollUs(0), convPending(false), convReadyUs(0) {
    prevRaw = INT32_MIN;
    prevRawChangeMs = 0;
}

ADS1220Manager::~ADS1220Manager() { end(); }

bool ADS1220Manager::begin(const Config& cfg_) {
    cfg = cfg_;
    Serial.println("[ADS1220] begin() - configuring pins and SPI");
    Serial.printf("[ADS1220] CS=%d DRDY=%d Vref=%.3fV Gain=%u SPS=%u\n", cfg.csPin, cfg.drdyPin, cfg.vref, cfg.gain, cfg.sampleRateSPS);
    pinMode(cfg.csPin, OUTPUT);
    csHigh();
    if (cfg.drdyPin >= 0) pinMode(cfg.drdyPin, INPUT);

    // Shared SPI bus
    SPI.begin();
    Serial.println("[ADS1220] SPI bus started");

    // Prepare NVS for calibration
    if (!prefs.begin("ads1220", false)) {
        // continue without persistence
    }
    loadCalibration();
    Serial.printf("[ADS1220] Calibration loaded: zeroV=%.6fV apv=%.3f A/V\n", zeroOffsetV, scaleAmpsPerVolt);

    // Reset and configure ads1220
    SPI.beginTransaction(spiSettings);
    Serial.println("[ADS1220] Sending RESET command");
    csLow(); spiWrite(CMD_RESET); csHigh(); delay(2);

    // Write registers 0..3 with baseline config
    Serial.println("[ADS1220] Writing baseline registers");
    writeRegisters();

    // Read back registers for verification
    uint8_t regs[4] = {0};
    bool regsOk = readRegisters(regs);
    if (regsOk) {
        Serial.printf("[ADS1220] Readback regs: %02X %02X %02X %02X\n", regs[0], regs[1], regs[2], regs[3]);
    } else {
        Serial.println("[ADS1220] ERROR: Failed to read registers (RREG)");
    }

    // Start continuous conversions
    Serial.println("[ADS1220] Sending START/SYNC (continuous conversions)\n");
    csLow(); spiWrite(CMD_START_SYNC); csHigh();
    SPI.endTransaction();

    // Sanity: attempt one raw sample and compute volts
    int32_t raw = 0; bool ok = false;
    SPI.beginTransaction(spiSettings);
    ok = readSampleRaw(raw);
    SPI.endTransaction();
    if (ok) {
        float v = rawToVolts(raw);
        float a = (v - zeroOffsetV) * scaleAmpsPerVolt;
        Serial.printf("[ADS1220] Initial sample: raw=%ld volts=%.6f V current=%.3f A\n", (long)raw, v, a);
    } else {
        Serial.println("[ADS1220] WARNING: Initial sample read failed");
    }

    initialized = true;
    Serial.println("[ADS1220] Initialization complete");
    return true;
}

void ADS1220Manager::end() {
    if (!initialized) return;
    SPI.end();
    initialized = false;
    prefs.end();
}

void ADS1220Manager::update() {
    if (!initialized) return;

    // If DRDY pin is used, only proceed when low
    if (cfg.drdyPin >= 0) {
        if (digitalRead(cfg.drdyPin) != LOW) return;
    } else {
        // Polling (no DRDY): ADC is in continuous conversion mode.
        // Read at or below the configured SPS so each RDATA fetch gets a fresh sample.
        const uint32_t sps = (cfg.sampleRateSPS > 0) ? cfg.sampleRateSPS : 20;
        const uint32_t periodUs = (sps > 0) ? (1000000UL / sps) : 50000UL;
        uint32_t nowUs = micros();
        if (lastPollUs != 0 && (uint32_t)(nowUs - lastPollUs) < periodUs) return;
        lastPollUs = nowUs;
    }

    int32_t raw;
    SPI.beginTransaction(spiSettings);
    bool ok = readSampleRaw(raw);
    SPI.endTransaction();
    if (!ok) return;

    // Watchdog: if raw hasn't changed for a long time, resync the ADC
    if (prevRaw == INT32_MIN || raw != prevRaw) {
        prevRaw = raw;
        prevRawChangeMs = millis();
    } else {
        uint32_t nowMs = millis();
        if (prevRawChangeMs != 0 && (nowMs - prevRawChangeMs) > 1500UL) {
            // Re-issue START/SYNC to kick conversions
            SPI.beginTransaction(spiSettings);
            sendStartSync();
            SPI.endTransaction();
            prevRawChangeMs = nowMs; // avoid spamming
        }
    }

    float volts = rawToVolts(raw);
    float amps = (volts - zeroOffsetV) * scaleAmpsPerVolt;

    // Simple IIR filter
    if (!filtInit) { filtCurrentA = amps; filtInit = true; }
    else { filtCurrentA = 0.8f * filtCurrentA + 0.2f * amps; }
    lastUpdateMs = millis();
}

float ADS1220Manager::getCurrentA() const {
    return filtCurrentA;
}

bool ADS1220Manager::calibrateZero(uint16_t avgSamples) {
    if (!initialized) return false;

    double acc = 0.0; uint16_t n = 0;
    for (uint16_t i = 0; i < avgSamples; ++i) {
        int32_t raw;
        SPI.beginTransaction(spiSettings);
        bool ok = readSampleRaw(raw);
        SPI.endTransaction();
        if (!ok) continue;
        acc += rawToVolts(raw);
        ++n; delay(5);
    }
    if (n == 0) return false;
    zeroOffsetV = (float)(acc / n);
    saveCalibration();
    return true;
}

bool ADS1220Manager::setScaleAmpsPerVolt(float apv) {
    if (apv <= 0.0f) return false;
    scaleAmpsPerVolt = apv;
    saveCalibration();
    return true;
}

bool ADS1220Manager::loadCalibration() {
    float z = prefs.getFloat("zeroV", NAN);
    float s = prefs.getFloat("apv", NAN);
    if (!isnan(z)) zeroOffsetV = z;
    if (!isnan(s)) scaleAmpsPerVolt = s;
    return true;
}

bool ADS1220Manager::saveCalibration() {
    prefs.putFloat("zeroV", zeroOffsetV);
    prefs.putFloat("apv", scaleAmpsPerVolt);
    return true;
}

bool ADS1220Manager::isValid(uint32_t maxAgeMs) const {
    if (!filtInit) return false;
    uint32_t now = millis();
    uint32_t age = (now >= lastUpdateMs) ? (now - lastUpdateMs) : (UINT32_MAX - lastUpdateMs + now + 1);
    return age <= maxAgeMs;
}

void ADS1220Manager::spiWrite(uint8_t b) { SPI.transfer(b); }
void ADS1220Manager::spiWriteBytes(const uint8_t* d, size_t n) { SPI.transfer((void*)d, n); }
void ADS1220Manager::spiReadBytes(uint8_t* d, size_t n) { for (size_t i=0;i<n;++i) d[i]=SPI.transfer(0xFF); }

void ADS1220Manager::sendReset() {
    csLow(); spiWrite(CMD_RESET); csHigh(); delayMicroseconds(50);
}

void ADS1220Manager::sendStartSync() {
    csLow(); spiWrite(CMD_START_SYNC); csHigh(); delayMicroseconds(50);
}

void ADS1220Manager::writeRegisters() {
    // Baseline config bytes
    // Reg0: AIN0-AIN1 differential, PGA bypass, Gain=1
    // ADS1220 Reg0: [MUX(7:4)=0000][PGA_BYP(3)=1][GAIN(2:0)=000]
    uint8_t reg0 = 0x08;

    // Map cfg.sampleRateSPS to DR bits (Reg1[7:5]) per ADS1220 datasheet
    uint8_t drBits = 0x00; // 20 SPS
    uint16_t sps = cfg.sampleRateSPS;
    if (sps >= 1000) drBits = 0xC0;        // 110 => 1000 SPS
    else if (sps >= 600) drBits = 0xA0;    // 101 => 600 SPS
    else if (sps >= 330) drBits = 0x80;    // 100 => 330 SPS
    else if (sps >= 175) drBits = 0x60;    // 011 => 175 SPS
    else if (sps >= 90)  drBits = 0x40;    // 010 => 90 SPS
    else if (sps >= 45)  drBits = 0x20;    // 001 => 45 SPS
    else                  drBits = 0x00;    // 000 => 20 SPS

    // Conversion mode: always continuous; if DRDY not used we still poll at or below SPS
    uint8_t convMode = 0x08; // Reg1 bit3

    uint8_t reg1 = drBits | convMode; // other bits 0 (default)
    uint8_t regs[4] = { reg0, reg1, 0x00, 0x00 };
    csLow();
    // WREG start at 0, write 4 registers (num-1=3)
    spiWrite(CMD_WREG | (0 << 2) | (4 - 1));
    spiWriteBytes(regs, 4);
    csHigh();
}

bool ADS1220Manager::readRegisters(uint8_t* out, uint8_t start, uint8_t count) {
    if (!out || count == 0 || start > 3 || (start + count) > 4) return false;
    csLow();
    spiWrite(CMD_RREG | ((start & 0x03) << 2) | ((count - 1) & 0x03));
    spiReadBytes(out, count);
    csHigh();
    return true;
}

bool ADS1220Manager::readSampleRaw(int32_t& raw) {
    // Issue RDATA
    csLow(); spiWrite(CMD_RDATA);
    uint8_t b[3]; spiReadBytes(b, 3); csHigh();

    // Assemble 24-bit signed (two's complement)
    int32_t v = ((int32_t)b[0] << 16) | ((int32_t)b[1] << 8) | (int32_t)b[2];
    if (v & 0x800000) v |= 0xFF000000; // sign extend
    raw = v;
    return true;
}

float ADS1220Manager::rawToVolts(int32_t raw) const {
    // LSB size for ADS1220 in bypassed PGA, Gain=1: Vref / 2^23
    float lsb = cfg.vref / 8388608.0f; // 2^23
    return raw * lsb;
}

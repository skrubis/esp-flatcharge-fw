#include "ADS1220Manager.h"

// ADS1220 commands
static constexpr uint8_t CMD_RESET = 0x06;
static constexpr uint8_t CMD_START_SYNC = 0x08;
static constexpr uint8_t CMD_RDATA = 0x10;
static constexpr uint8_t CMD_WREG = 0x40; // + (addr<<2) + (num-1)

ADS1220Manager::ADS1220Manager()
: initialized(false), spiSettings(1000000, MSBFIRST, SPI_MODE1),
  zeroOffsetV(0.0f), scaleAmpsPerVolt(1.0f/0.0067f), // ~149.25 A/V (6.7mV/A)
  filtInit(false), filtCurrentA(NAN), lastUpdateMs(0) {}

ADS1220Manager::~ADS1220Manager() { end(); }

bool ADS1220Manager::begin(const Config& cfg_) {
    cfg = cfg_;
    pinMode(cfg.csPin, OUTPUT);
    csHigh();
    if (cfg.drdyPin >= 0) pinMode(cfg.drdyPin, INPUT);

    // Shared SPI bus
    SPI.begin();

    // Prepare NVS for calibration
    if (!prefs.begin("ads1220", false)) {
        // continue without persistence
    }
    loadCalibration();

    // Reset and configure ads1220
    SPI.beginTransaction(spiSettings);
    csLow(); spiWrite(CMD_RESET); csHigh(); delay(2);

    // Write registers 0..3 with baseline config: [0x01, 0x20, 0x00, 0x00]
    writeRegisters();

    // Start continuous conversions
    csLow(); spiWrite(CMD_START_SYNC); csHigh();
    SPI.endTransaction();

    initialized = true;
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
        // polling delay - modest rate
    }

    int32_t raw;
    SPI.beginTransaction(spiSettings);
    bool ok = readSampleRaw(raw);
    SPI.endTransaction();
    if (!ok) return;

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
    uint8_t regs[4] = { 0x01, 0x20, 0x00, 0x00 };
    csLow();
    // WREG start at 0, write 4 registers (num-1=3)
    spiWrite(CMD_WREG | (0 << 2) | (4 - 1));
    spiWriteBytes(regs, 4);
    csHigh();
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

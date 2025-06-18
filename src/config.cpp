#include "config.h"

// System constants
const char* WIFI_SSID = "FLATCHARGE";
const char* WIFI_PASSWORD = "FLATCHARGE!";
const char* CONFIG_FILE = "/config.json";

// CAN bus constants
const long CAN_SPEED = CAN_125KBPS;
const uint8_t SPI_CAN1_CS = 14;
const uint8_t SPI_CAN2_CS = 17;
const uint8_t SPI_CAN3_CS = 18;

// I2C pins
const uint8_t I2C_SDA = 9;
const uint8_t I2C_SCL = 8;

// SPI pins
const uint8_t SPI_MOSI = 11;
const uint8_t SPI_MISO = 13;
const uint8_t SPI_SCK = 12;

// ESP32 GPIO pins
const uint8_t HEARTBEAT_LED = 48;
const uint8_t CONTROL_PILOT_IN = 2;

// GPIO expander pins
const uint8_t MCP23017_ADDRESS = 0x26;

// GPIO expander port A pins
const uint8_t GPIOEXP_SPI_CAN1_RST = 0;
const uint8_t GPIOEXP_SPI_CAN1_INT = 1;
const uint8_t GPIOEXP_SPI_CAN2_RST = 2;
const uint8_t GPIOEXP_SPI_CAN2_INT = 3;
const uint8_t GPIOEXP_SPI_CAN3_RST = 4;
const uint8_t GPIOEXP_SPI_CAN3_INT = 5;

// GPIO expander port B pins (add 8 to access via MCP23017)
const uint8_t GPIOEXP_EVSE_MODE_IDLE = 0;  // Port B pin 0 (8)
const uint8_t GPIOEXP_EVSE_MODE_CHARGE = 1;  // Port B pin 1 (9)
const uint8_t GPIOEXP_EVSE_MODE_CHARGE_VENT = 2;  // Port B pin 2 (10)
const uint8_t GPIOEXP_EVSE_MODE_FAULT = 3;  // Port B pin 3 (11)
const uint8_t GPIOEXP_TWAI_CAN_POWER = 4;  // Port B pin 4 (12)
const uint8_t GPIOEXP_SPI_CAN3_POWER = 5;  // Port B pin 5 (13)
const uint8_t GPIOEXP_SPI_CAN2_POWER = 6;  // Port B pin 6 (14)
const uint8_t GPIOEXP_SPI_CAN1_POWER = 7;  // Port B pin 7 (15)

// Flatpack2 constants
const long FP_SERIAL_ID = 0x05000000;
const long FP_LOGIN_ID = 0x05004000;
const long FP_REQUEST_ID = 0x05004000;
const long FP_DEFAULT_VOLTAGE_ID = 0x05010000;
const long FP_STATUS_ID_1 = 0x05014000;
const long FP_STATUS_ID_2 = 0x05014010;
const long FP_STATUS_ID_3 = 0x05014020;
const long FP_STATUS_ID_4 = 0x05014030;

// Flatpack limits
const float FP_MIN_VOLTAGE = 42.0;
const float FP_MAX_VOLTAGE = 58.0;
const float MAX_CURRENT_THREE_PHASE = 16.0;
const float MAX_CURRENT_SINGLE_PHASE = 32.0;

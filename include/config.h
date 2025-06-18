#pragma once

// WiFi configuration
#define WIFI_SSID "FLATCHARGE"
#define WIFI_PASSWORD "FLATCHARGE!"
#define WIFI_CHANNEL 1
#define WIFI_MAX_CONNECTIONS 4

// I2C configuration
#define I2C_SDA_PIN 9
#define I2C_SCL_PIN 8
#define I2C_FREQUENCY 400000
#define MCP23017_ADDRESS 0x26

// SPI configuration
#define VSPI_MOSI_PIN 11
#define VSPI_MISO_PIN 13
#define VSPI_SCK_PIN 12
#define VSPI_CAN1_CS_PIN 14
#define VSPI_CAN2_CS_PIN 17
#define VSPI_CAN3_CS_PIN 18

// MCP23017 GPIO pins
#define GPIOEXP_SPI_CAN1_POWER 7  // GPB7
#define GPIOEXP_SPI_CAN2_POWER 6  // GPB6
#define GPIOEXP_SPI_CAN3_POWER 5  // GPB5
#define GPIOEXP_TWAI_CAN_POWER 4  // GPB4

#define GPIOEXP_SPI_CAN1_INT 4    // GPA4
#define GPIOEXP_SPI_CAN1_RST 5    // GPA5
#define GPIOEXP_SPI_CAN2_INT 1    // GPA1
#define GPIOEXP_SPI_CAN2_RST 0    // GPA0
#define GPIOEXP_SPI_CAN3_INT 2    // GPA2
#define GPIOEXP_SPI_CAN3_RST 3    // GPA3

// EVSE Mode indicators
#define GPIOEXP_EVSE_MODE_IDLE 8           // GPB0
#define GPIOEXP_EVSE_MODE_CHARGE 9         // GPB1
#define GPIOEXP_EVSE_MODE_CHARGE_VENT 10   // GPB2
#define GPIOEXP_EVSE_MODE_FAULT 11         // GPB3

// GPIO pins
#define GPIO_EXPANDER_INT_PIN 1
#define CP_CHARGER_PWM_IN_PIN 2
#define TWAI_CAN_RX_PIN 5
#define TWAI_CAN_TX_PIN 6
#define IGNITION_IN_PIN 7

// CAN bus speed
#define CAN_SPEED CAN_125KBPS

// Flatpack settings
#define FP_DEFAULT_VOLTAGE 43.7    // Default voltage for flatpacks
#define FP_MIN_VOLTAGE 43.5        // Minimum voltage
#define FP_MAX_VOLTAGE 57.5        // Maximum voltage
#define FP_MAX_CURRENT 62.5        // Maximum current for 3000W FP

// Web server port
#define HTTP_PORT 80

// SPIFFS paths
#define CONFIG_FILE "/config.json"

// Maximum current draw limits
#define MAX_CURRENT_SINGLE_PHASE 32.0   // Amperes
#define MAX_CURRENT_THREE_PHASE 10.7    // Amperes per phase (32/3)

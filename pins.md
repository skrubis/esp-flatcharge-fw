ESP32-S3 Pins:
    GPIO EXPANDER INTERRUPT: GPIO1
    CP CHARGER PWM IN: GPIO2
    TWAI CAN RX: GPIO5
    TWAI CAN TX: GPIO6
    IGNITION IN: GPIO7
    I2C_SCL: GPIO9
    I2C_SDA: GPIO8
    VSPI_MOSI: GPIO11
    VSPI_SCK: GPIO12
    VSPI_MISO: GPIO13
    VSPI_CAN1_CS: GPIO14
    VSPI_CAN2_CS: GPIO17
    VSPI_CAN3_CS: GPIO18
    VSPI_CAN_3_INT: GPIO10
    VSPI_CAN_2_INT: GPIO21
    VSPI_CAN_1_INT: GPIO47


    
MCP23017 GPIO Expander (ADDRESS 0x26):
GPB0 - EVSE MODE IDLE
GPB1 - EVSE MODE CHARGE VENT NOT REQUIRED
GPB2 - EVSE MODE CHARGE VENT REQUIRED
GPB3 - EVSE MODE FAULT
GPB4 - ENABLE TWAI CAN POWER
GPB5 - ENABLE SPI_CAN_3 POWER
GPB6 - ENABLE SPI_CAN_2 POWER
GPB7 - ENABLE SPI_CAN_1 POWER

GPA0 - SPI_CAN2_RST
GPA3 - SPI_CAN3_RST
GPA5 - SPI_CAN1_RST

SPI CAN = 3x MCP2515 SPI TRANSCEIVERS
TWAI CAN = ESP32-S3 TWAI CAN

BOARD HAS NO PSRAM

# MCP2515 Configuration Reference

## MCP2515 Operating Modes:
    0x00 - Normal Mode: Standard operation, can transmit/receive messages, acknowledges received frames
    0x20 - Sleep Mode: Low power mode, wakes on CAN bus activity
    0x40 - Loopback Mode: For testing, transmissions internally routed to receive buffers
    0x60 - Listen-Only Mode: Can receive but not transmit or acknowledge frames (for bus monitoring)
    0x80 - Configuration Mode: For changing bit timing registers and other settings

## Bit Timing Configuration for 8MHz Crystal at 125kbps:

### Working Configurations:

1. **Normal Mode (preferred)**
   - CNF1=0x03, CNF2=0xF1, CNF3=0x85
   - BRP=4, PRSEG=0, PHSEG1=7, PHSEG2=5, SAM=1, SJW=4
   - Sample Point: 87.5%

2. **Normal Mode (alternate)**
   - CNF1=0x03, CNF2=0xAC, CNF3=0x04
   - BRP=4, PRSEG=2, PHSEG1=5, PHSEG2=4, SAM=0, SJW=1
   - Sample Point: 75%

3. **Listen-Only Mode**
   - CNF1=0x03, CNF2=0xB1, CNF3=0x05
   - BRP=4, PRSEG=1, PHSEG1=6, PHSEG2=5, SAM=0, SJW=1
   - Sample Point: 75%

### Notes:
- Removing pullup resistor on RXCAN pin resolved communication issues
- When using 8MHz crystal, these settings achieve 125kbps CAN bus speed
- To disable acceptance filters completely:
  - Set RXB0CTRL = 0x64 (accept all messages + rollover)
  - Set RXB1CTRL = 0x60 (accept all messages)
- Error counters and flags should be monitored for optimal configuration
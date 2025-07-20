We are building a firmware for a board which controls up to 3 (pirmarily more than one in series) Eltek flatpack2 power rectifiers in series or parallel to charge a battery. For charging a battery from AC type 2 socket we have CP PP pins, but that is out of scope for now, now we need to control the flatpacks.

M
    CAN speed is 125kbit/s.
To send requests to the FP (Flatpack) you first need to send a "login" message consisting of the FP serial number which is printed on the lable of the FP followed by two empty bytes (0x00). Send the serial number (ex 141471110820) in HEX and use the following ID: 0x05004804.
Arduino code using mcp_can library and MCP2515 CAN controller.

Code: Select all

unsigned char stmp1[8] = {0x14, 0x14, 0x71, 0x11, 0x08, 0x20, 0x00, 0x00};    //this is the serial number of the flatpack followed by two 00 bytes 
CAN.sendMsgBuf(0x05004804, 1, 8, stmp1);                                      //send message to log in

Within 5s of the "login" you then send your "request" with ID: 0x05FF4004.
Current in dA (Ampere times 10) in HEX with low byte first. For 16.0A translate 160 to HEX => 0x00A0, separate into 2 bytes and place as byte 0 & 1.
Voltage in cV (Voltage times 100) in HEX with low byte first. For 57.6V translate 5760 to HEX => 0x1680, separate into 2 bytes and place as byte 2 & 3 and also byte 4 & 5.
An over voltage protection level also needs to be set, above the requested voltage. Done in same manner as Voltage so for 59.5V translate 5950 to HEX => 0x173E, separate into 2 bytes and place as byte 6 & 7.

Code: Select all

unsigned char stmp2[8] = {0xA0, 0x00, 0x80, 0x16, 0x80, 0x16, 0x3E, 0x17};    //set rectifier to 16.0 amps (00 A0) 57.60 (16 80) and OVP to 59.5V (17 3E)
CAN.sendMsgBuf(0x05FF4004, 1, 8, stmp2); 

So one login followed by repeated requests with less then 5s interval is enough. For robustness I send both login and request every time.
If communication is lost the FP reverts back to the default voltage.
This default voltage can be set by sending first login and then a message with ID: 0x05009C00.
Byte 0 = 0x29, Byte 1 = 0x15, Byte 2= 0x00.
For 43.7V translate 4370 to HEX => 0x1112, separate into 2 bytes and place as byte 3 & 4.

Code: Select all

byte stmp2[5] = {0x29, 0x15, 0x00, 0x12, 0x11};                   //set rectifier permanently to 43.70 (11 12)
CAN.sendMsgBuf(0x05009C00, 1, 5, stmp2); 

Login followedby above message only needs to be sent once and then comes into effect after communication time-out (5s) or after power cycle. This will be remembered after power off. I use 43.7V as I found this the lowest stable voltage.

Voltage can be set between 43.5V and 57.5V. Current can be set between 0 and max 62.5A for a 3000W FP, we are using 2000W ones, so current limit is probably much lower..
NOTE: When using current limiting the FP will only maintain set current if it can do so with a voltage above 47.0V. If voltage needs to reduce further the current output will be max current, 62.5A so be careful when designing the charging algorithm.

You first need to init each canbus, look if there are messages on the bus, if yes, get serial number of the flatpack and use it for further login.


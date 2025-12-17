#pragma once
#include <Arduino.h>
#include <WiFi.h>
#include "secrets.h"
#include <esp_wifi.h>
#include <esp_now.h>
#include "driver/twai.h"

bool newDataToSend = false;

typedef struct
{
    uint32_t identifier;
    uint8_t data_length_code; /**< Data length code max value of 8 */
    byte dataByte0;
    byte dataByte1;
    byte dataByte2;
    byte dataByte3;
    byte dataByte4;
    byte dataByte5;
    byte dataByte6;
    byte dataByte7;
} esp_now_message_t;

esp_now_message_t outgoingMessage;

#define DEBUG 1
// Conditional definition for debugging if DEBUG is 1 then it will print to serial port.
// If DEBUG = 0 then the lines will be removed by the compiler.
#if DEBUG == 1
#define debug(x) Serial.print(x)
#define debugln(x) Serial.println(x)
#define debugg(x,y) Serial.print(x,y)
#define debugf(fmt, ...) Serial.printf((fmt), ##__VA_ARGS__)
#define debuglnf(fmt, ...) Serial.printlnf((fmt), ##__VA_ARGS__);
#else
#define debug(x)
#define debugln(x)
#define debugg(x,y)
#define debugf(fmt, ...)
#define debuglnf(fmt, ...)
#endif
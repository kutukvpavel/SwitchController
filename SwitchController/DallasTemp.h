/*
 * DallasTemp.h
 *
 * Created: 11.11.2018 3:57:13
 *  Author: Павел
 */ 

#pragma once

#ifndef DALLASTEMP_H_
#define DALLASTEMP_H_

#ifdef __cplusplus

#include "Onewire.h"

// Error Codes
#define DEVICE_DISCONNECTED_C -127

// OneWire commands
#define STARTCONVO      0x44  // Tells device to take a temperature reading and put it on the scratchpad
#define COPYSCRATCH     0x48  // Copy EEPROM
#define READSCRATCH     0xBE  // Read EEPROM
#define WRITESCRATCH    0x4E  // Write to EEPROM
#define RECALLSCRATCH   0xB8  // Reload from last known
#define READPOWERSUPPLY 0xB4  // Determine if device needs parasite power
#define ALARMSEARCH     0xEC  // Query bus for devices with an alarm condition

// Scratchpad locations
#define TEMP_LSB        0
#define TEMP_MSB        1
#define HIGH_ALARM_TEMP 2
#define LOW_ALARM_TEMP  3
#define CONFIGURATION   4
#define INTERNAL_BYTE   5
#define COUNT_REMAIN    6
#define COUNT_PER_C     7
#define SCRATCHPAD_CRC  8

// Device resolution
#define TEMP_9_BIT  0x1F //  9 bit
#define TEMP_10_BIT 0x3F // 10 bit
#define TEMP_11_BIT 0x5F // 11 bit
#define TEMP_12_BIT 0x7F // 12 bit

typedef uint8_t DeviceAddress[8];

uint8_t searchForDevices(uint8_t*);

void readScratchPad(const uint8_t*, uint8_t*);

void writeScratchPad(const uint8_t*, const uint8_t*);

// set resolution of a device to 9, 10, 11, or 12 bits
void setResolution(const uint8_t*);

// sends command for all devices on the bus to perform a temperature conversion
void requestTemperatures(void);

// returns temperature raw value (12 bit integer of 1/128 degrees C)
float getTemp(const uint8_t*);

typedef uint8_t ScratchPad[9];
#endif // __cplusplus

#endif /* DALLASTEMP_H_ */
/*
 * DallasTemp.cpp
 *
 * Created: 11.11.2018 3:57:23
 *  Author: Павел
 */ 

 #include "DallasTemp.h"

// initialise the bus
uint8_t searchForDevices(uint8_t* addressArray) {
	OWreset_search();
	uint8_t cnt = 0;
	while (OWsearch(addressArray)) 
	{
		addressArray += 8;
		++cnt;
	}
	return cnt;
}

void writeScratchPad(const uint8_t* deviceAddress,
		const uint8_t* scratchPad) {
	OWreset();
	OWselect(deviceAddress);
	OWwrite(WRITESCRATCH);
	OWwrite(scratchPad[HIGH_ALARM_TEMP]); // high alarm temp
	OWwrite(scratchPad[LOW_ALARM_TEMP]); // low alarm temp
	OWwrite(scratchPad[CONFIGURATION]);
	OWreset();
	OWselect(deviceAddress);
	OWwrite(COPYSCRATCH);
	_delay_ms(20); // <--- added 20ms delay to allow 10ms long EEPROM write operation (as specified by datasheet)
	OWreset();
}

void setResolution(const uint8_t* deviceAddress) {
	ScratchPad scratchPad;
	readScratchPad(deviceAddress, scratchPad);
	scratchPad[CONFIGURATION] = TEMP_9_BIT;
	writeScratchPad(deviceAddress, scratchPad);
}

// sends command for all devices on the bus to perform a temperature conversion
void requestTemperatures() {
	OWreset();
	OWskip();
	OWwrite(STARTCONVO);
}

void readScratchPad(const uint8_t* deviceAddress, uint8_t* scratchPad) {
	OWreset();
	OWselect(deviceAddress);
	OWwrite(READSCRATCH);
	for (uint8_t i = 0; i < 9; i++) {
		scratchPad[i] = OWread();
	}
	OWreset();
}

float getTemp(const uint8_t* deviceAddress) {
	ScratchPad scratchPad;
	readScratchPad(deviceAddress, scratchPad);
	if (OWcrc8(scratchPad, 8) == scratchPad[SCRATCHPAD_CRC])
	{
		int16_t fpTemperature = (((int16_t) scratchPad[TEMP_MSB]) << 11) | (((int16_t) scratchPad[TEMP_LSB]) << 3);
		return (float)fpTemperature * 0.0078125;
	}
	return 200;
}
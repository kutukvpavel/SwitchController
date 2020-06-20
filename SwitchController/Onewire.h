/*
 * Onewire.h
 *
 * Created: 11.11.2018 3:34:39
 *  Author: Павел
 */ 

#pragma once

#ifndef ONEWIRE_H_
#define ONEWIRE_H_

#ifdef __cplusplus

#include <stdint.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include "Libs/OptPin.h"

// You can exclude certain features from OneWire.  In theory, this
// might save some space.  In practice, the compiler automatically
// removes unused code (technically, the linker, using -fdata-sections
// and -ffunction-sections when compiling, and Wl,--gc-sections
// when linking), so most of these will not result in any code size
// reduction.  Well, unless you try to use the missing features
// and redesign your program to not need them!  ONEWIRE_CRC8_TABLE
// is the exception, because it selects a fast but large algorithm
// or a small but slow algorithm.

void OWbegin(pin_t* pin);

// Perform a 1-Wire reset cycle. Returns 1 if a device responds
// with a presence pulse.  Returns 0 if there is no device or the
// bus is shorted or otherwise held low for more than 250uS
uint8_t OWreset(void);

// Issue a 1-Wire rom select command, you do the reset first.
void OWselect(const uint8_t rom[8]);

// Issue a 1-Wire rom skip command, to address all on bus.
void OWskip(void);

// Write a byte. If 'power' is one then the wire is held high at
// the end for parasitically powered devices. You are responsible
// for eventually depowering it by calling depower() or doing
// another read or write.
void OWwrite(uint8_t v, uint8_t power = 0);

void OWwrite_bytes(const uint8_t *buf, uint16_t count, bool power = 0);

// Read a byte.
uint8_t OWread(void);

void OWread_bytes(uint8_t *buf, uint16_t count);

// Write a bit. The bus is always left powered at the end, see
// note in write() about that.
void OWwrite_bit(uint8_t v);

// Read a bit.
uint8_t OWread_bit(void);

// Stop forcing power onto the bus. You only need to do this if
// you used the 'power' flag to write() or used a write_bit() call
// and aren't about to do another read or write. You would rather
// not leave this powered if you don't have to, just in case
// someone shorts your bus.
void OWdepower(void);

// Clear the search state so that if will start from the beginning again.
void OWreset_search();

// Setup the search to find the device type 'family_code' on the next call
// to search(*newAddr) if it is present.
void OWtarget_search(uint8_t family_code);

// Look for the next device. Returns 1 if a new address has been
// returned. A zero might mean that the bus is shorted, there are
// no devices, or you have already retrieved all of them.  It
// might be a good idea to check the CRC to make sure you didn't
// get garbage.  The order is deterministic. You will always get
// the same devices in the same order.
bool OWsearch(uint8_t *newAddr, bool search_mode = true);

uint8_t OWcrc8(const uint8_t *addr, uint8_t len);

#endif // __cplusplus



#endif /* ONEWIRE_H_ */
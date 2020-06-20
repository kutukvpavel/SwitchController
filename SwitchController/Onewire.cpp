/*
* Onewire.cpp
*
* Created: 11.11.2018 3:34:27
*  Author: Павел
*/ 

#include "Onewire.h"

static pin_t* OWpin;
// global search state
static unsigned char ROM_NO[8];
static uint8_t LastDiscrepancy;
static uint8_t LastFamilyDiscrepancy;
static bool LastDeviceFlag;

void OWbegin(pin_t* pin)
{
	OWpin = pin;
#if PINS_STATIC
	pin_t::SetDirection(pin, pin_t::Input);
#else
	OWpin->SetDirection(pin_t::Input);
#endif
	#if ONEWIRE_SEARCH
	OWreset_search();
	#endif
}


// Perform the onewire OWreset function.  We will wait up to 250uS for
// the bus to come high, if it doesn't then it is broken or shorted
// and we return a 0;
//
// Returns 1 if a device asserted a presence pulse, 0 otherwise.
//
uint8_t OWreset(void)
{
	uint8_t r;
	uint8_t retries = 125;

	cli();
	#if PINS_STATIC
	pin_t::SetDirection(OWpin, pin_t::Input);
	#else
	OWpin->SetDirection(pin_t::Input);
	#endif
	sei();
	// wait until the wire is high... just in case
	do {
		if (--retries == 0) return 3;
		_delay_us(2);
	} while (!
	#if PINS_STATIC
	pin_t::Read(OWpin)
	#else
	OWpin->Read()
	#endif
	);

	cli();
	#if PINS_STATIC
	pin_t::Write(OWpin, false);
	#else
	OWpin->Write(false);
	#endif
	#if PINS_STATIC
	pin_t::SetDirection(OWpin, pin_t::Output);
	#else
	OWpin->SetDirection(pin_t::Output);
	#endif	// drive output low
	sei();
	_delay_us(500);
	cli();
	#if PINS_STATIC
	pin_t::SetDirection(OWpin, pin_t::Input);
	#else
	OWpin->SetDirection(pin_t::Input);
	#endif	// allow it to float
	_delay_us(50);
	r = !
	#if PINS_STATIC
	pin_t::Read(OWpin);
	#else
	OWpin->Read();
	#endif
	sei();
	_delay_us(410);
	return r;
}

//
// Write a bit. Port and bit is used to cut lookup time and provide
// more certain timing.
//
void OWwrite_bit(uint8_t v)
{
	if (v & 1) {
		cli();
		#if PINS_STATIC
		pin_t::Write(OWpin, false);
		#else
		OWpin->Write(false);
		#endif
		#if PINS_STATIC
		pin_t::SetDirection(OWpin, pin_t::Output);
		#else
		OWpin->SetDirection(pin_t::Output);
		#endif	// drive output low
		_delay_us(10);
		#if PINS_STATIC
		pin_t::Write(OWpin, true);
		#else
		OWpin->Write(true);
		#endif	// drive output high
		sei();
		_delay_us(55);
		} else {
		cli();
		#if PINS_STATIC
		pin_t::Write(OWpin, false);
		#else
		OWpin->Write(false);
		#endif
		#if PINS_STATIC
		pin_t::SetDirection(OWpin, pin_t::Output);
		#else
		OWpin->SetDirection(pin_t::Output);
		#endif	// drive output low
		_delay_us(65);
		#if PINS_STATIC
		pin_t::Write(OWpin, true);
		#else
		OWpin->Write(true);
		#endif	// drive output high
		sei();
		_delay_us(5);
	}
}

//
// Read a bit. Port and bit is used to cut lookup time and provide
// more certain timing.
//
uint8_t OWread_bit(void)
{
	uint8_t r;

	cli();
	#if PINS_STATIC
	pin_t::SetDirection(OWpin, pin_t::Output);
	#else
	OWpin->SetDirection(pin_t::Output);
	#endif
	#if PINS_STATIC
	pin_t::Write(OWpin, false);
	#else
	OWpin->Write(false);
	#endif
	_delay_us(3);
	#if PINS_STATIC
	pin_t::SetDirection(OWpin, pin_t::Input);
	#else
	OWpin->SetDirection(pin_t::Input);
	#endif	// let pin float, pull up will raise
	_delay_us(10);
	r = 
	#if PINS_STATIC
	pin_t::Read(OWpin);
	#else
	OWpin->Read();
	#endif
	sei();
	_delay_us(53);
	return r;
}

//
// Write a byte. The writing code uses the active drivers to raise the
// pin high, if you need power after the OWwrite (e.g. DS18S20 in
// parasite power mode) then set 'power' to 1, otherwise the pin will
// go tri-state at the end of the OWwrite to avoid heating in a short or
// other mishap.
//
void OWwrite(uint8_t v, uint8_t power /* = 0 */) {
	uint8_t bitMask;

	for (bitMask = 0x01; bitMask; bitMask <<= 1) {
		OWwrite_bit( (bitMask & v)?1:0);
	}
	if ( !power) {
		cli();
		#if PINS_STATIC
		pin_t::SetDirection(OWpin, pin_t::Input);
		#else
		OWpin->SetDirection(pin_t::Input);
		#endif
		#if PINS_STATIC
		pin_t::Write(OWpin, false);
		#else
		OWpin->Write(false);
		#endif
		sei();
	}
}

void OWwrite_bytes(const uint8_t *buf, uint16_t count, bool power /* = 0 */) {
	for (uint16_t i = 0 ; i < count ; i++)
	OWwrite(buf[i]);
	if (!power) {
		cli();
		#if PINS_STATIC
		pin_t::SetDirection(OWpin, pin_t::Input);
		#else
		OWpin->SetDirection(pin_t::Input);
		#endif
		#if PINS_STATIC
		pin_t::Write(OWpin, false);
		#else
		OWpin->Write(false);
		#endif
		sei();
	}
}

//
// Read a byte
//
uint8_t OWread() {
	uint8_t bitMask;
	uint8_t r = 0;

	for (bitMask = 0x01; bitMask; bitMask <<= 1) {
		if ( OWread_bit()) r |= bitMask;
	}
	return r;
}

void OWread_bytes(uint8_t *buf, uint16_t count) {
	for (uint16_t i = 0 ; i < count ; i++)
	buf[i] = OWread();
}

//
// Do a ROM select
//
void OWselect(const uint8_t rom[8])
{
	uint8_t i;

	OWwrite(0x55);           // Choose ROM

	for (i = 0; i < 8; i++) OWwrite(rom[i]);
}

//
// Do a ROM skip
//
void OWskip()
{
	OWwrite(0xCC);           // Skip ROM
}

void OWdepower()
{
	cli();
	#if PINS_STATIC
	pin_t::SetDirection(OWpin, pin_t::Input);
	#else
	OWpin->SetDirection(pin_t::Input);
	#endif
	sei();
}

//
// You need to use this function to start a search again from the beginning.
// You do not need to do it for the first search, though you could.
//
void OWreset_search()
{
	// OWreset the search state
	LastDiscrepancy = 0;
	LastDeviceFlag = false;
	LastFamilyDiscrepancy = 0;
	for(int i = 7; ; i--) {
		ROM_NO[i] = 0;
		if ( i == 0) break;
	}
}

// Setup the search to find the device type 'family_code' on the next call
// to search(*newAddr) if it is present.
//
void OWtarget_search(uint8_t family_code)
{
	// set the search state to find SearchFamily type devices
	ROM_NO[0] = family_code;
	for (uint8_t i = 1; i < 8; i++)
	ROM_NO[i] = 0;
	LastDiscrepancy = 64;
	LastFamilyDiscrepancy = 0;
	LastDeviceFlag = false;
}

//
// Perform a search. If this function returns a '1' then it has
// enumerated the next device and you may retrieve the ROM from the
// OWaddress variable. If there are no devices, no further
// devices, or something horrible happens in the middle of the
// enumeration then a 0 is returned.  If a new device is found then
// its address is copied to newAddr.  Use OWreset_search() to
// start over.
//
// --- Replaced by the one from the Dallas Semiconductor web site ---
//--------------------------------------------------------------------------
// Perform the 1-Wire Search Algorithm on the 1-Wire bus using the existing
// search state.
// Return TRUE  : device found, ROM number in ROM_NO buffer
//        FALSE : device not found, end of search
//
bool OWsearch(uint8_t *newAddr, bool search_mode /* = true */)
{
	uint8_t id_bit_number;
	uint8_t last_zero, rom_byte_number;
	bool    search_result;
	uint8_t id_bit, cmp_id_bit;

	unsigned char rom_byte_mask, search_direction;

	// initialize for search
	id_bit_number = 1;
	last_zero = 0;
	rom_byte_number = 0;
	rom_byte_mask = 1;
	search_result = false;

	// if the last call was not the last one
	if (!LastDeviceFlag) {
		// 1-Wire OWreset
		if (!OWreset()) {
			// OWreset the search
			LastDiscrepancy = 0;
			LastDeviceFlag = false;
			LastFamilyDiscrepancy = 0;
			return false;
		}

		// issue the search command
		if (search_mode == true) {
			OWwrite(0xF0);   // NORMAL SEARCH
			} else {
			OWwrite(0xEC);   // CONDITIONAL SEARCH
		}

		// loop to do the search
		do
		{
			// OWread a bit and its complement
			id_bit = OWread_bit();
			cmp_id_bit = OWread_bit();

			// check for no devices on 1-wire
			if ((id_bit == 1) && (cmp_id_bit == 1)) {
				break;
				} else {
				// all devices coupled have 0 or 1
				if (id_bit != cmp_id_bit) {
					search_direction = id_bit;  // bit OWwrite value for search
					} else {
					// if this discrepancy if before the Last Discrepancy
					// on a previous next then pick the same as last time
					if (id_bit_number < LastDiscrepancy) {
						search_direction = ((ROM_NO[rom_byte_number] & rom_byte_mask) > 0);
						} else {
						// if equal to last pick 1, if not then pick 0
						search_direction = (id_bit_number == LastDiscrepancy);
					}
					// if 0 was picked then record its position in LastZero
					if (search_direction == 0) {
						last_zero = id_bit_number;

						// check for Last discrepancy in family
						if (last_zero < 9)
						LastFamilyDiscrepancy = last_zero;
					}
				}

				// set or clear the bit in the ROM byte rom_byte_number
				// with mask rom_byte_mask
				if (search_direction == 1)
				ROM_NO[rom_byte_number] |= rom_byte_mask;
				else
				ROM_NO[rom_byte_number] &= ~rom_byte_mask;

				// serial number search direction OWwrite bit
				OWwrite_bit(search_direction);

				// increment the byte counter id_bit_number
				// and shift the mask rom_byte_mask
				id_bit_number++;
				rom_byte_mask <<= 1;

				// if the mask is 0 then go to new SerialNum byte rom_byte_number and OWreset mask
				if (rom_byte_mask == 0) {
					rom_byte_number++;
					rom_byte_mask = 1;
				}
			}
		}
		while(rom_byte_number < 8);  // loop until through all ROM bytes 0-7

		// if the search was successful then
		if (!(id_bit_number < 65)) {
			// search successful so set LastDiscrepancy,LastDeviceFlag,search_result
			LastDiscrepancy = last_zero;

			// check for last device
			if (LastDiscrepancy == 0) {
				LastDeviceFlag = true;
			}
			search_result = true;
		}
	}

	// if no device found then OWreset counters so next 'search' will be like a first
	if (!search_result || !ROM_NO[0]) {
		LastDiscrepancy = 0;
		LastDeviceFlag = false;
		LastFamilyDiscrepancy = 0;
		search_result = false;
		} else {
		for (int i = 0; i < 8; i++) newAddr[i] = ROM_NO[i];
	}
	return search_result;
}

// Dow-CRC using polynomial X^8 + X^5 + X^4 + X^0
// Tiny 2x16 entry CRC table created by Arjen Lentz
// See http://lentz.com.au/blog/calculating-crc-with-a-tiny-32-entry-lookup-table
static const uint8_t dscrc2x16_table[] = {
	0x00, 0x5E, 0xBC, 0xE2, 0x61, 0x3F, 0xDD, 0x83,
	0xC2, 0x9C, 0x7E, 0x20, 0xA3, 0xFD, 0x1F, 0x41,
	0x00, 0x9D, 0x23, 0xBE, 0x46, 0xDB, 0x65, 0xF8,
	0x8C, 0x11, 0xAF, 0x32, 0xCA, 0x57, 0xE9, 0x74
};

// Compute a Dallas Semiconductor 8 bit CRC. These show up in the ROM
// and the registers.  (Use tiny 2x16 entry CRC table)
uint8_t OWcrc8(const uint8_t *addr, uint8_t len)
{
	uint8_t crc = 0;

	while (len--) {
		crc = *addr++ ^ crc;  // just re-using crc as intermediate
		crc = *(dscrc2x16_table + (crc & 0x0f)) ^ *(dscrc2x16_table + 16 + ((crc >> 4) & 0x0f));
	}

	return crc;
}
/*
 * SwitchController.cpp
 * Created: 17.08.2018 20:40:33
 * Last edit: 19.01.2019
 * Author: Павел
 * Warning! Compile with LTO!
 * TODO: none
 *
 *				1	_   _	28
 *	 +5 (RESET)	PC6|.|_| |PC5 I0
 *		RXD 	PD0|	 |PC4 I1
 *		TXD 	PD1|	 |PC3 I2
 *		INT0	PD2|	 |PC2 I3
 *		INT1	PD3|	 |PC1 I4
 *	  O4 (LED)	PD4|	 |PC0 I5
 *				VCC|	 |GND
 *				GND|	 |AREF
 *		  XTAL	PB6|	 |AVCC
 *		  XTAL	PB7|	 |PB5 X
 *	I6 (BUTTON) PD5|	 |PB4 O0 (MOSFET)
 *			O3  PD6|	 |PB3 I/O
 *			O2  PD7|	 |PB2 I/O (1-Wire)
 *			O1  PB0|_____|PB1 +5 (??)
 *				14			15
 *
 * "Labels L${I/O/T}$#$\"<16\"; L$A[${I/O/T}$#]|I/O inversion I${O/I}${0/1}|Output O$#${0/1}; O$E|Mapping M$i#$o#|Override V$#${0/1}|Read R[${I/O/T}#]|Input normal N$#${0/1}|Probe P, Help H"
 */

#include <avr/io.h>
#include <util/delay.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <avr/eeprom.h>
#include <stdlib.h>
#include <limits.h>
#include "Libs/MyFunctions.h"
#include "Libs/OptPin.h"
#include "Libs/uart.h"
#include "Libs/ProgmemLate.h"
#include "DallasTemp.h"

#pragma region Macros,vars,declaration

#define PC_COMM_INTERVAL 54932U	//*16,384 mS == 15 min
#define PC_RESET_PULSE_TIME 100	//mS
#define DEVICE_NAME "SwitchController"
#define DEVICE_DESC "Labels L|I/O inversion I|Output O|Mapping M|Override V|Read R|Input normal N|Temperature limit T|Probe P|Help H"
#define OW_DDR DDRB
#define OW_PIN 2

//Pin number-related things
using inputs_t = uint8_t;
#define EEPROM_READ_INPUTS(var) eeprom_read_byte(&var)
#define EEPROM_WRITE_INPUTS(var, val) eeprom_update_byte(&var, val)
#define BVI(val) static_cast<inputs_t>(static_cast<inputs_t>(1U) << (val))
using outputs_t = uint8_t;
#define EEPROM_READ_OUTPUTS(var) eeprom_read_byte(&var)
#define EEPROM_WRITE_OUTPUTS(var, val) eeprom_update_byte(&var, val)
#define BVO(val) static_cast<outputs_t>(static_cast<outputs_t>(1U) << (val))

//Global
static const char line_feed[] = "\r\n";
static const char ok[] = "OK";
static inputs_t input_compare_buf;
static inputs_t input_state_buf = 0;
static inputs_t last_input_diff = 0;
static outputs_t output_compare_buf;
static outputs_t override_comapre_buf;
static inputs_t input_inv_buf;
static outputs_t output_inv_buf;
static uint8_t ow_count;
static uint8_t ow_devices[CHAR_BIT * 8];
static int8_t temperatures[CHAR_BIT];
static int8_t temp_high_buf;
static uint8_t ow_alarm = 0;
static volatile uint16_t time_cnt = 0;
static uint16_t last_comm_time = 0;

//EEPROM+PROGMEM section
EEMEM uint8_t is_first_run = 2;
EEMEM inputs_t input_normal = 0xFF;
EEMEM outputs_t output_last = 0;
EEMEM outputs_t manual_override = 0;
EEMEM inputs_t inputs_inverted = 0;
EEMEM outputs_t outputs_inverted = 0;
EEMEM uint8_t temp_high = 0x28; //40
EEMEM uint8_t txt_in1[16] = { 'I', 'n', ' ', '1', '\0' };
EEMEM uint8_t txt_in2[sizeof(txt_in1)] = { 'I', 'n', ' ', '2', '\0' };
EEMEM uint8_t txt_in3[sizeof(txt_in1)] = { 'I', 'n', ' ', '3', '\0' };
EEMEM uint8_t txt_in4[sizeof(txt_in1)] = { 'I', 'n', ' ', '4', '\0' };
EEMEM uint8_t txt_in5[sizeof(txt_in1)] = { 'I', 'n', ' ', '5', '\0' };
EEMEM uint8_t txt_in6[sizeof(txt_in1)] = { 'I', 'n', ' ', '6', '\0' };
EEMEM uint8_t txt_in7[sizeof(txt_in1)] = { 'B', 'u', 't', 't', 'o', 'n', '\0' };
EEMEM uint8_t txt_out1[sizeof(txt_in1)] = { 'M', 'O', 'S', 'F', 'E', 'T', '\0' };
EEMEM uint8_t txt_out2[sizeof(txt_in1)] = { 'O', 'u', 't', ' ', '2', '\0' };
EEMEM uint8_t txt_out3[sizeof(txt_in1)] = { 'O', 'u', 't', ' ', '3', '\0' };
EEMEM uint8_t txt_out4[sizeof(txt_in1)] = { 'O', 'u', 't', ' ', '4', '\0' };
EEMEM uint8_t txt_out5[sizeof(txt_in1)] = { 'L', 'E', 'D', '\0' };
EEMEM uint8_t txt_ds1[sizeof(txt_in1)] = { 'D', 'S', ' ', '1', '\0' };
EEMEM uint8_t txt_ds2[sizeof(txt_in1)] = { 'D', 'S', ' ', '2', '\0' };
EEMEM uint8_t txt_ds3[sizeof(txt_in1)] = { 'D', 'S', ' ', '3', '\0' };
EEMEM uint8_t txt_ds4[sizeof(txt_in1)] = { 'D', 'S', ' ', '4', '\0' };
EEMEM uint8_t txt_ds5[sizeof(txt_in1)] = { 'D', 'S', ' ', '5', '\0' };
EEMEM uint8_t txt_ds6[sizeof(txt_in1)] = { 'D', 'S', ' ', '6', '\0' };
EEMEM uint8_t txt_ds7[sizeof(txt_in1)] = { 'D', 'S', ' ', '7', '\0' };
EEMEM uint8_t txt_ds8[sizeof(txt_in1)] = { 'D', 'S', ' ', '8', '\0' };
PROGMEM_LATE uint8_t* const ds_names[] = { &(txt_ds1[0]), &(txt_ds2[0]), &(txt_ds3[0]), &(txt_ds4[0]), &(txt_ds5[0]), &(txt_ds6[0]), &(txt_ds7[0]), &(txt_ds8[0]) };
static const pin_t input_pins[] = { pin_t(DDRC, 0, &(txt_in1[0])), pin_t(DDRC, 1, &(txt_in2[0])), pin_t(DDRC, 2, &(txt_in3[0])),
									pin_t(DDRC, 3, &(txt_in4[0])), pin_t(DDRC, 4, &(txt_in5[0])), pin_t(DDRC, 5, &(txt_in6[0])), pin_t(DDRD, 5, &(txt_in7[0])) };
static const pin_t output_pins[] = { pin_t(DDRB, 4, &(txt_out1[0])), pin_t(DDRB, 0, &(txt_out2[0])), pin_t(DDRD, 7, &(txt_out3[0])), pin_t(DDRD, 6, &(txt_out4[0])),
								pin_t(DDRD, 4, &(txt_out5[0])) };
EEMEM uint8_t temp_auto_mapping[arraySize(temperatures)];
static int8_t temp_mapping_buf[arraySize(temp_auto_mapping)];
//EEMEM uint8_t temp_auto_disengage;
//static bool temp_disengage_buf;
EEMEM uint8_t pc_reset_mapping;
static int8_t pc_reset_buf;
EEMEM uint8_t auto_on_mapping[arraySize(input_pins)];
static int8_t mapping_compare_buf[arraySize(auto_on_mapping)];
EEMEM uint8_t dev_name[sizeof(DEVICE_NAME)] = DEVICE_NAME;
EEMEM uint8_t dev_desc[sizeof(DEVICE_DESC)] = DEVICE_DESC;
static pin_t ow_pin = pin_t(OW_DDR, OW_PIN, &(txt_ds8[0]));

//Declarations
void soft_reset(void);
void checkPCDowntime(void);
void processOWDevices(void);
void poll(void);
void actuate(outputs_t);
void actuate(uint8_t, bool);
void reaction(uint8_t, bool);
void cmd(char);
void flushSerial(void);
void reportError(PGM_P);
bool check_index_helper(int8_t, uint8_t, bool msg = true);
void l_helper(char* &p, const uint8_t n);

#pragma endregion Macros,vars,declaration

#ifdef DEBUG
void printMem()
{
	char buf[8];
	uart_puts(itoa(freeMemory(), buf, 10));
}
#endif

//Interrupts

ISR(TIMER0_OVF_vect)
{
	++time_cnt;	//Increment main timer (used for PC reset)
}

//Main

void soft_reset()
{
	while (!uart_ready()) _delay_ms(10);	//Make sure everything is sent
	cli();	//Disable interrupts, thus UART won't be able to receive commands before reset
	while (uart_getc() != UART_NO_DATA);	//Make sure we won't receive some partial commands once restarted (no delay needed due to previous line)
	while(true);	//Let WDT do its job
	__builtin_unreachable();
}

void loadEEPROM()
{
	uint8_t ifr = eeprom_read_byte(&is_first_run);	//Read first run indicator byte
	if (ifr == 1_ui8)	//If not first (or factory-reset) run actually read EEPROM
	{
		input_compare_buf = EEPROM_READ_INPUTS(input_normal);
		override_comapre_buf = EEPROM_READ_OUTPUTS(manual_override);
		actuate(EEPROM_READ_OUTPUTS(output_last));
		input_inv_buf = EEPROM_READ_INPUTS(inputs_inverted);
		output_inv_buf = EEPROM_READ_OUTPUTS(outputs_inverted);
		uint8_t temp;
		for (uint8_t i = 0; i < arraySize(mapping_compare_buf); ++i)
		{
			temp = eeprom_read_byte(&(auto_on_mapping[i]));
			mapping_compare_buf[i] = *reinterpret_cast<int8_t*>(&temp);
		}
		for (uint8_t i = 0; i < arraySize(temp_mapping_buf); ++i)
		{
			temp = eeprom_read_byte(&(temp_auto_mapping[i]));
			temp_mapping_buf[i] = *reinterpret_cast<int8_t*>(&temp);
		}
		temp = eeprom_read_byte(&temp_high);
		temp_high_buf = *reinterpret_cast<int8_t*>(&temp);
		temp = eeprom_read_byte(&pc_reset_mapping);
		pc_reset_buf = *reinterpret_cast<int8_t*>(&temp);
	}
	else //If it is first run populate EEPROM with defaults
	{
		//Regular variables
		EEPROM_WRITE_INPUTS(input_normal, 0xFF);
		EEPROM_WRITE_OUTPUTS(manual_override, 0x00);
		EEPROM_WRITE_OUTPUTS(output_last, 0x00);
		EEPROM_WRITE_INPUTS(inputs_inverted, 0x00);
		EEPROM_WRITE_OUTPUTS(outputs_inverted, 0x00);
		//Those, which require reinterpret casts
		int8_t negative_one = -1;
		for (uint8_t i = 0; i < arraySize(auto_on_mapping); ++i) eeprom_update_byte(&(auto_on_mapping[i]), *reinterpret_cast<uint8_t*>(&negative_one));
		for (uint8_t i = 0; i < arraySize(temp_auto_mapping); ++i) eeprom_update_byte(&(temp_auto_mapping[i]), *reinterpret_cast<uint8_t*>(&negative_one));
		eeprom_update_byte(&pc_reset_mapping, *reinterpret_cast<uint8_t*>(&negative_one));
		eeprom_update_byte(&temp_high, 0x28);
		//Set the flag and
		eeprom_write_byte(&is_first_run, 1_ui8);
		loadEEPROM();
	}
}

int main(void)
{
	static_assert(!((sizeof(inputs_t) * CHAR_BIT) < arraySize(input_pins) || (sizeof(outputs_t) * CHAR_BIT) < arraySize(output_pins)),
		"Too many inputs/outputs for current types of the variables.");
	//Setup watchdog
	wdt_reset();
	wdt_enable(WDTO_1S);
	//Setup timer
	// Clock source: System Clock
	// Clock value: 15,625 kHz
	TCCR0 = 0x05;
	TCNT0 = 0x00;
	TIMSK = 0x01;	//Overflow interrupt
	//Setup pins
	loadEEPROM();
	for (uint8_t i = 0; i < arraySize(output_pins); ++i) pin_t::SetDirection(&(output_pins[i]), pin_t::Output);
	//Setup serial interface
	sei();
	uart_init(UART_BAUD_SELECT(9600, F_CPU));
	//Discover 1-wire devices
	wdt_reset();
	OWbegin(&ow_pin);
	ow_count = searchForDevices(ow_devices);
	for (uint8_t i = 0; i < ow_count; ++i) setResolution(ow_devices + i * 8_ui8);
	//Request temperature conversions
	wdt_reset();
	requestTemperatures();
	_delay_ms(100);
	//Ready
	wdt_reset();
	flushSerial();
	cmd('P');

	unsigned int c;
	while(true)
	{
		wdt_reset();
		poll();
		wdt_reset();
		processOWDevices();
		wdt_reset();
		c = uart_getc();
		if (c != UART_NO_DATA)
		{
			last_comm_time = time_cnt;
			_delay_ms(2);
			cmd(static_cast<char>(c));
			wdt_reset();
		}
		else
		{
			checkPCDowntime();
		}
		_delay_ms(200);
	}
}

void checkPCDowntime()
{
	if (check_index_helper(pc_reset_buf, arraySize(output_pins), false)) return;
	if ((time_cnt - last_comm_time) > PC_COMM_INTERVAL)
	{
		last_comm_time = time_cnt;
		actuate(pc_reset_buf, true);
		_delay_ms(PC_RESET_PULSE_TIME);
		actuate(pc_reset_buf, false);
	}
}

void processOWDevices()
{
	char buf[16];
	for (uint8_t i = 0; i < ow_count; ++i)
	{
		uint8_t ind = i * 8_ui8;
		if (ow_devices[ind] != 0x28) continue;	//Not a DS18B20
		int16_t t = static_cast<int16_t>(getTemp(ow_devices + ind));
		if (abs(t) < 127)
		{
			temperatures[i] = static_cast<int8_t>(t);
			if (t > temp_high_buf)
			{
				if (!(ow_alarm & BV8(i)))
				{
					uart_puts("T:A:");
					uart_putc(NumToASCII(i));
					uart_putc(':');
					uart_puts(itoa(t, buf, 10));
					uart_puts(line_feed);
					ow_alarm |= BV8(i);
					if (!check_index_helper(temp_mapping_buf[i], arraySize(output_pins), false)) actuate(temp_mapping_buf[i], true);
				}
			}
			else
			{
				if (ow_alarm & BV8(i))
				{
					uart_puts("T:S:");
					uart_putc(NumToASCII(i));
					uart_putc(':');
					uart_puts(itoa(t, buf, 10));
					uart_puts(line_feed);
					ow_alarm &= ~BV8(i);
					if (!check_index_helper(temp_mapping_buf[i], arraySize(output_pins), false))
					{
						if (!(override_comapre_buf & BVO(temp_mapping_buf[i]))) actuate(temp_mapping_buf[i], false);
					}
				}
			}
		}
		else
		{
			uart_puts_P("E:Temp. ");
			uart_puts(itoa(t, buf, 10));
			uart_putc('@');
			uart_puts_EP(&(ds_names[i]), sizeof(txt_in1));
			uart_puts(line_feed);
		}
	}
	requestTemperatures();
}

void poll()
{
	uint8_t i;
	input_state_buf = 0;
	for (i = 0; i < arraySize(input_pins); ++i)	if (pin_t::Read(&(input_pins[i])) != ((input_inv_buf & BVI(i)) > 0)) input_state_buf |= BVI(i);
	inputs_t res = input_state_buf ^ input_compare_buf;
	inputs_t temp = res ^ last_input_diff;
	for (i = 0; i < arraySize(input_pins); ++i) if (temp & BVI(i)) reaction(i, res & BVI(i));
	last_input_diff = res;
}

void actuate(outputs_t states)
{
	for (uint8_t i = 0; i < arraySize(output_pins); ++i) pin_t::Write(&(output_pins[i]), ((states & BVO(i)) > 0) != ((output_inv_buf & BVO(i)) > 0));
	output_compare_buf = states;
}
void actuate(uint8_t pin_index, bool state)
{
	state = (state != ((output_inv_buf & BVO(pin_index)) > 0));
	pin_t::Write(&(output_pins[pin_index]), state);
	state ? output_compare_buf |= BVO(pin_index) : output_compare_buf &= static_cast<outputs_t>(~BVO(pin_index));
}

void reaction(uint8_t pin_index, bool active)
{
	//Send alarm message to PC
	char buf[4];
	uart_puts(active ? "A:" : "S:");
	uart_puts(itoa(pin_index, buf, 10));
	uart_puts(line_feed);
	//Binded outputs
	if (!check_index_helper(mapping_compare_buf[pin_index], arraySize(output_pins), false))
	{
		if (active || !(override_comapre_buf & BVO(mapping_compare_buf[pin_index])))
		{
			for (uint8_t i = 0; i < arraySize(mapping_compare_buf); ++i)
			{
				if (( pin_index != i ) && ( mapping_compare_buf[i] == mapping_compare_buf[pin_index] )
				&& ( (input_state_buf ^ input_compare_buf) & BVI(i) )) return;
			}
			actuate(mapping_compare_buf[pin_index], active);
		}
	}
}

#pragma region Helpers

//Populates the buffer with characters (including - and ., until \r or \0 or N/A) and returns argument letter
unsigned int cmd_helper(char* b, uint8_t size)
{
	uint8_t i = 0;
	unsigned int c = uart_getc();
	_delay_ms(1);
	if (static_cast<char>(c) == ':') c = uart_getc(); //Just a meaningless part of a header
	if (c == UART_NO_DATA)
	{
		reportError(PSTR_L("No data"));
		return UART_NO_DATA;
	}
	char read = static_cast<char>(c);
	if (isNumeric(read) || (read == '-') || (read == '.')) b[i++] = read;
	for (; i < size; i++)
	{
		_delay_ms(2);
		c = uart_getc();
		if (c != UART_NO_DATA)
		{
			b[i] = static_cast<char>(c);
			if ((b[i] == '\r') || (b[i] == '\n'))
			{
				b[i] = '\0';
				//flushSerial(); reportError takes care about it
				if (static_cast<char>(uart_peek()) == '\n') uart_getc();
				break;
			}
		}
		else { break; };
	}
	if (b[i] != '\0') //If no line ending mark or null character itself
	{
		reportError((i == size) ? PSTR_L("Input too long") : PSTR_L("No CRLF or NULL"));
		return UART_NO_DATA; //Abort transaction
	}
	return read;
}

//Checks if index value is inside supported range.
bool check_index_helper(int8_t val, uint8_t size, bool msg)
{
	if ((val < 0) || (val >= size))
	{
		if (msg) reportError(PSTR_L("Illegal index"));
		return true;
	}
	return false;
}

//For R/W virtual registers
template<typename T> bool cmd_helper_2(T* var, char* buf, uint8_t size, unsigned int c, uint8_t arr_size)
{
	if (static_cast<char>(c) == 'A')
	{
		intToBit(*var, buf);
		uart_puts(buf);
		return true;
	}
	if (*buf == ':') ++buf;
	uint8_t l = strnlen(buf, size);	//Argument letter has already been parsed out, only index:value left
	uint8_t i = arraySearch(buf, ':', l);
	int8_t n;
	if (i < l)
	{
		n = decodeInt(buf, 0, i);	//index goes first
		if (check_index_helper(n, arr_size)) return true;
		if (buf[++i] == '0') { *var &= static_cast<T>(~(static_cast<T>(1U) << n)); }
		else { *var |= static_cast<T>(static_cast<T>(1U) << n); };
	}
	else
	{
		if (l < (sizeof(T) * CHAR_BIT))
		{
			reportError(PSTR_L("Too short input"));
			return true;
		}
		else { bitToInt(var, buf); };
	}
	return false;
}

//For Mappings. Any Mapping command with 'C' argument refers to PC reset.
bool cmd_helper_3(int8_t* data, uint8_t size, char* buf, uint8_t s, unsigned int c, uint8_t* eep)
{
	if (static_cast<char>(c) == 'A')	//Read values
	{
		char* p = buf;
		for (uint8_t i = 0; i < size; ++i)
		{
			if (data[i] < 0_ui8)
			{
				*p++ = '-';
				*p++ = ',';
			}
			else
			{
				l_helper(p, data[i]);
			}
		}
		*(--p) = '\0';
		uart_puts(buf);
		return true;
	}
	uint8_t l = strnlen(buf, s);	//Check arguments
	uint8_t i = arraySearch(buf, ':', l);
	if (i == l)
	{
		reportError(PSTR_L("No argument"));
		return true;
	}
	if (static_cast<char>(c) == 'C')	//Process PC reset
	{
		++i;
		if (buf[i] == 'A')
		{
			char* p = buf;
			if (pc_reset_buf < 0_ui8)
			{
				*p++ = '-';
			}
			else
			{
				if (pc_reset_buf > 9_ui8) *p++ = '1';
				*p++ = NumToASCII(pc_reset_buf % 10_ui8);
			}
			*p = '\0';
			uart_puts(buf);
			return true;
		}
		int8_t o = decodeInt(buf, i, l - i); //Process output number to be binded
		if (check_index_helper(o, arraySize(output_pins), false)) o = -1;
		pc_reset_buf = o;
		eeprom_update_byte(&pc_reset_mapping, *reinterpret_cast<uint8_t*>(&o));
		return false;
	}
	//Actually process mapping data
	int8_t n = decodeInt(buf, 0, i++); //Process input number
	if (check_index_helper(n, size)) return true;
	int8_t o = decodeInt(buf, i, l - i); //Process output number to be binded
	if (check_index_helper(o, arraySize(output_pins), false)) o = -1;
	data[n] = o;
	eeprom_update_byte(eep + n, *reinterpret_cast<uint8_t*>(&o)); //Save
	return false;
}

//Returns true if parameter c is not an argument separator character
bool arg_check_helper(unsigned int c)
{
	if (static_cast<char>(c) != ':')
	{
		reportError(PSTR_L("Missing argument"));
		return true;
	}
	return false;
}

//Useful for Listings and Mappings (comma separated integer[0..19] output), increments passed string pointer
void l_helper(char* &p, const uint8_t n)
{
	if (n > 9_ui8) *p++ = '1';
	*p++ = NumToASCII(n % 10_ui8);
	*p++ = ',';
}

//Returns true if I/O index is missing. Parses index into passed integer variable. Messes with string buffer!
bool l_decode_helper(char* p, uint8_t &i)
{
	_delay_ms(1);
	p[0] = static_cast<char>(uart_getc());
	_delay_ms(1);
	p[1] = static_cast<char>(uart_getc());
	if ((p[1] == ':') || (p[1] == '\r'))
	{
		i = ASCIIToNum(p[0]);
	}
	else
	{
		_delay_ms(1);
		if (arg_check_helper(uart_getc())) return true;
		p[2] = '\0';
		i = decodeInt(p);
	}
	return false;
}

//Returns EEPROM pointers to the name of specified (type + index) I/O-entity
uint8_t* l_switch_helper(char a, uint8_t i)
{
	switch (a)
	{
		case 'T':
		return pgm_read_EEM_P(ds_names[i]);
		case 'O':
		return output_pins[i].Text;
		case 'I':
		default:
		return input_pins[i].Text;
	}
}

#pragma endregion Helpers

void cmd(char first_letter)
{
	char buf[32];
	switch (first_letter)
	{
		case 'F':
			uart_puts("F:");
			if (static_cast<char>(cmd_helper(buf, arraySize(buf))) == '1')	//cmd_helper returns UART_NO_DATA == 0x0100 --> 0x00 == '\0' only
			{
				eeprom_write_byte(&is_first_run, 0);
				uart_puts(ok);
				uart_puts(line_feed);
				soft_reset();	//Makes all the code below unreachable without else
				__builtin_unreachable();
			}
			uart_puts("False");
			break;
		case 'A':
			uart_puts_P("A:Resetting...\r\n");
			soft_reset();
			__builtin_unreachable();
			break;
		case 'P': //Show presence
			uart_puts("P:");
			//flushSerial(); -?
			uart_puts_E(dev_name, arraySize(dev_name));
			break;
		case 'H':
		{
			uart_puts("H:");
			for (uint8_t i = 0; i < sizeof(DEVICE_DESC); i += sizeof(txt_in1)) uart_puts_E(dev_desc + i, sizeof(txt_in1));
			break;
		}
		case 'L':	//Signal label. L:{I/O/T}:#[#]:"<=16"\r[\n]; L:{A=E}[:{I/O/T}:#[#]][\r[\n]]
		{
			uart_puts("L:");
			_delay_ms(1);
			if (arg_check_helper(uart_getc())) break;
			_delay_ms(1);
			char a = static_cast<char>(uart_getc());
			_delay_ms(1);
			if (a == 'A')	//A: reading mode (EEPROM)
			{
				if (static_cast<char>(uart_getc()) != ':')	//No arguments but E: get number of signals monitored
				{
					flushSerial();
					char* p = buf;
					l_helper(p, arraySize(input_pins));
					l_helper(p, arraySize(output_pins));
					*p++ = NumToASCII(ow_count);
					*p = '\0';
					uart_puts(buf);
					break;
				}
				_delay_ms(1);
				a = static_cast<char>(uart_getc());		//Get name of specified signal
				_delay_ms(1);
				if (arg_check_helper(uart_getc())) break;
				uint8_t i;
				if (l_decode_helper(buf, i)) break;
				_delay_ms(1);
				if (static_cast<char>(uart_peek()) == '\n') uart_getc();
				eeprom_read_block((void*)buf, (void*)l_switch_helper(a, i), sizeof(txt_in1));
				uart_puts(buf);
				break;
			}
			if (arg_check_helper(uart_getc())) break;
			uint8_t i;
			if (l_decode_helper(buf, i)) break;
			if (check_index_helper(i, sizeof(txt_in1))) break;
			_delay_ms(1);
			unsigned int c = uart_getc();
			uint8_t j = 0;
			while ((static_cast<char>(c) != '\r') && (c != UART_NO_DATA))
			{
				buf[j++] = static_cast<char>(c);
				if (j == sizeof(txt_in1))
				{
					reportError(PSTR_L("Too long argument"));
					break;
				}
				_delay_ms(1);
				c = uart_getc();
			}
			_delay_ms(1);
			if (static_cast<char>(uart_peek()) == '\n') uart_getc();
			buf[j++] = '\0';
			eeprom_update_block((void*)buf, (void*)l_switch_helper(a, i), j);
			uart_puts(ok);
			break;
		}
		case 'I': //Input inversion
		{
			uart_puts("I:");
			unsigned int c = cmd_helper(buf, arraySize(buf));
			if (c == UART_NO_DATA) break;
			char a = (*buf == ':') ? *(buf + 1) : *buf;
			if (static_cast<char>(c) == 'O')
			{
				if (cmd_helper_2(&output_inv_buf, buf, arraySize(buf), a, arraySize(output_pins))) break;
				EEPROM_WRITE_INPUTS(outputs_inverted, output_inv_buf); //Buffer is never changed outside of this routine
			}
			else
			{
				if (cmd_helper_2(&input_inv_buf, buf, arraySize(buf), a, arraySize(input_pins))) break;
				EEPROM_WRITE_INPUTS(inputs_inverted, input_inv_buf); //Buffer is never changed outside of this routine
			}
			uart_puts(ok);
			break;
		}
		case 'V': //Manual output override
		{
			uart_puts("V:");
			unsigned int c = cmd_helper(buf, arraySize(buf));
			if (c == UART_NO_DATA) break;
			if (cmd_helper_2(&override_comapre_buf, buf, arraySize(buf), c, arraySize(output_pins))) break;
			EEPROM_WRITE_OUTPUTS(manual_override, override_comapre_buf); //Buffer is never changed outside of this routine
			uart_puts(ok);
			break;
		}
		case 'N': //Normal input state
		{
			uart_puts("N:");
			unsigned int c = cmd_helper(buf, arraySize(buf));
			if (c == UART_NO_DATA) break;
			if (cmd_helper_2(&input_compare_buf, buf, arraySize(buf), c, arraySize(input_pins))) break;
			EEPROM_WRITE_INPUTS(input_normal, input_compare_buf); //Buffer is never changed outside of this routine
			uart_puts(ok);
			break;
		}
		case 'M': //Map output to input
		{
			uart_puts("M:");
			unsigned int c = cmd_helper(buf, arraySize(buf));
			if (c == UART_NO_DATA) break;
			if (cmd_helper_3(mapping_compare_buf, arraySize(mapping_compare_buf), buf, arraySize(buf), c, auto_on_mapping)) break;
			uart_puts(ok);
			break;
		}
		case 'G':
		{
			uart_puts("G:");
			unsigned int c = cmd_helper(buf, arraySize(buf));
			if (c == UART_NO_DATA) break;
			if (cmd_helper_3(temp_mapping_buf, arraySize(temp_mapping_buf), buf, arraySize(buf), c, temp_auto_mapping)) break;
			uart_puts(ok);
			break;
		}
		case 'R': //Read
		{
			uart_puts("R:"); //Header is common for all modes
			unsigned int c = cmd_helper(buf, arraySize(buf));
			if (c == UART_NO_DATA) break;
			if (*buf == ':') strrmv(buf, 1);
			if (!isNumeric(buf, arraySize(buf)))
			{
				reportError(PSTR_L("Out of range"));
				break;
			}
			if (static_cast<char>(c) == 'T')
			{
				int8_t n = decodeInt(buf);
				if (check_index_helper(n, ow_count)) break;
				uart_puts(itoa(temperatures[n], buf, 10));
			}
			else
			{
				intToBit(input_state_buf, buf);
				uart_puts(buf);
				uart_putc(':');
				intToBit(output_compare_buf, buf);
				uart_puts(buf);
			}
			break;
		}
		case 'O': //Turn output on
		{
			//Detect explicit on/off flag
			uart_puts("O:");
			unsigned int c = cmd_helper(buf, arraySize(buf));
			if (c == UART_NO_DATA) break; //Error-handling is inside of the helper routine
			if (static_cast<char>(c) == 'E')
			{
				intToBit(EEPROM_READ_OUTPUTS(output_last), buf);
				uart_puts(buf);
				break;
			}
			if (*buf == ':') strrmv(buf, 1);
			//Parse index:state
			uint8_t l = strnlen(buf, sizeof(buf));
			uint8_t i = arraySearch(buf, ':', l);
			if (i < l)
			{
				l = decodeInt(buf, 0, i);	//index goes first
				if (check_index_helper(l, arraySize(output_pins))) break;
			}
			else
			{
				reportError(PSTR_L("Missing argument"));
				break;
			}
			bool eep = false;
			switch (buf[i + 1])
			{
				case 'N':
					eep = true;
				case '1':
					actuate(l, true); //Set determined state
				break;
				case 'F':
					eep = true;
				case '0':
					actuate(l, false); //Set determined state
				break;
				default:
					actuate(output_compare_buf ^ BVO(l)); //Toggle
				break;
			}
			if (eep)
			{
				outputs_t temp = EEPROM_READ_OUTPUTS(output_last);
				if (output_compare_buf & BVO(l))
				{
					temp |= BVO(l);
				}
				else
				{
					temp &= static_cast<outputs_t>(~BVO(l));
				}
				EEPROM_WRITE_OUTPUTS(output_last, temp); //Save (only user-initiated output control is saved)
			}
			uart_puts(ok);
			break;
		}
		case 'T':
		{
			uart_puts("T:");
			unsigned int c = cmd_helper(buf, sizeof(buf));
			if (c == UART_NO_DATA) break;
			if (static_cast<char>(c) == 'A')
			{
				itoa(temp_high_buf, buf, 10);
				uart_puts(buf);
			}
			else
			{
				temp_high_buf = decodeInt(buf);
				eeprom_update_byte(&temp_high, *reinterpret_cast<uint8_t*>(&temp_high_buf));
				uart_puts(ok);
			}
			break;
		}
		default:
		return;
	}
	uart_puts(line_feed);
}

void flushSerial()
{
	_delay_ms(8);	//10 causes bug in Proteus (?)
	while (uart_getc() != UART_NO_DATA) _delay_ms(2);
}

void reportError(PGM_P desc)
{
	uart_puts("E:");
	uart_puts_p(desc);
	uart_puts(line_feed);
	flushSerial();
}
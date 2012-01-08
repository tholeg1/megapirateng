extern "C" {
  // AVR LibC Includes
  #include <inttypes.h>
  #include <avr/interrupt.h>
  #include "WConstants.h"
}

#include <FastSerial.h>

#include <AP_Common.h>
#include <AP_Math.h>		// ArduPilot Mega Vector/Matrix math Library
#include <I2C.h>
#include "AP_Baro_BMP085_Pirates.h"

#define BMP085_ADDRESS 0x77  //(0xEE >> 1)
#define READ_PRESS_TIMEOUT 25000 // reading press timeout, if oss=3, timout = 25ms
#define READ_TEMP_TIMEOUT 4500 // reading temp timeout, timout = 4,5ms

uint8_t AP_Baro_BMP085_Pirates::oss = 3;	// Over Sampling setting 3 = High resolution
bool AP_Baro::healthy = false;

// Public Methods //////////////////////////////////////////////////////////////
bool AP_Baro_BMP085_Pirates::init( AP_PeriodicProcess * scheduler )
{
	scheduler->stop();
	delay(10);
	init_hardware();
	scheduler->register_process( &AP_Baro_BMP085_Pirates::_update );
	scheduler->start();

	return healthy;
}

void AP_Baro_BMP085_Pirates::init_hardware()
{
	byte buff[22];

	// We read the calibration data registers
	if (I2c.read(BMP085_ADDRESS, 0xAA, 22, buff) != 0) {
		healthy = false;
		return;
	}

	ac1 = ((int)buff[0] << 8) | buff[1];
	ac2 = ((int)buff[2] << 8) | buff[3];
	ac3 = ((int)buff[4] << 8) | buff[5];
	ac4 = ((int)buff[6] << 8) | buff[7];
	ac5 = ((int)buff[8] << 8) | buff[9];
	ac6 = ((int)buff[10] << 8) | buff[11];
	b1 = ((int)buff[12] << 8) | buff[13];
	b2 = ((int)buff[14] << 8) | buff[15];
	mb = ((int)buff[16] << 8) | buff[17];
	mc = ((int)buff[18] << 8) | buff[19];
	md = ((int)buff[20] << 8) | buff[21];

	// Ensure we read right values at first reading
	Command_ReadTemp();
	delay(5);
	ReadTemp();	// On state 1 we read temp
	Command_ReadPress();
	delay(25);
	ReadPress();
	Calculate();
	
	Command_ReadTemp();	// Read Temp, first step in state machine, see below
}


static volatile long _baro_timer = 0;
/* Gates access to asynchronous state: */
static volatile bool _baro_updated = false;
static volatile uint8_t _baro_state = 1;
static volatile uint32_t _baro_read_timeout = READ_TEMP_TIMEOUT;

// Read the sensor. This is a state machine
// We read one time Temperature (state=1) and then 4 times Pressure (states 2-5)
void AP_Baro_BMP085_Pirates::_update(uint32_t tnow)
{
    if (tnow - _baro_timer < _baro_read_timeout) {
	    return; // wait for more than 10ms
    }

    _baro_timer = tnow;
		if (_baro_state == 1) {
			ReadTemp();	// On state 1 we read temp
	    _baro_state++;
			Command_ReadPress();
    } else if (_baro_state == 5) {
			ReadPress();
	    _baro_state = 1;	// Start again from state = 1
			Command_ReadTemp();	// Read Temp
	    _baro_updated = true;	// New pressure reading
    } else {
			ReadPress();
	    _baro_state++;
			Command_ReadPress();
	    _baro_updated = true;	// New pressure reading
    }
}

uint8_t AP_Baro_BMP085_Pirates::read()
{
	if (_baro_updated) {
		_baro_updated = false;
		Calculate();
	} else {
	}
	return healthy;
}

int32_t AP_Baro_BMP085_Pirates::get_pressure() {
    return Press;
}

int16_t AP_Baro_BMP085_Pirates::get_temperature() {
    return Temp;
}

float AP_Baro_BMP085_Pirates::get_altitude() {
    return 0.0; // TODO
}

int32_t AP_Baro_BMP085_Pirates::get_raw_pressure() {
    return RawPress;
}

int32_t AP_Baro_BMP085_Pirates::get_raw_temp() {
    return RawTemp;
}

// Private functions: /////////////////////////////////////////////////////////

// Send command to Read Pressure
void AP_Baro_BMP085_Pirates::Command_ReadPress()
{
	if (I2c.write(BMP085_ADDRESS, 0xF4, 0x34+(oss << 6)) != 0) {
		healthy = false;
	}
	_baro_read_timeout = READ_PRESS_TIMEOUT;
}

int32_t	AP_Baro_BMP085_Pirates::RawPress = 0;
int32_t	AP_Baro_BMP085_Pirates::_offset_press = 0;
int	AP_Baro_BMP085_Pirates::_press_filter[PRESS_FILTER_SIZE];
uint8_t	AP_Baro_BMP085_Pirates::_press_index = 0;

// Read Raw Pressure values
void AP_Baro_BMP085_Pirates::ReadPress()
{
	uint8_t buf[3];

	if (I2c.read(BMP085_ADDRESS, 0xF6, 3, buf) != 0) {
		healthy = false;
		return;
	}

	healthy = true;
	
	RawPress = (((long)buf[0] << 16) | ((long)buf[1] << 8) | ((long)buf[2])) >> (8 - oss);

	if(_offset_press == 0){
		_offset_press = RawPress;
		RawPress = 0;
	} else{
		RawPress -= _offset_press;
	}
	// filter
	_press_filter[_press_index++] = RawPress;

	if(_press_index >= PRESS_FILTER_SIZE)
		_press_index = 0;

	RawPress = 0;

	// sum our filter
	for (uint8_t i = 0; i < PRESS_FILTER_SIZE; i++){
		RawPress += _press_filter[i];
	}

	// grab result
	RawPress /= PRESS_FILTER_SIZE;
	//RawPress >>= 3;
	RawPress += _offset_press;
}

// Send Command to Read Temperature
void AP_Baro_BMP085_Pirates::Command_ReadTemp()
{
	if (I2c.write(BMP085_ADDRESS, 0xF4, 0x2E) != 0) {
		healthy = false;
	}
	_baro_read_timeout = READ_TEMP_TIMEOUT;
}

int32_t AP_Baro_BMP085_Pirates::RawTemp = 0;
int	AP_Baro_BMP085_Pirates::_temp_filter[TEMP_FILTER_SIZE];
long	AP_Baro_BMP085_Pirates::_offset_temp = 0;
uint8_t	AP_Baro_BMP085_Pirates::_temp_index = 0;

// Read Raw Temperature values
void AP_Baro_BMP085_Pirates::ReadTemp()
{
	uint8_t buf[2];

	if (I2c.read(BMP085_ADDRESS, 0xF6, 2, buf) != 0) {
		healthy = false;
		return;
	}
	RawTemp = buf[0];
	RawTemp = (RawTemp << 8) | buf[1];

	healthy = true;

	if (_offset_temp == 0){
		_offset_temp = RawTemp;
		RawTemp = 0;
	} else {
		RawTemp -= _offset_temp;
	}

	// filter
	_temp_filter[_temp_index++] = RawTemp;

	if(_temp_index >= TEMP_FILTER_SIZE)
		_temp_index = 0;

	RawTemp = 0;
	// sum our filter
	for(uint8_t i = 0; i < TEMP_FILTER_SIZE; i++){
		RawTemp += _temp_filter[i];
	}

	// grab result
	RawTemp /= TEMP_FILTER_SIZE;
	//RawTemp >>= 4;
	RawTemp += _offset_temp;	
}

// Calculate Temperature and Pressure in real units.
void AP_Baro_BMP085_Pirates::Calculate()
{
	long x1, x2, x3, b3, b5, b6, p;
	unsigned long b4, b7;
	int32_t tmp;

	// See Datasheet page 13 for this formulas
	// Based also on Jee Labs BMP085 example code. Thanks for share.
	// Temperature calculations
	x1 = ((long)RawTemp - ac6) * ac5 >> 15;
	x2 = ((long) mc << 11) / (x1 + md);
	b5 = x1 + x2;
	Temp = (b5 + 8) >> 4;

	// Pressure calculations
	b6 = b5 - 4000;
	x1 = (b2 * (b6 * b6 >> 12)) >> 11;
	x2 = ac2 * b6 >> 11;
	x3 = x1 + x2;
	//b3 = (((int32_t) ac1 * 4 + x3)<<oss + 2) >> 2; // BAD
	//b3 = ((int32_t) ac1 * 4 + x3 + 2) >> 2;  //OK for oss=0
	tmp = ac1;
	tmp = (tmp*4 + x3)<<oss;
	b3 = (tmp+2)/4;
	x1 = ac3 * b6 >> 13;
	x2 = (b1 * (b6 * b6 >> 12)) >> 16;
	x3 = ((x1 + x2) + 2) >> 2;
	b4 = (ac4 * (uint32_t) (x3 + 32768)) >> 15;
	b7 = ((uint32_t) RawPress - b3) * (50000 >> oss);
	p = b7 < 0x80000000 ? (b7 * 2) / b4 : (b7 / b4) * 2;

	x1 = (p >> 8) * (p >> 8);
	x1 = (x1 * 3038) >> 16;
	x2 = (-7357 * p) >> 16;
	Press = p + ((x1 + x2 + 3791) >> 4);
}

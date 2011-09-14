// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

//
// NMEA parser, adapted by Michael Smith from TinyGPS v9:
//
// TinyGPS - a small GPS library for Arduino providing basic NMEA parsing
// Copyright (C) 2008-9 Mikal Hart
// All rights reserved.
//
// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 2.1 of the License, or (at your option) any later version.
//
// This library is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// Lesser General Public License for more details.
//

/// @file	GPS_Ublox_i2c.cpp
/// @brief	NMEA protocol parser
///
/// This is a lightweight NMEA parser, derived originally from the
/// TinyGPS parser by Mikal Hart.
///

#include <FastSerial.h>
#include <AP_Common.h>

#include <avr/pgmspace.h>
#include <ctype.h>
#include <stdint.h>

#include <Wire.h>

#include "GPS_Ublox_i2c.h"

#define GPS_ADDRESS 66  // Address of GPS module on i2c bus

// ublox init messages /////////////////////////////////////////////////////////
//
// Note that we will only see a ublox in NMEA mode if we are explicitly configured
// for NMEA.  GPS_AUTO will try to set any ublox unit to binary mode as part of
// the autodetection process.
//
// We don't attempt to send $PUBX,41 as the unit must already be talking NMEA
// and we don't know the baudrate.
//
const char* GPS_Ublox_i2c::_ublox_init_string[]  =
        {"$PUBX,40,GGA,1,0,0,0,0,0*5B\r\n",
		"$PUBX,40,VTG,1,0,0,0,0,0*5F\r\n",	
		"$PUBX,40,RMC,1,0,0,0,0,0*46\r\n",
        "$PUBX,40,GSA,0,0,0,0,0,0*4E\r\n",
        "$PUBX,40,GSV,0,0,0,0,0,0*59\r\n",
        "$PUBX,40,GLL,0,0,0,0,0,0*5C\r\n"};	
		
byte ublox_set_5hz[14]={0xB5 ,0x62 ,0x06 ,0x08 ,0x06 ,0x00 ,0xC8 ,0x00 ,
		   0x01 ,0x00 ,0x01 ,0x00 ,0xDE ,0x6A};  

// NMEA message identifiers ////////////////////////////////////////////////////
//
const char GPS_Ublox_i2c::_gprmc_string[] PROGMEM = "GPRMC";
const char GPS_Ublox_i2c::_gpgga_string[] PROGMEM = "GPGGA";
const char GPS_Ublox_i2c::_gpvtg_string[] PROGMEM = "GPVTG";

// Convenience macros //////////////////////////////////////////////////////////
//
#define DIGIT_TO_VAL(_x)	(_x - '0')

// Constructors ////////////////////////////////////////////////////////////////
GPS_Ublox_i2c::GPS_Ublox_i2c(Stream *s) :
	GPS(s)
{
}

// Public Methods //////////////////////////////////////////////////////////////
void GPS_Ublox_i2c::init(void)
{
	Wire.begin();
	
	Wire.beginTransmission(GPS_ADDRESS);
	Wire.send(ublox_set_5hz, 14);
	Wire.endTransmission();
	delay(100);
	
	for(int i = 0; i < 6; i++)
	{
		Wire.beginTransmission(GPS_ADDRESS);
		Wire.send((char *)_ublox_init_string[i]);
		Wire.endTransmission();
	}
	
    idleTimeout = 20000;
	gps_timeout = 0;
}

bool GPS_Ublox_i2c::read(void)
{
	int numc;
	bool parsed = false;
	if(millis() - gps_timeout < 50) {
		return false;
	}
	
	Wire.requestFrom(GPS_ADDRESS, 32);
	numc = Wire.available();
	while (numc--) {
		char c = Wire.receive();
		//Serial.print(c);
		if(c >= 0)
		{
			if (_decode(c)) {
				parsed = true;
			}
		}
	}
	gps_timeout = millis();
	return parsed;
}

bool GPS_Ublox_i2c::_decode(char c)
{
	bool valid_sentence = false;

	switch (c) {
	case ',': // term terminators
		_parity ^= c;
	case '\r':
	case '\n':
	case '*':
		if (_term_offset < sizeof(_term)) {
			_term[_term_offset] = 0;
			valid_sentence = _term_complete();
		}
		++_term_number;
		_term_offset = 0;
		_is_checksum_term = c == '*';
		return valid_sentence;

	case '$': // sentence begin
		_term_number = _term_offset = 0;
		_parity = 0;
		_sentence_type = _GPS_SENTENCE_OTHER;
		_is_checksum_term = false;
		_gps_data_good = false;
		return valid_sentence;
	}

	// ordinary characters
	if (_term_offset < sizeof(_term) - 1)
		_term[_term_offset++] = c;
	if (!_is_checksum_term)
		_parity ^= c;

	return valid_sentence;
}

//
// internal utilities
//
int GPS_Ublox_i2c::_from_hex(char a)
{
	if (a >= 'A' && a <= 'F')
		return a - 'A' + 10;
	else if (a >= 'a' && a <= 'f')
		return a - 'a' + 10;
	else
		return a - '0';
}

unsigned long GPS_Ublox_i2c::_parse_decimal()
{
	char *p = _term;
	unsigned long ret = 100UL * atol(p);
	while (isdigit(*p))
		++p;
	if (*p == '.') {
		if (isdigit(p[1])) {
			ret += 10 * (p[1] - '0');
			if (isdigit(p[2]))
				ret += p[2] - '0';
		}
	}
	return ret;
}

unsigned long GPS_Ublox_i2c::_parse_degrees()
{
	char *p, *q;
	uint8_t deg = 0, min = 0;
	unsigned int frac_min = 0;

	// scan for decimal point or end of field
	for (p = _term; isdigit(*p); p++)
		;
	q = _term;

	// convert degrees
	while ((p - q) > 2) {
		if (deg)
			deg *= 10;
		deg += DIGIT_TO_VAL(*q++);
	}

	// convert minutes
	while (p > q) {
		if (min)
			min *= 10;
		min += DIGIT_TO_VAL(*q++);
	}

	// convert fractional minutes
	// expect up to four digits, result is in
	// ten-thousandths of a minute
	if (*p == '.') {
		q = p + 1;
		for (int i = 0; i < 4; i++) {
			frac_min *= 10;
			if (isdigit(*q))
				frac_min += *q++ - '0';
		}
	}
	return deg * 100000UL + (min * 10000UL + frac_min) / 6;
}

// Processes a just-completed term
// Returns true if new sentence has just passed checksum test and is validated
bool GPS_Ublox_i2c::_term_complete()
{
	// handle the last term in a message
	if (_is_checksum_term) {
		uint8_t checksum = 16 * _from_hex(_term[0]) + _from_hex(_term[1]);
		if (checksum == _parity) {
			if (_gps_data_good) {
				switch (_sentence_type) {
				case _GPS_SENTENCE_GPRMC:
					time			= _new_time;
					date			= _new_date;
					if(_new_latitude!=0) fc_lat_prev=_new_latitude; else _new_latitude=fc_lat_prev;
					if(_new_longitude!=0) fc_lon_prev=_new_longitude; else _new_longitude=fc_lon_prev;
					latitude		= _new_latitude * 100;	// degrees*10e5 -> 10e7
					longitude		= _new_longitude * 100;	// degrees*10e5 -> 10e7
					ground_speed	= _new_speed;
					ground_course	= _new_course;
					fix				= true;
					break;
				case _GPS_SENTENCE_GPGGA:
					altitude		= _new_altitude;
					time			= _new_time;
					if(_new_latitude!=0) fc_lat_prev=_new_latitude; else _new_latitude=fc_lat_prev;
					if(_new_longitude!=0) fc_lon_prev=_new_longitude; else _new_longitude=fc_lon_prev;
					latitude		= _new_latitude * 100;	// degrees*10e5 -> 10e7
					longitude		= _new_longitude * 100;	// degrees*10e5 -> 10e7
					num_sats		= _new_satellite_count;
					hdop			= _new_hdop;
					fix				= true;
					break;
				case _GPS_SENTENCE_VTG:
					ground_speed	= _new_speed;
					ground_course	= _new_course;
					// VTG has no fix indicator, can't change fix status
					break;
				}
			} else {
				switch (_sentence_type) {
				case _GPS_SENTENCE_GPRMC:
				case _GPS_SENTENCE_GPGGA:
					// Only these sentences give us information about
					// fix status.
					fix = false;
				}
			}
			// we got a good message
			return true;
		}
		// we got a bad message, ignore it
		return false;
	}

	// the first term determines the sentence type
	if (_term_number == 0) {
		if (!strcmp_P(_term, _gprmc_string)) {
			_sentence_type = _GPS_SENTENCE_GPRMC;
		} else if (!strcmp_P(_term, _gpgga_string)) {
			_sentence_type = _GPS_SENTENCE_GPGGA;
		} else if (!strcmp_P(_term, _gpvtg_string)) {
			_sentence_type = _GPS_SENTENCE_VTG;
			// VTG may not contain a data qualifier, presume the solution is good
			// unless it tells us otherwise.
			_gps_data_good = true;
		} else {
			_sentence_type = _GPS_SENTENCE_OTHER;
		}
		return false;
	}

	// 10 = RMC, 20 = GGA, 30 = VTG
	if (_sentence_type != _GPS_SENTENCE_OTHER && _term[0]) {
		switch (_sentence_type + _term_number) {
		// operational status
		//
		case 12: // validity (RMC)
			_gps_data_good = _term[0] == 'A';
			break;
		case 26: // Fix data (GGA)
			_gps_data_good = _term[0] > '0';
			break;
		case 39: // validity (VTG) (we may not see this field)
			_gps_data_good = _term[0] != 'N';
			break;
		case 27: // satellite count (GGA)
			_new_satellite_count = atol(_term);
			break;
		case 28: // HDOP (GGA)
			_new_hdop = _parse_decimal();
			break;

			// time and date
			//
		case 11: // Time (RMC)
		case 21: // Time (GGA)
			_new_time = _parse_decimal();
			break;
		case 19: // Date (GPRMC)
			_new_date = atol(_term);
			break;

			// location
			//
		case 13: // Latitude
		case 22:
			_new_latitude = _parse_degrees();
			break;
		case 14: // N/S
		case 23:
			if (_term[0] == 'S')
				_new_latitude = -_new_latitude;
			break;
		case 15: // Longitude
		case 24:
			_new_longitude = _parse_degrees();
			break;
		case 16: // E/W
		case 25:
			if (_term[0] == 'W')
				_new_longitude = -_new_longitude;
			break;
		case 29: // Altitude (GPGGA)
			_new_altitude = _parse_decimal();
			break;

			// course and speed
			//
		case 17: // Speed (GPRMC)
			_new_speed = (_parse_decimal() * 514) / 1000; 	// knots-> m/sec, approximiates * 0.514
			break;
		case 37: // Speed (VTG)
			_new_speed = _parse_decimal();
			break;
		case 18: // Course (GPRMC)
		case 31: // Course (VTG)
			_new_course = _parse_decimal();
			break;
		}
	}

	return false;
}

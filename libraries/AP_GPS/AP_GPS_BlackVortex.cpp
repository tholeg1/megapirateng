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

/// @file	AP_GPS_BLACKVORTEX.cpp
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

#include "AP_GPS_BlackVortex.h"

// NMEA message identifiers ////////////////////////////////////////////////////
//
const char AP_GPS_BLACKVORTEX::_gprmc_string[] PROGMEM = "GPRMC";
const char AP_GPS_BLACKVORTEX::_gpgga_string[] PROGMEM = "GPGGA";
const char AP_GPS_BLACKVORTEX::_gpvtg_string[] PROGMEM = "GPVTG";

// Convenience macros //////////////////////////////////////////////////////////
//
#define DIGIT_TO_VAL(_x)	(_x - '0')

// Constructors ////////////////////////////////////////////////////////////////
AP_GPS_BLACKVORTEX::AP_GPS_BLACKVORTEX(Stream *s) :
	GPS(s)
{
	FastSerial	*fs = (FastSerial *)_port;

	// Re-open the port with enough receive buffering for the messages we expect
	// and very little tx buffering, since we don't care about sending.
	// Leave the port speed alone as we don't actually know at what rate we're running...
	//
	fs->begin(0, 200, 16);
}

// Public Methods //////////////////////////////////////////////////////////////
void AP_GPS_BLACKVORTEX::init(void)
{
	byte ublox_set_5hz[14]={0xB5 ,0x62 ,0x06 ,0x08 ,0x06 ,0x00 ,0xC8 ,0x00 ,
  				                0x01 ,0x00 ,0x01 ,0x00 ,0xDE ,0x6A};   
	byte ublox_set_384[28]={0xB5 ,0x62 ,0x06 ,0x00 ,0x14 ,0x00 ,0x01 ,0x00 ,
  			           	      0x00 ,0x00 ,0xD0 ,0x08 ,0x00 ,0x00 ,0x00 ,0x96 ,
                   				0x00 ,0x00 ,0x07 ,0x00 ,0x02 ,0x00 ,0x00 ,0x00 ,
                   				0x00 ,0x00 ,0x92 ,0x8A};
  uint8_t i;

	FastSerial	*fs = (FastSerial *)_port;

	if (callback == NULL) callback = delay;

	fs->begin(9600);
	for (i=0;i<14;i++) fs->write(ublox_set_5hz[i]);
	callback(100);
	for (i=0;i<28;i++) fs->write(ublox_set_384[i]);
	callback(1000);
	fs->begin(38400);

	idleTimeout = 1200;
}

bool AP_GPS_BLACKVORTEX::read(void)
{
	int numc;
	bool parsed = false;

	numc = _port->available();
	while (numc--) {
		if (_decode(_port->read())) {
			parsed = true;
		}
	}
	return parsed;
}

bool AP_GPS_BLACKVORTEX::_decode(char c)
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
int AP_GPS_BLACKVORTEX::_from_hex(char a)
{
	if (a >= 'A' && a <= 'F')
		return a - 'A' + 10;
	else if (a >= 'a' && a <= 'f')
		return a - 'a' + 10;
	else
		return a - '0';
}

unsigned long AP_GPS_BLACKVORTEX::_parse_decimal()
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

unsigned long AP_GPS_BLACKVORTEX::_parse_degrees()
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
bool AP_GPS_BLACKVORTEX::_term_complete()
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
				case _GPS_SENTENCE_GPVTG:
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
			_sentence_type = _GPS_SENTENCE_GPVTG;
			// VTG may not contain a data qualifier, presume the solution is good
			// unless it tells us otherwise.
			_gps_data_good = true;
		} else {
			_sentence_type = _GPS_SENTENCE_OTHER;
		}
		return false;
	}

	// 32 = RMC, 64 = GGA, 96 = VTG
	if (_sentence_type != _GPS_SENTENCE_OTHER && _term[0]) {
		switch (_sentence_type + _term_number) {
		// operational status
		//
		case _GPS_SENTENCE_GPRMC + 2: // validity (RMC)
			_gps_data_good = _term[0] == 'A';
			break;
		case _GPS_SENTENCE_GPGGA + 6: // Fix data (GGA)
			_gps_data_good = _term[0] > '0';
			break;
		case _GPS_SENTENCE_GPVTG + 9: // validity (VTG) (we may not see this field)
			_gps_data_good = _term[0] != 'N';
			break;
		case _GPS_SENTENCE_GPGGA + 7: // satellite count (GGA)
			_new_satellite_count = atol(_term);
			break;
		case _GPS_SENTENCE_GPGGA + 8: // HDOP (GGA)
			_new_hdop = _parse_decimal();
			break;

			// time and date
			//
		case _GPS_SENTENCE_GPRMC + 1: // Time (RMC)
		case _GPS_SENTENCE_GPGGA + 1: // Time (GGA)
			_new_time = _parse_decimal();
			break;
		case _GPS_SENTENCE_GPRMC + 9: // Date (GPRMC)
			_new_date = atol(_term);
			break;

			// location
			//
		case _GPS_SENTENCE_GPRMC + 3: // Latitude
		case _GPS_SENTENCE_GPGGA + 2:
			_new_latitude = _parse_degrees();
			break;
		case _GPS_SENTENCE_GPRMC + 4: // N/S
		case _GPS_SENTENCE_GPGGA + 3:
			if (_term[0] == 'S')
				_new_latitude = -_new_latitude;
			break;
		case _GPS_SENTENCE_GPRMC + 5: // Longitude
		case _GPS_SENTENCE_GPGGA + 4:
			_new_longitude = _parse_degrees();
			break;
		case _GPS_SENTENCE_GPRMC + 6: // E/W
		case _GPS_SENTENCE_GPGGA + 5:
			if (_term[0] == 'W')
				_new_longitude = -_new_longitude;
			break;
		case _GPS_SENTENCE_GPGGA + 9: // Altitude (GPGGA)
			_new_altitude = _parse_decimal();
			break;

			// course and speed
			//
		case _GPS_SENTENCE_GPRMC + 7: // Speed (GPRMC)
		case _GPS_SENTENCE_GPVTG + 5: // Speed (VTG)
			_new_speed = (_parse_decimal() * 514) / 1000; 	// knots-> m/sec, approximiates * 0.514
			break;
		case _GPS_SENTENCE_GPRMC + 8: // Course (GPRMC)
		case _GPS_SENTENCE_GPVTG + 1: // Course (VTG)
			_new_course = _parse_decimal();
			break;
		}
	}

	return false;
}

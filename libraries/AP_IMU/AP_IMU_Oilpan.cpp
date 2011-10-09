// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
//
//
//	AP_IMU.cpp - IMU Sensor Library for Ardupilot Mega
//		Code by Michael Smith, Doug Weibel, Jordi Muñoz and Jose Julio. DIYDrones.com
//
//	This library works with the ArduPilot Mega and "Oilpan"
//
//	This library is free software; you can redistribute it and/or
//	modify it under the terms of the GNU Lesser General Public
//	License as published by the Free Software Foundation; either
//	version 2.1 of the License, or (at your option) any later version.
//

/// @file	AP_IMU.h
/// @brief	IMU driver for the APM oilpan

#include <FastSerial.h>
#include <AP_Common.h>

#include <avr/eeprom.h>

#include "AP_IMU_Oilpan.h"

// XXX secret knowledge about the APM/oilpan wiring
//
#define A_LED_PIN   13
#define C_LED_PIN   30

// Sensors: GYROX, GYROY, GYROZ, ACCELX, ACCELY, ACCELZ
const uint8_t AP_IMU_Oilpan::_sensors[6]        = { 1, 2, 0, 4, 5, 6};	// Channel assignments on the APM oilpan
const int8_t  AP_IMU_Oilpan::_sensor_signs[6]	= {	1,-1,-1, 1,-1,-1};  // Channel orientation vs. normal

void
AP_IMU_Oilpan::init(Start_style style, void (*callback)(unsigned long t))
{
    // if we are warm-starting, load the calibration data from EEPROM and go
    //
    if (WARM_START == style) {
        _sensor_cal.load();
    } else {

        // do cold-start calibration for both accel and gyro
        _init_gyro(callback);
        _init_accel(callback);

        // save calibration
        _sensor_cal.save();
    }
}

/**************************************************/

void
AP_IMU_Oilpan::init_gyro(void (*callback)(unsigned long t))
{
    _init_gyro(callback);
    _sensor_cal.save();
}

void
AP_IMU_Oilpan::_init_gyro(void (*callback)(unsigned long t))
{
	int flashcount = 0;
	int tc_temp;
	float adc_in;
	float prev[3] = {0,0,0};
	float total_change;
	float max_offset;
  int adc_values[6];

	// cold start
	tc_temp = _adc->Ch(_gyro_temp_ch);
 	callback(500);
	Serial.printf_P(PSTR("Init Gyro"));

	for(int c = 0; c < 25; c++){				// Mostly we are just flashing the LED's here to tell the user to keep the IMU still
		digitalWrite(A_LED_PIN, LOW);
		digitalWrite(C_LED_PIN, HIGH);
		callback(20);

    _adc->Ch6(_sensors, adc_values);

		digitalWrite(A_LED_PIN, HIGH);
		digitalWrite(C_LED_PIN, LOW);
		callback(20);
	}

	for (int j = 0; j <= 2; j++)
	    _sensor_cal[j] = 500;		// Just a large value to load prev[j] the first time

	do {
    // get 6 sensor values
    _adc->Ch6(_sensors, adc_values);

		for (int j = 0; j <= 2; j++){
			prev[j]     = _sensor_cal[j];
			_sensor_cal[j]	= adc_values[j];
		}

		for(int i = 0; i < 50; i++){

      // get 6 sensor values
      _adc->Ch6(_sensors, adc_values);

			for (int j = 0; j < 3; j++){
				adc_in = adc_values[j];
				// filter
				_sensor_cal[j] = _sensor_cal[j] * 0.9 + adc_in * 0.1;
			}

			callback(20);
			if(flashcount == 5) {
				Serial.printf_P(PSTR("*"));
				digitalWrite(A_LED_PIN, LOW);
				digitalWrite(C_LED_PIN, HIGH);
			}

			if(flashcount >= 10) {
				flashcount = 0;
				digitalWrite(C_LED_PIN, LOW);
			  digitalWrite(A_LED_PIN, HIGH);
			}
			flashcount++;
		}

		total_change    = fabs(prev[0] - _sensor_cal[0]) + fabs(prev[1] - _sensor_cal[1]) +fabs(prev[2] - _sensor_cal[2]);
		max_offset      = (_sensor_cal[0] > _sensor_cal[1]) ? _sensor_cal[0] : _sensor_cal[1];
		max_offset      = (max_offset > _sensor_cal[2]) ? max_offset : _sensor_cal[2];
		callback(500);
	} while (  total_change > _gyro_total_cal_change || max_offset > _gyro_max_cal_offset);
	digitalWrite(C_LED_PIN, LOW);
	digitalWrite(A_LED_PIN, LOW);
}

void
AP_IMU_Oilpan::save()
{
    _sensor_cal.save();
}

void
AP_IMU_Oilpan::init_accel(void (*callback)(unsigned long t))
{
    _init_accel(callback);
    _sensor_cal.save();
}

void
AP_IMU_Oilpan::_init_accel(void (*callback)(unsigned long t))
{
	int flashcount = 0;
	float adc_in;
	float prev[6] = {0,0,0};
	float total_change;
	float max_offset;
  int adc_values[6];

	// cold start
 	callback(500);

	Serial.printf_P(PSTR("Init Accel"));

	for (int j=3; j<=5; j++) _sensor_cal[j] = 500;		// Just a large value to load prev[j] the first time

	do {
        _adc->Ch6(_sensors, adc_values);

		for (int j = 3; j <= 5; j++){
			prev[j] = _sensor_cal[j];
			_sensor_cal[j]	= adc_values[j];
		}

		for(int i = 0; i < 50; i++){		// We take some readings...

			callback(20);

            _adc->Ch6(_sensors, adc_values);

			for (int j = 3; j < 6; j++){
				adc_in 	    	= adc_values[j];
				_sensor_cal[j]	= _sensor_cal[j] * 0.9 + adc_in * 0.1;
			}

			if(flashcount == 5) {
				Serial.printf_P(PSTR("*"));
				digitalWrite(A_LED_PIN, LOW);
				digitalWrite(C_LED_PIN, HIGH);
			}

			if(flashcount >= 10) {
				flashcount = 0;
				digitalWrite(C_LED_PIN, LOW);
				digitalWrite(A_LED_PIN, HIGH);
			}
			flashcount++;
		}

		// null gravity from the Z accel
		_sensor_cal[5] += _gravity * _sensor_signs[5];

		total_change = fabs(prev[3] - _sensor_cal[3]) + fabs(prev[4] - _sensor_cal[4]) +fabs(prev[5] - _sensor_cal[5]);
		max_offset = (_sensor_cal[3] > _sensor_cal[4]) ? _sensor_cal[3] : _sensor_cal[4];
		max_offset = (max_offset > _sensor_cal[5]) ? max_offset : _sensor_cal[5];

		callback(500);
	} while (  total_change > _accel_total_cal_change || max_offset > _accel_max_cal_offset);

	digitalWrite(C_LED_PIN, LOW);
	digitalWrite(A_LED_PIN, LOW);
	Serial.printf_P(PSTR(" "));
}

/**************************************************/
// Returns the temperature compensated raw gyro value
//---------------------------------------------------

float
AP_IMU_Oilpan::_sensor_in(uint8_t channel, int adc_value)
{
    float   adc_in;

    // get the compensated sensor value
    //
    adc_in = adc_value;

    // adjust for sensor sign and apply calibration offset
    //
    if (_sensor_signs[channel] < 0) {
        adc_in = _sensor_cal[channel] - adc_in;
    } else {
        adc_in = adc_in - _sensor_cal[channel];
    }

    return adc_in;
}


bool
AP_IMU_Oilpan::update(void)
{
	// MPNG: Temperature not used
  int adc_values[6];

  _sample_time = _adc->Ch6(_sensors, adc_values);

	// convert corrected gyro readings to delta acceleration
	//
	_gyro.x = _gyro_gain_x * _sensor_in(0, adc_values[0]);
	_gyro.y = _gyro_gain_y * _sensor_in(1, adc_values[1]);
	_gyro.z = _gyro_gain_z * _sensor_in(2, adc_values[2]);

	// convert corrected accelerometer readings to acceleration
	//
	_accel.x = _accel_scale * _sensor_in(3, adc_values[3]);
	_accel.y = _accel_scale * _sensor_in(4, adc_values[4]);
	_accel.z = _accel_scale * _sensor_in(5, adc_values[5]);

	_accel_filtered.x = _accel_filtered.x * .98 + _accel.x * .02;
	_accel_filtered.y = _accel_filtered.y * .98 + _accel.y * .02;
	_accel_filtered.z = _accel_filtered.z * .98 + _accel.z * .02;

	// always updated
	return true;
}

// -*- tab-width: 4; Mode: C++; c-basic-offset: 3; indent-tabs-mode: t -*-
/*
	AP_RangeFinder.cpp - Arduino Library for Sharpe GP2Y0A02YK0F
	infrared proximity sensor
	Code by Jose Julio and Randy Mackay. DIYDrones.com

	This library is free software; you can redistribute it and/or
    modify it under the terms of the GNU Lesser General Public
    License as published by the Free Software Foundation; either
    version 2.1 of the License, or (at your option) any later version.

	This has the basic functions that all RangeFinders need implemented
*/

// AVR LibC Includes
#include "WConstants.h"
#include "RangeFinder.h"



// Public Methods //////////////////////////////////////////////////////////////
void RangeFinder::set_analog_port(int analogPort)
{
    // store the analog port to be used
    _analogPort = analogPort;
	pinMode(analogPort, INPUT);
}

void RangeFinder::set_orientation(int x, int y, int z)
{
    orientation_x = x;
	orientation_y = y;
	orientation_z = z;
}

// Read Sensor data - only the raw_value is filled in by this parent class
int RangeFinder::read()
{
	//raw_value = _ap_adc->Ch(7) ; // in MPNG we use ADC library for sonar value and custom made filter with blackjack and...(realy the same)
	//distance = raw_value;
	return _ap_adc->Ch(7);
}


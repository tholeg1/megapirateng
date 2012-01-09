// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-
//
// Test for GPS_Ublox_i2c
//

#include <FastSerial.h>
#include <AP_Common.h>
#include <GPS_Ublox_i2c.h>
#include <stdio.h>
#include <Wire.h>

FastSerialPort0(Serial);

GPS_Ublox_i2c Ublox_i2c_gps(NULL);
GPS *gps = &Ublox_i2c_gps;

#define T6 1000000
#define T7 10000000

void setup()
{
	Serial.begin(38400);

	Serial.println("GPS Ublox_I2C library test");
	gps->init();
}

void loop()
{
	gps->update();
	if (gps->new_data){
		if (gps->fix) {
			Serial.printf("\nLat: %.7f Lon: %.7f Alt: %.2fm GSP: %.2fm/s CoG: %d SAT: %d TIM: %lu",
						  (float)gps->latitude / T7,
						  (float)gps->longitude / T7,
						  (float)gps->altitude / 100.0,
						  (float)gps->ground_speed / 100.0,
						  (int)gps->ground_course / 100,
						  gps->num_sats,
						  gps->time);
		} else {
			Serial.println("No fix");
		}
		gps->new_data = false;
	}
}

// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

// Example config file. Take a look at config.h. Any term define there can be overridden by defining it here.

// Select your sensor board
#define PIRATES_SENSOR_BOARD PIRATES_ALLINONE
/*
	PIRATES_ALLINONE
	PIRATES_FFIMU
	PIRATES_FREEIMU
	PIRATES_BLACKVORTEX
	PIRATES_FREEIMU_4 // New FreeIMU 0.4.1 with MPU6000, MS5611 and 5883L
*/

// RC configuration
//#define SERIAL_PPM ENABLED
#define TX_CHANNEL_SET	TX_mwi
/*
	TX_set1							//Graupner/Spektrum												PITCH,YAW,THROTTLE,ROLL,AUX1,AUX2,CAMPITCH,CAMROLL
	TX_standard					//standard  PPM layout Robbe/Hitec/Sanwa	ROLL,PITCH,THROTTLE,YAW,MODE,AUX2,CAMPITCH,CAMROLL
	TX_standard_mode6		//standard, Mode channel is 6  						ROLL,PITCH,THROTTLE,YAW,AUX1,MODE,CAMPITCH,CAMROLL
	TX_set2							//some Hitec/Sanwa/others									PITCH,ROLL,THROTTLE,YAW,AUX1,AUX2,CAMPITCH,CAMROLL
	TX_mwi							//MultiWii layout													ROLL,THROTTLE,PITCH,YAW,AUX1,AUX2,CAMPITCH,CAMROLL
*/

// Select your baro sensor
#define CONFIG_BARO AP_BARO_BMP085_PIRATES
/*
	AP_BARO_BMP085_PIRATES
	AP_BARO_MS5611_I2C
*/

#define LED_SEQUENCER DISABLED
#define MAX_SONAR_RANGE 400

#define OSD_PROTOCOL OSD_PROTOCOL_NONE
	/*
		OSD_PROTOCOL_NONE
		OSD_PROTOCOL_SYBERIAN
		OSD_PROTOCOL_REMZIBI  // Read more at: http://www.rcgroups.com/forums/showthread.php?t=921467
	*/

// New in 2.0.43, but unused in MegairateNG
// MPNG: Piezo uses AN5 pin in ArduCopter, we uses AN5 for CLI switch
#define PIEZO	DISABLED	
#define PIEZO_LOW_VOLTAGE	DISABLED
#define PIEZO_ARMING		DISABLED

// For BlackVortex, just set PIRATES_SENSOR_BOARD as PIRATES_BLACKVORTEX, GPS will select automatically 
#define GPS_PROTOCOL GPS_PROTOCOL_NONE
	/*
	options:
	GPS_PROTOCOL_NONE 	without GPS
	GPS_PROTOCOL_NMEA
	GPS_PROTOCOL_SIRF
	GPS_PROTOCOL_UBLOX
	GPS_PROTOCOL_IMU
	GPS_PROTOCOL_MTK
	GPS_PROTOCOL_HIL
	GPS_PROTOCOL_MTK16
	GPS_PROTOCOL_AUTO	auto select GPS
	GPS_PROTOCOL_UBLOX_I2C
	GPS_PROTOCOL_BLACKVORTEX
	*/
	
#define SERIAL0_BAUD			 115200	// If one want a wireless modem (like APC220) on the console port, lower that to 57600. Default is 115200 
#define SERIAL2_BAUD			 38400	// GPS port bps
#define SERIAL3_BAUD			 57600	// default telemetry BPS rate = 57600

#define FRAME_CONFIG QUAD_FRAME
	/*
	options:
	QUAD_FRAME
	TRI_FRAME
	HEXA_FRAME
	Y6_FRAME
	OCTA_FRAME
	HELI_FRAME
	*/

#define FRAME_ORIENTATION X_FRAME
	/*
	PLUS_FRAME
	X_FRAME
	V_FRAME
	*/

# define CH7_OPTION		CH7_DO_NOTHING
	/*
	CH7_DO_NOTHING
	CH7_SET_HOVER
	CH7_FLIP
	CH7_SIMPLE_MODE
	CH7_RTL
	CH7_AUTO_TRIM
	CH7_ADC_FILTER (experimental)
	CH7_SAVE_WP
	*/

#define ACCEL_ALT_HOLD 0		// disabled by default, work in progress


//#define RATE_ROLL_I 	0.18
//#define RATE_PITCH_I	0.18
//#define MOTORS_JD880
//#define MOTORS_JD850


// agmatthews USERHOOKS
// the choice of function names is up to the user and does not have to match these
// uncomment these hooks and ensure there is a matching function un your "UserCode.pde" file
//#define USERHOOK_FASTLOOP userhook_FastLoop();
#define USERHOOK_50HZLOOP userhook_50Hz();
//#define USERHOOK_MEDIUMLOOP userhook_MediumLoop();
//#define USERHOOK_SLOWLOOP userhook_SlowLoop();
//#define USERHOOK_SUPERSLOWLOOP userhook_SuperSlowLoop();
#define USERHOOK_INIT userhook_init();

// the choice of includeed variables file (*.h) is up to the user and does not have to match this one
// Ensure the defined file exists and is in the arducopter directory
#define USERHOOK_VARIABLES "UserVariables.h"


// to enable, set to 1
// to disable, set to 0
#define AUTO_THROTTLE_HOLD 1



# define LOGGING_ENABLED		DISABLED
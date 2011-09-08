// Example config file. Take a look at confi.h. Any term define there can be overridden by defining it here.

// GPS is auto-selected
#define GPS_PROTOCOL GPS_PROTOCOL_NMEA
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
	*/


#define MAG_ORIENTATION		ROTATION_YAW_270

#define SERIAL0_BAUD			115200	// If one want a wireless modem (like APC220) on the console port, lower that to 57600. Default is 115200 
#define SERIAL2_BAUD			 38400	// GPS port bps
#define SERIAL3_BAUD			 57600	// default telemetry BPS rate = 57600

#define CLI_ENABLED ENABLED

// for motors arm|disarm
#define ARM_DELAY 10		// one second
#define DISARM_DELAY 10		// one second

// for level
#define LEVEL_DELAY 120 	// twelve seconds for set level
#define AUTO_LEVEL_DELAY 250	// twentyfive seconds for enable infly autolevel mode

//#define BROKEN_SLIDER		0		// 1 = yes (use Yaw to enter CLI mode)

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


//#define CHANNEL_6_TUNING CH6_NONE
	/*
	CH6_NONE
	CH6_STABILIZE_KP
	CH6_STABILIZE_KI
	CH6_RATE_KP
	CH6_RATE_KI
	CH6_THROTTLE_KP
	CH6_THROTTLE_KD
	CH6_YAW_KP
	CH6_YAW_KI
	CH6_YAW_RATE_KP
	CH6_YAW_RATE_KI
	CH6_TOP_BOTTOM_RATIO
	CH6_PMAX
    CH6_RELAY
	*/

// experimental!!
// Yaw is controled by targeting home. you will not have Yaw override.
// flying too close to home may induce spins.
//#define SIMPLE_LOOK_AT_HOME 0

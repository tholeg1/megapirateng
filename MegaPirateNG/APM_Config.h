// Example config file. Take a look at confi.h. Any term define there can be overridden by defining it here.

// GPS is auto-selected
#define GPS_PROTOCOL GPS_PROTOCOL_NMEA

#define MAG_ORIENTATION		ROTATION_YAW_270

#define  SONAR_TYPE  MAX_SONAR_XL // don't change!!!

#define HIL_MODE	HIL_MODE_DISABLED

#define GCS_PROTOCOL  GCS_PROTOCOL_MAVLINK

#define SERIAL0_BAUD			115200
#define SERIAL2_BAUD			 38400
#define SERIAL3_BAUD			 57600

#define CLI_ENABLED ENABLED

// for motors arm|disarm
#define ARM_DELAY 10	// one secon
#define DISARM_DELAY 10	// one secon

// for level
#define LEVEL_DELAY 120 // twelve seconds
#define AUTO_LEVEL_DELAY 250 // twentyfive seconds

//#define LOITER_TEST 1

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

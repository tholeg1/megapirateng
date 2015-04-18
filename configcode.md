# Supported IMU boards and sensors #
in libraries/AP\_ADC/AP\_ADC\_ADS7844.cpp
  * AllInOne: #define ALLINONE
  * AllInOne2: #define ALLINONE
  * FFIMU: #define FFIMU
  * ITG3205+BMA020 (minimal sensors list): #define ALLINONE and #define BMA\_020, then you need "nulled" barometer library (comment all about I2C (Wire.something))

# Supported range finder sensors #
in libraries/AP\_ADC/AP\_ADC\_ADS7844.cpp
  * DYP-ME007: [DYP-ME007](dypme007.md)
  * DYP-ME007v2: [DYP-ME007](dypme007.md)

# Supported frames #
## frame configuration ##
in APM\_Config.h #define FRAME\_CONFIG
  * QUAD\_FRAME
  * TRI\_FRAME
  * HEXA\_FRAME
  * Y6\_FRAME
  * OCTA\_FRAME
## frame orientation ##
in APM\_Config.h #define FRAME\_ORIENTATION
  * for QUAD\_FRAME: PLUS\_FRAME, X\_FRAME or V\_FRAME (something between X and Y - good for FPV)
  * for HEXA\_FRAME:
  * for OCTA\_FRAME:

# Supported GPS protocols #
in APM\_Config.h #define GPS\_PROTOCOL
  * GPS\_PROTOCOL\_NONE - without GPS
  * GPS\_PROTOCOL\_NMEA
  * GPS\_PROTOCOL\_SIRF
  * GPS\_PROTOCOL\_UBLOX
  * GPS\_PROTOCOL\_IMU
  * GPS\_PROTOCOL\_MTK
  * GPS\_PROTOCOL\_HIL
  * GPS\_PROTOCOL\_MTK16
  * GPS\_PROTOCOL\_UBLOX\_I2C - AllInOne2 UBLOX GPS connected via i2c
  * GPS\_PROTOCOL\_AUTO - auto select (not recommend)
for change port speed: #define SERIAL2\_BAUD 38400

# Use APC220 #
if you want use APC220 Wireless Communication Module for serial connection, change SERIAL0\_BAUD from default 115200 to 57600 and use it speed in APM Planner
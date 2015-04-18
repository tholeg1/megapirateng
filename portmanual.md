# Introduction #

ArduCopter2 board looks like the same as Arduino MEGA, except aditional 328p chip


# Add new libraries #
  * APM\_RC (we don't have 328 chip on arduino mega, so need new for rc)
  * AP\_ADC (we have another IMU board and sonar)
  * AP\_Compass (better to use old and tested on MP lib)

# Existing AC2 libraries #
  * APM\_BMP085 (comment all about BMP085\_EOC)
  * AP\_IMU\_Oilpan.cpp (`#define A_LED_PIN 13`, `#define C_LED_PIN 31`)
  * RangeFinder.cpp (read() must return `_ap_adc->Ch(7)`)

# ArduCopter2 code to MegaPirateNG code #
  * ArduCopterMega.pde (comment SPI.h and DataFlash.h, GPS on Serial2)
  * defines.h (comment AN5 59 we use it for CLI, change SLIDE\_SWITCH\_PIN to 59, A\_LED\_PIN 13, B\_LED\_PIN 31, C\_LED\_PIN 30)
  * config.h (define SONAR\_TYPE, HIL\_MODE and GCS\_PROTOCOL and disable LOGGING\_ENABLED)
  * System.pde (comment data logging, enter in CLI, gps to Serial2)
  * Setup.pde (we need some additional code for CLI parameters set, rc trim and gps)

# New features #
  * led.pde by Syberian's (in ArduCopterMega.pde add sq\_led\_heartbeat(); in medium\_loop)
  * sonar filter (2.0.40\_TEST5)
  * I2C GPS for AllInOne2
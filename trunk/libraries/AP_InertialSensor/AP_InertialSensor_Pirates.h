
#ifndef __AP_INERTIAL_SENSOR_PIRATES_H__
#define __AP_INERTIAL_SENSOR_PIRATES_H__

#include <string.h>
#include <stdint.h>

#include "../AP_PeriodicProcess/AP_PeriodicProcess.h"
#include "../AP_Math/AP_Math.h"
#include "AP_InertialSensor.h"

class AP_InertialSensor_Pirates : public AP_InertialSensor
{
  public:

  AP_InertialSensor_Pirates();

  void init( AP_PeriodicProcess * scheduler );

  /* Concrete implementation of AP_InertialSensor functions: */
  bool update();
  float gx();
  float gy();
  float gz();
  void get_gyros( float * );
  float ax();
  float ay();
  float az();
  void get_accels( float * );
  void get_sensors( float * );
  float temperature();
  uint32_t sample_time();
  void reset_sample_time();

  static void read();
  static void hardware_init();

  private:

  Vector3f _gyro;
  Vector3f _accel;
  float _temp;

  uint32_t _last_sample_micros;

  float _temp_to_celsius( uint16_t );

  static const float _accel_scale;
  static const float _gyro_scale;

  static const uint8_t _gyro_data_index[3];
  static const  int8_t _gyro_data_sign[3];

  static const uint8_t _accel_data_index[3];
  static const  int8_t _accel_data_sign[3];

  static const uint8_t _temp_data_index;

  static int16_t _data[7];
};

#endif // __AP_INERTIAL_SENSOR_PIRATES_H__

#include <FastSerial.h>

#include <AP_I2C.h>
#include "AP_InertialSensor_Pirates.h"

#include <wiring.h>

//*****************************
// Select your INS board type:
// #define FFIMU
#define ALLINONE
// #define FREEIMU_35

// #define BMA_020 // do you have it?

// *********************
// I2C general functions
// *********************
#define I2C_PULLUPS_DISABLE        PORTD &= ~(1<<0); PORTD &= ~(1<<1); 

#ifdef ALLINONE
#define BMA180_A 0x82
#else
#define BMA180_A 0x80
#endif

#ifdef BMA_020
#define ACC_DIV 25.812
#else
#define ACC_DIV 28
#endif


// ITG-3200 14.375 LSB/degree/s
const float AP_InertialSensor_Pirates::_gyro_scale = 0.0012141421; // ToRad(1/14.375)
const float AP_InertialSensor_Pirates::_accel_scale = 9.81 / 2730.0;

/* pch: I believe the accel and gyro indicies are correct
 *      but somone else should please confirm.
 */
const uint8_t AP_InertialSensor_Pirates::_gyro_data_index[3]  = { 5, 4, 6 };
const int8_t  AP_InertialSensor_Pirates::_gyro_data_sign[3]   = { -1, 1, -1 };

const uint8_t AP_InertialSensor_Pirates::_accel_data_index[3] = { 1, 0, 2 };
const int8_t  AP_InertialSensor_Pirates::_accel_data_sign[3]  = { 1, -1, -1 };
	
const uint8_t AP_InertialSensor_Pirates::_temp_data_index = 3;

AP_InertialSensor_Pirates::AP_InertialSensor_Pirates()
{
  _gyro.x = 0;
  _gyro.y = 0;
  _gyro.z = 0;
  _accel.x = 0;
  _accel.y = 0;
  _accel.z = 0;
  _temp = 0;
}

void AP_InertialSensor_Pirates::init( AP_PeriodicProcess * scheduler )
{
    hardware_init();
    scheduler->register_process( &AP_InertialSensor_Pirates::read );
}

// accumulation in ISR - must be read with interrupts disabled
// the sum of the values since last read
static volatile int32_t _sum[7];
static volatile int8_t rawADC_ITG3200[8];
static volatile int8_t rawADC_BMA180[6];

// how many values we've accumulated since last read
static volatile uint16_t _count;


/*================ AP_INERTIALSENSOR PUBLIC INTERFACE ==================== */

bool AP_InertialSensor_Pirates::update( void )
{
	int32_t sum[7];
	uint16_t count;
	float count_scale;

	// wait for at least 1 sample
	while (_count == 0) /* nop */;

	// disable interrupts for mininum time
	cli();
	for (int i=0; i<7; i++) {
		sum[i] = _sum[i];
		_sum[i] = 0;
	}
	count = _count;
	_count = 0;
	sei();

	count_scale = 1.0 / count;

	_gyro.x = _gyro_scale * _gyro_data_sign[0] * sum[_gyro_data_index[0]] * count_scale;
	_gyro.y = _gyro_scale * _gyro_data_sign[1] * sum[_gyro_data_index[1]] * count_scale;
	_gyro.z = _gyro_scale * _gyro_data_sign[2] * sum[_gyro_data_index[2]] * count_scale;

	_accel.x = _accel_scale * _accel_data_sign[0] * sum[_accel_data_index[0]] * count_scale;
	_accel.y = _accel_scale * _accel_data_sign[1] * sum[_accel_data_index[1]] * count_scale;
	_accel.z = _accel_scale * _accel_data_sign[2] * sum[_accel_data_index[2]] * count_scale;

	_temp    = _temp_to_celsius(sum[_temp_data_index] * count_scale);

	return true;
}

float AP_InertialSensor_Pirates::gx() { return _gyro.x; }
float AP_InertialSensor_Pirates::gy() { return _gyro.y; }
float AP_InertialSensor_Pirates::gz() { return _gyro.z; }

void AP_InertialSensor_Pirates::get_gyros( float * g )
{
  g[0] = _gyro.x;
  g[1] = _gyro.y;
  g[2] = _gyro.z;
}

float AP_InertialSensor_Pirates::ax() { return _accel.x; }
float AP_InertialSensor_Pirates::ay() { return _accel.y; }
float AP_InertialSensor_Pirates::az() { return _accel.z; }

void AP_InertialSensor_Pirates::get_accels( float * a )
{
  a[0] = _accel.x;
  a[1] = _accel.y;
  a[2] = _accel.z;
}

void AP_InertialSensor_Pirates::get_sensors( float * sensors )
{
  sensors[0] = _gyro.x;
  sensors[1] = _gyro.y;
  sensors[2] = _gyro.z;
  sensors[3] = _accel.x;
  sensors[4] = _accel.y;
  sensors[5] = _accel.z;
}

float AP_InertialSensor_Pirates::temperature() { return _temp; }

uint32_t AP_InertialSensor_Pirates::sample_time()
{
  uint32_t us = micros();
  /* XXX rollover creates a major bug */
  uint32_t delta = us - _last_sample_micros;
  reset_sample_time();
  return delta;
}

void AP_InertialSensor_Pirates::reset_sample_time()
{
    _last_sample_micros = micros();
}

/*================ HARDWARE FUNCTIONS ==================== */

void AP_InertialSensor_Pirates::read()
{
static uint8_t i;

  i2c.rep_start(0XD0);     // I2C write direction ITG3200
  i2c.write(0X1B);         // Start multiple read from Temp register
  i2c.rep_start(0XD0 +1);  // I2C read direction => 1
  for(i = 0; i< 7; i++) {
  rawADC_ITG3200[i]= i2c.readAck();}
  rawADC_ITG3200[7]= i2c.readNak();
	 _sum[3] =  ((rawADC_ITG3200[0]<<8) | rawADC_ITG3200[1]); // temperature
	#ifdef ALLINONE
	  _sum[0] =  ((rawADC_ITG3200[6]<<8) | rawADC_ITG3200[7]); //g yaw
	  _sum[1] =  ((rawADC_ITG3200[4]<<8) | rawADC_ITG3200[5]); //g roll
	  _sum[2] =- ((rawADC_ITG3200[2]<<8) | rawADC_ITG3200[3]); //g pitch
	#endif
	#ifdef FREEIMU_35
	  _sum[0] =  ((rawADC_ITG3200[6]<<8) | rawADC_ITG3200[7]); //g yaw
	  _sum[1] =  ((rawADC_ITG3200[4]<<8) | rawADC_ITG3200[5]); //g roll
	  _sum[2] =- ((rawADC_ITG3200[2]<<8) | rawADC_ITG3200[3]); //g pitch
	#endif
	#ifdef FFIMU
	  _sum[0] =  ((rawADC_ITG3200[6]<<8) | rawADC_ITG3200[7]); //g yaw
	  _sum[2] =  ((rawADC_ITG3200[4]<<8) | rawADC_ITG3200[5]); //g roll
	  _sum[1] =  ((rawADC_ITG3200[2]<<8) | rawADC_ITG3200[3]); //g pitch
	#endif

	#ifndef BMA_020
	  i2c.rep_start(BMA180_A);     // I2C write direction BMA 180
	  i2c.write(0x02);         // Start multiple read at reg 0x02 acc_x_lsb
	  i2c.rep_start(BMA180_A +1);  // I2C read direction => 1
	  for( i = 0; i < 5; i++) {
	    rawADC_BMA180[i]=i2c.readAck();}
	  rawADC_BMA180[5]= i2c.readNak();
	#else // BMA020
	  i2c.rep_start(0x70);
	  i2c.write(0x02);
	  i2c.write(0x71);  
	  i2c.rep_start(0x71);
	  for( i = 0; i < 5; i++) {
	    rawADC_BMA180[i]=i2c.readAck();}
	  rawADC_BMA180[5]= i2c.readNak();
	#endif  
	  
	#ifdef ALLINONE
	  _sum[4] +=  ((rawADC_BMA180[3]<<8) | (rawADC_BMA180[2])) >> 2; //a pitch
	  _sum[5] += -((rawADC_BMA180[1]<<8) | (rawADC_BMA180[0])) >> 2; //a roll
	  _sum[6] +=  ((rawADC_BMA180[5]<<8) | (rawADC_BMA180[4])) >> 2; //a yaw
	#endif
	#ifdef FREEIMU_35
	  _sum[4] +=  ((rawADC_BMA180[3]<<8) | (rawADC_BMA180[2])) >> 2; //a pitch
	  _sum[5] += -((rawADC_BMA180[1]<<8) | (rawADC_BMA180[0])) >> 2; //a roll
	  _sum[6] +=  ((rawADC_BMA180[5]<<8) | (rawADC_BMA180[4])) >> 2; //a yaw
	#endif
	#ifdef FFIMU
	  _sum[5] +=  ((rawADC_BMA180[3]<<8) | (rawADC_BMA180[2])) >> 2; //a pitch
	  _sum[4] +=  ((rawADC_BMA180[1]<<8) | (rawADC_BMA180[0])) >> 2; //a roll
	  _sum[6] +=  ((rawADC_BMA180[5]<<8) | (rawADC_BMA180[4])) >> 2; //a yaw
	#endif

  _count++;
  if (_count == 0) {
	  // rollover - v unlikely
	  memset((void*)_sum, 0, sizeof(_sum));
  }
}

void AP_InertialSensor_Pirates::hardware_init()
{
	int i;
	i2c.init();
	
	I2C_PULLUPS_DISABLE
	
	// GYRO
	//=== ITG3200 INIT
	delay(10);  
	TWBR = ((16000000L / 400000L) - 16) / 2; // change the I2C clock rate to 400kHz
	i2c.rep_start(0xd0+0);  // I2C write direction 
	i2c.write(0x3E);                   // Power Management register
	i2c.write(0x80);                   //   reset device
	delay(5);
	i2c.rep_start(0xd0+0);  // I2C write direction 
	i2c.write(0x15);                   // register Sample Rate Divider
	i2c.write(0x3+1);                    //   7: 1000Hz/(3+1) = 250Hz . 
	delay(5);
	i2c.rep_start(0xd0+0);  // I2C write direction 
	i2c.write(0x16);                   // register DLPF_CFG - low pass filter configuration & sample rate
	i2c.write(0x18+4);                   //   Internal Sample Rate 1kHz, 1..6: 1=200hz, 2-100,3-50,4-20,5-10,6-5
	delay(5);
	i2c.rep_start(0xd0+0);  // I2C write direction 
	i2c.write(0x3E);                   // Power Management register
	i2c.write(0x03);                   //   PLL with Z Gyro reference
	delay(100);

	// ACCEL
	delay(10);
	#ifndef BMA_020
		//===BMA180 INIT
		i2c.rep_start(BMA180_A+0);   // I2C write direction 
		i2c.write(0x0D);                   // ctrl_reg0
		i2c.write(1<<4);                   // Set bit 4 to 1 to enable writing
		i2c.rep_start(BMA180_A+0);       
		i2c.write(0x35);          
		i2c.write(3<<1);                   // range set to 3.  2730 1G raw data.  With /10 divisor on acc_ADC, more in line with other sensors and works with the GUI
		i2c.rep_start(BMA180_A+0);
		i2c.write(0x20);                   // bw_tcs reg: bits 4-7 to set bw
		i2c.write(0<<4);                   // bw to 10Hz (low pass filter)
	#else
		byte control;				// BMA020 INIT
		
		i2c.rep_start(0x70);     // I2C write direction
		i2c.write(0x15);         // 
		i2c.write(0x80);         // Write B10000000 at 0x15 init BMA020
		
		i2c.rep_start(0x70);     // 
		i2c.write(0x14);         //  
		i2c.write(0x71);         // 
		i2c.rep_start(0x71);     //
		control = i2c.readNak();
		control = control >> 5;  //ensure the value of three fist bits of reg 0x14 see BMA020 documentation page 9
		control = control << 2;
		control = control | 0x00; //Range 2G 00
		control = control << 3;
		control = control | 0x00; //Bandwidth 25 Hz 000
		i2c.rep_start(0x70);     // I2C write direction
		i2c.write(0x14);         // Start multiple read at reg 0x32 ADX
		i2c.write(control);
	#endif
	delay(10);  
}

float AP_InertialSensor_Pirates::_temp_to_celsius ( uint16_t regval )
{
	return (35.0 + ((float) (regval + 13200)) / 280);
}

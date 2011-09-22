/*
	APM_ADC.cpp - ADC ADS7844 Library for Ardupilot Mega
Total rewrite by Syberian:

Full I2C sensors replacement:
ITG3200, BMA180

Integrated analog Sonar on the ADC channel 7 (in centimeters)
//D10 (PORTL.1) = input from sonar
//D9 (PORTL.2) = sonar Tx (trigger)
//The smaller altitude then lower the cycle time
*/
extern "C" {
  // AVR LibC Includes
  #include <inttypes.h>
  #include <avr/interrupt.h>
  #include "WConstants.h"
}

#include "AP_ADC_ADS7844.h"

static volatile uint16_t 		_filter[8][ADC_FILTER_SIZE];
static volatile uint8_t			_filter_index;


//*****************************
// Select your IMU board type:
// #define FFIMU
#define ALLINONE
// #define BMA_020 // do you have it?

// *********************
// I2C general functions
// *********************
#define I2C_PULLUPS_DISABLE        PORTC &= ~(1<<4); PORTC &= ~(1<<5);
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



// Mask prescaler bits : only 5 bits of TWSR defines the status of each I2C request
#define TW_STATUS_MASK	(1<<TWS7) | (1<<TWS6) | (1<<TWS5) | (1<<TWS4) | (1<<TWS3)
#define TW_STATUS       (TWSR & TW_STATUS_MASK)
int neutralizeTime;


void i2c_init(void) {
    I2C_PULLUPS_DISABLE
  TWSR = 0;        // no prescaler => prescaler = 1
  TWBR = ((16000000L / 400000L) - 16) / 2; // change the I2C clock rate
  TWCR = 1<<TWEN;  // enable twi module, no interrupt
}
void waitTransmissionI2C() {
  uint8_t count = 255;
  while (count-->0 && !(TWCR & (1<<TWINT)) );
  if (count<2) { //we are in a blocking state => we don't insist
    TWCR = 0;  //and we force a reset on TWINT register
    neutralizeTime = micros(); //we take a timestamp here to neutralize the value during a short delay after the hard reset
  }
}

void i2c_rep_start(uint8_t address) {
  TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN) | (1<<TWSTO); // send REAPEAT START condition
  waitTransmissionI2C(); // wait until transmission completed
  TWDR = address; // send device address
  TWCR = (1<<TWINT) | (1<<TWEN);
  waitTransmissionI2C(); // wail until transmission completed
}

void i2c_write(uint8_t data ) {	
  TWDR = data; // send data to the previously addressed device
  TWCR = (1<<TWINT) | (1<<TWEN);
  waitTransmissionI2C(); // wait until transmission completed
}

uint8_t i2c_readAck() {
  TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWEA);
  waitTransmissionI2C();
  return TWDR;
}

uint8_t i2c_readNak(void) {
  TWCR = (1<<TWINT) | (1<<TWEN);
  waitTransmissionI2C();
  return TWDR;
}
int     adc_value[8]   = { 0, 0, 0, 0, 0, 0, 0, 0 };
float     adc_flt[8]   = { 0, 0, 0, 0, 0, 0, 0, 0 };
int rawADC_ITG3200[6],rawADC_BMA180[6];
long adc_read_timeout=0;
static uint32_t last_ch6_micros;


// Constructors ////////////////////////////////////////////////////////////////
AP_ADC_ADS7844::AP_ADC_ADS7844()
{
}

// Public Methods //////////////////////////////////////////////////////////////
void AP_ADC_ADS7844::Init(void)
{
 int i;
i2c_init();
//=== ITG3200 INIT
for (i=0;i<8;i++) adc_flt[i]=0;

 delay(10);  
  TWBR = ((16000000L / 400000L) - 16) / 2; // change the I2C clock rate to 400kHz
 
  i2c_rep_start(0xd0+0);  // I2C write direction 
  i2c_write(0x3E);                   // Power Management register
  i2c_write(0x80);                   //   reset device
  delay(5);
  i2c_rep_start(0xd0+0);  // I2C write direction 
  i2c_write(0x15);                   // register Sample Rate Divider
  i2c_write(0x4);                    //   7: 1000Hz/(4+1) = 250Hz . 
  delay(5);
  i2c_rep_start(0xd0+0);  // I2C write direction 
  i2c_write(0x16);                   // register DLPF_CFG - low pass filter configuration & sample rate
  i2c_write(0x18+4);                   //   Internal Sample Rate 1kHz, 1..6: 1=200hz, 2-100,3-50,4-20,5-10,6-5
  delay(5);
  i2c_rep_start(0xd0+0);  // I2C write direction 
  i2c_write(0x3E);                   // Power Management register
  i2c_write(0x03);                   //   PLL with Z Gyro reference
  delay(100);
  
  

delay(10);
#ifndef BMA_020
 //===BMA180 INIT
  i2c_rep_start(BMA180_A+0);   // I2C write direction 
  i2c_write(0x0D);                   // ctrl_reg0
  i2c_write(1<<4);                   // Set bit 4 to 1 to enable writing
  i2c_rep_start(BMA180_A+0);       
  i2c_write(0x35);          
  i2c_write(3<<1);                   // range set to 3.  2730 1G raw data.  With /10 divisor on acc_ADC, more in line with other sensors and works with the GUI
  i2c_rep_start(BMA180_A+0);
  i2c_write(0x20);                   // bw_tcs reg: bits 4-7 to set bw
  i2c_write(0<<4);                   // bw to 10Hz (low pass filter)
#else
  byte control;				// BMA020 INIT
  
  i2c_rep_start(0x70);     // I2C write direction
  i2c_write(0x15);         // 
  i2c_write(0x80);         // Write B10000000 at 0x15 init BMA020

  i2c_rep_start(0x70);     // 
  i2c_write(0x14);         //  
  i2c_write(0x71);         // 
  i2c_rep_start(0x71);     //
  control = i2c_readNak();
 
  control = control >> 5;  //ensure the value of three fist bits of reg 0x14 see BMA020 documentation page 9
  control = control << 2;
  control = control | 0x00; //Range 2G 00
  control = control << 3;
  control = control | 0x00; //Bandwidth 25 Hz 000
 
  i2c_rep_start(0x70);     // I2C write direction
  i2c_write(0x14);         // Start multiple read at reg 0x32 ADX
  i2c_write(control);
#endif
 delay(10);  
 
 // Sonar INIT
//=======================
//D48 (PORTL.1) = sonar input
//D47 (PORTL.2) = sonar Tx (trigger)
//The smaller altitude then lower the cycle time

 // 0.034 cm/micros
//PORTL&=B11111001; 
//DDRL&=B11111101;
//DDRL|=B00000100;

PORTH&=B10111111; // H6 -d9  - sonar TX
DDRH |=B01000000;

PORTB&=B11101111; // B4 -d10 - sonar Echo
DDRB &=B11101111;

last_ch6_micros = micros(); 

//PORTG|=B00000011; // buttons pullup

//div64 = 0.5 us/bit
//resolution =0.136cm
//full range =11m 33ms
// Using timer5, warning! Timer5 also share with RC PPM decoder
  TCCR5A =0; //standard mode with overflow at A and OC B and C interrupts
  TCCR5B = (1<<CS11); //Prescaler set to 8, resolution of 0.5us
  TIMSK5=B00000111; // ints: overflow, capture, compareA
  OCR5A=65510; // approx 10m limit, 33ms period
  OCR5B=3000;
  
}

// Sonar read interrupts
volatile char sonar_meas=0;
volatile unsigned int sonar_data=0, sonar_data_start=0, pre_sonar_data=0; // Variables for calculating length of Echo impulse
ISR(TIMER5_COMPA_vect) // This event occurs when counter = 65510
{
		if (sonar_meas == 0) // sonar_meas=1 if we not found Echo pulse, so skip this measurement
				sonar_data = 0;
		PORTH|=B01000000; // set Sonar TX pin to 1 and after ~12us set it to 0 (below) to start new measurement
} 

ISR(TIMER5_OVF_vect) // Counter overflowed, 12us elapsed
{
	PORTH&=B10111111; // set TX pin to 0, and wait for 1 on Echo pin (below)
	sonar_meas=0; // Clean "Measurement finished" flag
}

ISR(PCINT0_vect)
{
	if (PINB & B00010000) { 
		sonar_data_start = TCNT5; // We got 1 on Echo pin, remeber current counter value
	} else {
		sonar_data=TCNT5-sonar_data_start; // We got 0 on Echo pin, calculate impulse length in counter ticks
		sonar_meas=1; // Set "Measurement finished" flag
	}
}

void i2c_ACC_getADC () { // ITG3200 read data
static uint8_t i;

  i2c_rep_start(0XD0);     // I2C write direction ITG3200
  i2c_write(0X1D);         // Start multiple read
  i2c_rep_start(0XD0 +1);  // I2C read direction => 1
  for(i = 0; i< 5; i++) {
  rawADC_ITG3200[i]= i2c_readAck();}
  rawADC_ITG3200[5]= i2c_readNak();
#ifdef ALLINONE
  adc_value[0] =  ((rawADC_ITG3200[4]<<8) | rawADC_ITG3200[5]); //g yaw
  adc_value[1] =  ((rawADC_ITG3200[2]<<8) | rawADC_ITG3200[3]); //g roll
  adc_value[2] =- ((rawADC_ITG3200[0]<<8) | rawADC_ITG3200[1]); //g pitch
#endif
#ifdef FFIMU
  adc_value[0] =  ((rawADC_ITG3200[4]<<8) | rawADC_ITG3200[5]); //g yaw
  adc_value[2] =  ((rawADC_ITG3200[2]<<8) | rawADC_ITG3200[3]); //g roll
  adc_value[1] =  ((rawADC_ITG3200[0]<<8) | rawADC_ITG3200[1]); //g pitch
#endif

#ifndef BMA_020
  i2c_rep_start(BMA180_A);     // I2C write direction BMA 180
  i2c_write(0x02);         // Start multiple read at reg 0x02 acc_x_lsb
  i2c_rep_start(BMA180_A +1);  // I2C read direction => 1
  for( i = 0; i < 5; i++) {
    rawADC_BMA180[i]=i2c_readAck();}
  rawADC_BMA180[5]= i2c_readNak();
#else // BMA020
  i2c_rep_start(0x70);
  i2c_write(0x02);
  i2c_write(0x71);  
  i2c_rep_start(0x71);
  for( i = 0; i < 5; i++) {
    rawADC_BMA180[i]=i2c_readAck();}
  rawADC_BMA180[5]= i2c_readNak();
#endif  
  
#ifdef ALLINONE
  adc_value[4] =  ((rawADC_BMA180[3]<<8) | (rawADC_BMA180[2])) >> 2; //a pitch
  adc_value[5] = -((rawADC_BMA180[1]<<8) | (rawADC_BMA180[0])) >> 2; //a roll
  adc_value[6] =  ((rawADC_BMA180[5]<<8) | (rawADC_BMA180[4])) >> 2; //a yaw
#endif
#ifdef FFIMU
  adc_value[5] =  ((rawADC_BMA180[3]<<8) | (rawADC_BMA180[2]))/ACC_DIV; //a pitch
  adc_value[4] =  ((rawADC_BMA180[1]<<8) | (rawADC_BMA180[0]))/ACC_DIV; //a roll
  adc_value[6] =  ((rawADC_BMA180[5]<<8) | (rawADC_BMA180[4]))/ACC_DIV; //a yaw
#endif
}

// Read one channel value
int AP_ADC_ADS7844::Ch(unsigned char ch_num)         
{char i;int flt;
	if (ch_num==7) { //ch7 is occupied by Differential pressure sensor
			if ( (sonar_data < 354) && (pre_sonar_data > 0) ) {	//wrong data from sonar (3cm * 118 = 354), use previous value
				sonar_data=pre_sonar_data;
			} else {
				pre_sonar_data=sonar_data;
			}
			return(sonar_data / 118); // Magic conversion sonar_data to cm
	} else  { // channels 0..6
		if ( (millis()-adc_read_timeout )  > 2 )  //each read is spaced by 3ms else place old values
		{  adc_read_timeout = millis();
			i2c_ACC_getADC ();
		}
		return(adc_value[ch_num]);
	}
}

// Read 6 channel values
uint32_t AP_ADC_ADS7844::Ch6(const uint8_t *channel_numbers, int *result)
{
	i2c_ACC_getADC (); // Read sensors gyros each 4ms (because DCM called this method from fast_loop which run at 250Hz)
		
	for (uint8_t i=0; i<6; i++) {
		result[i] = adc_value[channel_numbers[i]];
	}
	
	return 4000; // Gyro sample rate 250Hz
}

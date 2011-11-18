extern "C" {
  // AVR LibC Includes
  #include <inttypes.h>
  #include <avr/interrupt.h>
  #include "WConstants.h"
}
#include <AP_I2C.h>
#include "APM_MS5611.h"

#define I2C_PULLUPS_DISABLE        PORTD &= ~(1<<0); PORTD &= ~(1<<1); 
#define MS561101BA_ADDRESS 0x76

// registers of the device
#define MS561101BA_PRESSURE    0x40
#define MS561101BA_TEMPERATURE 0x50
#define MS561101BA_RESET       0x1E

// OSR (Over Sampling Ratio) constants
#define MS561101BA_OSR_256  0x00
#define MS561101BA_OSR_512  0x02
#define MS561101BA_OSR_1024 0x04
#define MS561101BA_OSR_2048 0x06
#define MS561101BA_OSR_4096 0x08

#define OSR MS561101BA_OSR_4096

// Public Methods //////////////////////////////////////////////////////////////
void APM_MS5611_Class::Init(int initialiseWireLib)
{
  I2C_PULLUPS_DISABLE
	i2c.init();
  delay(10);
  i2c.writeReg(MS561101BA_ADDRESS, MS561101BA_RESET, 0); 
  delay(10);
  union {uint16_t val; uint8_t raw[2]; } data;
  delay(10);
  for(uint8_t i=0;i<6;i++) {
    i2c.rep_start(MS561101BA_ADDRESS + 0);
    i2c.write(0xA2+2*i);
    i2c.rep_start(MS561101BA_ADDRESS + 1);//I2C read direction => 1
    data.raw[1] = i2c.readAck();  // read a 16 bit register
    data.raw[0] = i2c.readNak();
    c[i+1] = data.val;
  } 
}

// Read the sensor. This is a state machine
// We read one time Temperature (state=1) and then 4 times Pressure (states 2-5)
uint8_t APM_MS5611_Class::Read()
{
  if (micros() < deadline) return (0); 
  deadline = micros();
  switch (state) {
    case 0: 
  		i2c.rep_start(MS561101BA_ADDRESS+0);      // I2C write direction
  		i2c.write(MS561101BA_TEMPERATURE + OSR);  // register selection
  		i2c.stop(); 
      state++; 
      deadline += 15000; //according to the specs, the pause should be at least 8.22ms
      break;
    case 1: 
  		i2c.rep_start(MS561101BA_ADDRESS + 0);
  		i2c.write(0);
  		i2c.rep_start(MS561101BA_ADDRESS + 1);
  		ut.raw[2] = i2c.readAck();
  		ut.raw[1] = i2c.readAck();
  		ut.raw[0] = i2c.readNak(); 
  		
  		// Calucalte temperature
  		RawTemp = ut.val;
		  dT = ut.val - ((uint32_t)c[5] << 8); // dT  = D2 - TREF = D2  - C5 * 2^8
		  // Convert to actual temperature and scale it to 0.1C accuracy
  		Temp = (2000 + dT * (c[6] >> 23)) / 10; // TEMP  = 20°C + dT * TEMPSENS = 2000 + dT  * C6  / 2^23
      state++;
      break;
    case 2: 
  		i2c.rep_start(MS561101BA_ADDRESS+0);      // I2C write direction
  		i2c.write(MS561101BA_PRESSURE + OSR);     // register selection
  		i2c.stop(); 
      state++; 
      deadline += 15000; //according to the specs, the pause should be at least 8.22ms
      break;
    case 3: 
		  i2c.rep_start(MS561101BA_ADDRESS + 0);
		  i2c.write(0);
		  i2c.rep_start(MS561101BA_ADDRESS + 1);
		  up.raw[2] = i2c.readAck();
		  up.raw[1] = i2c.readAck();
		  up.raw[0] = i2c.readNak(); 
		  RawPress = up.val;
			if(_offset_press == 0){
				_offset_press = RawPress;
				RawPress = 0;
			}else{
				RawPress -= _offset_press;
			}
			// TODO: Implement second order temperature compensation described at page 8 in datasheet
			// dT calculated at temp reading
		  int64_t off  = ((uint32_t)c[2] <<16) + ((dT * c[4]) >> 7);
		  int64_t sens = ((uint32_t)c[1] <<15) + ((dT * c[3]) >> 8);
		  Press = (( (up.val * sens ) >> 21) - off) >> 15; // Press is mBar with 0.01mBar precision and also it same as Pa with 1.0Pa precision (we need result in Pa)
      state = 0; 
      deadline += 30000;
      break;
  }
  return(1);
}
#ifndef AP_I2C_h
#define AP_I2C_h

#include <inttypes.h>

class AP_I2C
{
  private:
		int neutralizeTime;

  public:
		AP_I2C() {};
		void init(void);
		void waitTransmissionI2C();
		void rep_start(uint8_t address);
		void write(uint8_t data );
		uint8_t readAck();
		uint8_t readNak(void);
		void stop(void);
		void writeReg(uint8_t add, uint8_t reg, uint8_t val); 
};

extern AP_I2C i2c;

#endif
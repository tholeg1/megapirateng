extern "C" {
  #include <stdlib.h>
  #include <string.h>
  #include <inttypes.h>
  #include "WConstants.h"
}

#include "AP_I2C.h"

// Mask prescaler bits : only 5 bits of TWSR defines the status of each I2C request
#define TW_STATUS_MASK	(1<<TWS7) | (1<<TWS6) | (1<<TWS5) | (1<<TWS4) | (1<<TWS3)
#define TW_STATUS       (TWSR & TW_STATUS_MASK)

void AP_I2C::init(void) {
  TWSR = 0;        // no prescaler => prescaler = 1
  TWBR = ((16000000L / 400000L) - 16) / 2; // change the I2C clock rate
  TWCR = 1<<TWEN;  // enable twi module, no interrupt
}
void AP_I2C::waitTransmissionI2C() {
  uint16_t count = 255;
  while (!(TWCR & (1<<TWINT))) {
    count--;
    if (count==0) {              //we are in a blocking state => we don't insist
      TWCR = 0;                  //and we force a reset on TWINT register
      neutralizeTime = micros(); //we take a timestamp here to neutralize the value during a short delay
      break;
    }
  } 
}

void AP_I2C::rep_start(uint8_t address) {
  TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN) | (1<<TWSTO); // send REAPEAT START condition
  waitTransmissionI2C(); // wait until transmission completed
  TWDR = address; // send device address
  TWCR = (1<<TWINT) | (1<<TWEN);
  waitTransmissionI2C(); // wail until transmission completed
}

void AP_I2C::write(uint8_t data ) {	
  TWDR = data; // send data to the previously addressed device
  TWCR = (1<<TWINT) | (1<<TWEN);
  waitTransmissionI2C(); // wait until transmission completed
}

uint8_t AP_I2C::readAck() {
  TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWEA);
  waitTransmissionI2C();
  return TWDR;
}

uint8_t AP_I2C::readNak(void) {
  TWCR = (1<<TWINT) | (1<<TWEN);
  waitTransmissionI2C();
  return TWDR;
}

void AP_I2C::stop(void) {
  TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWSTO);
} 
void AP_I2C::writeReg(uint8_t add, uint8_t reg, uint8_t val) {
  rep_start(add+0);  // I2C write direction
  write(reg);        // register selection
  write(val);        // value to write in register
  stop();
} 

// Preinstantiate Objects //////////////////////////////////////////////////////

AP_I2C i2c = AP_I2C();
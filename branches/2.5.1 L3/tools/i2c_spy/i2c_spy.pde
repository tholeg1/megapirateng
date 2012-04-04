// I2C device detector for MegaPirate Flight Controller
// by Syberian
#include <Wire.h>

void setup()
{
  Wire.begin();
  Serial.begin(115200);
Serial.println("I2C devices detector");
Serial.println("=================================");
Serial.println();
}
void loop()
{
for(int i=0;i<128;i++)  {
  Wire.requestFrom(i, 1);
  while(Wire.available())
  { 
    byte c = Wire.receive();
    Serial.print("Detected device addr: 0x");
    Serial.print(i<<1,HEX);
    switch (i<<1)
    { case 0x3c: Serial.println(" HMC5883/43 (compass)");break;
      case 0xD0: Serial.println(" ITG3200 (gyro)");break;
      case 0x82: Serial.println(" BMA180 (accel) Allinone board");break;
      case 0x80: Serial.println(" BMA180 (accel) FFIMU or BB");break;
      case 0xEE: Serial.println(" BMP085 (baro)");break;
      default: Serial.println(" unknown device!");break;
    }  }}
Serial.println("=================================");
Serial.println("Cycle is over");
while(1);
}
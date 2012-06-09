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
    byte c = Wire.read();
    Serial.print("Detected device addr: 0x");
    Serial.print(i,HEX);
    switch (i)
    { case 0x1E: Serial.println(" HMC5883/43 (compass)");break;
      case 0x40: Serial.println(" BMA180 (accel) FFIMU or BB");break;
      case 0x41: Serial.println(" BMA180 (accel) Allinone board");break;
      case 0x68: Serial.println(" ITG3200 (gyro), MPU6050 (gyro+accel)");break;
      case 0x69: Serial.println(" MPU6050 (gyro+accel)");break;
      case 0x76: Serial.println(" MS5611 (baro)");break;
      case 0x77: Serial.println(" BMP085 (baro), MS5611 (baro)");break;
      default: Serial.println(" unknown device!");break;
    }  }}
Serial.println("=================================");
Serial.println("Cycle is over");
while(1);
}

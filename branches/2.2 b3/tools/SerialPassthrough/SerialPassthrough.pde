#include <FastSerial.h>

FastSerialPort0(Serial);
FastSerialPort2(Serial2);

void setup(void)
{
	Serial.begin(115200);
	Serial2.begin(38400);
	Serial.println("Start");
}

void
loop(void)
{
    byte    c;
    if (Serial2.available()){
    	c = Serial2.read();
      Serial.write(c);
    }
    if (Serial.available()){
    	c = Serial.read();
      Serial2.write(c);
    }
}




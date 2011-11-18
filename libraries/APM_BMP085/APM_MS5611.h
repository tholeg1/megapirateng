#ifndef APM_MS5611_h
#define APM_MS5611_h

class APM_MS5611_Class
{
  public:
	APM_MS5611_Class() {};  // Constructor
	int32_t	_offset_press; // Raw pressure at ground ?
	int32_t RawTemp;
	int16_t Temp;
	int32_t RawPress;
	int32_t Press;
	//int Altitude;
	uint8_t oss;
	//int32_t Press0;  // Pressure at sea level

	void Init(int initialiseWireLib = 1);
	uint8_t Read();

  private:
  uint16_t c[7]; // Calibration coefs
  union {uint32_t val; uint8_t raw[4]; } ut; //uncompensated T
  union {uint32_t val; uint8_t raw[4]; } up; //uncompensated P
  // State machine
  uint8_t  state;
  uint32_t deadline;
  int64_t dT; // Difference between actual and reference temperature
};

#endif

#ifndef AP_RangeFinder_ME007_H
#define AP_RangeFinder_ME007_H

#include "RangeFinder.h"

#define AP_RANGEFINDER_ME007_MIN_DISTANCE 20
#define AP_RANGEFINDER_ME007_MAX_DISTANCE 400

class AP_RangeFinder_ME007 : public RangeFinder
{
 // public:
  public:
	AP_RangeFinder_ME007(AP_AnalogSource *source, ModeFilter *filter);

	int convert_raw_to_distance(int _raw_value) { return _raw_value/118; }   // read value from analog port and return distance in cm
};
#endif

// -*- tab-width: 4; Mode: C++; c-basic-offset: 3; indent-tabs-mode: t -*-

// AVR LibC Includes
#include "WConstants.h"
#include "AP_RangeFinder_ME007.h"

// Constructor //////////////////////////////////////////////////////////////

AP_RangeFinder_ME007::AP_RangeFinder_ME007(AP_AnalogSource *source,
                                                     ModeFilter *filter) :
	RangeFinder(source, filter)
{
	max_distance = AP_RANGEFINDER_ME007_MAX_DISTANCE;
	min_distance = AP_RANGEFINDER_ME007_MIN_DISTANCE;
}

// Public Methods //////////////////////////////////////////////////////////////

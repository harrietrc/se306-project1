#include "time_conversion.hpp"
#include <math.h>

namespace time_conversion {
	/* Length of a simulated day in seconds. See time_conversion.hpp */
	const int DAY_LENGTH = 120;
	
	/* Converts a number of hours in the simulator to seconds in real time. */
	int simHoursToRealSecs(double hours) {
		return floor(DAY_LENGTH/24*hours);
	}

	int getDayLength() {
		return DAY_LENGTH;
	}

}

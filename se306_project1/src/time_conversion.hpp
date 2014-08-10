namespace time_conversion {
	/* The length of a simulated day in seconds. */
	extern const int DAY_LENGTH; // We can remove the global variable by hardcoding an appropriate value
								 // once we find one that works with the robot speed we want. Not even sure this needs to be global.

	/** 
		Converts a number of hours in the simulator to seconds in real time. 
		hours = number of hours in simulator time
		return = seconds in real time
	*/
	int simHoursToRealSecs(double hours);

	/** 
		Converts a number of times for an event to occur in a certain period in the simulation to a real time frequency. 
		times = the number of times for an event to occur
		period = the period in hours in simulation time
		return = a rate in real time
	*/
	int simFreqToRealSecs(int times, int period);

	/**
		 This is pretty nonsensical while DAY_LENGTH is global, but I'll either make that constant not global or remove this later.
		 Gets the length of a simulation day in real time seconds.
		 double = length of a simulation day in seconds.
	*/
	int getDayLength(); 

}
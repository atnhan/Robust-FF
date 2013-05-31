/*
 * Clock.h
 *
 *  Created on: May 28, 2013
 *      Author: Tuan A. Nguyen
 */

#ifndef CLOCK_H_
#define CLOCK_H_

#include <ctime>

class Clock {

	// Start and end of the clock
	clock_t start, end;

public:
	Clock() {
		start = clock();
	}
	virtual ~Clock() {}

	// Restart the clock
	void restart() {
		start = clock();
	}

	// Stop the clock
	void stop() {
		end = clock();
	}

	// Get the time elapsed from the last restart
	double time() const {
		return ((double)(end - start))/CLOCKS_PER_SEC;
	}

};

#endif /* CLOCK_H_ */

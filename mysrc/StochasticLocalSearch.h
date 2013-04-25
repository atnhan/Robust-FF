/*
 * StochasticLocalSearch.h
 *
 *  Created on: Apr 1, 2013
 *      Author: tanguye1
 */

#ifndef STOCHASTICLOCALSEARCH_H_
#define STOCHASTICLOCALSEARCH_H_

#include "Search.h"

class StochasticLocalSearch: public Search {
	int max_restarts;
	double noise;
	int max_steps;

	// All information we need to decide next best local move
	struct NeighborInfo {
		int rp_length;
		double lower_robustness;
		double upper_robustness;
	};

public:
	StochasticLocalSearch(State *init, State *goals, int max_restarts, int max_steps, double noise);
	virtual ~StochasticLocalSearch();

	bool run();

};

#endif /* STOCHASTICLOCALSEARCH_H_ */

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
	int max_runs;
	int max_restarts;
	double noise;
	int max_steps;

public:
	StochasticLocalSearch(State *init, State *goals, int max_restarts, int max_steps, double noise);
	virtual ~StochasticLocalSearch();

	void run();

	// Find a plan with better robustness than a threshold
	bool improve(double current_best_robustness, std::vector<int>& new_plan);
};

#endif /* STOCHASTICLOCALSEARCH_H_ */

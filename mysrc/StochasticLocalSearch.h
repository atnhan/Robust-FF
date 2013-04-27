/*
 * StochasticLocalSearch.h
 *
 *  Created on: Apr 1, 2013
 *      Author: tanguye1
 */

#ifndef STOCHASTICLOCALSEARCH_H_
#define STOCHASTICLOCALSEARCH_H_

#include "Search.h"
#include <boost/random/uniform_int_distribution.hpp>
#include <boost/random/uniform_real_distribution.hpp>
#include <boost/random/mersenne_twister.hpp>

class StochasticLocalSearch: public Search {

	// Random generator
	typedef boost::mt19937 base_generator_type;
	base_generator_type generator;

	// Search parameters
	int max_restarts;
	double noise;
	int max_steps;

public:
	StochasticLocalSearch(State *init, State *goals, int max_restarts, int max_steps, double noise);
	virtual ~StochasticLocalSearch();

	bool run();

};

#endif /* STOCHASTICLOCALSEARCH_H_ */

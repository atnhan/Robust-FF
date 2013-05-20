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

	// The two distribution for random number generation
	boost::random::uniform_int_distribution<> int_dist;		// The range can change
	boost::random::uniform_real_distribution<> real_dist;	// By default: [0, 1)


	// Search parameters
	int max_restarts;		// The number of times to restart from the initial state
	double noise;
	int max_steps;

	int max_attempts;		// The number of times to attempt escaping the local minimum

	//
	int initial_depth_bound;
	int max_iterations;
	int probes_at_depth;

	// The search neighbor
	struct NEIGHBOR {
		int action;		// Action to apply
		int h;			// Length of the relaxed plan from the resulting state
		double robustness;	// Robustness of the plan prefix + "action" + relaxed plan
		double h_weight;	// Weight of "h" wrt all other neighbors
		double prob;		// Probability for selection
	};

	// Using local search to find a better state than the current one
	State *local_search_for_a_better_state(StripsEncoding* e,
			double &current_robustness, double& better_robustness);

	// Sample a set of next actions from a given state
	void sample_next_actions(StripsEncoding* e, std::vector<int>& actions);

	// Sample a next state of a given state
	// Return true if a next state is found from which there is a relaxed plan with at least the specified
	// robustness threshold
	bool sample_next_state(StripsEncoding* e, double current_robustness, NEIGHBOR& selected_neighbor);

public:
	StochasticLocalSearch(State *init, State *goals, int max_restarts, int max_steps, double noise);
	virtual ~StochasticLocalSearch();

	bool run();

};

#endif /* STOCHASTICLOCALSEARCH_H_ */

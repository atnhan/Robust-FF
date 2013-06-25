/*
 * StochasticLocalSearch.h
 *
 *  Created on: Apr 1, 2013
 *      Author: tanguye1
 */

#ifndef STOCHASTICLOCALSEARCH_H_
#define STOCHASTICLOCALSEARCH_H_

#include "Search.h"
#include "Clock.h"
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

	// Various types of time spent during the search
	struct TIMER {
		// Time spent by the local search
		double search_time;

		// Time spent to maintain the encoding
		double clause_set_construction_time;

		// Relaxed plan extraction time
		double rp_time;

		// Robustness computation time (exact or approximate)
		double robustness_computation_time;

		// Constructor
		TIMER() {
			search_time = 0;
			clause_set_construction_time = 0;
			rp_time = 0;
			robustness_computation_time = 0;
		}

		// Get the total time
		double total() {
			return search_time + clause_set_construction_time + rp_time + robustness_computation_time;
		}
	};
	TIMER timer;

	// The clock to record time
	Clock clock;

	// The search neighbor
	struct NEIGHBOR {
		int action;		// Action to apply
		int h;			// Length of the relaxed plan from the resulting state
		double robustness;	// Robustness of the plan prefix + "action" + relaxed plan
		double h_weight;	// Weight of "h" wrt all other neighbors
		double prob;		// Probability for selection

		void print() {
			std::cout<<"Action: "<<action<<", h: "<<h<<", robustness: "<<robustness<<", h_weight: "<<h_weight<<", prob: "<<prob;
		}
	};

	struct NEIGHBOR_COMPARE {
		bool operator() (NEIGHBOR a, NEIGHBOR b) {
			return (a.prob < b.prob);
		}
	} neighbor_comparison_obj;

	// Using local search to find a better state than the current one
	bool local_search_for_a_better_state(StripsEncoding* e,
			double current_robustness, int h, int& next_h, double& next_robustness, int& fail_count, int tab = 0);

	// Sample a set of next actions from a given state
	bool sample_next_actions(StripsEncoding* e, double robustness_threshold, int n, std::vector<int>& actions, int tab = 0);

	// Sample a next state of a given state
	// Return true if a next state is found from which there is a relaxed plan with at least the specified
	// robustness threshold
	bool sample_next_state(StripsEncoding* e, double current_robustness, int h, double heuristic_bias, NEIGHBOR& selected_neighbor, int tab = 0);

	// Sample k distinct integers from 0 to n-1
	void sample_k(int k, int n, std::vector<int>& result);

	// Update the experiment analysis file
	void update_experiment_analysis_file();

public:

	/*
	 * SEARCH PARAMETERS
	 */

	// If we only consider FF helpful actions in sampling next actions
	static bool FF_helpful_actions;

	// The number of times to restart from the initial state
	static int max_restarts;

	// The maximal number of iterations needs to do local search for escaping plateaus, from the current local minimum state
	static int max_iterations;

	// The initial depth bound for each probes. Doubled when the number of "probes_at_depth" reaches.
	static int initial_depth_bound;

	// For each depth, construct at most this number of probes
	static int probes_at_depth;

	// The number of successor states, randomly sampled.
	static int neighborhood_size;

	// The number of "fails" during the local search. A fail is when a complete probe has been constructed but we cannot
	// find a better state. Doubled every time we restart the search from the initial state.
	static int fail_bound;

	// The max and min of heuristic-bias parameter (beta, in Coles's work)
	static double max_heuristic_bias;
	static double min_heuristic_bias;

	StochasticLocalSearch(State *init, State *goals, double desired_robustness = 1.0);
	virtual ~StochasticLocalSearch();

	bool run();
};

#endif /* STOCHASTICLOCALSEARCH_H_ */

/*
 * StochasticLocalSearch.cpp
 *
 *  Created on: Apr 1, 2013
 *      Author: tanguye1
 */

#include "StochasticLocalSearch.h"
#include "assert.h"
#include <vector>
using namespace std;


StochasticLocalSearch::StochasticLocalSearch(State *init, State *goals, int max_restarts, int max_steps, double noise):
	Search(init, goals) {
	assert(max_restarts > 0);
	assert(max_steps > 0);
	assert(noise >= 0.0);
	this->max_restarts = max_restarts;
	this->max_steps = max_steps;
	this->noise = noise;
}

StochasticLocalSearch::~StochasticLocalSearch() {

}

void StochasticLocalSearch::run() {
	const State *current;
	const double initial_robustness = 0;
	double current_robustness;
	double current_best_robustness;

	int sat_result;
	double sat_prob;
	double running_time;
	e->evaluate_robustness(sat_result, sat_prob, running_time, goals);
	current_best_robustness = sat_prob;

	for (int i = 0; i< max_runs; i++) {

	}

}

// Find a plan with better robustness than a threshold
bool StochasticLocalSearch::improve(double current_best_robustness, std::vector<int>& new_plan) {
	const State *current_state;
	double current_robustness;
	int sat_result;
	double sat_prob;
	double running_time;

	// We try to find a better plan in at most "max_runs" episodes
	vector<int> applicable_actions;
	vector<double> new_plan_robustness;
	for (int i=0;i<max_runs;i++) {

		bool found = false;
		current_state = init;
		current_robustness = 0;
		for (int j=0;j<max_steps;j++) {

			if (current_robustness > current_best_robustness)
				return true;

			// (1) Get applicable actions
			get_applicable_actions(current_state, applicable_actions);

			// (2) Compute robustness of all "new plan + <an applicable action>"
			for (int k=0;k<applicable_actions.size();k++) {

			}
		}
	}
	return false;
}







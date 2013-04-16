/*
 * StochasticLocalSearch.cpp
 *
 *  Created on: Apr 1, 2013
 *      Author: tanguye1
 */

#include "StochasticLocalSearch.h"
#include "assert.h"
#include <vector>
#include "StripsEncoding.h"
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

bool StochasticLocalSearch::run() {
	const State *current_state;

	best_plan.actions.clear();
	best_plan.robustness = 0;

	// We try to find a better plan in at most "max_restarts" restarts from the initial state
	for (int i=0;i<max_restarts;i++) {

		// When we're here: we have a current best robustness for the current best plan
		current_state = init;	// Restart from the initial state
		StripsEncoding *e = new StripsEncoding(init);

		for (int j=0;j<max_steps;j++) {

			bool better_plan_found = false;

			// (1) Get applicable actions
			vector<int> applicable_actions;
			get_applicable_actions(current_state, applicable_actions);

			// (2) Evaluate the quality of "plan prefix + <an applicable action>"
			vector<NeighborInfo> neighborhood;	// one for each applicable action
			neighborhood.reserve(applicable_actions.size());
			for (int k=0;k<applicable_actions.size();k++) {

				// If the estimated robustness of "plan prefix + this action" wrt the goals is
				// not less than the current best robustness, then we check its exact robustness
				double lower, upper;
				e->append(applicable_actions[k]);
				ClauseSet all_clauses(e->get_clauses());
				ClauseSet goal_clauses;
				bool goals_present = e->check_goals(goals, goal_clauses);
				if (goals_present) {
					all_clauses.add_clauses(goal_clauses);
					lower = all_clauses.lower_wmc();
					upper = all_clauses.upper_wmc();
					// We use lower bound to recognize a better plan. May use upper bound as well?
					if (lower >= best_plan.robustness) {
						int sat_result;
						double sat_prob;
						double running_time;

						all_clauses.wmc(sat_result, sat_prob, running_time);
						assert(lower < sat_prob);

						// Record the new best plan
						best_plan.actions.clear();
						for (int l=0;l<e->get_actions().size();l++) {
							int op = e->get_actions().at(l);
							best_plan.actions.push_back(op);
						}
						best_plan.actions.push_back(applicable_actions[k]);
						best_plan.robustness = sat_prob;

						better_plan_found = true;
						break;	// out of considering other applicable actions
					}
				}

				//
			}

			// If we find a better plan, we are done for this improvement iteration
			// We will restart from the initial state for finding even better plan
			if (better_plan_found)
				break;

			// Otherwise, we need to make local move

		}
	}

	if (best_plan.actions.size() > 0 && best_plan.robustness > 0)
		return true;
	return false;
}







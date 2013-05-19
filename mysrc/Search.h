/*
 * Search.h
 *
 *  Created on: Mar 30, 2013
 *      Author: tanguye1
 */

#ifndef SEARCH_H_
#define SEARCH_H_
#include "../ff.h"
#include <vector>
#include "StripsEncoding.h"
#include "RelaxedPlan.h"

class Search {
protected:

	struct Plan {
		std::vector<int> actions;
		double robustness;
	};

	State *init;
	State *goals;
	std::vector<Plan> plans;
	Plan best_plan;

	// Optionally, we can set to find a plan with at least some robustness value
	double desired_robustness;

	/*
	 * FUNCTIONS
	 */

	// Get applicable actions for a given state
	void get_applicable_actions(const State* state, std::vector<int>& actions, const RelaxedPlan* rp = 0, bool helpful_actions = true);

	// Get all applicable actions for a given state
	void get_all_applicable_actions(const State* state, std::vector<int>& actions);

	// Get "FF-helpful" actions
	void get_FF_helpful_actions(const State* state, std::vector<int>& actions, const RelaxedPlan* rp);

public:
	Search(State* init, State* goals);
	virtual ~Search();

	virtual bool run() = 0;

	// If we are using FF-helpful action
	static bool FF_helpful_actions;

};

#endif /* SEARCH_H_ */

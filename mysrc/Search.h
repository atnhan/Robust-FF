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
	const State *goals;
	std::vector<Plan> plans;
	Plan best_plan;

	/*
	 * FUNCTIONS
	 */

	// Get applicable actions for a given state
	void get_applicable_actions(const State* state, std::vector<int>& actions, const RelaxedPlan* rp = 0, bool helpful_actions = true);

public:
	Search(State* init, State* goals);
	virtual ~Search();

	virtual bool run() = 0;

	// If we are using FF-helpful action
	static bool FF_helpful_actions;

};

#endif /* SEARCH_H_ */

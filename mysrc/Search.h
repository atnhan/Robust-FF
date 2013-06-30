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
#include <climits>
#include "StripsEncoding.h"
#include "RelaxedPlan.h"

#define INF_HEU		INT_MAX

class Search {
protected:

	struct Plan {
		std::vector<int> actions;
		double robustness;
		int id;
	};

	State *init;
	State *goals;
	std::vector<Plan> plans;
	Plan best_plan;

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

	// Check if an action sequence, incorporated in a StripsEncoding, satisfies the "goals"
	// with at least a threshold
	// + check_type: < 0  if using lower bound, == 0 if exact, and > 0 if upper bound
	bool robustness_check(const StripsEncoding *e, int check_type, double robustness_threshold);

	// Output plans to streams
	friend std::ostream& operator<<(std::ostream& os, const std::vector<Plan>& plans);
	friend std::ostream& operator<<(std::ostream& os, const Plan& plan);


public:

	Search(State* init, State* goals, double desired_robustness = 1.0);
	virtual ~Search();

	virtual bool run(FILE *log) = 0;

};

#endif /* SEARCH_H_ */

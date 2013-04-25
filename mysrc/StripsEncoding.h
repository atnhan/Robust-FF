/*
 * StripsEncoding.h
 *
 *  Created on: Feb 7, 2013
 *      Author: tuan
 */

#ifndef STRIPSENCODING_H_
#define STRIPSENCODING_H_

#include "../ff.h"
#include <vector>
#include <set>
#include <utility>
#include <boost/unordered_map.hpp>
#include "ClauseSet.h"

class StripsEncoding {

protected:

	// Sequence of actions. Only a prefix of this is the actual plan prefix.
	std::vector<int> actions;

	// Sequence of states
	std::vector<State*> states;		// NOTE: including both the initial state and the last state. So number of states = number of actions + 1

	// Part of the action sequence that is the current plan prefix
	int plan_prefix_length;

	// Clauses for all actions in the current plan prefix
	ClauseSet plan_prefix_clauses;

	// Clause set for preconditions and possible preconditions for each action
	typedef boost::unordered_map<int/*proposition*/, ClauseSet> PRE_2_CLAUSE_SET;
	typedef boost::unordered_map<int/*proposition*/, ClauseSet> POSS_PRE_2_CLAUSE_SET;
	struct ActionClauses {
		PRE_2_CLAUSE_SET pre_clauses;
		POSS_PRE_2_CLAUSE_SET poss_pre_clauses;
	};
	std::vector<ActionClauses> action_clauses;	// same size with "actions"

	// Get the latest level at which the truth value of a fact is "confirmed"
	// NOTE: "ft" might definitely be false at the state at "level"
	// (i.e., it has been deleted by some actions before it, otherwise be false in the initial state)
	int get_confirmed_level(int ft,int level) const;

	// Construct the set of clauses for TRUE truth value of a fact at a level
	bool supporting_constraints(int ft, int level, ClauseSet& clauses) const;

public:
	StripsEncoding(State *init);
	virtual ~StripsEncoding();

	// Append an action, and update the clauses
	void append(int action);

	// Remove the last action. Return false if trying to cut the plan prefix
	bool remove_last();

	// Append an action and extend the plan prefix. Update the clause set.
	void extend_plan_prefix(int action);

	// Check the goals: return false if any goal proposition is not in the last state
	// *New* constraints enforced on goal propositions are also returned
	bool check_goals(const State *goals, ClauseSet& clauses);

	// Collect all clauses for an action at position "i", and all actions
	void get_clauses(int i, ClauseSet& cs) const;
	void get_clauses(ClauseSet& cs) const;

	// Evaluate the robustness of the current plan prefix, optionally with a goal set using weighted model counting.
	void evaluate_plan_prefix(CACHET_OUTPUT& r, const State *goals = 0);

	// Gets
	const std::vector<int>& get_actions() const {
		return actions;
	}
	const std::vector<State*>& get_states() const {
		return states;
	}

	// The last state (after the last action)
	const State* get_last_state() const {
		return states[states.size()-1];
	}

	friend class RelaxedPlan;

	// UNIT TESTS
	friend void test_evaluate_plan_robustness(std::string filename, State *initial_state, State* goal_state);
	friend void test_relaxed_planning_graph(std::string partial_sol_file, State *initial_state, State* goal_state);
};

#endif /* STRIPSENCODING_H_ */

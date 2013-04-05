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
#include "ClauseSet.h"

class StripsEncoding {

protected:

	std::vector<int> actions;
	std::vector<State*> states;		// NOTE: including both the initial state and the last state. So number of states = number of actions + 1
	ClauseSet *clauses;

	// Get the latest level at which the truth value of a fact is "confirmed"
	// NOTE: "ft" might definitely be false at the state at "level"
	// (i.e., it has been deleted by some actions before it, otherwise be false in the initial state)
	int get_confirmed_level(int ft,int level) const;

	// Construct the set of clauses for TRUE truth value of a fact at a level
	bool supporting_constraints(int ft, int level, ClauseSet& clauses) const;

	// Add new clause set
	void add_clause(const Clause& c);
	void add_clauses(const ClauseSet& cs);

	// Write the clause set into a CNF file
	bool write_cnf_file(const char* filename, State *goals = 0, std::vector<float> *weights = 0);

	// Read answer file by the model counting (Cachet)
	void read_weighted_model_counting_answer_file(int& satresult,double& sat_prob, double& rtime);

public:
	StripsEncoding(State *init);
	virtual ~StripsEncoding();

	// Append an action, and update the clauses
	// Return true if succeeds.
	bool append(int action);

	// Check the goals: return false if any goal proposition is not in the last state
	// *New* constraints enforced on goal propositions are also returned
	bool check_goals(State *goals, ClauseSet& cs);

	// Evaluate the robustness of the current action sequence, optionally with a goal set
	bool evaluate_robustness(int& satresult,double& sat_prob, double& rtime, State *goals = 0);

	// Gets
	const std::vector<int>& get_actions() const {
		return actions;
	}
	const std::vector<State*>& get_states() const {
		return states;
	}
	const ClauseSet& get_clauses() const {
		return *clauses;
	}

	// The last state (after the last action)
	const State* get_last_state() const {
		return states[states.size()-1];
	}

	friend class RelaxedPlan;
};

#endif /* STRIPSENCODING_H_ */

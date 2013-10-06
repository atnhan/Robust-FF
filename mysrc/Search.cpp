/*
 * Search.cpp
 *
 *  Created on: Mar 30, 2013
 *      Author: tanguye1
 */

#include "Search.h"
#include "Helpful.h"
#include <vector>
#include <iostream>
using namespace std;

extern void source_to_dest( State *dest, State *source );
extern void make_state( State *S, int n );

Search::Search(State* init, State* goals) {
	this->init = (State*) calloc(1, sizeof(State));
	make_state(this->init, gnum_ft_conn);
	this->init->max_F = gnum_ft_conn;
	source_to_dest(this->init, init);

	this->goals = (State*) calloc(1, sizeof(State));
	make_state(this->goals, gnum_ft_conn);
	this->goals->max_F = gnum_ft_conn;
	source_to_dest(this->goals, goals);

}

Search::~Search() {
	if (goals) {
		if (goals->F)
			free(goals->F);
		if (goals->known_F)
			free(goals->known_F);
		free(goals);
	}
	if (init) {
		if (init->F)
			free(init->F);
		if (init->known_F)
			free(init->known_F);
		free(init);
	}
}

ostream& operator<<(std::ostream& os, const std::vector<Search::Plan>& plans) {

	// Domain file
	os<<"# Domain file: ";
	os<<gcmd_line.ops_file_name<<endl;

	// Problem file
	os<<"# Problem file: ";
	os<<gcmd_line.fct_file_name<<endl;

	// Number of plans
	os<<"# Number of plans"<<endl;
	os<<plans.size()<<endl;

	for (size_t i = 0; i < plans.size(); i++) {
		const Search::Plan& p = plans[i];

		os<<"#"<<endl;
		os<<"# Plan "<<i+1<<"/"<<plans.size()<<endl;

		// Robustness
		os<<"# Robustness"<<endl;
		os<<p.robustness<<endl;

		// Number of actions
		os<<"# Number of actions"<<endl;
		os<<p.actions.size()<<endl;

		for (size_t j=0; j<p.actions.size(); j++) {
			Action *a = gop_conn[p.actions[j]].action;
			os<<a->name<<" ";
			for (size_t k=0;k<a->num_name_vars;k++) {
				os<<gconstants[a->name_inst_table[k]];
			}
			os<<endl;
		}
	}

	return os;
}

ostream& operator<<(std::ostream& os, const Search::Plan& plan) {
	// Domain file
	os<<"# Domain file: ";
	os<<gcmd_line.ops_file_name<<endl;

	// Problem file
	os<<"# Problem file: ";
	os<<gcmd_line.fct_file_name<<endl;

	// Plan ID
	os<<"# Plan ID: "<<plan.id<<endl;

	// Number of actions
	os<<"# Number of actions"<<endl;
	os<<plan.actions.size()<<endl;

	for (size_t j=0; j<plan.actions.size(); j++) {
		Action *a = gop_conn[plan.actions[j]].action;
		os<<a->name<<" ";
		for (size_t k=0;k<a->num_name_vars;k++) {
			os<<gconstants[a->name_inst_table[k]]<<" ";
		}
		os<<endl;
	}

	return os;
}

// Get all applicable actions for a given state
void Search::get_all_applicable_actions(const State* state, std::vector<int>& actions) {
	actions.clear();
	for (int op=0;op<gnum_op_conn;op++) {
		if (applicable_action(op, state))
			actions.push_back(op);
	}
}

// Get "FF-helpful" actions
void Search::get_FF_helpful_actions(const State* state, std::vector<int>& actions, const RelaxedPlan* rp) {
	assert(rp != 0);
	rp->get_FF_helpful_actions(actions);
}

// Get applicable actions for a given state
void Search::get_applicable_actions(const State* state, std::vector<int>& actions,
		const RelaxedPlan* rp, bool helpful_actions) {

	if (!helpful_actions) {
		for (int op=0;op<gnum_op_conn;op++) {
			if (applicable_action(op, state))
				actions.push_back(op);
		}
	}
	else {
		// The relaxed plan must be
		assert(rp != 0);
		rp->get_FF_helpful_actions(actions);
	}

#ifndef NDEBUG
	for (int i=0;i<actions.size();i++) {
		assert(applicable_action(actions[i], state));
	}
#endif
}

// Check if an action sequence, incorporated in a StripsEncoding, satisfies the "goals"
// with at least a threshold
// + check_type: < 0  if using lower bound, == 0 if exact, and > 0 if upper bound
bool Search::robustness_check(const StripsEncoding *e, int check_type, double robustness_threshold) {
	ClauseSet all_clauses;
	e->get_clauses(all_clauses);

	// Check if the goals present in the last state, and get its clause set
	ClauseSet goal_clauses;
	bool goals_present = e->check_goals(goals, goal_clauses);
	if (!goals_present)
		return false;

	// Add goal clauses into the set of all clauses
	all_clauses.add_clauses(goal_clauses);

	double robustness;
	if (check_type < 0) {
		robustness = all_clauses.lower_wmc();
	}
	else if (check_type == 0) {
		CACHET_OUTPUT o;
		all_clauses.wmc(o);
		robustness = o.prob;
	}
	else {
		robustness = all_clauses.upper_wmc();
	}

	//if ()
}



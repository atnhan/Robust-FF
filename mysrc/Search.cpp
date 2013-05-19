/*
 * Search.cpp
 *
 *  Created on: Mar 30, 2013
 *      Author: tanguye1
 */

#include "Search.h"
#include "Helpful.h"

extern void source_to_dest( State *dest, State *source );
extern void make_state( State *S, int n );

bool Search::FF_helpful_actions = true;

Search::Search(State* init, State* goals) {
	this->init = (State*) calloc(1, sizeof(State));
	make_state(this->init, gnum_ft_conn);
	this->init->max_F = gnum_ft_conn;
	source_to_dest(this->init, init);

	this->goals = (State*) calloc(1, sizeof(State));
	make_state(this->goals, gnum_ft_conn);
	this->goals->max_F = gnum_ft_conn;
	source_to_dest(this->goals, goals);

	this->desired_robustness = 1.0;
}

Search::~Search() {
	if (goals) {
		delete goals;
		goals = 0;
	}
	if (init) {
		delete init;
		init = 0;
	}
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


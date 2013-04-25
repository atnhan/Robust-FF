/*
 * Search.cpp
 *
 *  Created on: Mar 30, 2013
 *      Author: tanguye1
 */

#include "Search.h"
#include "Helpful.h"

Search::Search(State* init, State* goals) {
	this->goals = goals;
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


// Get applicable actions for a given state
void Search::get_applicable_actions(const State* state, std::vector<int>& actions,
		const RelaxedPlan* rp, bool helpful_actions) {

	if (!helpful_actions) {
		for (int op=0;op<gnum_op_conn;op++) {
			bool applicable = true;
			for (int i=0;i<gop_conn[op].num_E;i++) {
				int ef = gop_conn[op].E[i];
				for (int j=0;j<gef_conn[ef].num_PC;j++) {
					int p = gef_conn[ef].PC[j];
					if (!is_in_state(p, state)) {
						applicable = false;
						break;
					}
				}
			}
			if (applicable) actions.push_back(op);
		}
	}
	else {
		// The relaxed plan must be
		assert(rp != 0);
		rp->get_FF_helpful_actions(actions);
	}
}

/*
 * StripsEncoding.cpp
 *
 *  Created on: Feb 7, 2013
 *      Author: tuan
 */

#include "StripsEncoding.h"
#include "Helpful.h"
#include <iostream>
#include <assert.h>
#include <string.h>
using namespace std;

extern int result_to_dest( State *dest, State *source, int op );
extern void source_to_dest( State *dest, State *source );
extern void make_state( State *S, int n );

StripsEncoding::StripsEncoding(State *init) {
	plan_prefix_length = 0;
	State *init_state = (State*) calloc(1, sizeof(State));
	make_state(init_state, gnum_ft_conn);
	init_state->max_F = gnum_ft_conn;
	source_to_dest(init_state, init);
	this->states.push_back(init_state);
//	this->clauses = new ClauseSet;
}

StripsEncoding::~StripsEncoding() {
	for (int i=0;i<this->states.size();i++)
		if (this->states[i])
			free(this->states[i]);

//	if (clauses)
//		delete clauses;
}

void StripsEncoding::append(int action) {
	assert(action >= 0 && action < gnum_op_conn);

//	cout<<"IN APPEND..."<<endl;

	State *current_state = this->get_last_state();

	State *resulting_state = (State*) calloc(1, sizeof(State));
	make_state(resulting_state, gnum_ft_conn);
	resulting_state->max_F = gnum_ft_conn;
	result_to_dest(resulting_state, current_state, action);

	this->states.push_back(resulting_state);
	this->actions.push_back(action);

	// Clauses associated with the new action
	ActionClauses new_action_clauses;

	// Update the clause set
	int level = this->actions.size()-1;

	assert(gop_conn[action].num_E == 1);
	int n_ef = gop_conn[action].E[0];

	// First, for known preconditions
	for (int i=0;i<gef_conn[n_ef].num_PC;i++) {
		int ft = gef_conn[n_ef].PC[i];
		ClauseSet cs;
		bool success = supporting_constraints(ft, level, cs);
		assert(success);

		// Update clauses for this precondition
		if (cs.size()) {
			new_action_clauses.pre_clauses[ft] = cs;

			//
//			cout<<cs<<endl;
		}
	}

	// Second, for possible preconditions
	for (int i = 0; i < gef_conn[n_ef].num_poss_PC;i++) {
		int ft = gef_conn[n_ef].poss_PC[i];
		int bvar = get_bool_var(ft, action, POSS_PRE);
		assert(bvar > 0);

		if (is_in_state(ft, this->states[level])) {
			ClauseSet cs;
			bool success = supporting_constraints(ft, level, cs);
			assert(success);

			// Since "ft" is possible precondition, the above constraints need only when it is realized
			// as a precondition of the action.
			ClauseSet temp_cs;
			for (ClauseSet::const_iterator itr = cs.cbegin(); itr != cs.cend(); itr++) {
				Clause c = *itr;
				c.add_literal(-bvar);
				temp_cs.add_clause(c);
			}

			// Update clauses for this possible precondition
			if (temp_cs.size()) {
				new_action_clauses.poss_pre_clauses[ft] = temp_cs;

				//
//				cout<<temp_cs<<endl;
			}
		}
		else {	// If "ft" is false at the level, we don't need to find supporting clauses (i.e., they are empty)
				// This action is applicable only when this possible precondition is not realized
			Clause c;
			c.add_literal(-bvar);

			// Update clauses for this possible precondition
			ClauseSet cs;
			cs.add_clause(c);
			new_action_clauses.poss_pre_clauses[ft] = cs;

			//
//			cout<<cs<<endl;
		}
	}

	this->action_clauses.push_back(new_action_clauses);

	//


	//
//	cout<<"DONE APPEND."<<endl;
}

// Remove the last action
bool StripsEncoding::remove_last() {
	// Action in plan prefix cannot be removed!
	assert(plan_prefix_length < actions.size());

	// Cannot remove action from an empty plan
	assert(actions.size() > 0);

	// Remove the last action, and the last state.
	actions.pop_back();
	action_clauses.pop_back();
	State* last_state = states[states.size()-1];
	free(last_state);
	last_state = 0;
	states.pop_back();
	return true;
}

// Append an action and extend the plan prefix. Update the clause set.
void StripsEncoding::extend_plan_prefix(int action) {
	append(action);
	plan_prefix_length++;
	ClauseSet clauses;
	get_clauses(actions.size()-1, clauses);
	plan_prefix_clauses.add_clauses(clauses);
}

int StripsEncoding::get_confirmed_level(int ft,int level) const
{
	int n = actions.size();
	if (n == 0)
		return 0;

	if (level < 0 && level > n)
	{
		return -1;
	}

	// If "ft" is after the last action, then we only check if it is add or delete effect of the last action
	if (level == n) {

		if (is_add(ft, actions[n-1]) || is_del(ft, actions[n-1]))
			return n;

		for (int l = n-1; l >= 1; l--) {
			// Check if it is a precondition
			if (is_pre(ft, actions[l])) return l;
			// Check if it is add or delete effect
			if (is_add(ft, actions[l-1]) || is_del(ft, actions[l-1])) return l;
		}
	}
	else {
		for (int l = level; l >= 1; l--) {
			// Check if it is a precondition
			if (is_pre(ft, actions[l])) return l;
			// Check if it is add or delete effect
			if (is_add(ft, actions[l-1]) || is_del(ft, actions[l-1])) return l;
		}
	}

	// All fact values are "confirmed at the initial state
	return 0;
}

// Construct the set of clauses for TRUE truth value of a fact at a level
bool StripsEncoding::supporting_constraints(int ft, int level, ClauseSet& clauses) const {
	if (clauses.size())
		clauses.clear();

	if (level < 0 || level > this->actions.size())
		return false;

	if (!is_in_state(ft, this->states[level]))	// There's no way to satisfy this fact
		return false;

	int confirmed_level = get_confirmed_level(ft, level);

	// If "ft" is false at the confirmed level, we first need "establishment constraints"
	if (!is_in_state(ft, this->states[confirmed_level])) {
		Clause c;
		int k;
		for (k = confirmed_level; k < level; k++)
			if (is_poss_add(ft, this->actions[k])) {
				int bvar = get_bool_var(ft, this->actions[k], POSS_ADD);
				c.add_literal(bvar);
			}
		clauses.add_clause(c);
	}

	// Protection constraints
	for (int k = confirmed_level; k < level; k++)
		if (is_poss_del(ft, this->actions[k])) {
			Clause c;
			int bvar = get_bool_var(ft, this->actions[k], POSS_DEL);
			c.add_literal(-bvar);

			for (int j=k+1;j<level;j++) {
				if (is_poss_add(ft, this->actions[j])) {
					bvar = get_bool_var(ft, this->actions[j], POSS_ADD);
					c.add_literal(bvar);
				}
			}
			clauses.add_clause(c);
		}

	return true;

}

// Add new clause or clause set
//void StripsEncoding::add_clauses(const ClauseSet& cs) {
//	clauses->add_clauses(cs);
//}
//
//void StripsEncoding::add_clause(const Clause& c) {
//	clauses->add_clause(c);
//}

bool StripsEncoding::check_goals(const State *goals, ClauseSet& clauses) {
	if (!goals) {
		clauses.clear();
		return true;
	}

	int n = this->actions.size();
	State *current_state = this->get_last_state();
	for (int i=0;i<goals->num_F;i++) {
		int ft = goals->F[i];
		if (!is_in_state(ft, current_state)) {
			clauses.clear();
			return false;
		}
		ClauseSet ft_clauses;
		bool success = supporting_constraints(ft, n, ft_clauses);
		assert(success);

		// Add new clauses
		clauses.add_clauses(ft_clauses);
	}

	return true;
}

void StripsEncoding::get_clauses(int k, ClauseSet& clauses) const {
	assert(k >= 0 && k < actions.size());

	//
//	cout<<"IN GET_CLAUSES..."<<endl;

	int op = actions[k];
	for (int i=0;i<gop_conn[op].num_E;i++) {
		int ef = gop_conn[op].E[i];

		// Known preconditions
		for (int j=0;j<gef_conn[ef].num_PC;j++) {
			int p = gef_conn[ef].PC[j];
			if (action_clauses[k].pre_clauses.find(p) != action_clauses[k].pre_clauses.end()) {
				clauses.add_clauses(action_clauses[k].pre_clauses.at(p));
			}
		}

		// Possible preconditions
		for (int j=0;j<gef_conn[ef].num_poss_PC;j++) {
			int p = gef_conn[ef].poss_PC[j];
			if (action_clauses[k].poss_pre_clauses.find(p) != action_clauses[k].poss_pre_clauses.end()) {
				clauses.add_clauses(action_clauses[k].poss_pre_clauses.at(p));
			}
		}
	}

	//
//	cout<<clauses;
//	cout<<"DONE GET_CLAUSES."<<endl;
}

void StripsEncoding::get_clauses(ClauseSet& clauses) const {
	if (plan_prefix_length == actions.size()) {
		clauses = plan_prefix_clauses;
		return;
	}

	clauses = plan_prefix_clauses;
	for (int k=plan_prefix_length; k<actions.size(); k++) {
		ClauseSet s;
		get_clauses(k, s);
		clauses.add_clauses(s);
	}
}


void StripsEncoding::evaluate_plan_prefix(CACHET_OUTPUT& r, const State *goals) {
	assert(actions.size() == action_clauses.size());
	assert(states.size() == actions.size()+1);

	// Add clauses for goals
	ClauseSet goal_clauses;
	if (goals) {
		if (!check_goals(goals, goal_clauses)) {
			r.prob = 0;
			r.time = 0;
			return;
		}
	}

}



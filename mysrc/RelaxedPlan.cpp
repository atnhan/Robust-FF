/*
 * RelaxedPlan.cpp
 *
 *  Created on: Feb 23, 2013
 *      Author: tanguye1
 */

#include "RelaxedPlan.h"
#include "Helpful.h"
#include <algorithm>
#include <assert.h>
#include <boost/foreach.hpp>
using namespace std;

extern void source_to_dest( State *dest, State *source );
extern void make_state( State *S, int n );


RelaxedPlan::RelaxedPlan(StripsEncoding *e, State *goals) {
	if (!goals) {
		this->current = 0;
		this->goals = 0;
		this->e = 0;
		return;
	}
	this->current = (State*) calloc(1, sizeof(State));
	make_state(this->current, gnum_ft_conn);
	this->current->max_F = gnum_ft_conn;

	const std::vector<State*>& states = e->get_states();
	int plan_length = e->get_actions().size();
	source_to_dest(this->current, states[plan_length]);

	this->goals = (State*) calloc(1, sizeof(State));
	make_state(this->goals, gnum_ft_conn);
	this->goals->max_F = gnum_ft_conn;
	source_to_dest(this->goals, goals);

	this->e = e;
	this->facts_in_rpg = new vector<bool>(gnum_ft_conn, false);
	this->actions_in_rpg = new vector<bool>(gnum_op_conn, false);

}

RelaxedPlan::~RelaxedPlan() {
	if (current) {
		free(current);
		current = 0;
	}
	if (goals) {
		free(goals);
		goals = 0;
	}

	for (int i=0;i<P.size();i++) {
		if (P[i]) {
			delete P[i];
			P[i] = 0;
		}
	}
	P.clear();

	for (int i=0;i<A.size();i++) {
		if (A[i]) {
			delete A[i];
			A[i] = 0;
		}
	}
	A.clear();

	if (facts_in_rpg) delete facts_in_rpg;
	if (actions_in_rpg) delete actions_in_rpg;

}

void RelaxedPlan::build_relaxed_planning_graph(int max_length) {
	bool goals_in_rpg = false;

	// Initialization
	initialize_fact_layer();

	for (int i=0; i<max_length; i++) {
		if (!grow_action_layer()) {
			cout<<endl<<"FAIL TO GROW ACTION LAYER!"<<endl;
			exit(1);
		}

		if (!grow_fact_layer()) {
			cout<<endl<<"FAIL TO GROW FACT LAYER!"<<endl;
			exit(1);
		}

		if (stop_growing()) break;
	}
}

void RelaxedPlan::extract() {

	// The queue to store all unsupported chosen actions
	UNSUPPORTED_ACTION_QUEUE Q;

	// Known and possible preconditions that need to be supported
	vector<int> unsup_pres;
	vector<int> unsup_poss_pres;

	// The level at which all goals appear. So this is also the layer at which GOAL_ACTION is put
	int n = P.size() - 1;
	num_chosen_actions.reserve(n+1);
	for (int i=0;i<=n;i++) num_chosen_actions[i] = 0;

	// The last fact layer
	const FactLayer& last_fact_layer = *(P[n]);

	// Create the step containing goal action of the relaxed plan
	RP_STEP *goal_step = new RP_STEP;
	goal_step->a = GOAL_ACTION;
	for (int i=0;i<goals->num_F; i++) {
		int g = goals->F[i];
		goal_step->s[g] = true;
		// WORTH TO TRY: if g is already in the current state, only take the clauses
		// from there...
		(goal_step->pre_clauses)[g] = last_fact_layer.at(g).best_clauses;
	}

	// Add the goal step to the relaxed plan
	rp.push_back(goal_step);
	num_chosen_actions[n] = 1;

	// Initialize Q with the goal action
	UnsupportedAction goal_action;
	goal_action.a = GOAL_ACTION;
	goal_action.l = n;
	goal_action.s_ptr = &goal_step->s;
	Q.push(goal_action);

	// Extracting actions in the relaxed planning graph to support actions in Q
	while (!Q.empty()) {
		int op = Q.top().a;
		int l_op = Q.top().l;
		const RP_STATE *state_before_op = Q.top().s_ptr;
		Q.pop();

#ifndef NDEBUG
		// This action must be found in the relaxed plan
		list<RP_STEP*>::iterator itr = rp.begin();
		for (; itr != rp.end(); itr++)
			if ((*itr)->a == op)
				break;
		assert(itr != rp.end());
#endif
		// Make sure these are cleared before being filled
		unsup_pres.clear();
		unsup_poss_pres.clear();
		if (op == GOAL_ACTION) {
			for (int i = 0; i < goals->num_F; i++) {
				int g = goals->F[i];
				// If "g" has been in the state before "op", we don't need to support it
				if (state_before_op->find(g) != state_before_op->end())
					continue;
				unsup_pres.push_back(g);
			}
		}
		else {
			for (int i = 0; i < gop_conn[op].num_E; i++) {
				int ef = gop_conn[op].E[i];
				for (int j = 0; j < gef_conn[ef].num_PC; j++) {
					int g = gef_conn[ef].PC[j];
					// If "g" has been in the state before "op", we don't need to support it
					if (state_before_op->find(g) != state_before_op->end())
						continue;
					unsup_pres.push_back(g);
				}
				for (int j = 0; j < gef_conn[ef].num_poss_PC; j++) {
					int g = gef_conn[ef].poss_PC[j];
					// If "g" has been in the state before "op", we don't need to support it
					if (state_before_op->find(g) != state_before_op->end())
						continue;
					unsup_poss_pres.push_back(g);
				}
			}
		}

		FactLayer& current_fact_layer = *(P[l_op]);	// All known and possible preconditions of "a" belong to this fact layer

		// For each unsupported precondition, we add an action to support it.
		for (int i = 0; i < unsup_pres.size(); i++) {
			int g = unsup_pres[i];

			// The first layer in the RPG that "g" appears
			int first_layer = current_fact_layer[g].first_layer;

			// Consider *all* actions from "first_layer" to "l_a - 1" which (possibly) support "g"
			// OPTIONS: only consider actions with the best robustness at each layer
			// THIS IS "HEAVY" STEP!!!!
			double best_robustness = -1;
			int best_supporting_action;
			int layer_of_best_supporting_action;
			for (int j = 0; j < gft_conn[g].num_A; j++) {
				int supporting_action = gft_conn[g].A[j];
				for (int l = first_layer; l < l_op; l++) {
					double r = evaluate_candidate_action(supporting_action, l);
					if (best_robustness < r) {
						best_robustness = r;
						best_supporting_action = supporting_action;
						layer_of_best_supporting_action = l;
					}
				}
			}

			// Add the best supporting action into the current relaxed plan...
			insert_action_into_relaxed_plan(best_supporting_action, layer_of_best_supporting_action);
		}
	}
}

double RelaxedPlan::evaluate_candidate_action(int action, int layer) {

	// Pointers to all clause sets
	vector<ClauseSet*> clause_sets;

	// Number of steps before the layer
	int count = 0;
	for (int l=0;l<layer;l++)
		count += num_chosen_actions[l];

	// The state before "a"
	RP_STATE s;
	if (count == 0) {
		for (int i=0;i<current->num_F; i++)
			s[current->F[i]] = true;
	}
	else {
		RELAXED_PLAN::iterator itr = rp.begin();
		for (int i=0;i<count;i++)
			itr++;
		s = (*itr)->s;
	}

	return 0;
}

// Insert action "a" at layer "l" of the RPG into the current relaxed plan
// Note that we order "a" in front of all chosen actions at the same layer
void RelaxedPlan::insert_action_into_relaxed_plan(int action, int layer) {

	// The new step to be inserted
	RP_STEP *new_step = new RP_STEP;

	// Number of steps before the layer
	int count = 0;
	for (int l=0;l<layer;l++)
		count += num_chosen_actions[l];

	// We now insert the new step, and keep the iterator
	RELAXED_PLAN::iterator new_itr;
	if (count == 0) {
		rp.push_front(new_step);
		new_itr = rp.begin();
	}
	else {
		RELAXED_PLAN::iterator itr = rp.begin();
		for (int i=0;i<count-1;i++)
			itr++;
		new_itr = rp.insert(itr, new_step);
	}

	// Update the step's action
	new_step->a = action;

	// Update the step's state
	if (new_itr == rp.begin()) {
		for (int i=0;i<current->num_F; i++)
			new_step->s[current->F[i]] = true;
	}
	else {
		RELAXED_PLAN::iterator itr = new_itr;
		itr--;
		int prev_action = (*itr)->a;
		const RP_STATE& prev_state = (*itr)->s;
		for (RP_STATE::const_iterator i = prev_state.begin(); i != prev_state.end(); i++) {
			if (i->second)
				new_step->s[i->first] = true;
		}
		// Add known and possible effects of "prev_action" into "s", if they were not present
		for (int i = 0; i < gop_conn[prev_action].num_E; i++) {
			int ef = gop_conn[prev_action].E[i];
			for (int j = 0; j < gef_conn[ef].num_A; j++)
				new_step->s[gef_conn[ef].A[j]] = true;

			for (int j = 0; j < gef_conn[ef].num_poss_A; j++)
				new_step->s[gef_conn[ef].poss_A[j]] = true;
		}
	}

	// Update states for all actions after "action" with its known and possible add effects
	RELAXED_PLAN::iterator itr = new_itr;
	itr++;
	while (itr != rp.end()) {
		RP_STEP& step = **itr;
		for (int j = 0; j < gop_conn[action].num_E; j++) {
			int ef = gop_conn[action].E[j];
			for (int k = 0; k < gef_conn[ef].num_A; k++)
				step.s[gef_conn[ef].A[k]] = true;
			for (int k = 0; k < gef_conn[ef].num_poss_A; k++)
				step.s[gef_conn[ef].poss_A[k]] = true;
		}
		itr++;
	}

	// Update clauses for known and possible preconditions of "action"
	for (int i=0;i<gop_conn[action].num_E;i++) {
		int ef = gop_conn[action].E[i];
		for (int j=0;j<gef_conn[ef].num_PC;j++) {
			int p = gef_conn[ef].PC[j];
			ClauseSet cs;
			bool success = supporting_constraints(p, new_itr, cs);
			assert(success);
			(*new_itr)->pre_clauses[p] = cs;
		}
		for (int j=0;j<gef_conn[ef].num_poss_PC;j++) {
			int p = gef_conn[ef].poss_PC[j];
			int bvar = get_bool_var(p, action, POSS_PRE);
			ClauseSet cs;
			bool success = supporting_constraints(p, new_itr, cs);
			assert(success);
			// These clauses are needed only if the possible precondition is realized
			ClauseSet temp_cs;
			for (ClauseSet::const_iterator itr = cs.cbegin(); itr != cs.cend(); itr++) {
				Clause c = *itr;
				c.insert(-bvar);
				temp_cs.add_clause(c);
			}
			(*new_itr)->poss_pre_clauses[p] = temp_cs;
		}
	}

	// Update clauses for known and possible preconditions of actions after "action" in the relaxed plan
	// Note that we only need to update for those that are known or possible add effects of "action"
	itr = new_itr;
	itr++;
	while (itr != rp.end()) {
		RP_STEP& step = **itr;
		for (int j = 0; j < gop_conn[action].num_E; j++) {
			int ef = gop_conn[action].E[j];
			for (int k = 0; k < gef_conn[ef].num_PC; k++) {
				int p = gef_conn[ef].PC[k];
				if (!is_add(p, action) || !is_poss_add(p, action))
					continue;
				ClauseSet cs;
				bool success = supporting_constraints(p, itr, cs);
				assert(success);
				(*itr)->pre_clauses[p] = cs;
			}

			for (int k = 0; k < gef_conn[ef].num_poss_PC; k++) {
				int p = gef_conn[ef].poss_PC[k];
				if (!is_add(p, action) || !is_poss_add(p, action))
					continue;
				int bvar = get_bool_var(p, action, POSS_PRE);
				ClauseSet cs;
				bool success = supporting_constraints(p, itr, cs);
				assert(success);
				// These clauses are needed only if the possible precondition is realized
				ClauseSet temp_cs;
				for (ClauseSet::const_iterator itr2 = cs.cbegin(); itr2 != cs.cend(); itr2++) {
					Clause c = *itr2;
					c.insert(-bvar);
					temp_cs.add_clause(c);
				}
				(*itr)->poss_pre_clauses[p] = temp_cs;
			}
		}
		itr++;
	}
}

void RelaxedPlan::get_confirmed_step_or_level(int p, RELAXED_PLAN::iterator& the_step_itr,
							pair<int, RELAXED_PLAN::iterator>& output) {
#ifndef NDEBUG
	// Make sure "p" is in the state pointed to by "the_step_itr"
	RP_STATE& the_start_state = (*the_step_itr)->s;
	assert(the_start_state.find(p) != the_start_state.end());
#endif
	RELAXED_PLAN::iterator begin_itr = rp.begin();
	// If "p" is at the first step, then its value is confirmed at some level of the plan prefix
	if (the_step_itr == begin_itr) {
		int n = e->get_confirmed_level(p, e->get_actions().size());
		if (n == e->get_actions().size()) {
			output.first = -1;
			output.second = begin_itr;
		}
	}
	else {
		output.first = 0;	// by default: every fact is confirmed at the initial state
		output.second = the_step_itr;
		--output.second;
		while (output.second != begin_itr) {
			int a = (*output.second)->a;
			if (is_add(p, a)) {	// We ignore known delete effect here in the relaxed plan
				output.first = -1;
				output.second++;
				break;
			}
			else if (is_pre(p, a)) {
				output.first = -1;
				break;
			}
			--output.second;
		}
		// If the confirmed step could be found above, we return.
		if (output.first < 0)
			return;

		// Now we check the "begin_itr" separately
		assert(output.second == begin_itr);
		int a = (*output.second)->a;
		if (is_add(p, a)) {	// We ignore known delete effect here in the relaxed plan
			output.first = -1;
			output.second++;
		}
		else if (is_pre(p, a)) {
			output.first = -1;
		}
		else {
			// Now we know for sure that the truth value of "p" is determined
			// either by some actions in the plan prefix, or in the initial state
			int n = e->get_confirmed_level(p, e->get_actions().size());
			if (n == e->get_actions().size()) {
				output.first = -1;
				output.second = begin_itr;
			}
		}
	}
}

bool RelaxedPlan::supporting_constraints(int p, RELAXED_PLAN::iterator& the_step_itr, ClauseSet& clauses) {
	pair<int, RELAXED_PLAN::iterator> confirmed_pos;
	get_confirmed_step_or_level(p, the_step_itr, confirmed_pos);
	if (confirmed_pos.first >= 0) {
		const vector<State*>& states = e->get_states();
		const vector<int>& actions = e->get_actions();
		// If "p" is false at the confirmed level, we first need establishment constraints
		if (!is_in_state(p, states[confirmed_pos.first])) {
			Clause c;
			// part of the plan prefix
			for(int i=confirmed_pos.first;i<actions.size();i++) {
				if (is_poss_add(p, actions[i])) {
					int bvar = get_bool_var(p, actions[i], POSS_ADD);
					c.insert(bvar);
				}
			}
			// part of the relaxed plan prefix
			RELAXED_PLAN::iterator itr = rp.begin();
			while (itr != the_step_itr) {
				int a = (*itr)->a;
				if (is_poss_add(p, a)) {
					int bvar = get_bool_var(p, a, POSS_ADD);
					c.insert(bvar);
				}
				itr++;
			}
			clauses.add_clause(c);
		}

		// Now protection constraints
		// Note: Actions with possible delete effects are in the plan prefix only
		// Known delete effects are ignored for actions in the relaxed plan prefix
		for(int i=confirmed_pos.first;i<actions.size();i++) {
			if (is_poss_del(p, actions[i])) {
				Clause c;
				int bvar = get_bool_var(p, actions[i], POSS_DEL);
				c.insert(-bvar);
				// Actions in plan prefix and relaxed plan are considered
				// First, part of the plan prefix
				for (int j=i+1;j<actions.size();j++)
					if (is_poss_add(p, actions[j])) {
						int bvar = get_bool_var(p, actions[j], POSS_ADD);
						c.insert(bvar);
					}
				// Second, part of the relaxed plan
				RELAXED_PLAN::iterator itr = rp.begin();
				while (itr != the_step_itr) {
					int a = (*itr)->a;
					if (is_poss_add(p, a)) {
						int bvar = get_bool_var(p, a, POSS_ADD);
						c.insert(bvar);
					}
					itr++;
				}
				clauses.add_clause(c);
			}
		}
	}
	else {
		// If "p" is confirmed false at the first step of the relaxed plan,
		// we first need establishment constraints
		if (confirmed_pos.second == rp.begin() && P[0]->find(p) == P[0]->end()) {
			Clause c;
			RELAXED_PLAN::iterator itr = confirmed_pos.second;
			while (itr != the_step_itr) {
				if (is_poss_add(p, (*itr)->a)) {
					int bvar = get_bool_var(p, (*itr)->a, POSS_ADD);
					c.insert(bvar);
				}
				itr++;
			}
			clauses.add_clause(c);
		}

		// Now protection constraints
		// IMPORTANT: one may argue that we don't need protection constraints in the relaxed plan
		RELAXED_PLAN::iterator itr = confirmed_pos.second;
		while (itr != the_step_itr) {
			if (is_poss_del(p, (*itr)->a)) {
				Clause c;
				int bvar = get_bool_var(p, (*itr)->a, POSS_DEL);
				c.insert(-bvar);

				RELAXED_PLAN::iterator itr2 = itr;
				itr2++;
				while (itr2 != the_step_itr) {
					if (is_poss_add(p, (*itr2)->a)) {
						bvar = get_bool_var(p, (*itr)->a, POSS_ADD);
						c.insert(bvar);
					}
				}
				clauses.add_clause(c);
			}
			itr++;
		}
	}
	return true;
}

void RelaxedPlan::initialize_fact_layer() {
	assert(P.size() == 0);

	int n = e->get_actions().size();	// Plan prefix length
	FactLayer *new_fact_layer = new FactLayer;
	for (int i=0;i<current->num_F;i++) {
		int ft = current->F[i];
		FactNode node;
		bool success = e->supporting_constraints(ft, n, node.best_clauses);
		node.best_robustness = node.best_clauses.estimate_robustness(e->get_clauses());
		node.best_supporting_action = e->get_actions()[e->get_actions().size()-1];
		node.in_rp = false;
		node.first_layer = 0;
		(*new_fact_layer)[ft] = node;
		// Mark this fact as being present in the RPG
		(*facts_in_rpg)[ft] = true;
	}
	P.push_back(new_fact_layer);
}

bool RelaxedPlan::fact_present(int ft, int l) {
	assert (ft >= 0 && ft < gnum_ft_conn);
	assert(l >= 0 && l < P.size());

	if (l == P.size() - 1)
		return (*facts_in_rpg)[ft];
	else
		return (P[l]->find(ft) != P[l]->end());
}

bool RelaxedPlan::action_present(int a, int l) {
	assert(a >= 0 && a < gnum_op_conn);
	assert(l >= 0 && l < A.size());

	if (l == A.size() - 1)
		return (*actions_in_rpg)[a];
	else
		return (A[l]->find(a) != A[l]->end());
}

bool RelaxedPlan::grow_action_layer() {
	// We cannot grow action layer if:
	// + There is no fact layer, or
	// + The number of action layers >= fact layers
	if (P.size() == 0 || A.size() >= P.size())
		return false;

#ifndef NDEBUG
	// We want to test if the set of known PCs of all actions in the RPG is subset of the current facts in the RPG
	set<int> all_known_PCs;
#endif

	int n = P.size()-1;
	ActionLayer * new_action_layer = new ActionLayer;

	for (int ef = 0; ef < gnum_ef_conn; ef++) {
		int op = gef_conn[ef].op;

		// Check if this action is applicable in the current fact layers
		// Possible preconditions ignored.
		bool applicable = true;

		// We only need to check applicability
		// when action "op" has not been present in the previous layer
		if (!(*actions_in_rpg)[op]) {
			for (int i=0;i<gef_conn[ef].num_PC;i++) {
				int ft = gef_conn[ef].PC[i];
				applicable = fact_present(ft, n);
				if (!applicable) break;
			}
		}

		if (!applicable) continue;

		// Information node for this action
		ActionNode node;

		// first, known preconditions
		for (int i=0;i<gef_conn[ef].num_PC;i++) {
			int ft = gef_conn[ef].PC[i];
			ClauseSet ft_cs = (*P[n])[ft].best_clauses;
			node.clauses.add_clauses(ft_cs);
		}

		// second, possible preconditions
		for (int i=0;i<gef_conn[ef].num_poss_PC;i++) {
			int ft = gef_conn[ef].poss_PC[i];
			int bvar = get_bool_var(ft, op, POSS_PRE);
			assert(bvar > 0);

			// TWO CASES: (1) this fact is in the current fact layer, (2) it is not
			if (P[n]->find(ft) != P[n]->end()) {
				ClauseSet ft_cs = (*P[n])[ft].best_clauses;
				ClauseSet temp_ft_cs;
				for (ClauseSet::const_iterator itr = ft_cs.cbegin(); itr != ft_cs.cend(); itr++) {
					Clause c = *itr;
					c.insert(-bvar);
					temp_ft_cs.add_clause(c);
				}
				node.clauses.add_clauses(temp_ft_cs);
			}
			else {
				ClauseSet cs;
				Clause c;
				c.insert(-bvar);
				cs.add_clause(c);
				node.clauses.add_clauses(cs);
			}
		}
		node.robustness = node.clauses.estimate_robustness(e->get_clauses());
		node.in_rp = false;

		// Add this node into the layer action and its clause set into the action layer
		(*new_action_layer)[op] = node;

		// Mark this action being present in the relaxed planning graph
		(*actions_in_rpg)[op] = true;

#ifndef NDEBUG
		for (int i=0;i<gef_conn[ef].num_PC;i++) {
			all_known_PCs.insert(gef_conn[ef].PC[i]);
		}
#endif
	}

#ifndef NDEBUG
	// Test if all known preconditions of actions in the RPG are present in the facts present in the RPG
	for (set<int>::const_iterator itr = all_known_PCs.begin(); itr != all_known_PCs.end(); itr++) {
		int ft = *itr;
		assert((*facts_in_rpg)[ft]);
	}
#endif

	A.push_back(new_action_layer);
	return true;
}

bool RelaxedPlan::grow_fact_layer() {
	// We cannot grow fact layers if:
	// + There's no action layer. Call initialization function instead.
	// + The number of fact layers > action layers
	if (A.size() == 0 || P.size() > A.size())
		return false;

	int n = A.size() - 1;
	ActionLayer& current_action_layer = *(A[n]);
	FactLayer& current_fact_layer = *(P[n]);
	FactLayer *new_fact_layer = new FactLayer;
	for (int ft = 0; ft < gnum_ft_conn; ft++) {

		bool will_be_added = false;	// Whether this fact will be added into the RPG at this iteration
		int best_supporting_action;
		ClauseSet best_clauses;
		double best_robustness = -1;

		// Collect actions certainly/possibly supporting this fact
		vector<int> certainly_supporting_actions;
		vector<int> possibly_supporting_actions;
		for (int i = 0; i < gft_conn[ft].num_A; i++) {
			int ef = gft_conn[ft].A[i];
			int op = gef_conn[ef].op;
			if (action_present(op, n)) {	// the action must be in the RPG
				certainly_supporting_actions.push_back(op);
			}
		}
		for (int i = 0; i < gft_conn[ft].num_poss_A; i++) {
			int ef = gft_conn[ft].poss_A[i];
			int op = gef_conn[ef].op;
			if (action_present(op, n)) {
				possibly_supporting_actions.push_back(op);
			}
		}

		// Now consider each such action
		if (certainly_supporting_actions.size() || possibly_supporting_actions.size()) {
			will_be_added = true;

			// Among actions certainly adding this fact, find the one with the highest robustness
			for (int i = 0; i < certainly_supporting_actions.size(); i++) {
				int op = certainly_supporting_actions[i];
				if (current_action_layer[op].robustness > best_robustness) {
					best_robustness = current_action_layer[op].robustness;
					best_supporting_action = op;
					best_clauses = current_action_layer[op].clauses;
				}
			}

			// Similarly, find the best actions possibly adding this fact
			for (int i = 0; i < possibly_supporting_actions.size(); i++) {
				int op = possibly_supporting_actions[i];
				int bvar = get_bool_var(ft, op, POSS_ADD);
				ClauseSet cs = current_action_layer[op].clauses;
				Clause c;
				c.insert(bvar);
				cs.add_clause(c);		// CAREFUL: this very like removes many clauses that are superset of "c"

				double r = cs.estimate_robustness(e->get_clauses());
				if (r > best_robustness) {
					best_robustness = r;
					best_supporting_action = op;
					best_clauses = cs;
				}
			}
		}

		// If this fact has been present in the RPG, then it will be at the next layer
		if (current_fact_layer.find(ft) != current_fact_layer.end()) {
			if (best_robustness <= current_fact_layer[ft].best_robustness) {	// NOTE: using "<=" makes NOOP preferable
				will_be_added = true;
				best_clauses = current_fact_layer[ft].best_clauses;
				best_robustness = current_fact_layer[ft].best_robustness;
				best_supporting_action = NOOP;
			}
		}

		if (will_be_added) {

#ifndef NDEBUG
			if (current_fact_layer.find(ft) != current_fact_layer.end()) {
				assert(best_robustness >= current_fact_layer[ft].best_robustness);
			}
#endif

			// Add this fact and its associated clause set into the fact layer
			FactNode node;
			node.best_clauses = best_clauses;
			node.best_robustness = best_robustness;
			node.best_supporting_action = best_supporting_action;
			node.in_rp = false;
			if (!(*facts_in_rpg)[ft])
				node.first_layer = n+1;
			else {
				node.first_layer = current_fact_layer[ft].first_layer;
			}

			(*new_fact_layer)[ft] = node;

			// Mark this fact being in the RPG
			(*facts_in_rpg)[ft] = true;
		}
	}

#ifndef NDEBUG
	// We want to test if all add and possible add effects of actions are present in the set of facts of the RPG
	set<int> all_add_and_possible_add_effects;
	for (int op=0; op<gnum_op_conn; op++) {
		if (!(*actions_in_rpg)[op]) continue;
		for (int i=0;i<gop_conn[op].num_E;i++) {
			int ef = gop_conn[op].E[i];
			for (int j = 0;j < gef_conn[ef].num_A; j++) {
				int ft = gef_conn[ef].A[j];
				assert(new_fact_layer->find(ft) != new_fact_layer->end());
			}
			for (int j = 0;j < gef_conn[ef].num_poss_A; j++) {
				int ft = gef_conn[ef].poss_A[j];
				assert(new_fact_layer->find(ft) != new_fact_layer->end());
			}
		}
	}
#endif

	P.push_back(new_fact_layer);
	return true;
}

bool RelaxedPlan::goals_present() {
	for (int i = 0; i< goals->num_F; i++) {
		int ft = goals->F[i];
		if (!(*facts_in_rpg)[ft]) return false;
	}
	return true;
}

bool RelaxedPlan::goals_present(FactLayer& fact_layer) {
	for (int i = 0; i< goals->num_F; i++) {
		int ft = goals->F[i];
		if (fact_layer.find(ft) == fact_layer.end()) return false;
	}
	return true;
}

bool RelaxedPlan::same_fact_layers(FactLayer& factlayer_1, FactLayer& factlayer_2) {

	if (factlayer_1.size() != factlayer_2.size()) return false;

	// Consider each fact "ft"
	for (int ft = 0; ft < gnum_ft_conn; ft++) {
		bool found_1 = (factlayer_1.find(ft) != factlayer_1.end());	// If this fact is in the first layer
		bool found_2 = (factlayer_2.find(ft) != factlayer_2.end());	// If it is in the second layer
		if ((found_1 && !found_2) || (!found_1 && found_2))	// If it is in only one layer, the two layers are different
			return false;

		// Here, "ft" could be in both, or neither.

		// For the first case, we check if the sets of clauses present are the same
		if (found_1 && found_2) {
			if (!(factlayer_1[ft].best_clauses == factlayer_2[ft].best_clauses)) return false;
		}
	}
	return true;
}

bool RelaxedPlan::stop_growing() {
	assert(A.size() <= P.size());

	if (!goals_present()) return false;

	if (A.size() == P.size()) return false;

	int n = P.size()-1;
	if (n == 0) return false;
	FactLayer& current_fact_layer = *(P[n]);
	FactLayer& previous_fact_layer = *(P[n-1]);

	// Check if the current fact layer and the last fact layer are exactly the same
	if (!same_fact_layers(current_fact_layer, previous_fact_layer))
		return false;
	return true;
}















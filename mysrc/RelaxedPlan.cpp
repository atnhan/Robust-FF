/*
 * RelaxedPlan.cpp
 *
 *  Created on: Feb 23, 2013
 *      Author: tanguye1
 */

#include "RelaxedPlan.h"
#include "Helpful.h"
#include <vector>
#include <algorithm>
#include <assert.h>
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
	if (current)
		free(current);
	if (goals)
		free(goals);

	for (int i=0;i<P.size();i++) {
		if (P[i]) {
			delete P[i];
		}
	}
	P.clear();

	for (int i=0;i<A.size();i++) {
		if (A[i]) {
			delete A[i];
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
			break;
		}
		if (!grow_fact_layer()) {
			cout<<endl<<"FAIL TO GROW FACT LAYER!"<<endl;
			break;
		}
	}
//
//	int l = 1;
//	while (true) {
//		if (!grow_action_layer()) {
//			cout<<endl<<"FAIL TO GROW ACTION LAYER!"<<endl;
//			break;
//		}
//		if (!grow_fact_layer()) {
//			cout<<endl<<"FAIL TO GROW FACT LAYER!"<<endl;
//			break;
//		}
//		if (stop_growing()) {
//			cout<<endl<<"STOPPING CONDITIONS MEET!"<<endl;
//			break;
//		}
//		if (++l > max_length)
//			break;
//	}

}

void RelaxedPlan::initialize_fact_layer() {
	assert(P.size() == 0);

	int n = e->get_actions().size();	// Plan prefix length
	FactLayer *new_fact_layer = new FactLayer;
	for (int i=0;i<current->num_F;i++) {
		int ft = current->F[i];
		FactNode node;
		bool success = e->supporting_constraints(ft, n, node.clauses);
		node.robustness = node.clauses.estimate_robustness(e->get_clauses());
		node.best_supporting_action = e->get_actions()[e->get_actions().size()-1];
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
			ClauseSet ft_cs = (*P[n])[ft].clauses;
			node.clauses.add_clauses(ft_cs);
		}

		// second, possible preconditions
		for (int i=0;i<gef_conn[ef].num_poss_PC;i++) {
			int ft = gef_conn[ef].poss_PC[i];
			int bvar = get_bool_var(ft, op, POSS_PRE);
			assert(bvar > 0);

			ClauseSet ft_cs = (*P[n])[ft].clauses;
			ClauseSet temp_ft_cs;
			for (ClauseSet::const_iterator itr = ft_cs.cbegin(); itr != ft_cs.cend(); itr++) {
				Clause c = *itr;
				c.insert(-bvar);
				temp_ft_cs.add_clause(c);
			}

			node.clauses.add_clauses(temp_ft_cs);
		}
		node.robustness = node.clauses.estimate_robustness(e->get_clauses());

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
		double best_robustness;

		// Optional initialization: if this fact has been present in the RPG, then it will be at the next layer
		// The current best choice is NOOP
		if (current_fact_layer.find(ft) != current_fact_layer.end()) {
			will_be_added = true;
			best_supporting_action = NOOP;
			best_clauses = current_fact_layer[ft].clauses;
			best_robustness = current_fact_layer[ft].robustness;
		}
		else {
			best_robustness = 0;
		}

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
				}
			}

			// Similarly, find the best actions possibly adding this fact
			bool flag = false;	// true if the best action comes from one of these "possible" actions
			for (int i = 0; i < possibly_supporting_actions.size(); i++) {
				int op = possibly_supporting_actions[i];
				int bvar = get_bool_var(ft, op, POSS_PRE);
				ClauseSet cs;
				for (ClauseSet::const_iterator itr = current_action_layer[op].clauses.begin(); itr != current_action_layer[op].clauses.end(); itr++) {
					Clause c = *itr;
					c.insert(-bvar);
					cs.add_clause(c);
				}
				double r = cs.estimate_robustness(e->get_clauses());
				if (r > best_robustness) {
					best_robustness = r;
					best_supporting_action = op;
					flag = true;
				}
			}

			if (!flag) {
				best_clauses = current_action_layer[best_supporting_action].clauses;
				best_robustness = current_action_layer[best_supporting_action].robustness;
			}
			else {
				int bvar = get_bool_var(ft, best_supporting_action, POSS_PRE);
				for (ClauseSet::const_iterator itr = current_action_layer[best_supporting_action].clauses.begin();
						itr != current_action_layer[best_supporting_action].clauses.end(); itr++) {
					Clause c = *itr;
					c.insert(-bvar);
					best_clauses.add_clause(c);
				}
				best_robustness = best_clauses.estimate_robustness(e->get_clauses());
			}

		}

		if (will_be_added) {
			// Add this fact and its associated clause set into the fact layer
			FactNode node;
			node.clauses = best_clauses;
			node.robustness = best_robustness;
			node.best_supporting_action = best_supporting_action;
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
	for (int ft = 0; ft < gnum_ft_conn; ft++) {
		bool found_1 = (factlayer_1.find(ft) != factlayer_1.end());
		bool found_2 = (factlayer_2.find(ft) != factlayer_2.end());
		if ((found_1 && !found_2) || (!found_1 && found_2))
			return false;

		// In case the set of facts are the same, we check the set of clauses present
		if (found_1 && found_2) {
			if (factlayer_1[ft].clauses == factlayer_2[ft].clauses) return false;
		}
	}
	return true;
}

bool RelaxedPlan::stop_growing() {
	assert(A.size() <= P.size());
	if (!goals_present()) return false;
	if (A.size() == P.size()) return false;
	int n = P.size()-1;
	if (n <= 1) return false;
	FactLayer& current_fact_layer = *(P[n]);
	FactLayer& last_fact_layer = *(P[n-1]);

	// Check if the current fact layer and the last fact layer are exactly the same
	if (!same_fact_layers(current_fact_layer, last_fact_layer))
		return false;
	return true;
}















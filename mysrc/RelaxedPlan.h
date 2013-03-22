/*
 * RelaxedPlan.h
 *
 *  Created on: Feb 23, 2013
 *      Author: tanguye1
 */

#ifndef RELAXEDPLAN_H_
#define RELAXEDPLAN_H_

#include "../ff.h"
#include "ClauseSet.h"
#include "StripsEncoding.h"
#include "Helpful.h"
#include <vector>
#include <string>
#include <map>
#include <queue>
#include <list>
#include <utility>

class RelaxedPlan {

#define NOOP -1
#define GOAL_ACTION		-1
#define MAX_RELAXED_PLAN_LENGTH		200

	State *current;
	State *goals;
	StripsEncoding *e;


	/**************************************************************************************************
	 * RELAXED PLANNING GRAPH
	 **************************************************************************************************/

	// All information we want to store at each fact and action node in the relaxed planning graph
	struct FactNode {

		// First layer at which a node appears
		int first_layer;

		// INFORMATION ON THE BEST SUPPORTING ACTION
		int best_supporting_action;	// The supporting action with the best robustness. Possible add effect taken into account in robustness estimation
		ClauseSet best_clauses;		// Clause set derived from the best supporting action
		double best_robustness;		// Best robustness computed from the best supporting action, including NOOP

		bool in_rp; // TRUE if this action node is selected
	};

	struct ActionNode {

		// First layer at which an action node occurs
		int first_layer;

		ClauseSet clauses;	// Clauses constructed from preconditions and possible preconditions
		double robustness;	// and its according estimate robustness

		bool in_rp;		// TRUE if this action node is selected
	};

	// Proposition and action layers.
	typedef std::map<int, FactNode> FactLayer;
	typedef std::map<int, ActionNode> ActionLayer;
	std::vector<FactLayer*> P;
	std::vector<ActionLayer*> A;

	// For O(1) checking the present of facts and actions in a layer of the relaxed planning graph
	// Used in growing fact and action layers
	std::vector<bool> *facts_in_rpg;
	std::vector<bool> *actions_in_rpg;

	// Initialize the first fact layer
	void initialize_fact_layer();

	// Check if a fact and action is in a fact layer
	bool fact_present(int ft, int l);
	bool action_present(int a, int l);

	// "Grow" the next action/fact layer. Return "false" if no new actions/facts added
	bool grow_action_layer();
	bool grow_fact_layer();

	// Check if a goal set present in the last layer
	bool goals_present();
	bool goals_present(FactLayer& fact_layer);

	// Check if two fact layers are the same
	bool same_fact_layers(FactLayer& factlayer_1, FactLayer& factlayer_2);

	// Check if we should stop growing the RPG
	bool stop_growing();

	/**************************************************************************************************
	 * RELAXED PLAN
	 **************************************************************************************************/

	// In the relaxed plan, actions at the same layer of the relaxed planning graph are stored in a list
	// For each action, we also store the state before it. We use this information to construct constraints
	// for the correctness of the relaxed plan after being totally ordered.
	struct RP_STEP {
		int a;	// the action at the step
		State *s;	// the state right before the action

		// Each known and possible precondition of the action is associated
		// with a clause set derived from possile add and delete of actions in the relaxed plan
		std::map<int, ClauseSet> clause_set_of_preconditions;
	};
	typedef std::vector<std::list<RP_STEP> > RELAXED_PLAN;
	RELAXED_PLAN rp;

	// Unsupported actions chosen during the relaxed plan extraction are stored in a queue
	struct UnsupportedAction {
		int action;
		int layer;
		const State *state;		// State before the action in the relaxed plan
	};

	// The function to compare two chosen actions.
	// This function must return true when "a1" is ordered before "a2"
	// The priority queue will pop the greatest element
	class unsupported_action_comparison {
	public:
		bool operator() (const UnsupportedAction& a1, const UnsupportedAction& a2) const {
			if (a1.layer != a2.layer)
				return (a1.layer > a2.layer);	// prefer actions at earlier layers to be supported first (like DFS)
			return true;
		}
	};

	typedef std::priority_queue<UnsupportedAction, std::vector<UnsupportedAction>, unsupported_action_comparison> UNSUPPORTED_ACTION_QUEUE;

	// Evaluate a candidate action "a", which is at layer "l" of the RPG, wrt the current relaxed plan.
	double evaluate_candidate_action(int a, int l);

	// Insert an action "a" at layer "l" into a relaxed plan
	void insert_action_into_relaxed_plan(int a, int l);

public:

	RelaxedPlan(StripsEncoding *e, State *goals);
	virtual ~RelaxedPlan();

	// Create relaxed planning graph.
	void build_relaxed_planning_graph(int max_length = 100);

	// Extract the relaxed plan
	void extract();

	// Gets
	const State& get_current_state() const {
		return *current;
	}
	const State& get_goals() const {
		return *goals;
	}

	// UNIT TEST
	friend void test_relaxed_plan(std::string partial_sol_file, State *initial_state, State* goal_state);
	friend void print_fact_layer(FactLayer& fact_layer, int style = 0);
	friend void print_action_layer(ActionLayer& action_layer, int style = 0);
	friend void print_relaxed_planning_graph(RelaxedPlan& rp, int option_f=0, int option_a=0);
	friend void print_fact_node(FactNode& node);
	friend void print_action_node(ActionNode& node);
	friend void print_action_through_layers(RelaxedPlan& rp, int op, int num_layers);
};

#endif /* RELAXEDPLAN_H_ */

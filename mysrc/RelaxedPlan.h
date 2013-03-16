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
#include <vector>
#include <string>
#include <map>

class RelaxedPlan {

#define NOOP -1

	State *current;
	State *goals;
	StripsEncoding *e;

	// All information we want to store at each fact and action node in the relaxed planning graph
	struct FactNode {

		// First layer at which a node appears
		int first_layer;

		/*
		 * INFORMATION ON THE BEST SUPPORTING ACTION
		 */
		int best_supporting_action;	// The supporting action with the best robustness. Possible add effect taken into account in robustness estimation
		ClauseSet best_clauses;		// Clause set derived from the best supporting action
		double best_robustness;		// Best robustness computed from the best supporting action, including NOOP

		/*
		 * OTHER INFORMATION
		 */
		bool in_rp; // TRUE if this action node is selected
	};

	struct ActionNode {

		// First layer at which an action node occurs
		int first_layer;

		ClauseSet clauses;	// Clauses constructed from preconditions and possible preconditions
		double robustness;	// and its according estimate robustness

		bool in_rp;		// TRUE if this action node is selected
	};

	// The (totally ordered) relaxed plan


	/*
	 * FUNCTIONS ON FACT AND ACTION LAYERS
	 */
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

	/*
	 * STRATEGY TO EXTRACT RELAXED PLANS
	 */
	void most_robust_rp_extraction();

public:

	// Ways to extract relaxed plans
	enum RELAXED_PLAN_TYPE {FF, MOST_ROBUST};

	RelaxedPlan(StripsEncoding *e, State *goals);
	virtual ~RelaxedPlan();

	// Create relaxed planning graph.
	void build_relaxed_planning_graph(int max_length = 100);

	// Extract the relaxed plan
	void extract(RELAXED_PLAN_TYPE t);

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

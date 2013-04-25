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
#include <boost/unordered_set.hpp>
#include <boost/unordered_map.hpp>

class RelaxedPlan {

#define NOOP -1
#define GOAL_ACTION		-1
#define MAX_RELAXED_PLAN_LENGTH		200
#define MAX_RPG_LENGTH	500

	const State *current;
	const State *goals;
	const StripsEncoding *e;

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

		bool in_rp; // TRUE if this fact node is selected
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

	/**************************************************************************************************
	 * RELAXED PLAN
	 **************************************************************************************************/

	// In the relaxed plan, actions at the same layer of the relaxed planning graph are stored in a list
	// For each action, we also store (1) the state before it, (2) clause set associated with known preconditions
	// (3) clause set associated with possible preconditions.
	typedef boost::unordered_map<int, int> RP_STATE;	// A proposition is in the state if and only if (a) it is in the map,
														// AND (b) the second field is POSITIVE (the number of add or possible added effects
														// supporting this proposition at the state)
	typedef boost::unordered_map<int, ClauseSet*> PRE_2_CLAUSES;
	typedef boost::unordered_map<int, ClauseSet*> POSS_PRE_2_CLAUSES;
	struct RP_STEP {
		int a;	// the action at the step
		int layer;	// the layer of the action in the RPG
		RP_STATE s;	// the state right before the action

		// Each known and possible precondition of the action is associated
		// with a clause set derived from possible add and delete of actions in the relaxed plan
		PRE_2_CLAUSES pre_clauses;
		POSS_PRE_2_CLAUSES poss_pre_clauses;
	};
	typedef std::list<RP_STEP*> RELAXED_PLAN;	// NOTE: including the step containing the unique goal action
	RELAXED_PLAN rp;
	std::vector<int> num_chosen_actions;	// Number of chosen actions in the relaxed plan at each layer

	// Set of facts added or possibly added by actions in the first action layer, who has been selected into the relaxed plan
	boost::unordered_set<int> possibly_supported_facts_at_1st_fact_layer;

	// Unsupported actions chosen during the relaxed plan extraction are stored in a queue
	struct UnsupportedAction {
		int a;	// action
		int l;	// layer of the RPG
		const RP_STATE *s_ptr;		// State before the action in the relaxed plan
	};

	// The function to compare two chosen actions.
	// This function must return true when "a1" is ordered before "a2"
	// The priority queue will pop the greatest element
	class unsupported_action_comparison {
	public:
		bool operator() (const UnsupportedAction& a1, const UnsupportedAction& a2) const {
			if (a1.l != a2.l)
				return (a1.l > a2.l);	// prefer actions at earlier layers to be supported first (like DFS)
			return true;
		}
	};

	typedef std::priority_queue<UnsupportedAction, std::vector<UnsupportedAction>, unsupported_action_comparison> UNSUPPORTED_ACTION_QUEUE;

	// Evaluate a candidate action "a", which is at layer "l" of the RPG, wrt the current relaxed plan.
	double evaluate_candidate_action(int a, int l);

	// Insert an action "a" at layer "l" into a relaxed plan
	void insert_action_into_relaxed_plan(int a, int l);

	// Get confirmed step in the relaxed plan OR the confirmed level in the current plan prefix
	// for proposition "p" in the rp-state contained in the iterator "itr"
	// The output is a pair:
	// + the first field is the level of the plan prefix; it is -1 if "p" is confirmed by a step in the current relaxed plan
	// + the second field is the confirmed step in the current relaxed plan; it must not be used if the first field is >= 0
	void get_confirmed_step_or_level(int p, RELAXED_PLAN::iterator& the_step_itr,
								std::pair<int, RELAXED_PLAN::iterator>& output);

	// Constructing supporting constraints for fact "p" at position pointed to by "the_step_itr" in the relaxed plan
	bool supporting_constraints(int p, RELAXED_PLAN::iterator& the_step_itr, ClauseSet& clauses);

	// Check if a proposition is in a RP-STATE
	bool in_rp_state(int p, const RP_STATE& s) const;

public:
	// If possible delete effects should be considered in evaluating robustness during
	// RPG construction and RP extraction
	static bool poss_del_in_rp;

	RelaxedPlan(const StripsEncoding *e, const State *init, const State *goals);
	virtual ~RelaxedPlan();

	// Create relaxed planning graph.
	// Return false if there is a goal proposition that cannot be achieved at the last layer
	// (which won't change if the RPG continues to grow)
	bool build_relaxed_planning_graph();

	// Extract the relaxed plan.
	// Return the length of the relaxed plan, and the robustness of {the plan prefix + the relaxed plan}
	void extract(std::pair<int, double>& result);

	// Get FF-style helpful actions
	void get_FF_helpful_actions(std::vector<int>& helpful_actions) const;

	// Gets
	int length() {
		return rp.size();	// Number of steps in the relaxed plan, including the goal step
	}

	const State& get_current_state() const {
		return *current;
	}
	const State& get_goals() const {
		return *goals;
	}

	// UNIT TEST
	friend void test_relaxed_planning_graph(std::string partial_sol_file, State *initial_state, State* goal_state);
	friend void test_relaxed_plan(std::string partial_sol_file, State *initial_state, State* goal_state);
	friend void print_rp_step(RelaxedPlan& rp, int step);
	friend void print_fact_layer(FactLayer& fact_layer, int style = 0);
	friend void print_action_layer(ActionLayer& action_layer, int style = 0);
	friend void print_relaxed_planning_graph(RelaxedPlan& rp, int option_f=0, int option_a=0);
	friend void print_fact_node(FactNode& node);
	friend void print_action_node(ActionNode& node);
	friend void print_action_through_layers(RelaxedPlan& rp, int op, int num_layers);
};

#endif /* RELAXEDPLAN_H_ */

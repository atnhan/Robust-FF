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
#define INIT_ACTION		-2
#define GOAL_ACTION		-3

#define MAX_RELAXED_PLAN_LENGTH		200
#define MAX_RPG_LENGTH	500

	const State *current;
	const State *goals;
	const StripsEncoding *e;

	// Time spent by the Weighted Model Counting module
	double wmc_time;

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

	// Create relaxed planning graph.
	// Return false if there is a goal proposition that cannot be achieved at the last layer
	// (which won't change if the RPG continues to grow)
	bool build_relaxed_planning_graph();

	// Check if a fact and action is in a fact layer
	bool fact_present(int ft, int l);
	bool action_present(int a, int l);

	// Check if an action "op" at layer "l" is applicable (wrt the fact layer before it)
	inline bool applicable_action_in_rpg(int op, int l);

	// "Grow" the next action/fact layer. Return "false" if no new actions/facts added
	bool grow_action_layer();
	bool grow_fact_layer();

	// Check if a goal set present in the last layer
	bool goals_present();
	bool goals_present(FactLayer& fact_layer);

	// Check if two fact layers are the same
	bool same_fact_layers(FactLayer& factlayer_1, FactLayer& factlayer_2);

	// Estimate probability of goals in the RPG
	double goals_prob_in_rpg();

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
		int a;	// the action at the step. IMPORTANT NOTE: THIS INCLUDE THE GOAL_ACTION!!!
		int layer;	// the layer of the action in the RPG
		RP_STATE s;	// the state right before the action

		// Each known and possible precondition of the action is associated
		// with a clause set derived from possible add and delete of actions in the relaxed plan
		PRE_2_CLAUSES pre_clauses;
		POSS_PRE_2_CLAUSES poss_pre_clauses;
	};

	//typedef std::list<boost::shared_ptr<RP_STEP> > RELAXED_PLAN;	// NOTE: including the step containing the unique goal action

	typedef std::list<RP_STEP*> RELAXED_PLAN;	// NOTE: including the step containing the unique goal action

	RELAXED_PLAN rp;
	std::vector<int> num_chosen_actions;	// Number of chosen actions in the relaxed plan at each layer

	// The clauses derived from actions in the relaxed plan
	// This multiset is updated when an action is inserted into the relaxed plan
	boost::unordered_multiset<Clause, boost::hash<Clause> > rp_clauses;

	// Facts in the first fact layer. Used for extracting FF-helpful actions
	std::vector<bool> known_and_possible_adds_of_actions_in_first_layer;

	//-------
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
	//-------

	// Each subgoal is attached with one particular action, at a specific layer of the RPG
	// and whether it is known or possible precondition of the action
	struct SubGoal {
		int g;	// the subgoal
		int op;	// the action
		int l;	// the layer of the RPG
		bool possible_precondition;		// whether it is a possible precondition
		RP_STATE *state_before_op;		// State before the action in the relaxed plan

		SubGoal(int g, int op, int l, bool possible_precondition, RP_STATE* s_ptr);

		void print() const {
			std::cout<<"g: "<<g<<", op: "<<op<<", l: "<<l<<", poss_pred?: "<<possible_precondition;
		}
	};

	class SubGoalComparison {
	public:
		// Return TRUE if "g1 < g2". The queue's top is the largest element.
		bool operator() (const SubGoal& g1, const SubGoal& g2) const;
	};
	typedef std::priority_queue<SubGoal, std::vector<SubGoal>, SubGoalComparison> SubGoalQueue;

	void print_subgoals(SubGoalQueue Q);
	//-------

//	// Evaluate a candidate action "a", which is at layer "l" of the RPG, wrt the current relaxed plan.
//	double evaluate_candidate_action(int a, int l);

	// Collect all clauses associated with rp_steps [begin, end) of the relaxed plan for evaluation
	// Note that for preconditions not supported in the rp_states, we (optionally) use clause sets
	// constructed in the RPG.
	void collect_rp_step_clauses_for_heuristics(RELAXED_PLAN::iterator begin, RELAXED_PLAN::iterator end, ClauseSet& clauses) const;

	// Construct clauses for a new RP_STEP "from scratch". Optionally, preconditions not present in rp_states
	// are associated with clause sets constructed in the RPG
	void construct_rp_step_clauses_for_heuristics(RELAXED_PLAN::iterator rp_step_itr, ClauseSet& clauses);

	// Update rp_states before actions after a new step RP_STEP is removed
	void update_rp_states_for_step_removal(RELAXED_PLAN::iterator& rp_step_itr);

	// Evaluate a candidate action "a", which is at layer "l" of the RPG, wrt the current relaxed plan.
	double evaluate_candidate_action(int a, int l);

	//-------
	// FUNCTIONS THAT HELP INSERTING AN ACTION INTO THE RELAXED PLAN

	// Update state before an RP_STEP
	// If required, return the number of known preconditions that are not supported
	int update_rp_state(const RELAXED_PLAN::iterator& rp_step_itr, bool compute_unsupported_known_preconditions = true);

	// Update states of all steps after a particular step.
	// If required, return the number of unsupported known preconditions that are supported by the new action
	int update_rp_states_after(const RELAXED_PLAN::iterator& rp_step_itr, bool compute_new_supported_preconditions = true);

	// Update all clauses for a new RP_STEP
	void update_rp_step_clauses(const RELAXED_PLAN::iterator& rp_step_itr);

	// Update all clauses for steps after a particular step
	void update_rp_step_clauses_after(const RELAXED_PLAN::iterator& rp_step_itr);

	// Insert an action "a" at layer "l" into a relaxed plan.
	// Return the new RP_STEP
	//boost::shared_ptr<RP_STEP> insert_action_into_relaxed_plan(int a, int l);
	RP_STEP *insert_action_into_relaxed_plan(int a, int l);
	//-------

	// Create an RP_STEP containing the goal action
	//boost::shared_ptr<RP_STEP> create_rp_step_for_goals();
	RP_STEP *create_rp_step_for_goals();

	// Get confirmed step in the relaxed plan OR the confirmed level in the current plan prefix
	// for proposition "p" in the rp-state contained in the iterator "itr"
	// The output is a pair:
	// + the first field is the level of the plan prefix; it is -1 if "p" is confirmed by a step in the current relaxed plan
	// + the second field is the confirmed step in the current relaxed plan; it must not be used if the first field is >= 0
	void get_confirmed_step_or_level(int p, const RELAXED_PLAN::iterator& the_step_itr,
								std::pair<int, RELAXED_PLAN::iterator>& output);

	// Constructing supporting constraints for fact "p" at position pointed to by "the_step_itr" in the relaxed plan
	bool supporting_constraints(int p, const RELAXED_PLAN::iterator& the_step_itr, ClauseSet& clauses);

	// Check if a proposition is in a RP-STATE
	bool in_rp_state(int p, const RP_STATE& s) const;

	// Estimate the robustness of the plan prefix + the current relaxed plan, depending on the flags
	double compute_robustness();

	// As the above function: Estimate both lower bound, upper bound of the plan prefix + the current relaxed plan
	// Except: the upper bound is compared against the robustness threshold, and if it is better then the exact robustness will also be computed
	// Return false if there exists unsupported preconditions in the current relaxed plan
	bool compute_robustness(double& lower, double& upper, double& exact);


	// The number of unsupported known preconditions
	int num_unsupported_known_preconditions;

	// The robustness threshold: we want to find a plan with more than this robustness
	double robustness_threshold;

	/*
	 * FOR DEBUGGING PURPOSES
	 */
	// Check if there exists any unsupported known preconditions in the relaxed plan
	// For debugging purpose
	bool unsupported_known_precondition_exists();

	// The set of unsupported known preconditions, together with its action. Note that in the relaxed plan
	// each action occurs at most once
	// First: precondition (proposition) id
	// Second: the action
	std::set<std::pair<int, int> > unsupported_known_precondition_set;

	void print_unsupported_known_precondition_set() {
		for (std::set<std::pair<int,int> >::const_iterator itr = unsupported_known_precondition_set.begin();
				itr != unsupported_known_precondition_set.end(); itr++) {
			std::cout<<"("<<itr->first<<", "<<itr->second<<") ";
		}
	}

	// The queue
	boost::unordered_set<std::pair<int, int> > SubgoalSet;
	void print_subgoal_set() {
		for (boost::unordered_set<std::pair<int, int> >::const_iterator itr = SubgoalSet.begin();
				itr != SubgoalSet.end(); itr++) {
			std::cout<<"("<<itr->first<<", "<<itr->second<<") ";
		}
	}

public:

	/*********************************************************************************************
	 * DESIGN CHOICES IN THE RELAXED PLAN EXTRACTION
	 *********************************************************************************************/

	// Each way to extract relaxed plans can be characterized by the following features:
	// + F1 (true/false): whether fact and action nodes are associated with clause sets
	// (and will be used in some ways)
	// + F2 ("first-layer-selection"/"most-robust-selection"):
	// >> "first-layer-selection": supporting actions for a subgoal are selected at the first layer
	// at which the subgoal appears in the RPG, breaking ties using the number of conditions (similar to FF heuristic)
	// >> "most-robust-selection": supporting actions for a subgoal are the most robust ones (using clause sets
	// associated with actions)
	// + F3 (true/false): whether robustness of the plan prefix + partial relaxed plan is assessed during the extraction
	// (in order to stop as soon as it exceeds the current robustness threshold)
	//
	// Valid combinations of features:
	// + F1 = false, F2 = "first-layer-selection", F3 = false (PURE_FF_RP)
	// + F1 = true, F2 = "first-layer-selection", F3 = true
	// + F1 = true, F2 = "most-robust-selection", F3 = true (SHORT_ROBUST_RP)
	// + F1 = true, F2 = "most-robust-selection", F3 = false (MOST_ROBUST_RP)

	enum RELAXED_PLAN_TYPES {

		PURE_FF_RP,
		SHORT_ROBUST_RP,
		MOST_ROBUST_RP

	};

	// If possible delete effects should be considered in evaluating robustness during
	// RPG construction and RP extraction
	static bool ignore_poss_del_in_rp;

	// If lower bound or upper bound on probability of clause sets should be used
	static bool use_lower_bound_in_rp;
	static bool use_upper_bound_in_rp;

	// When we evaluate a candidate action, whether we consider how the current actions affect
	// it, and how it affects the current actions in the relaxed plan
	static bool candidate_actions_affect_current_actions;
	static bool current_actions_affect_candidate_action;

	// Whether clauses from the RPG are attached to FALSE known/possible preconditions of
	// of actions in the relaxed plan (i.e., the known preconditions that are not present in the
	// state before it)
	static bool clauses_from_rpg_for_false_preconditions;

	/*********************************************************************************************/

	// If we are searching for a plan with more than a robustness threshold
	static bool use_robustness_threshold;

	// Constructors
	RelaxedPlan(const StripsEncoding *e, const State *init, const State *goals, double robustness_threshold = 0);
	virtual ~RelaxedPlan();

	// Extract the relaxed plan
	// Return true if a relaxed plan is found. The "result" pair contains its length and
	// the approximate robustness of the plan prefix + the relaxed plan
	bool extract(std::pair<int, double>& result);

	bool extract_most_robust_rp(std::pair<int, double>& result);

	// Extract the FF-like relaxed plan
	bool extract_pure_ff_heuristic(std::pair<int, double>& result);

	// A version of "extract" with multiple options for heuristics
	bool extract(std::pair<int, double>& result, RELAXED_PLAN_TYPES o);

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

	double get_wmc_time() {
		return wmc_time;
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

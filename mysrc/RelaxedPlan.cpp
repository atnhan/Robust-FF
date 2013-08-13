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

// DEFAULT OPTIONS

bool RelaxedPlan::ignore_poss_del_in_rp = true;	// Possible delete effects ignored

bool RelaxedPlan::use_lower_bound_in_rp = false;	// With both of these FALSE, the default
bool RelaxedPlan::use_upper_bound_in_rp = false;	// computation of WMC for SAT formula is exact!

bool RelaxedPlan::use_robustness_threshold = true;	// The extraction of relaxed plans will stop as
													// soon as its estimated/exact robustness is more than this threshold

bool RelaxedPlan::clauses_from_rpg_for_false_preconditions = true;	// Use clauses from the RPG construction for preconditions that
																	// are known to be false given the current partial relaxed plan

bool RelaxedPlan::current_actions_affect_candidate_action = true;	// The effect of current actions on a candidate action (after them) is considered
bool RelaxedPlan::candidate_actions_affect_current_actions = true;	// and that of a candidate action on current actions (after it)

RelaxedPlan::RELAXED_PLAN_TYPES RelaxedPlan::rp_types = INCREMENTAL_ROBUSTNESS_RP;

int RelaxedPlan::num_rp_calls = 0;
int RelaxedPlan::num_successful_rp_calls = 0;
int RelaxedPlan::num_better_supporting_action_checks_in_rpg = 0;
int RelaxedPlan::num_better_supporting_actions_found_in_rpg = 0;
int RelaxedPlan::num_rp_robustness_increasing_checks = 0;
int RelaxedPlan::num_rp_robustness_increasing_check_success = 0;

RelaxedPlan::RelaxedPlan(const StripsEncoding *e, const State *init, const State *goals, double robustness_threshold) {
	assert(e && goals);

	this->current = init;
	this->goals = goals;
	this->e = e;
	this->facts_in_rpg = new vector<bool>(gnum_ft_conn, false);
	this->actions_in_rpg = new vector<bool>(gnum_op_conn, false);
	this->robustness_threshold = robustness_threshold;

	this->known_and_possible_adds_of_actions_in_first_layer.resize(gnum_ft_conn, false);

	this->wmc_time = 0;
}

RelaxedPlan::~RelaxedPlan() {

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

	RELAXED_PLAN::iterator itr = rp.begin();
	while (itr != rp.end()) {
		for (PRE_2_CLAUSES::iterator itr2 = (*itr)->pre_clauses.begin(); itr2 != (*itr)->pre_clauses.end(); itr2++) {
			if (itr2->second) {
				delete itr2->second;
				itr2->second = 0;
			}
		}

		for (POSS_PRE_2_CLAUSES::iterator itr2 = (*itr)->poss_pre_clauses.begin();
				itr2 != (*itr)->poss_pre_clauses.end(); itr2++) {
			if (itr2->second) {
				delete itr2->second;
				itr2->second = 0;
			}
		}

		RP_STEP *temp = *itr;
		itr++;
		if (temp) {
			delete temp;
			temp = 0;
		}

	}

}

bool RelaxedPlan::build_relaxed_planning_graph() {

//#define DEBUG_BUILD_RPG
#ifdef DEBUG_BUILD_RPG
	cout<<"Begin build_relaxed_planning graph..."<<endl<<endl;
#endif

	bool goals_in_rpg = false;

	// Initialization
	initialize_fact_layer();

	double goals_prob = 0;
	int length = 0;
	while (true) {
		if (!grow_action_layer()) {
			cout<<endl<<"FAIL TO GROW ACTION LAYER!"<<endl;
			exit(1);
		}

		if (!grow_fact_layer()) {
			cout<<endl<<"FAIL TO GROW FACT LAYER!"<<endl;
			exit(1);
		}

		if (++length >= MAX_RPG_LENGTH) {
			cout<<"Error! Increase max rpg length! File "<<__FILE__<<", line "<<__LINE__<<endl;
			exit(1);
		}

		// CHECKING STOPPING CONDITION
		int n = P.size() - 1;	// the last fact layer
		FactLayer& current_fact_layer = *(P[n]);
		FactLayer& previous_fact_layer = *(P[n-1]);

		// If we want to find a plan with more than a robustness threshold
		// stop as soon as the (estimate) goal probability is more than expected
		if (RelaxedPlan::use_robustness_threshold) {

			// If this happens, all total-ordered sequence of the
			// best actions have robustness greater than the threshold
			if ((goals_prob = goals_prob_in_rpg()) > robustness_threshold)
				break;
		}

		// We stop only when the last two fact layers are the same
		// In that case, if all goals present then the RPG is built successfully;
		// otherwise, its construction fails
		if (!same_fact_layers(current_fact_layer, previous_fact_layer)) {
			continue;
		}
		else if (!goals_present()) {	// same fact layers, but there exists a goals not present

#ifdef DEBUG_BUILD_RPG
			cout<<"End build_relaxed_planning graph... FALSE "<<__LINE__<<endl<<endl;
#endif

			return false;
		}
		else
			break;	// Two fact layers are exactly the same, and all goals are present
	}

#ifdef DEBUG_BUILD_RPG
	cout<<"End build_relaxed_planning graph... TRUE "<<__LINE__<<endl;
	cout<<"RPG length: "<<P.size()<<endl;
	cout<<"Goals prob: "<<goals_prob<<endl<<endl;
#endif

	return true;
}

bool RelaxedPlan::build_ff_relaxed_planning_graph() {

//#define DEBUG_BUILD_RPG
#ifdef DEBUG_BUILD_RPG
	cout<<"Begin build_relaxed_planning graph..."<<endl<<endl;
#endif

	bool goals_in_rpg = false;

	// Initialization
	initialize_ff_fact_layer();

	double goals_prob = 0;
	int length = 0;
	while (true) {
		if (!grow_ff_action_layer()) {
			cout<<endl<<"FAIL TO GROW ACTION LAYER!"<<endl;
			exit(1);
		}

		if (!grow_ff_fact_layer()) {
			cout<<endl<<"FAIL TO GROW FACT LAYER!"<<endl;
			exit(1);
		}

		if (++length >= MAX_RPG_LENGTH) {
			cout<<"Error! Increase max rpg length! File "<<__FILE__<<", line "<<__LINE__<<endl;
			exit(1);
		}

		// CHECKING STOPPING CONDITION
		int n = P.size() - 1;	// the last fact layer
		FactLayer& current_fact_layer = *(P[n]);
		if (goals_present())
			break;

		FactLayer& previous_fact_layer = *(P[n-1]);
		if (same_ff_fact_layers(current_fact_layer, previous_fact_layer))
			return false;
	}

#ifdef DEBUG_BUILD_RPG
	cout<<"End build_relaxed_planning graph... TRUE "<<__LINE__<<endl;
	cout<<"RPG length: "<<P.size()<<endl;
	cout<<"Goals prob: "<<goals_prob<<endl<<endl;
#endif

	return true;
}


// Create an RP_STEP containing the goal action
//boost::shared_ptr<RelaxedPlan::RP_STEP> RelaxedPlan::create_rp_step_for_goals() {
RelaxedPlan::RP_STEP *RelaxedPlan::create_rp_step_for_goals() {
	// The level at which all goals appear. So this is also the layer at which GOAL_ACTION is put
	int n = P.size() - 1;

	// The first and last fact layer
	const FactLayer& first_fact_layer = *(P[0]);
	const FactLayer& last_fact_layer = *(P[n]);

	// Create the step
	//boost::shared_ptr<RP_STEP> goal_step (new RP_STEP);
	RP_STEP *goal_step = new RP_STEP;
	goal_step->a = GOAL_ACTION;
	goal_step->layer = n;

	// Update the rp_state before it
	// first, for known propositions
	for (int i=0;i<current->num_known_F;i++) {
		int f = current->known_F[i];
		goal_step->s[f] = std::make_pair(1,0);
	}
	// second, for propositions that are not certainly known, but have probability of being true
	for (int i=0;i<current->num_F;i++) {
		int f = current->F[i];
		if (goal_step->s.find(f) == goal_step->s.end())
			goal_step->s[f] = std::make_pair(0,1);
	}

	// Update its clause set, and count the number of goals not in the current rp_state
	for (int i=0;i<goals->num_F; i++) {
		int g = goals->F[i];

		// If "g" is already in the current state, then we take the clauses for the fact at the current state
		if (in_rp_state(g, goal_step->s)) {
			assert(first_fact_layer.find(g) != first_fact_layer.end());

			if (first_fact_layer.at(g).best_clauses.size())
				(goal_step->pre_clauses)[g] = new ClauseSet(first_fact_layer.at(g).best_clauses);
		}
		else {

			num_unsupported_known_preconditions++;

#ifdef DEBUG_CREATE_RP_STEP_FOR_GOALS
			cout<<"A goal not in current state: ";
			print_ft_name(g);
			cout<<endl<<endl;
#endif

//#define DEBUG_UNSUPPORTED_KNOWN_PRECONDITIONS
#ifdef DEBUG_UNSUPPORTED_KNOWN_PRECONDITIONS
			unsupported_known_precondition_set.insert(make_pair(g, GOAL_ACTION));
#endif
		}
	}

	return goal_step;
}

// Check if there exists any unsupported known preconditions in the relaxed plan
bool RelaxedPlan::unsupported_known_precondition_exists() {
	bool unsupported_known_preconditions = false;
	for (RELAXED_PLAN::iterator itr = rp.begin(); itr != rp.end() && !unsupported_known_preconditions; itr++) {
		int a = (*itr)->a;
		if (a != GOAL_ACTION) {
			int ef = gop_conn[a].E[0];
			for (int i=0;i<gef_conn[ef].num_PC && !unsupported_known_preconditions;i++) {
				int p = gef_conn[ef].PC[i];
				if (!in_rp_state(p, (*itr)->s)) {
					unsupported_known_preconditions = true;
//					cout<<"Unsupported known pc: "<<p<<endl;
//					cout<<"Op: "<<a<<endl<<endl;
				}
			}
		}
		else {
			for (int i=0;i<goals->num_F && !unsupported_known_preconditions; i++) {
				int g = goals->F[i];
				if (!in_rp_state(g, (*itr)->s)) {
					unsupported_known_preconditions = true;
//					cout<<"Unsupported known pc (goal): "<<g<<endl;
//					cout<<"Op: "<<a<<endl<<endl;
				}
			}
		}
	}
	return unsupported_known_preconditions;
}

bool RelaxedPlan::extract_incremental_robustness_rp(pair<int, double>& result) {

//#define DEBUG_EXTRACT
//
//#define DEBUG_UNSUPPORTED_KNOWN_PRECONDITIONS

#ifdef DEBUG_EXTRACT
	cout<<"Begin extract_01... Robustness threshold: "<<robustness_threshold<<endl<<endl;
#endif


	if (!build_relaxed_planning_graph()) {

#ifdef DEBUG_EXTRACT
		cout<<"End extract_01..."<<__LINE__<<endl<<endl;
#endif

		return false;
	}

	// The level at which all goals appear. So this is also the layer at which GOAL_ACTION is put
	int n = P.size() - 1;

	// The first and last fact layer
	const FactLayer& first_fact_layer = *(P[0]);
	const FactLayer& last_fact_layer = *(P[n]);

	// Initialize the number of actions chosen to the relaxed plan at each layer of the RPG
	num_chosen_actions.reserve(n+1);
	for (int i=0;i<=n;i++) num_chosen_actions[i] = 0;

	// Initialize the number of unsupported known preconditions
	num_unsupported_known_preconditions = 0;

	// Create the step containing goal action of the relaxed plan
	RP_STEP *goal_step = create_rp_step_for_goals();

	// Initialize the relaxed plan with the goal step, and increase the counter
	// REMINDER: release memory for "rp"
	assert(rp.size() == 0);
	rp.push_back(goal_step);
	num_chosen_actions[n] = 1;

	// The robustness of plan prefix + the current relaxed plan (0 if there exists an unsupported known preconditions in the RP)
	// Note that this can be either lower bound, upper bound or exact value of the robustness, depending on the flags
	// "-use_lower_bound_in_rp" and "-use_upper_bound_in_rp;"
	double current_robustness;

	// The lower bound, upper bound of the robustness of the plan prefix + the empty relaxed plan
	double lower_robustness, upper_robustness, exact_robustness;
	if (compute_robustness(lower_robustness, upper_robustness, exact_robustness)) {	// Succeed only if no unsupported known preconditions exist

		if (RelaxedPlan::use_lower_bound_in_rp) {
			current_robustness = lower_robustness;
		}
		else if (RelaxedPlan::use_upper_bound_in_rp) {
			current_robustness = upper_robustness;
		}
		else {
			current_robustness = exact_robustness;
		}

		// Check if the empty relaxed plan is enough
		if (RelaxedPlan::use_robustness_threshold && exact_robustness > robustness_threshold) {

			result.first = 0; // Don't count the goal step
			if (exact_robustness > robustness_threshold)
				result.second = exact_robustness;
			else
				result.second = current_robustness;

#ifdef DEBUG_EXTRACT
				cout<<"End extract_01..."<<__LINE__<<endl<<endl;
#endif

				return true;	// EMPTY RELAXED PLAN RETURNED!!!
		}
	}
	// Otherwise, there exists unsupported known goals in the empty RP. So the robustness is 0.
	else {
		current_robustness = 0;
	}

//	// Compute the current robustness of the plan prefix + current EMPTY relaxed plan
//	// Check it against the robustness threshold
//	double current_robustness = compute_robustness();
//
//#ifdef DEBUG_EXTRACT
//	cout<<"Current robustness: "<<current_robustness<<endl<<endl;
//#endif
//
//	if (RelaxedPlan::use_robustness_threshold && current_robustness > robustness_threshold) {
//		result.first = 0;	// Don't count the goal step
//		result.second = current_robustness;
//
//#ifdef DEBUG_EXTRACT
//		cout<<"End extract_01..."<<__LINE__<<endl<<endl;
//#endif
//
//		return true;	// Empty relaxed plan!
//	}

	// During the extraction of the relaxed plan, we keep a robustness value
	// of the current relaxed plan. We use a clause set constructed taking into account
	// clause sets in the RPG, thus quite different from the set used to compute robustness
	// Note: when the relaxed plan does not have any unsupported known preconditions, it is equal to
	// the robustness of the relaxed plan
	double current_robustness_for_heuristics;
	ClauseSet current_clauses_for_heuristics;

	// Clause set of the current plan prefix
	ClauseSet clauses_of_plan_prefix;
	e->get_clauses(clauses_of_plan_prefix);

	// The queue to store all subgoals
	SubGoalQueue Q;

	// Initialize Q with the all the top level goals
	for (int i=0;i<goals->num_F;i++) {

		int g = goals->F[i];

		assert(last_fact_layer.find(g) != last_fact_layer.end());

		// Ignore goals that are certainly known in the current state
		if (RelaxedPlan::ignore_poss_del_in_rp && is_known_in_state(g, current)) {

#ifdef DEBUG_EXTRACT
			cout<<"Subgoal ignored (known in current state): "<<g<<endl<<endl;
#endif
			continue;
		}

		// Now the goal is either not in the state or in the state but not certainly known to be true

		SubGoal subgoal(g, GOAL_ACTION, n, false, &goal_step->s);
		Q.push(subgoal);

#ifdef DEBUG_UNSUPPORTED_KNOWN_PRECONDITIONS
		SubgoalSet.insert(make_pair(g, GOAL_ACTION));
#endif


#ifdef DEBUG_EXTRACT
		cout<<"New subgoal added: ";
		subgoal.print();
		cout<<endl<<endl;

		cout<<"Current Q'top: ";
		Q.top().print();
		cout<<endl<<endl;
#endif

		// Initialize clause sets for heuristics associated with these goals
		if (is_in_state(g, current)) {
			ClauseSet cs;
			e->supporting_constraints(g, e->get_actions().size(), cs);
			if (cs.size())
				current_clauses_for_heuristics.add_clauses(cs);
		}
		else if (RelaxedPlan::clauses_from_rpg_for_false_preconditions) {
			const FactNode& g_node = last_fact_layer.at(g);
			if (g_node.best_clauses.size())
				current_clauses_for_heuristics.add_clauses(g_node.best_clauses);
		}
	}

	// Add all clauses of the current plan prefix
	current_clauses_for_heuristics.add_clauses(clauses_of_plan_prefix);

	// Compute the current "robustness for heuristics"
	if (RelaxedPlan::use_lower_bound_in_rp)
		current_robustness_for_heuristics = current_clauses_for_heuristics.lower_wmc();
	else if (RelaxedPlan::use_upper_bound_in_rp)
		current_robustness_for_heuristics = current_clauses_for_heuristics.upper_wmc();
	else {
		CACHET_OUTPUT o;
		current_clauses_for_heuristics.wmc(o);
		current_robustness_for_heuristics = o.prob;
		wmc_time += o.time;
	}

#ifdef DEBUG_EXTRACT
	cout<<"current_clauses_for_heuristics: "<<current_clauses_for_heursitcis<<endl<<endl;
	cout<<"current_robustness_for_heuristics: "<<current_robustness_for_heuristics<<endl<<endl;
#endif


#ifdef DEBUG_EXTRACT
	cout<<"Num unsupported known preconditions: "<<num_unsupported_known_preconditions<<endl<<endl;
	cout<<"In Q-loop"<<endl<<endl;
#endif

#ifdef DEBUG_UNSUPPORTED_KNOWN_PRECONDITIONS
	cout<<"Unsupported known precondition set: ";
	print_unsupported_known_precondition_set();
	cout<<endl<<endl;

	cout<<"Subgoal set: ";
	print_subgoal_set();
	cout<<endl<<endl;
#endif

	// Extracting actions in the relaxed planning graph to support subgoals in Q
	while (!Q.empty()) {

		// Retrieve the next subgoal
		SubGoal subgoal = Q.top();
		Q.pop();

#ifdef DEBUG_UNSUPPORTED_KNOWN_PRECONDITIONS
		SubgoalSet.erase(make_pair(subgoal.g, subgoal.op));
#endif


#ifdef DEBUG_EXTRACT
		TAB(2);	cout<<"Subgoal popped: "<<subgoal.g<<"..."<<endl<<endl;
		TAB(2); cout<<"State before subgoal:";
		const RP_STATE& sg = *subgoal.state_before_op;
		for (int ft=0;ft<gnum_ft_conn;ft++) {
			if (in_rp_state(ft, sg))
				cout<<"F"<<ft<<" ";
		}
		cout<<endl<<endl;
#endif

#ifndef NDEBUG
		// This action must be found in the relaxed plan
		RELAXED_PLAN::iterator rp_itr = rp.begin();
		for (; rp_itr != rp.end(); rp_itr++)
			if ((*rp_itr)->a == subgoal.op)
				break;
		assert(rp_itr != rp.end());
#endif

		// If this subgoal is not in its fact layer in the RPG, which also means
		// it is the possible precondition subgoal, we simply ignore this subgoal
		// (since there is not any best supporting action for it)
		if (!fact_present(subgoal.g, subgoal.l)) {
			assert(subgoal.possible_precondition);

#ifdef DEBUG_EXTRACT
			TAB(3);
			cout<<"Subgoal ignored (possible precondition not in fact layer "<<subgoal.l<<")."<<endl<<endl;
#endif


			// Continue with the next subgoal
			continue;
		}

		// Find the best action supporting this subgoal.
		// CURRENT STRATEGY: take the one found in the RPG construction
		int l = subgoal.l;
		while (l > 0 && P[l]->at(subgoal.g).best_supporting_action == NOOP) {
			l--;
		}

		if (l==0) {

			if (!fact_present(subgoal.g, l))
				assert(subgoal.possible_precondition);
			else {
				assert(P[l]->at(subgoal.g).best_supporting_action == e->get_actions()[e->get_actions().size()-1] ||
						P[l]->at(subgoal.g).best_supporting_action == INIT_ACTION);
			}

#ifdef DEBUG_EXTRACT
			TAB(3);
			cout<<"Subgoal ignored (possible precondition not in fact layer "<<l<<")."<<endl<<endl;
#endif


			// The best supporting action is the last action in the plan prefix, or the INIT_ACTION.
			// Continue with the next subgoal
			continue;

		}

		const int candidate_action = (P[l])->at(subgoal.g).best_supporting_action;
		int layer_of_candidate_action = l - 1;
		assert(candidate_action >= 0 && candidate_action < gnum_op_conn);

		// If this candidate action has been chosen, we don't add it again
		// We're done with this subgoal
		if (A[layer_of_candidate_action]->at(candidate_action).in_rp) {

#ifdef DEBUG_EXTRACT
			TAB(3);
			cout<<"Candidate action "<<candidate_action<<" ignored."<<endl<<endl;
#endif


			continue;	// Consider next subgoals
		}

		// Pointer to the new RP_STEP (if insertion happens)
		//boost::shared_ptr<RP_STEP> new_rp_step;
		RP_STEP *new_rp_step = 0;

		// Case 1: this subgoal is not present in the state before it
		if (!in_rp_state(subgoal.g, *subgoal.state_before_op)) {

			// If this subgoal is known precondition of its action, then
			// the candidate action will be inserted
			if (!subgoal.possible_precondition) {
				assert(current_robustness <= 0);
				new_rp_step = insert_action_into_relaxed_plan(candidate_action, layer_of_candidate_action);

				// Recompute the "robustness for heuristics"
				ClauseSet cs_for_heuristics;
				collect_rp_step_clauses_for_heuristics(rp.begin(), rp.end(), cs_for_heuristics);

				ClauseSet cs_for_plan_prefix;
				e->get_clauses(cs_for_plan_prefix);
				cs_for_heuristics.add_clauses(cs_for_plan_prefix);

				if (RelaxedPlan::use_lower_bound_in_rp)
					current_robustness_for_heuristics = cs_for_heuristics.lower_wmc();
				else if (RelaxedPlan::use_upper_bound_in_rp)
					current_robustness_for_heuristics = cs_for_heuristics.upper_wmc();
				else {
					CACHET_OUTPUT o;
					cs_for_heuristics.wmc(o);
					current_robustness_for_heuristics = o.prob;
					wmc_time += o.time;
				}
				//

#ifdef DEBUG_EXTRACT
				TAB(3);	cout<<"Candidate action "<<candidate_action<<" inserted."<<endl<<endl;
				TAB(3);	cout<<"Current robustness for heuristics: "<<current_robustness_for_heuristics<<endl<<endl;
				cout<<"-- RP-STEPS --"<<endl<<endl;
				for (int i=0;i<this->length()-1;i++) {
					print_rp_step(*this, i);
					cout<<endl<<endl;
				}
#endif

			}
			// Otherwise (i.e., it is possible precondition), since it is okay for
			// a possible precondition not having any support in a relaxed plan,
			// we only insert the candidate action if it increases the robustness
			else {

				RelaxedPlan::num_rp_robustness_increasing_checks++;

				// Evaluate the candidate action, return the POTENTIAL robustness
				// In computing this potential robustness, for preconditions not present in the rp_states
				// we use the clause sets constructed during the RPG construction
				double new_robustness_for_heuristics = evaluate_candidate_action(candidate_action, layer_of_candidate_action);

				// We insert it only if the "robustness for heuristics" increases
				if (new_robustness_for_heuristics > current_robustness_for_heuristics) {
					new_rp_step = insert_action_into_relaxed_plan(candidate_action, layer_of_candidate_action);
					current_robustness_for_heuristics = new_robustness_for_heuristics;

					RelaxedPlan::num_rp_robustness_increasing_check_success++;

#ifdef DEBUG_EXTRACT
					TAB(3);	cout<<"Candidate action "<<candidate_action<<" inserted."<<endl<<endl;
					TAB(3);	cout<<"Current robustness for heuristics: "<<current_robustness_for_heuristics<<endl<<endl;

					cout<<"-- RP-STEPS --"<<endl<<endl;
					for (int i=0;i<this->length()-1;i++) {
						print_rp_step(*this, i);
						cout<<endl<<endl;
					}
#endif

				}
			}
		}

		// Case 2: this subgoal is present in the state before it
		// Evaluate the action and insert only if it increases the robustness
		else {

			RelaxedPlan::num_rp_robustness_increasing_checks++;

			// Evaluate the candidate action, return the POTENTIAL robustness
			// In computing this potential robustness, for preconditions not present in the rp_states
			// we use the clause sets constructed during the RPG construction
			double new_robustness_for_heuristics = evaluate_candidate_action(candidate_action, layer_of_candidate_action);

			// We insert it only if the "robustness for heuristics" increases
			if (new_robustness_for_heuristics > current_robustness_for_heuristics) {
				new_rp_step = insert_action_into_relaxed_plan(candidate_action, layer_of_candidate_action);
				current_robustness_for_heuristics = new_robustness_for_heuristics;

				RelaxedPlan::num_rp_robustness_increasing_check_success++;

#ifdef DEBUG_EXTRACT
				TAB(3); cout<<"Candidate action "<<candidate_action<<" ignored."<<endl<<endl;
				TAB(3); cout<<"Current robustness for heuristics: "<<current_robustness_for_heuristics<<endl<<endl;

				cout<<"-- RP-STEPS --"<<endl<<endl;
				for (int i=0;i<this->length()-1;i++) {
					print_rp_step(*this, i);
					cout<<endl<<endl;
				}
#endif

			}
		} // end of Case 2

		// If the new action is inserted
		if (new_rp_step) {

			// Whenever we insert a new action into the relaxed plan, we re-compute its robustness
			// This will quickly return 0 if there exists unsupported known preconditions
			current_robustness = compute_robustness();

#ifdef DEBUG_EXTRACT
			TAB(3); cout<<"current_robustness: "<<current_robustness<<endl<<endl;
#endif

			// RELAXED PLAN FOUND!!!!
			if (RelaxedPlan::use_robustness_threshold && current_robustness > robustness_threshold) {

#ifdef DEBUG_EXTRACT
				cout<<"Out Q-loop "<<__LINE__<<endl<<endl;
#endif

				break;
			}

			//
			// Add new subgoals
			assert(gop_conn[candidate_action].num_E == 1);
			int ef = gop_conn[candidate_action].E[0];
			for (int i=0;i<gef_conn[ef].num_PC;i++) {
				int p = gef_conn[ef].PC[i];

				// This can be improved: we need only check if "p" is known at the rp_state
				// To do this, known facts at rp_states must be maintained
				if (RelaxedPlan::ignore_poss_del_in_rp && is_known_in_state(p, current)) {

#ifdef DEBUG_EXTRACT
					TAB(3); cout<<"Subgoal ignored (known in current state): "<<p<<endl<<endl;
#endif

					continue;
				}

				SubGoal subgoal(p, candidate_action, layer_of_candidate_action, false, &new_rp_step->s);
				Q.push(subgoal);

#ifdef DEBUG_UNSUPPORTED_KNOWN_PRECONDITIONS
				SubgoalSet.insert(make_pair(p, candidate_action));
#endif


#ifdef DEBUG_EXTRACT
				TAB(3); cout<<"New subgoal added: ";
				subgoal.print();
				cout<<endl<<endl;

				TAB(3); cout<<"Current Q'top: ";
				Q.top().print();
				cout<<endl<<endl;
#endif


			}
			for (int i=0;i<gef_conn[ef].num_poss_PC;i++) {
				int p = gef_conn[ef].poss_PC[i];

				if (RelaxedPlan::ignore_poss_del_in_rp && is_known_in_state(p, current))
					continue;
				SubGoal subgoal(p, candidate_action, layer_of_candidate_action, true, &new_rp_step->s);
				Q.push(subgoal);

#ifdef DEBUG_UNSUPPORTED_KNOWN_PRECONDITIONS
				SubgoalSet.insert(make_pair(p, candidate_action));
#endif


#ifdef DEBUG_EXTRACT
				TAB(3); cout<<"New subgoal added: ";
				subgoal.print();
				cout<<endl<<endl;

				TAB(3); cout<<"Current Q'top: ";
				Q.top().print();
				cout<<endl<<endl;
#endif
			}

#ifdef DEBUG_UNSUPPORTED_KNOWN_PRECONDITIONS
				cout<<"Unsupported known precondition set: ";
				print_unsupported_known_precondition_set();
				cout<<endl<<endl;

				cout<<"Subgoal set: ";
				print_subgoal_set();
				cout<<endl<<endl;

				// Check if all unsupported known preconditions present in the queue
				for (std::set<std::pair<int,int> >::const_iterator itr = unsupported_known_precondition_set.begin();
						itr != unsupported_known_precondition_set.end(); itr++) {
					assert(SubgoalSet.find(make_pair(itr->first, itr->second)) != SubgoalSet.end());
				}
#endif
		}

	} // out of Q-loop

#ifdef DEBUG_EXTRACT
	cout<<"Out Q-loop "<<__LINE__<<endl<<endl;
	cout<<"Num unsupported known preconditions: "<<num_unsupported_known_preconditions<<endl<<endl;
	cout<<"current_robustness_for_heuristics: "<<current_robustness_for_heuristics<<endl<<endl;
	cout<<"current_robustness: "<<compute_robustness()<<endl<<endl;
	cout<<"RP length: "<<rp.size()<<endl<<endl;
	cout<<"End extract_01..."<<__LINE__<<endl<<endl;
#endif

#ifdef DEBUG_UNSUPPORTED_KNOWN_PRECONDITIONS
	cout<<"Unsupported known precondition set: ";
	for (set<pair<int,int> >::const_iterator itr = unsupported_known_precondition_set.begin();
			itr != unsupported_known_precondition_set.end(); itr++) {
		cout<<"("<<itr->first<<", "<<itr->second<<") ";
	}
	cout<<endl<<endl;
#endif


#ifndef NDEBUG
	assert(!unsupported_known_precondition_exists());
#endif

	// This must be satisfied, since a trivial relaxed plan includes all "best actions", with which
	// all known preconditions are supported
	assert(num_unsupported_known_preconditions == 0);

	// This must be true, because when all known preconditions are supported, clause sets from RPG no longer need
	//assert(current_robustness_for_heuristics == current_robustness);

	result.first = rp.size() - 1;	// Ignore the goal step
	result.second = current_robustness;

	// Check if the relaxed plan returned has enough robustness
	if (current_robustness <= robustness_threshold) {
		return false;
	}

	return true;
}

bool RelaxedPlan::extract_locally_incremental_robustness_rp(pair<int, double>& result) {

//#define DEBUG_EXTRACT
//
//#define DEBUG_UNSUPPORTED_KNOWN_PRECONDITIONS

#ifdef DEBUG_EXTRACT
	cout<<"Begin extract_01... Robustness threshold: "<<robustness_threshold<<endl<<endl;
#endif


	if (!build_relaxed_planning_graph()) {

#ifdef DEBUG_EXTRACT
		cout<<"End extract_01..."<<__LINE__<<endl<<endl;
#endif

		return false;
	}

	// The level at which all goals appear. So this is also the layer at which GOAL_ACTION is put
	int n = P.size() - 1;

	// The first and last fact layer
	const FactLayer& first_fact_layer = *(P[0]);
	const FactLayer& last_fact_layer = *(P[n]);

	// Initialize the number of actions chosen to the relaxed plan at each layer of the RPG
	num_chosen_actions.reserve(n+1);
	for (int i=0;i<=n;i++) num_chosen_actions[i] = 0;

	// Initialize the number of unsupported known preconditions
	num_unsupported_known_preconditions = 0;

	// Create the step containing goal action of the relaxed plan
	RP_STEP *goal_step = create_rp_step_for_goals();

	// Initialize the relaxed plan with the goal step, and increase the counter
	// REMINDER: release memory for "rp"
	assert(rp.size() == 0);
	rp.push_back(goal_step);
	num_chosen_actions[n] = 1;

	// The robustness of plan prefix + the current relaxed plan (0 if there exists an unsupported known preconditions in the RP)
	// Note that this can be either lower bound, upper bound or exact value of the robustness, depending on the flags
	// "-use_lower_bound_in_rp" and "-use_upper_bound_in_rp;"
	double current_robustness;

	// The lower bound, upper bound of the robustness of the plan prefix + the empty relaxed plan
	double lower_robustness, upper_robustness, exact_robustness;
	if (compute_robustness(lower_robustness, upper_robustness, exact_robustness)) {	// Succeed only if no unsupported known preconditions exist

		if (RelaxedPlan::use_lower_bound_in_rp) {
			current_robustness = lower_robustness;
		}
		else if (RelaxedPlan::use_upper_bound_in_rp) {
			current_robustness = upper_robustness;
		}
		else {
			current_robustness = exact_robustness;
		}

		// Check if the empty relaxed plan is enough
		if (RelaxedPlan::use_robustness_threshold && exact_robustness > robustness_threshold) {

			result.first = 0; // Don't count the goal step
			if (exact_robustness > robustness_threshold)
				result.second = exact_robustness;
			else
				result.second = current_robustness;

#ifdef DEBUG_EXTRACT
				cout<<"End extract_01..."<<__LINE__<<endl<<endl;
#endif

				return true;	// EMPTY RELAXED PLAN RETURNED!!!
		}
	}
	// Otherwise, there exists unsupported known goals in the empty RP. So the robustness is 0.
	else {
		current_robustness = 0;
	}

	// Clause set of the current plan prefix
	ClauseSet clauses_of_plan_prefix;
	e->get_clauses(clauses_of_plan_prefix);

	// The queue to store all subgoals
	SubGoalQueue Q;

	// Initialize Q with the all the top level goals
	for (int i=0;i<goals->num_F;i++) {

		int g = goals->F[i];

		assert(last_fact_layer.find(g) != last_fact_layer.end());

		// Ignore goals that are certainly known in the current state
		if (RelaxedPlan::ignore_poss_del_in_rp && is_known_in_state(g, current)) {

#ifdef DEBUG_EXTRACT
			cout<<"Subgoal ignored (known in current state): "<<g<<endl<<endl;
#endif
			continue;
		}

		// Now the goal is either not in the state or in the state but not certainly known to be true

		SubGoal subgoal(g, GOAL_ACTION, n, false, &goal_step->s);
		Q.push(subgoal);

#ifdef DEBUG_UNSUPPORTED_KNOWN_PRECONDITIONS
		SubgoalSet.insert(make_pair(g, GOAL_ACTION));
#endif


#ifdef DEBUG_EXTRACT
		cout<<"New subgoal added: ";
		subgoal.print();
		cout<<endl<<endl;

		cout<<"Current Q'top: ";
		Q.top().print();
		cout<<endl<<endl;
#endif

	}

#ifdef DEBUG_EXTRACT
	cout<<"current_clauses_for_heuristics: "<<current_clauses_for_heursitcis<<endl<<endl;
	cout<<"current_robustness_for_heuristics: "<<current_robustness_for_heuristics<<endl<<endl;
#endif


#ifdef DEBUG_EXTRACT
	cout<<"Num unsupported known preconditions: "<<num_unsupported_known_preconditions<<endl<<endl;
	cout<<"In Q-loop"<<endl<<endl;
#endif

#ifdef DEBUG_UNSUPPORTED_KNOWN_PRECONDITIONS
	cout<<"Unsupported known precondition set: ";
	print_unsupported_known_precondition_set();
	cout<<endl<<endl;

	cout<<"Subgoal set: ";
	print_subgoal_set();
	cout<<endl<<endl;
#endif

	// Extracting actions in the relaxed planning graph to support subgoals in Q
	while (!Q.empty()) {

		// Retrieve the next subgoal
		SubGoal subgoal = Q.top();
		Q.pop();

#ifdef DEBUG_UNSUPPORTED_KNOWN_PRECONDITIONS
		SubgoalSet.erase(make_pair(subgoal.g, subgoal.op));
#endif


#ifdef DEBUG_EXTRACT
		TAB(2);	cout<<"Subgoal popped: "<<subgoal.g<<"..."<<endl<<endl;
		TAB(2); cout<<"State before subgoal:";
		const RP_STATE& sg = *subgoal.state_before_op;
		for (int ft=0;ft<gnum_ft_conn;ft++) {
			if (in_rp_state(ft, sg))
				cout<<"F"<<ft<<" ";
		}
		cout<<endl<<endl;
#endif

#ifndef NDEBUG
		// This action must be found in the relaxed plan
		RELAXED_PLAN::iterator rp_itr = rp.begin();
		for (; rp_itr != rp.end(); rp_itr++)
			if ((*rp_itr)->a == subgoal.op)
				break;
		assert(rp_itr != rp.end());
#endif

		// If this subgoal is not in its fact layer in the RPG, which also means
		// it is the possible precondition subgoal, we simply ignore this subgoal
		// (since there is not any best supporting action for it)
		if (!fact_present(subgoal.g, subgoal.l)) {
			assert(subgoal.possible_precondition);

#ifdef DEBUG_EXTRACT
			TAB(3);
			cout<<"Subgoal ignored (possible precondition not in fact layer "<<subgoal.l<<")."<<endl<<endl;
#endif

			// Continue with the next subgoal
			continue;
		}

		// Find the best action supporting this subgoal.
		// CURRENT STRATEGY: take the one found in the RPG construction
		int l = subgoal.l;
		while (l > 0 && P[l]->at(subgoal.g).best_supporting_action == NOOP) {
			l--;
		}

		if (l==0) {

			if (!fact_present(subgoal.g, l))
				assert(subgoal.possible_precondition);
			else {
				assert(P[l]->at(subgoal.g).best_supporting_action == e->get_actions()[e->get_actions().size()-1] ||
						P[l]->at(subgoal.g).best_supporting_action == INIT_ACTION);
			}

#ifdef DEBUG_EXTRACT
			TAB(3);
			cout<<"Subgoal ignored (possible precondition not in fact layer "<<l<<")."<<endl<<endl;
#endif


			// The best supporting action is the last action in the plan prefix, or the INIT_ACTION.
			// Continue with the next subgoal
			continue;

		}

		const int candidate_action = (P[l])->at(subgoal.g).best_supporting_action;
		int layer_of_candidate_action = l - 1;
		assert(candidate_action >= 0 && candidate_action < gnum_op_conn);

		// If this candidate action has been chosen, we don't add it again
		// We're done with this subgoal
		if (A[layer_of_candidate_action]->at(candidate_action).in_rp) {

#ifdef DEBUG_EXTRACT
			TAB(3);
			cout<<"Candidate action "<<candidate_action<<" ignored."<<endl<<endl;
#endif


			continue;	// Consider next subgoals
		}

		// Pointer to the new RP_STEP (if insertion happens)
		//boost::shared_ptr<RP_STEP> new_rp_step;
		RP_STEP *new_rp_step = 0;

		// Case 1: this subgoal is not present in the state before it
		// Whether it is known or possible precondition, inserting the supporting action always
		// helps increase *its* probability of being true!
		if (!in_rp_state(subgoal.g, *subgoal.state_before_op)) {

			new_rp_step = insert_action_into_relaxed_plan(candidate_action, layer_of_candidate_action);

		}

		// Case 2: this subgoal is present in the state before it
		else {
			// Sub-case 1: if this subgoal is *known* at its rp-state, then adding the supporting action
			// does not increase the subgoal probability of being true
			// So for this sub-case, we simply do not insert this supporting action

			// Sub-case 2:  otherwise, inserting this supporting action will increase the probability of being
			// true of this subgoal. The amount of this increasing is: probability for this supporting action
			// to be executable \times the probability of this subgoal being its realized add effect
			RelaxedPlan::num_rp_robustness_increasing_checks++;

			if (!known_in_rp_state(subgoal.g, *subgoal.state_before_op)) {
				new_rp_step = insert_action_into_relaxed_plan(candidate_action, layer_of_candidate_action);
				RelaxedPlan::num_rp_robustness_increasing_check_success++;
			}
		} // end of Case 2

		// If the new action is inserted
		if (new_rp_step) {

			// Whenever we insert a new action into the relaxed plan, we re-compute its robustness
			// This will quickly return 0 if there exists unsupported known preconditions
			current_robustness = compute_robustness();

#ifdef DEBUG_EXTRACT
			TAB(3); cout<<"current_robustness: "<<current_robustness<<endl<<endl;
#endif

			// RELAXED PLAN FOUND!!!!
			if (RelaxedPlan::use_robustness_threshold && current_robustness > robustness_threshold) {

#ifdef DEBUG_EXTRACT
				cout<<"Out Q-loop "<<__LINE__<<endl<<endl;
#endif

				break;
			}

			//
			// Add new subgoals
			assert(gop_conn[candidate_action].num_E == 1);
			int ef = gop_conn[candidate_action].E[0];
			for (int i=0;i<gef_conn[ef].num_PC;i++) {
				int p = gef_conn[ef].PC[i];

				// This can be improved: we need only check if "p" is known at the rp_state
				// To do this, known facts at rp_states must be maintained
				if (RelaxedPlan::ignore_poss_del_in_rp && is_known_in_state(p, current)) {

#ifdef DEBUG_EXTRACT
					TAB(3); cout<<"Subgoal ignored (known in current state): "<<p<<endl<<endl;
#endif

					continue;
				}

				SubGoal subgoal(p, candidate_action, layer_of_candidate_action, false, &new_rp_step->s);
				Q.push(subgoal);

#ifdef DEBUG_UNSUPPORTED_KNOWN_PRECONDITIONS
				SubgoalSet.insert(make_pair(p, candidate_action));
#endif


#ifdef DEBUG_EXTRACT
				TAB(3); cout<<"New subgoal added: ";
				subgoal.print();
				cout<<endl<<endl;

				TAB(3); cout<<"Current Q'top: ";
				Q.top().print();
				cout<<endl<<endl;
#endif


			}
			for (int i=0;i<gef_conn[ef].num_poss_PC;i++) {
				int p = gef_conn[ef].poss_PC[i];

				if (RelaxedPlan::ignore_poss_del_in_rp && is_known_in_state(p, current))
					continue;
				SubGoal subgoal(p, candidate_action, layer_of_candidate_action, true, &new_rp_step->s);
				Q.push(subgoal);

#ifdef DEBUG_UNSUPPORTED_KNOWN_PRECONDITIONS
				SubgoalSet.insert(make_pair(p, candidate_action));
#endif


#ifdef DEBUG_EXTRACT
				TAB(3); cout<<"New subgoal added: ";
				subgoal.print();
				cout<<endl<<endl;

				TAB(3); cout<<"Current Q'top: ";
				Q.top().print();
				cout<<endl<<endl;
#endif
			}

#ifdef DEBUG_UNSUPPORTED_KNOWN_PRECONDITIONS
				cout<<"Unsupported known precondition set: ";
				print_unsupported_known_precondition_set();
				cout<<endl<<endl;

				cout<<"Subgoal set: ";
				print_subgoal_set();
				cout<<endl<<endl;

				// Check if all unsupported known preconditions present in the queue
				for (std::set<std::pair<int,int> >::const_iterator itr = unsupported_known_precondition_set.begin();
						itr != unsupported_known_precondition_set.end(); itr++) {
					assert(SubgoalSet.find(make_pair(itr->first, itr->second)) != SubgoalSet.end());
				}
#endif
		}

	} // out of Q-loop

#ifdef DEBUG_EXTRACT
	cout<<"Out Q-loop "<<__LINE__<<endl<<endl;
	cout<<"Num unsupported known preconditions: "<<num_unsupported_known_preconditions<<endl<<endl;
	cout<<"current_robustness_for_heuristics: "<<current_robustness_for_heuristics<<endl<<endl;
	cout<<"current_robustness: "<<compute_robustness()<<endl<<endl;
	cout<<"RP length: "<<rp.size()<<endl<<endl;
	cout<<"End extract_01..."<<__LINE__<<endl<<endl;
#endif

#ifdef DEBUG_UNSUPPORTED_KNOWN_PRECONDITIONS
	cout<<"Unsupported known precondition set: ";
	for (set<pair<int,int> >::const_iterator itr = unsupported_known_precondition_set.begin();
			itr != unsupported_known_precondition_set.end(); itr++) {
		cout<<"("<<itr->first<<", "<<itr->second<<") ";
	}
	cout<<endl<<endl;
#endif


#ifndef NDEBUG
	assert(!unsupported_known_precondition_exists());
#endif

	// This must be satisfied, since a trivial relaxed plan includes all "best actions", with which
	// all known preconditions are supported
	assert(num_unsupported_known_preconditions == 0);

	// This must be true, because when all known preconditions are supported, clause sets from RPG no longer need
	//assert(current_robustness_for_heuristics == current_robustness);

	result.first = rp.size() - 1;	// Ignore the goal step
	result.second = current_robustness;

	// Check if the relaxed plan returned has enough robustness
	if (current_robustness <= robustness_threshold) {
		return false;
	}

	return true;
}

bool RelaxedPlan::extract_greedy_robustness_rp(pair<int, double>& result) {

//#define DEBUG_EXTRACT
//
//#define DEBUG_UNSUPPORTED_KNOWN_PRECONDITIONS

#ifdef DEBUG_EXTRACT
	cout<<"Begin extract_01... Robustness threshold: "<<robustness_threshold<<endl<<endl;
#endif


	if (!build_relaxed_planning_graph()) {

#ifdef DEBUG_EXTRACT
		cout<<"End extract_01..."<<__LINE__<<endl<<endl;
#endif

		return false;
	}

	// The level at which all goals appear. So this is also the layer at which GOAL_ACTION is put
	int n = P.size() - 1;

	// The first and last fact layer
	const FactLayer& first_fact_layer = *(P[0]);
	const FactLayer& last_fact_layer = *(P[n]);

	// Initialize the number of actions chosen to the relaxed plan at each layer of the RPG
	num_chosen_actions.reserve(n+1);
	for (int i=0;i<=n;i++) num_chosen_actions[i] = 0;

	// Initialize the number of unsupported known preconditions
	num_unsupported_known_preconditions = 0;

	// Create the step containing goal action of the relaxed plan
	RP_STEP *goal_step = create_rp_step_for_goals();

	// Initialize the relaxed plan with the goal step, and increase the counter
	// REMINDER: release memory for "rp"
	assert(rp.size() == 0);
	rp.push_back(goal_step);
	num_chosen_actions[n] = 1;

	// The robustness of plan prefix + the current relaxed plan (0 if there exists an unsupported known preconditions in the RP)
	// Note that this can be either lower bound, upper bound or exact value of the robustness, depending on the flags
	// "-use_lower_bound_in_rp" and "-use_upper_bound_in_rp;"
	double current_robustness;

	// The lower bound, upper bound of the robustness of the plan prefix + the empty relaxed plan
	double lower_robustness, upper_robustness, exact_robustness;
	if (compute_robustness(lower_robustness, upper_robustness, exact_robustness)) {	// Succeed only if no unsupported known preconditions exist

		if (RelaxedPlan::use_lower_bound_in_rp) {
			current_robustness = lower_robustness;
		}
		else if (RelaxedPlan::use_upper_bound_in_rp) {
			current_robustness = upper_robustness;
		}
		else {
			current_robustness = exact_robustness;
		}

		// Check if the empty relaxed plan is enough
		if (RelaxedPlan::use_robustness_threshold && exact_robustness > robustness_threshold) {

			result.first = 0; // Don't count the goal step
			if (exact_robustness > robustness_threshold)
				result.second = exact_robustness;
			else
				result.second = current_robustness;

#ifdef DEBUG_EXTRACT
				cout<<"End extract_01..."<<__LINE__<<endl<<endl;
#endif

				return true;	// EMPTY RELAXED PLAN RETURNED!!!
		}
	}
	// Otherwise, there exists unsupported known goals in the empty RP. So the robustness is 0.
	else {
		current_robustness = 0;
	}

	// Clause set of the current plan prefix
	ClauseSet clauses_of_plan_prefix;
	e->get_clauses(clauses_of_plan_prefix);

	// The queue to store all subgoals
	SubGoalQueue Q;

	// Initialize Q with the all the top level goals
	for (int i=0;i<goals->num_F;i++) {

		int g = goals->F[i];

		assert(last_fact_layer.find(g) != last_fact_layer.end());

		// Ignore goals that are certainly known in the current state
		if (RelaxedPlan::ignore_poss_del_in_rp && is_known_in_state(g, current)) {

#ifdef DEBUG_EXTRACT
			cout<<"Subgoal ignored (known in current state): "<<g<<endl<<endl;
#endif
			continue;
		}

		// Now the goal is either not in the state or in the state but not certainly known to be true

		SubGoal subgoal(g, GOAL_ACTION, n, false, &goal_step->s);
		Q.push(subgoal);

#ifdef DEBUG_UNSUPPORTED_KNOWN_PRECONDITIONS
		SubgoalSet.insert(make_pair(g, GOAL_ACTION));
#endif


#ifdef DEBUG_EXTRACT
		cout<<"New subgoal added: ";
		subgoal.print();
		cout<<endl<<endl;

		cout<<"Current Q'top: ";
		Q.top().print();
		cout<<endl<<endl;
#endif

	}

#ifdef DEBUG_EXTRACT
	cout<<"current_clauses_for_heuristics: "<<current_clauses_for_heursitcis<<endl<<endl;
	cout<<"current_robustness_for_heuristics: "<<current_robustness_for_heuristics<<endl<<endl;
#endif


#ifdef DEBUG_EXTRACT
	cout<<"Num unsupported known preconditions: "<<num_unsupported_known_preconditions<<endl<<endl;
	cout<<"In Q-loop"<<endl<<endl;
#endif

#ifdef DEBUG_UNSUPPORTED_KNOWN_PRECONDITIONS
	cout<<"Unsupported known precondition set: ";
	print_unsupported_known_precondition_set();
	cout<<endl<<endl;

	cout<<"Subgoal set: ";
	print_subgoal_set();
	cout<<endl<<endl;
#endif

	// Extracting actions in the relaxed planning graph to support subgoals in Q
	while (!Q.empty()) {

		// Retrieve the next subgoal
		SubGoal subgoal = Q.top();
		Q.pop();

#ifdef DEBUG_UNSUPPORTED_KNOWN_PRECONDITIONS
		SubgoalSet.erase(make_pair(subgoal.g, subgoal.op));
#endif


#ifdef DEBUG_EXTRACT
		TAB(2);	cout<<"Subgoal popped: "<<subgoal.g<<"..."<<endl<<endl;
		TAB(2); cout<<"State before subgoal:";
		const RP_STATE& sg = *subgoal.state_before_op;
		for (int ft=0;ft<gnum_ft_conn;ft++) {
			if (in_rp_state(ft, sg))
				cout<<"F"<<ft<<" ";
		}
		cout<<endl<<endl;
#endif

#ifndef NDEBUG
		// This action must be found in the relaxed plan
		RELAXED_PLAN::iterator rp_itr = rp.begin();
		for (; rp_itr != rp.end(); rp_itr++)
			if ((*rp_itr)->a == subgoal.op)
				break;
		assert(rp_itr != rp.end());
#endif

		// If this subgoal is not in its fact layer in the RPG, which also means
		// it is the possible precondition subgoal, we simply ignore this subgoal
		// (since there is not any best supporting action for it)
		if (!fact_present(subgoal.g, subgoal.l)) {
			assert(subgoal.possible_precondition);

#ifdef DEBUG_EXTRACT
			TAB(3);
			cout<<"Subgoal ignored (possible precondition not in fact layer "<<subgoal.l<<")."<<endl<<endl;
#endif

			// Continue with the next subgoal
			continue;
		}

		// Find the best action supporting this subgoal.
		// CURRENT STRATEGY: take the one found in the RPG construction
		int l = subgoal.l;
		while (l > 0 && P[l]->at(subgoal.g).best_supporting_action == NOOP) {
			l--;
		}

		if (l==0) {

			if (!fact_present(subgoal.g, l))
				assert(subgoal.possible_precondition);
			else {
				assert(P[l]->at(subgoal.g).best_supporting_action == e->get_actions()[e->get_actions().size()-1] ||
						P[l]->at(subgoal.g).best_supporting_action == INIT_ACTION);
			}

#ifdef DEBUG_EXTRACT
			TAB(3);
			cout<<"Subgoal ignored (possible precondition not in fact layer "<<l<<")."<<endl<<endl;
#endif


			// The best supporting action is the last action in the plan prefix, or the INIT_ACTION.
			// Continue with the next subgoal
			continue;

		}

		const int candidate_action = (P[l])->at(subgoal.g).best_supporting_action;
		int layer_of_candidate_action = l - 1;
		assert(candidate_action >= 0 && candidate_action < gnum_op_conn);

		// If this candidate action has been chosen, we don't add it again
		// We're done with this subgoal
		if (A[layer_of_candidate_action]->at(candidate_action).in_rp) {

#ifdef DEBUG_EXTRACT
			TAB(3);
			cout<<"Candidate action "<<candidate_action<<" ignored."<<endl<<endl;
#endif


			continue;	// Consider next subgoals
		}

		// Pointer to the new RP_STEP (if insertion happens)
		//boost::shared_ptr<RP_STEP> new_rp_step;
		RP_STEP *new_rp_step = 0;

		// Case 1: this subgoal is not present in the state before it
		// Whether it is known or possible precondition, inserting the supporting action always
		// helps increase *its* probability of being true!
		if (!in_rp_state(subgoal.g, *subgoal.state_before_op)) {

			new_rp_step = insert_action_into_relaxed_plan(candidate_action, layer_of_candidate_action);

		}

		// Case 2: this subgoal is present in the state before it
		else if (false) { // THE ONLY DIFFERENCE WITH LOCALLY_INCREMENTAL_ROBUSTNESS
			// Sub-case 1: if this subgoal is *known* at its rp-state, then adding the supporting action
			// does not increase the subgoal probability of being true
			// So for this sub-case, we simply do not insert this supporting action

			// Sub-case 2:  otherwise, inserting this supporting action will increase the probability of being
			// true of this subgoal. The amount of this increasing is: probability for this supporting action
			// to be executable \times the probability of this subgoal being its realized add effect
			RelaxedPlan::num_rp_robustness_increasing_checks++;

			if (!known_in_rp_state(subgoal.g, *subgoal.state_before_op)) {
				new_rp_step = insert_action_into_relaxed_plan(candidate_action, layer_of_candidate_action);
				RelaxedPlan::num_rp_robustness_increasing_check_success++;
			}
		} // end of Case 2

		// If the new action is inserted
		if (new_rp_step) {

			// Whenever we insert a new action into the relaxed plan, we re-compute its robustness
			// This will quickly return 0 if there exists unsupported known preconditions
			current_robustness = compute_robustness();

#ifdef DEBUG_EXTRACT
			TAB(3); cout<<"current_robustness: "<<current_robustness<<endl<<endl;
#endif

			// RELAXED PLAN FOUND!!!!
			if (RelaxedPlan::use_robustness_threshold && current_robustness > robustness_threshold) {

#ifdef DEBUG_EXTRACT
				cout<<"Out Q-loop "<<__LINE__<<endl<<endl;
#endif

				break;
			}

			//
			// Add new subgoals
			assert(gop_conn[candidate_action].num_E == 1);
			int ef = gop_conn[candidate_action].E[0];
			for (int i=0;i<gef_conn[ef].num_PC;i++) {
				int p = gef_conn[ef].PC[i];

				// This can be improved: we need only check if "p" is known at the rp_state
				// To do this, known facts at rp_states must be maintained
				if (RelaxedPlan::ignore_poss_del_in_rp && is_known_in_state(p, current)) {

#ifdef DEBUG_EXTRACT
					TAB(3); cout<<"Subgoal ignored (known in current state): "<<p<<endl<<endl;
#endif

					continue;
				}

				SubGoal subgoal(p, candidate_action, layer_of_candidate_action, false, &new_rp_step->s);
				Q.push(subgoal);

#ifdef DEBUG_UNSUPPORTED_KNOWN_PRECONDITIONS
				SubgoalSet.insert(make_pair(p, candidate_action));
#endif


#ifdef DEBUG_EXTRACT
				TAB(3); cout<<"New subgoal added: ";
				subgoal.print();
				cout<<endl<<endl;

				TAB(3); cout<<"Current Q'top: ";
				Q.top().print();
				cout<<endl<<endl;
#endif


			}
			for (int i=0;i<gef_conn[ef].num_poss_PC;i++) {
				int p = gef_conn[ef].poss_PC[i];

				if (RelaxedPlan::ignore_poss_del_in_rp && is_known_in_state(p, current))
					continue;
				SubGoal subgoal(p, candidate_action, layer_of_candidate_action, true, &new_rp_step->s);
				Q.push(subgoal);

#ifdef DEBUG_UNSUPPORTED_KNOWN_PRECONDITIONS
				SubgoalSet.insert(make_pair(p, candidate_action));
#endif


#ifdef DEBUG_EXTRACT
				TAB(3); cout<<"New subgoal added: ";
				subgoal.print();
				cout<<endl<<endl;

				TAB(3); cout<<"Current Q'top: ";
				Q.top().print();
				cout<<endl<<endl;
#endif
			}

#ifdef DEBUG_UNSUPPORTED_KNOWN_PRECONDITIONS
				cout<<"Unsupported known precondition set: ";
				print_unsupported_known_precondition_set();
				cout<<endl<<endl;

				cout<<"Subgoal set: ";
				print_subgoal_set();
				cout<<endl<<endl;

				// Check if all unsupported known preconditions present in the queue
				for (std::set<std::pair<int,int> >::const_iterator itr = unsupported_known_precondition_set.begin();
						itr != unsupported_known_precondition_set.end(); itr++) {
					assert(SubgoalSet.find(make_pair(itr->first, itr->second)) != SubgoalSet.end());
				}
#endif
		}

	} // out of Q-loop

#ifdef DEBUG_EXTRACT
	cout<<"Out Q-loop "<<__LINE__<<endl<<endl;
	cout<<"Num unsupported known preconditions: "<<num_unsupported_known_preconditions<<endl<<endl;
	cout<<"current_robustness_for_heuristics: "<<current_robustness_for_heuristics<<endl<<endl;
	cout<<"current_robustness: "<<compute_robustness()<<endl<<endl;
	cout<<"RP length: "<<rp.size()<<endl<<endl;
	cout<<"End extract_01..."<<__LINE__<<endl<<endl;
#endif

#ifdef DEBUG_UNSUPPORTED_KNOWN_PRECONDITIONS
	cout<<"Unsupported known precondition set: ";
	for (set<pair<int,int> >::const_iterator itr = unsupported_known_precondition_set.begin();
			itr != unsupported_known_precondition_set.end(); itr++) {
		cout<<"("<<itr->first<<", "<<itr->second<<") ";
	}
	cout<<endl<<endl;
#endif


#ifndef NDEBUG
	assert(!unsupported_known_precondition_exists());
#endif

	// This must be satisfied, since a trivial relaxed plan includes all "best actions", with which
	// all known preconditions are supported
	assert(num_unsupported_known_preconditions == 0);

	// This must be true, because when all known preconditions are supported, clause sets from RPG no longer need
	//assert(current_robustness_for_heuristics == current_robustness);

	result.first = rp.size() - 1;	// Ignore the goal step
	result.second = current_robustness;

	// Check if the relaxed plan returned has enough robustness
	if (current_robustness <= robustness_threshold) {
		return false;
	}

	return true;
}


bool RelaxedPlan::extract_rp_with_all_most_robust_supporting_actions(pair<int, double>& result) {

//#define DEBUG_EXTRACT
//
//#define DEBUG_UNSUPPORTED_KNOWN_PRECONDITIONS

#ifdef DEBUG_EXTRACT
	cout<<"Begin extract_01... Robustness threshold: "<<robustness_threshold<<endl<<endl;
#endif


	if (!build_relaxed_planning_graph()) {

#ifdef DEBUG_EXTRACT
		cout<<"End extract_01..."<<__LINE__<<endl<<endl;
#endif

		return false;
	}

	// The level at which all goals appear. So this is also the layer at which GOAL_ACTION is put
	int n = P.size() - 1;

	// The first and last fact layer
	const FactLayer& first_fact_layer = *(P[0]);
	const FactLayer& last_fact_layer = *(P[n]);

	// Initialize the number of actions chosen to the relaxed plan at each layer of the RPG
	num_chosen_actions.reserve(n+1);
	for (int i=0;i<=n;i++) num_chosen_actions[i] = 0;

	// Initialize the number of unsupported known preconditions
	num_unsupported_known_preconditions = 0;

	// Create the step containing goal action of the relaxed plan
	RP_STEP *goal_step = create_rp_step_for_goals();

	// Initialize the relaxed plan with the goal step, and increase the counter
	// REMINDER: release memory for "rp"
	assert(rp.size() == 0);
	rp.push_back(goal_step);
	num_chosen_actions[n] = 1;

	// The robustness of plan prefix + the current relaxed plan (0 if there exists an unsupported known preconditions in the RP)
	// Note that this can be either lower bound, upper bound or exact value of the robustness, depending on the flags
	// "-use_lower_bound_in_rp" and "-use_upper_bound_in_rp;"
	double current_robustness;

	// The lower bound, upper bound of the robustness of the plan prefix + the empty relaxed plan
	double lower_robustness, upper_robustness, exact_robustness;
	if (compute_robustness(lower_robustness, upper_robustness, exact_robustness)) {	// Succeed only if no unsupported known preconditions exist

		if (RelaxedPlan::use_lower_bound_in_rp) {
			current_robustness = lower_robustness;
		}
		else if (RelaxedPlan::use_upper_bound_in_rp) {
			current_robustness = upper_robustness;
		}
		else {
			current_robustness = exact_robustness;
		}

		// Check if the empty relaxed plan is enough
		if (RelaxedPlan::use_robustness_threshold && exact_robustness > robustness_threshold) {

			result.first = 0; // Don't count the goal step
			if (exact_robustness > robustness_threshold)
				result.second = exact_robustness;
			else
				result.second = current_robustness;

#ifdef DEBUG_EXTRACT
				cout<<"End extract_01..."<<__LINE__<<endl<<endl;
#endif

				return true;	// EMPTY RELAXED PLAN RETURNED!!!
		}
	}
	// Otherwise, there exists unsupported known goals in the empty RP. So the robustness is 0.
	else {
		current_robustness = 0;
	}

	// Clause set of the current plan prefix
	ClauseSet clauses_of_plan_prefix;
	e->get_clauses(clauses_of_plan_prefix);

	// The queue to store all subgoals
	SubGoalQueue Q;

	// Initialize Q with the all the top level goals
	for (int i=0;i<goals->num_F;i++) {

		int g = goals->F[i];

		assert(last_fact_layer.find(g) != last_fact_layer.end());

		// Ignore goals that are certainly known in the current state
		if (RelaxedPlan::ignore_poss_del_in_rp && is_known_in_state(g, current)) {

#ifdef DEBUG_EXTRACT
			cout<<"Subgoal ignored (known in current state): "<<g<<endl<<endl;
#endif
			continue;
		}

		// Now the goal is either not in the state or in the state but not certainly known to be true

		SubGoal subgoal(g, GOAL_ACTION, n, false, &goal_step->s);
		Q.push(subgoal);

#ifdef DEBUG_UNSUPPORTED_KNOWN_PRECONDITIONS
		SubgoalSet.insert(make_pair(g, GOAL_ACTION));
#endif


#ifdef DEBUG_EXTRACT
		cout<<"New subgoal added: ";
		subgoal.print();
		cout<<endl<<endl;

		cout<<"Current Q'top: ";
		Q.top().print();
		cout<<endl<<endl;
#endif

	}

#ifdef DEBUG_EXTRACT
	cout<<"current_clauses_for_heuristics: "<<current_clauses_for_heursitcis<<endl<<endl;
	cout<<"current_robustness_for_heuristics: "<<current_robustness_for_heuristics<<endl<<endl;
#endif


#ifdef DEBUG_EXTRACT
	cout<<"Num unsupported known preconditions: "<<num_unsupported_known_preconditions<<endl<<endl;
	cout<<"In Q-loop"<<endl<<endl;
#endif

#ifdef DEBUG_UNSUPPORTED_KNOWN_PRECONDITIONS
	cout<<"Unsupported known precondition set: ";
	print_unsupported_known_precondition_set();
	cout<<endl<<endl;

	cout<<"Subgoal set: ";
	print_subgoal_set();
	cout<<endl<<endl;
#endif

	// Extracting actions in the relaxed planning graph to support subgoals in Q
	while (!Q.empty()) {

		// Retrieve the next subgoal
		SubGoal subgoal = Q.top();
		Q.pop();

#ifdef DEBUG_UNSUPPORTED_KNOWN_PRECONDITIONS
		SubgoalSet.erase(make_pair(subgoal.g, subgoal.op));
#endif


#ifdef DEBUG_EXTRACT
		TAB(2);	cout<<"Subgoal popped: "<<subgoal.g<<"..."<<endl<<endl;
		TAB(2); cout<<"State before subgoal:";
		const RP_STATE& sg = *subgoal.state_before_op;
		for (int ft=0;ft<gnum_ft_conn;ft++) {
			if (in_rp_state(ft, sg))
				cout<<"F"<<ft<<" ";
		}
		cout<<endl<<endl;
#endif

#ifndef NDEBUG
		// This action must be found in the relaxed plan
		RELAXED_PLAN::iterator rp_itr = rp.begin();
		for (; rp_itr != rp.end(); rp_itr++)
			if ((*rp_itr)->a == subgoal.op)
				break;
		assert(rp_itr != rp.end());
#endif

		// If this subgoal is not in its fact layer in the RPG, which also means
		// it is the possible precondition subgoal, we simply ignore this subgoal
		// (since there is not any best supporting action for it)
		if (!fact_present(subgoal.g, subgoal.l)) {
			assert(subgoal.possible_precondition);

#ifdef DEBUG_EXTRACT
			TAB(3);
			cout<<"Subgoal ignored (possible precondition not in fact layer "<<subgoal.l<<")."<<endl<<endl;
#endif

			// Continue with the next subgoal
			continue;
		}

		// Find the best action supporting this subgoal.
		// CURRENT STRATEGY: take the one found in the RPG construction
		int l = subgoal.l;
		while (l > 0 && P[l]->at(subgoal.g).best_supporting_action == NOOP) {
			l--;
		}

		if (l==0) {

			if (!fact_present(subgoal.g, l))
				assert(subgoal.possible_precondition);
			else {
				assert(P[l]->at(subgoal.g).best_supporting_action == e->get_actions()[e->get_actions().size()-1] ||
						P[l]->at(subgoal.g).best_supporting_action == INIT_ACTION);
			}

#ifdef DEBUG_EXTRACT
			TAB(3);
			cout<<"Subgoal ignored (possible precondition not in fact layer "<<l<<")."<<endl<<endl;
#endif


			// The best supporting action is the last action in the plan prefix, or the INIT_ACTION.
			// Continue with the next subgoal
			continue;

		}

		const int candidate_action = (P[l])->at(subgoal.g).best_supporting_action;
		int layer_of_candidate_action = l - 1;
		assert(candidate_action >= 0 && candidate_action < gnum_op_conn);

		// If this candidate action has been chosen, we don't add it again
		// We're done with this subgoal
		if (A[layer_of_candidate_action]->at(candidate_action).in_rp) {

#ifdef DEBUG_EXTRACT
			TAB(3);
			cout<<"Candidate action "<<candidate_action<<" ignored."<<endl<<endl;
#endif


			continue;	// Consider next subgoals
		}

		// Always insert the candidate action
		RP_STEP *new_rp_step = insert_action_into_relaxed_plan(candidate_action, layer_of_candidate_action);;

		// If the new action is inserted
		if (new_rp_step) {

			// Whenever we insert a new action into the relaxed plan, we re-compute its robustness
			// This will quickly return 0 if there exists unsupported known preconditions
			current_robustness = compute_robustness();

#ifdef DEBUG_EXTRACT
			TAB(3); cout<<"current_robustness: "<<current_robustness<<endl<<endl;
#endif

			// RELAXED PLAN FOUND!!!!
			if (RelaxedPlan::use_robustness_threshold && current_robustness > robustness_threshold) {

#ifdef DEBUG_EXTRACT
				cout<<"Out Q-loop "<<__LINE__<<endl<<endl;
#endif

				break;
			}

			//
			// Add new subgoals
			assert(gop_conn[candidate_action].num_E == 1);
			int ef = gop_conn[candidate_action].E[0];
			for (int i=0;i<gef_conn[ef].num_PC;i++) {
				int p = gef_conn[ef].PC[i];

				// This can be improved: we need only check if "p" is known at the rp_state
				// To do this, known facts at rp_states must be maintained
				if (RelaxedPlan::ignore_poss_del_in_rp && is_known_in_state(p, current)) {

#ifdef DEBUG_EXTRACT
					TAB(3); cout<<"Subgoal ignored (known in current state): "<<p<<endl<<endl;
#endif

					continue;
				}

				SubGoal subgoal(p, candidate_action, layer_of_candidate_action, false, &new_rp_step->s);
				Q.push(subgoal);

#ifdef DEBUG_UNSUPPORTED_KNOWN_PRECONDITIONS
				SubgoalSet.insert(make_pair(p, candidate_action));
#endif


#ifdef DEBUG_EXTRACT
				TAB(3); cout<<"New subgoal added: ";
				subgoal.print();
				cout<<endl<<endl;

				TAB(3); cout<<"Current Q'top: ";
				Q.top().print();
				cout<<endl<<endl;
#endif

			}

			for (int i=0;i<gef_conn[ef].num_poss_PC;i++) {
				int p = gef_conn[ef].poss_PC[i];

				if (RelaxedPlan::ignore_poss_del_in_rp && is_known_in_state(p, current))
					continue;
				SubGoal subgoal(p, candidate_action, layer_of_candidate_action, true, &new_rp_step->s);
				Q.push(subgoal);

#ifdef DEBUG_UNSUPPORTED_KNOWN_PRECONDITIONS
				SubgoalSet.insert(make_pair(p, candidate_action));
#endif


#ifdef DEBUG_EXTRACT
				TAB(3); cout<<"New subgoal added: ";
				subgoal.print();
				cout<<endl<<endl;

				TAB(3); cout<<"Current Q'top: ";
				Q.top().print();
				cout<<endl<<endl;
#endif
			}

#ifdef DEBUG_UNSUPPORTED_KNOWN_PRECONDITIONS
				cout<<"Unsupported known precondition set: ";
				print_unsupported_known_precondition_set();
				cout<<endl<<endl;

				cout<<"Subgoal set: ";
				print_subgoal_set();
				cout<<endl<<endl;

				// Check if all unsupported known preconditions present in the queue
				for (std::set<std::pair<int,int> >::const_iterator itr = unsupported_known_precondition_set.begin();
						itr != unsupported_known_precondition_set.end(); itr++) {
					assert(SubgoalSet.find(make_pair(itr->first, itr->second)) != SubgoalSet.end());
				}
#endif
		}

	} // out of Q-loop

#ifdef DEBUG_EXTRACT
	cout<<"Out Q-loop "<<__LINE__<<endl<<endl;
	cout<<"Num unsupported known preconditions: "<<num_unsupported_known_preconditions<<endl<<endl;
	cout<<"current_robustness_for_heuristics: "<<current_robustness_for_heuristics<<endl<<endl;
	cout<<"current_robustness: "<<compute_robustness()<<endl<<endl;
	cout<<"RP length: "<<rp.size()<<endl<<endl;
	cout<<"End extract_01..."<<__LINE__<<endl<<endl;
#endif

#ifdef DEBUG_UNSUPPORTED_KNOWN_PRECONDITIONS
	cout<<"Unsupported known precondition set: ";
	for (set<pair<int,int> >::const_iterator itr = unsupported_known_precondition_set.begin();
			itr != unsupported_known_precondition_set.end(); itr++) {
		cout<<"("<<itr->first<<", "<<itr->second<<") ";
	}
	cout<<endl<<endl;
#endif


#ifndef NDEBUG
	assert(!unsupported_known_precondition_exists());
#endif

	// Compute the robustness after the relaxed plan is built
	current_robustness = compute_robustness();

	// This must be satisfied, since a trivial relaxed plan includes all "best actions", with which
	// all known preconditions are supported
	assert(num_unsupported_known_preconditions == 0);

	// This must be true, because when all known preconditions are supported, clause sets from RPG no longer need
	//assert(current_robustness_for_heuristics == current_robustness);

	result.first = rp.size() - 1;	// Ignore the goal step
	result.second = current_robustness;

	// Check if the relaxed plan returned has enough robustness
	if (current_robustness <= robustness_threshold) {
		return false;
	}

	return true;
}

bool RelaxedPlan::extract_pure_ff_heuristic(pair<int, double>& result) {

//#define DEBUG_EXTRACT
//
//#define DEBUG_UNSUPPORTED_KNOWN_PRECONDITIONS

#ifdef DEBUG_EXTRACT
	cout<<"Begin extract_01... Robustness threshold: "<<robustness_threshold<<endl<<endl;
#endif


	if (!build_ff_relaxed_planning_graph()) {

#ifdef DEBUG_EXTRACT
		cout<<"End extract_01..."<<__LINE__<<endl<<endl;
#endif

		return false;
	}

	// The level at which all goals appear. So this is also the layer at which GOAL_ACTION is put
	int n = P.size() - 1;

	// The first and last fact layer
	const FactLayer& first_fact_layer = *(P[0]);
	const FactLayer& last_fact_layer = *(P[n]);

	// Initialize the number of actions chosen to the relaxed plan at each layer of the RPG
	num_chosen_actions.reserve(n+1);
	for (int i=0;i<=n;i++) num_chosen_actions[i] = 0;

	// Initialize the number of unsupported known preconditions
	num_unsupported_known_preconditions = 0;

	// Create the step containing goal action of the relaxed plan
	RP_STEP *goal_step = create_rp_step_for_goals();

	// Initialize the relaxed plan with the goal step, and increase the counter
	// REMINDER: release memory for "rp"
	assert(rp.size() == 0);
	rp.push_back(goal_step);
	num_chosen_actions[n] = 1;

	// The robustness of plan prefix + the current relaxed plan (0 if there exists an unsupported known preconditions in the RP)
	// Note that this can be either lower bound, upper bound or exact value of the robustness, depending on the flags
	// "-use_lower_bound_in_rp" and "-use_upper_bound_in_rp;"
	double current_robustness;

	// The lower bound, upper bound of the robustness of the plan prefix + the empty relaxed plan
	double lower_robustness, upper_robustness, exact_robustness;
	if (compute_robustness(lower_robustness, upper_robustness, exact_robustness)) {	// Succeed only if no unsupported known preconditions exist

		if (RelaxedPlan::use_lower_bound_in_rp) {
			current_robustness = lower_robustness;
		}
		else if (RelaxedPlan::use_upper_bound_in_rp) {
			current_robustness = upper_robustness;
		}
		else {
			current_robustness = exact_robustness;
		}

		// Check if the empty relaxed plan is enough
		if (RelaxedPlan::use_robustness_threshold && exact_robustness > robustness_threshold) {

			result.first = 0; // Don't count the goal step
			if (exact_robustness > robustness_threshold)
				result.second = exact_robustness;
			else
				result.second = current_robustness;

#ifdef DEBUG_EXTRACT
				cout<<"End extract_01..."<<__LINE__<<endl<<endl;
#endif

				return true;	// EMPTY RELAXED PLAN RETURNED!!!
		}
	}
	// Otherwise, there exists unsupported known goals in the empty RP. So the robustness is 0.
	else {
		current_robustness = 0;
	}

	// Clause set of the current plan prefix
	ClauseSet clauses_of_plan_prefix;
	e->get_clauses(clauses_of_plan_prefix);

	// The queue to store all subgoals
	SubGoalQueue Q;

	// Initialize Q with the all the top level goals
	for (int i=0;i<goals->num_F;i++) {

		int g = goals->F[i];

		assert(last_fact_layer.find(g) != last_fact_layer.end());

		// Ignore goals that are possibly or certainly known in the current state
		// Note: for non-FF like heuristic, we check "certainly known" only
		if (RelaxedPlan::ignore_poss_del_in_rp && is_in_state(g, current)) {

#ifdef DEBUG_EXTRACT
			cout<<"Subgoal ignored (in current state): "<<g<<endl<<endl;
#endif
			continue;
		}

		// Now the goal is either not in the state or in the state but not certainly known to be true

		SubGoal subgoal(g, GOAL_ACTION, n, false, &goal_step->s);
		Q.push(subgoal);

#ifdef DEBUG_UNSUPPORTED_KNOWN_PRECONDITIONS
		SubgoalSet.insert(make_pair(g, GOAL_ACTION));
#endif


#ifdef DEBUG_EXTRACT
		cout<<"New subgoal added: ";
		subgoal.print();
		cout<<endl<<endl;

		cout<<"Current Q'top: ";
		Q.top().print();
		cout<<endl<<endl;
#endif

	}

#ifdef DEBUG_EXTRACT
	cout<<"current_clauses_for_heuristics: "<<current_clauses_for_heursitcis<<endl<<endl;
	cout<<"current_robustness_for_heuristics: "<<current_robustness_for_heuristics<<endl<<endl;
#endif


#ifdef DEBUG_EXTRACT
	cout<<"Num unsupported known preconditions: "<<num_unsupported_known_preconditions<<endl<<endl;
	cout<<"In Q-loop"<<endl<<endl;
#endif

#ifdef DEBUG_UNSUPPORTED_KNOWN_PRECONDITIONS
	cout<<"Unsupported known precondition set: ";
	print_unsupported_known_precondition_set();
	cout<<endl<<endl;

	cout<<"Subgoal set: ";
	print_subgoal_set();
	cout<<endl<<endl;
#endif

	// Extracting actions in the relaxed planning graph to support subgoals in Q
	while (!Q.empty()) {

		// Retrieve the next subgoal
		SubGoal subgoal = Q.top();
		Q.pop();

#ifdef DEBUG_UNSUPPORTED_KNOWN_PRECONDITIONS
		SubgoalSet.erase(make_pair(subgoal.g, subgoal.op));
#endif


#ifdef DEBUG_EXTRACT
		TAB(2);	cout<<"Subgoal popped: "<<subgoal.g<<"..."<<endl<<endl;
		TAB(2); cout<<"State before subgoal:";
		const RP_STATE& sg = *subgoal.state_before_op;
		for (int ft=0;ft<gnum_ft_conn;ft++) {
			if (in_rp_state(ft, sg))
				cout<<"F"<<ft<<" ";
		}
		cout<<endl<<endl;
#endif

#ifndef NDEBUG
		// This action must be found in the relaxed plan
		RELAXED_PLAN::iterator rp_itr = rp.begin();
		for (; rp_itr != rp.end(); rp_itr++)
			if ((*rp_itr)->a == subgoal.op)
				break;
		assert(rp_itr != rp.end());
#endif

		// If this subgoal is not in its fact layer in the RPG, which also means
		// it is the possible precondition subgoal, we simply ignore this subgoal
		// (since there is not any best supporting action for it)
		if (!fact_present(subgoal.g, subgoal.l)) {
			assert(subgoal.possible_precondition);

#ifdef DEBUG_EXTRACT
			TAB(3);
			cout<<"Subgoal ignored (possible precondition not in fact layer "<<subgoal.l<<")."<<endl<<endl;
#endif

			// Continue with the next subgoal
			continue;
		}

		// If this subgoal presents in the corresponding rp-state, it is considered "supported"
		// as in classical planning
		RELAXED_PLAN::iterator rp_itr_before = rp.begin();
		for (; rp_itr_before != rp.end(); rp_itr_before++)
			if ((*rp_itr_before)->a == subgoal.op)
				break;
		assert(rp_itr_before != rp.end());
		if (rp_itr_before == rp.begin()) {
			if (is_in_state(subgoal.g, current))
				continue;
		}
		else {
			rp_itr_before--;
			const RP_STATE& prev_rp_state = (*rp_itr_before)->s;
			if (in_rp_state(subgoal.g, prev_rp_state))
				continue;
		}

		// Find the best action supporting this subgoal.
		// FF-like strategy: among all actions at the first layer of this subgoal that (possibly) add it,
		// we choose the one with the least difficulty level.
		// The difficulty of an action is the summation of the first layers of its preconditions and possible preconditions
		int l = subgoal.l;
		int first_layer = P[l]->at(subgoal.g).first_layer;
		if (first_layer == 0) // The best supporting action is the last action in the plan prefix, or the INIT_ACTION.
			continue;

		assert(!fact_present(subgoal.g, first_layer-1));

		int candidate_action = -1;
		int layer_of_candidate_action = first_layer - 1;
		int min_difficulty = 10000;

		// Check actions having subgoal as a known and possible add effect (the former is preferred)
		for (int i = 0; i < gft_conn[subgoal.g].num_A; i++) {
			int ef = gft_conn[subgoal.g].A[i];	// action having "subgoal" as an add effect
			int op = gef_conn[ef].op;
			if (!action_present(op, layer_of_candidate_action)) continue;

			// Compute the difficulty of this action
			int difficulty=0;
			for (int j=0;j<gef_conn[ef].num_PC;j++) {
				int p = gef_conn[ef].PC[j];	// known precondition
				assert(fact_present(p, layer_of_candidate_action));
				difficulty += P[layer_of_candidate_action]->at(p).first_layer;
			}
			for (int j=0;j<gef_conn[ef].num_poss_PC;j++) {
				int p = gef_conn[ef].poss_PC[j];	// possible precondition
				if (fact_present(p, layer_of_candidate_action))	// possible precondition of an action may not be present at a fact layer
					difficulty += P[layer_of_candidate_action]->at(p).first_layer;
			}
			// Choose the action with minimum difficulty
			if (min_difficulty > difficulty) {
				min_difficulty = difficulty;
				candidate_action = op;
			}
		}

		for (int i = 0; i < gft_conn[subgoal.g].num_poss_A; i++) {
			int ef = gft_conn[subgoal.g].poss_A[i];	// action having "subgoal" as an add effect
			int op = gef_conn[ef].op;
			if (!action_present(op, layer_of_candidate_action)) continue;

			// Compute the difficulty of this action
			int difficulty=0;
			for (int j=0;j<gef_conn[ef].num_PC;j++) {
				int p = gef_conn[ef].PC[j];	// known precondition
				assert(fact_present(p, layer_of_candidate_action));
				difficulty += P[layer_of_candidate_action]->at(p).first_layer;
			}
			for (int j=0;j<gef_conn[ef].num_poss_PC;j++) {
				int p = gef_conn[ef].poss_PC[j];	// possible precondition
				if (fact_present(p, layer_of_candidate_action))
					difficulty += P[layer_of_candidate_action]->at(p).first_layer;
			}
			// Choose the action with minimum difficulty
			if (min_difficulty > difficulty) {
				min_difficulty = difficulty;
				candidate_action = op;
			}
		}
		assert(candidate_action >= 0 && candidate_action < gnum_op_conn);

		// If this candidate action has been chosen, we don't add it again
		// We're done with this subgoal
		if (A[layer_of_candidate_action]->at(candidate_action).in_rp) {

#ifdef DEBUG_EXTRACT
			TAB(3);
			cout<<"Candidate action "<<candidate_action<<" ignored."<<endl<<endl;
#endif


			continue;	// Consider next subgoals
		}

		// Always insert the candidate action
		RP_STEP *new_rp_step = insert_action_into_relaxed_plan(candidate_action, layer_of_candidate_action);;

		// If the new action is inserted
		if (new_rp_step) {

#ifdef DEBUG_EXTRACT
			TAB(3); cout<<"current_robustness: "<<current_robustness<<endl<<endl;
#endif

			//
			// Add new subgoals
			assert(gop_conn[candidate_action].num_E == 1);
			int ef = gop_conn[candidate_action].E[0];
			for (int i=0;i<gef_conn[ef].num_PC;i++) {
				int p = gef_conn[ef].PC[i];

				// This can be improved: we need only check if "p" is known at the rp_state
				// To do this, known facts at rp_states must be maintained
				if (RelaxedPlan::ignore_poss_del_in_rp && is_known_in_state(p, current)) {

#ifdef DEBUG_EXTRACT
					TAB(3); cout<<"Subgoal ignored (known in current state): "<<p<<endl<<endl;
#endif

					continue;
				}

				SubGoal subgoal(p, candidate_action, layer_of_candidate_action, false, &new_rp_step->s);
				Q.push(subgoal);

#ifdef DEBUG_UNSUPPORTED_KNOWN_PRECONDITIONS
				SubgoalSet.insert(make_pair(p, candidate_action));
#endif


#ifdef DEBUG_EXTRACT
				TAB(3); cout<<"New subgoal added: ";
				subgoal.print();
				cout<<endl<<endl;

				TAB(3); cout<<"Current Q'top: ";
				Q.top().print();
				cout<<endl<<endl;
#endif

			}

			for (int i=0;i<gef_conn[ef].num_poss_PC;i++) {
				int p = gef_conn[ef].poss_PC[i];

				if (RelaxedPlan::ignore_poss_del_in_rp && is_known_in_state(p, current))
					continue;
				SubGoal subgoal(p, candidate_action, layer_of_candidate_action, true, &new_rp_step->s);
				Q.push(subgoal);

#ifdef DEBUG_UNSUPPORTED_KNOWN_PRECONDITIONS
				SubgoalSet.insert(make_pair(p, candidate_action));
#endif


#ifdef DEBUG_EXTRACT
				TAB(3); cout<<"New subgoal added: ";
				subgoal.print();
				cout<<endl<<endl;

				TAB(3); cout<<"Current Q'top: ";
				Q.top().print();
				cout<<endl<<endl;
#endif
			}

#ifdef DEBUG_UNSUPPORTED_KNOWN_PRECONDITIONS
				cout<<"Unsupported known precondition set: ";
				print_unsupported_known_precondition_set();
				cout<<endl<<endl;

				cout<<"Subgoal set: ";
				print_subgoal_set();
				cout<<endl<<endl;

				// Check if all unsupported known preconditions present in the queue
				for (std::set<std::pair<int,int> >::const_iterator itr = unsupported_known_precondition_set.begin();
						itr != unsupported_known_precondition_set.end(); itr++) {
					assert(SubgoalSet.find(make_pair(itr->first, itr->second)) != SubgoalSet.end());
				}
#endif
		}

	} // out of Q-loop

#ifdef DEBUG_EXTRACT
	cout<<"Out Q-loop "<<__LINE__<<endl<<endl;
	cout<<"Num unsupported known preconditions: "<<num_unsupported_known_preconditions<<endl<<endl;
	cout<<"current_robustness_for_heuristics: "<<current_robustness_for_heuristics<<endl<<endl;
	cout<<"current_robustness: "<<compute_robustness()<<endl<<endl;
	cout<<"RP length: "<<rp.size()<<endl<<endl;
	cout<<"End extract_01..."<<__LINE__<<endl<<endl;
#endif

#ifdef DEBUG_UNSUPPORTED_KNOWN_PRECONDITIONS
	cout<<"Unsupported known precondition set: ";
	for (set<pair<int,int> >::const_iterator itr = unsupported_known_precondition_set.begin();
			itr != unsupported_known_precondition_set.end(); itr++) {
		cout<<"("<<itr->first<<", "<<itr->second<<") ";
	}
	cout<<endl<<endl;
#endif


#ifndef NDEBUG
	assert(!unsupported_known_precondition_exists());
#endif

	// Compute the robustness after the relaxed plan is built
	current_robustness = compute_robustness();

	// This must be satisfied, since a trivial relaxed plan includes all "best actions", with which
	// all known preconditions are supported
	assert(num_unsupported_known_preconditions == 0);

	// This must be true, because when all known preconditions are supported, clause sets from RPG no longer need
	//assert(current_robustness_for_heuristics == current_robustness);

	result.first = rp.size() - 1;	// Ignore the goal step
	result.second = current_robustness;

	// Check if the relaxed plan returned has enough robustness
	if (current_robustness <= robustness_threshold) {
		return false;
	}

	return true;
}

bool RelaxedPlan::extract_robust_ff_heuristic(pair<int, double>& result) {

//#define DEBUG_EXTRACT
//
//#define DEBUG_UNSUPPORTED_KNOWN_PRECONDITIONS

#ifdef DEBUG_EXTRACT
	cout<<"Begin extract_01... Robustness threshold: "<<robustness_threshold<<endl<<endl;
#endif


	if (!build_relaxed_planning_graph()) {

#ifdef DEBUG_EXTRACT
		cout<<"End extract_01..."<<__LINE__<<endl<<endl;
#endif

		return false;
	}

	// The level at which all goals appear. So this is also the layer at which GOAL_ACTION is put
	int n = P.size() - 1;

	// The first and last fact layer
	const FactLayer& first_fact_layer = *(P[0]);
	const FactLayer& last_fact_layer = *(P[n]);

	// Initialize the number of actions chosen to the relaxed plan at each layer of the RPG
	num_chosen_actions.reserve(n+1);
	for (int i=0;i<=n;i++) num_chosen_actions[i] = 0;

	// Initialize the number of unsupported known preconditions
	num_unsupported_known_preconditions = 0;

	// Create the step containing goal action of the relaxed plan
	RP_STEP *goal_step = create_rp_step_for_goals();

	// Initialize the relaxed plan with the goal step, and increase the counter
	// REMINDER: release memory for "rp"
	assert(rp.size() == 0);
	rp.push_back(goal_step);
	num_chosen_actions[n] = 1;

	// The robustness of plan prefix + the current relaxed plan (0 if there exists an unsupported known preconditions in the RP)
	// Note that this can be either lower bound, upper bound or exact value of the robustness, depending on the flags
	// "-use_lower_bound_in_rp" and "-use_upper_bound_in_rp;"
	double current_robustness;

	// The lower bound, upper bound of the robustness of the plan prefix + the empty relaxed plan
	double lower_robustness, upper_robustness, exact_robustness;
	if (compute_robustness(lower_robustness, upper_robustness, exact_robustness)) {	// Succeed only if no unsupported known preconditions exist

		if (RelaxedPlan::use_lower_bound_in_rp) {
			current_robustness = lower_robustness;
		}
		else if (RelaxedPlan::use_upper_bound_in_rp) {
			current_robustness = upper_robustness;
		}
		else {
			current_robustness = exact_robustness;
		}

		// Check if the empty relaxed plan is enough
		if (RelaxedPlan::use_robustness_threshold && exact_robustness > robustness_threshold) {

			result.first = 0; // Don't count the goal step
			if (exact_robustness > robustness_threshold)
				result.second = exact_robustness;
			else
				result.second = current_robustness;

#ifdef DEBUG_EXTRACT
				cout<<"End extract_01..."<<__LINE__<<endl<<endl;
#endif

				return true;	// EMPTY RELAXED PLAN RETURNED!!!
		}
	}
	// Otherwise, there exists unsupported known goals in the empty RP. So the robustness is 0.
	else {
		current_robustness = 0;
	}

	// Clause set of the current plan prefix
	ClauseSet clauses_of_plan_prefix;
	e->get_clauses(clauses_of_plan_prefix);

	// The queue to store all subgoals
	SubGoalQueue Q;

	// Initialize Q with the all the top level goals
	for (int i=0;i<goals->num_F;i++) {

		int g = goals->F[i];

		assert(last_fact_layer.find(g) != last_fact_layer.end());

		// Ignore goals that are possibly or certainly known in the current state
		// Note: for non-FF like heuristic, we check "certainly known" only
		if (RelaxedPlan::ignore_poss_del_in_rp && is_in_state(g, current)) {

#ifdef DEBUG_EXTRACT
			cout<<"Subgoal ignored (in current state): "<<g<<endl<<endl;
#endif
			continue;
		}

		// Now the goal is either not in the state or in the state but not certainly known to be true

		SubGoal subgoal(g, GOAL_ACTION, n, false, &goal_step->s);
		Q.push(subgoal);

#ifdef DEBUG_UNSUPPORTED_KNOWN_PRECONDITIONS
		SubgoalSet.insert(make_pair(g, GOAL_ACTION));
#endif


#ifdef DEBUG_EXTRACT
		cout<<"New subgoal added: ";
		subgoal.print();
		cout<<endl<<endl;

		cout<<"Current Q'top: ";
		Q.top().print();
		cout<<endl<<endl;
#endif

	}

#ifdef DEBUG_EXTRACT
	cout<<"current_clauses_for_heuristics: "<<current_clauses_for_heursitcis<<endl<<endl;
	cout<<"current_robustness_for_heuristics: "<<current_robustness_for_heuristics<<endl<<endl;
#endif


#ifdef DEBUG_EXTRACT
	cout<<"Num unsupported known preconditions: "<<num_unsupported_known_preconditions<<endl<<endl;
	cout<<"In Q-loop"<<endl<<endl;
#endif

#ifdef DEBUG_UNSUPPORTED_KNOWN_PRECONDITIONS
	cout<<"Unsupported known precondition set: ";
	print_unsupported_known_precondition_set();
	cout<<endl<<endl;

	cout<<"Subgoal set: ";
	print_subgoal_set();
	cout<<endl<<endl;
#endif

	// Extracting actions in the relaxed planning graph to support subgoals in Q
	while (!Q.empty()) {

		// Retrieve the next subgoal
		SubGoal subgoal = Q.top();
		Q.pop();

#ifdef DEBUG_UNSUPPORTED_KNOWN_PRECONDITIONS
		SubgoalSet.erase(make_pair(subgoal.g, subgoal.op));
#endif


#ifdef DEBUG_EXTRACT
		TAB(2);	cout<<"Subgoal popped: "<<subgoal.g<<"..."<<endl<<endl;
		TAB(2); cout<<"State before subgoal:";
		const RP_STATE& sg = *subgoal.state_before_op;
		for (int ft=0;ft<gnum_ft_conn;ft++) {
			if (in_rp_state(ft, sg))
				cout<<"F"<<ft<<" ";
		}
		cout<<endl<<endl;
#endif

#ifndef NDEBUG
		// This action must be found in the relaxed plan
		RELAXED_PLAN::iterator rp_itr = rp.begin();
		for (; rp_itr != rp.end(); rp_itr++)
			if ((*rp_itr)->a == subgoal.op)
				break;
		assert(rp_itr != rp.end());
#endif

		// If this subgoal is not in its fact layer in the RPG, which also means
		// it is the possible precondition subgoal, we simply ignore this subgoal
		// (since there is not any best supporting action for it)
		if (!fact_present(subgoal.g, subgoal.l)) {
			assert(subgoal.possible_precondition);

#ifdef DEBUG_EXTRACT
			TAB(3);
			cout<<"Subgoal ignored (possible precondition not in fact layer "<<subgoal.l<<")."<<endl<<endl;
#endif

			// Continue with the next subgoal
			continue;
		}

		// If this subgoal presents in the corresponding rp-state, it is considered "supported"
		// as in classical planning
		RELAXED_PLAN::iterator rp_itr_before = rp.begin();
		for (; rp_itr_before != rp.end(); rp_itr_before++)
			if ((*rp_itr_before)->a == subgoal.op)
				break;
		assert(rp_itr_before != rp.end());
		if (rp_itr_before == rp.begin()) {
			if (is_in_state(subgoal.g, current))
				continue;
		}
		else {
			rp_itr_before--;
			const RP_STATE& prev_rp_state = (*rp_itr_before)->s;
			if (in_rp_state(subgoal.g, prev_rp_state))
				continue;
		}

		// Find the best action supporting this subgoal.
		// FF-like strategy: among all actions at the first layer of this subgoal that (possibly) add it,
		// we choose the one with the least difficulty level.
		// The difficulty of an action is the summation of the first layers of its preconditions and possible preconditions
		int l = subgoal.l;
		int first_layer = P[l]->at(subgoal.g).first_layer;
		if (first_layer == 0) // The best supporting action is the last action in the plan prefix, or the INIT_ACTION.
			continue;

		assert(!fact_present(subgoal.g, first_layer-1));

		int candidate_action = -1;
		int layer_of_candidate_action = first_layer - 1;
		ActionLayer& action_layer = *(A[layer_of_candidate_action]);
		int best_action_robustness = 0;

		// Check actions having subgoal as a known and possible add effect (the former is preferred)
		for (int i = 0; i < gft_conn[subgoal.g].num_A; i++) {
			int ef = gft_conn[subgoal.g].A[i];	// action having "subgoal" as an add effect
			int op = gef_conn[ef].op;
			if (!action_present(op, layer_of_candidate_action)) continue;
			double action_robustness = action_layer[op].robustness;
			// Choose the action with the highest robustness
			if (best_action_robustness < action_robustness) {
				best_action_robustness = action_robustness;
				candidate_action = op;
			}
		}

		for (int i = 0; i < gft_conn[subgoal.g].num_poss_A; i++) {
			int ef = gft_conn[subgoal.g].poss_A[i];	// action having "subgoal" as an add effect
			int op = gef_conn[ef].op;
			if (!action_present(op, layer_of_candidate_action)) continue;
			double action_robustness = action_layer[op].robustness;
			// Choose the action with the highest robustness
			if (best_action_robustness < action_robustness) {
				best_action_robustness = action_robustness;
				candidate_action = op;
			}
		}
		assert(candidate_action >= 0 && candidate_action < gnum_op_conn);

		// If this candidate action has been chosen, we don't add it again
		// We're done with this subgoal
		if (A[layer_of_candidate_action]->at(candidate_action).in_rp) {

#ifdef DEBUG_EXTRACT
			TAB(3);
			cout<<"Candidate action "<<candidate_action<<" ignored."<<endl<<endl;
#endif


			continue;	// Consider next subgoals
		}

		// Always insert the candidate action
		RP_STEP *new_rp_step = insert_action_into_relaxed_plan(candidate_action, layer_of_candidate_action);;

		// If the new action is inserted
		if (new_rp_step) {

#ifdef DEBUG_EXTRACT
			TAB(3); cout<<"current_robustness: "<<current_robustness<<endl<<endl;
#endif

			//
			// Add new subgoals
			assert(gop_conn[candidate_action].num_E == 1);
			int ef = gop_conn[candidate_action].E[0];
			for (int i=0;i<gef_conn[ef].num_PC;i++) {
				int p = gef_conn[ef].PC[i];

				// This can be improved: we need only check if "p" is known at the rp_state
				// To do this, known facts at rp_states must be maintained
				if (RelaxedPlan::ignore_poss_del_in_rp && is_known_in_state(p, current)) {

#ifdef DEBUG_EXTRACT
					TAB(3); cout<<"Subgoal ignored (known in current state): "<<p<<endl<<endl;
#endif

					continue;
				}

				SubGoal subgoal(p, candidate_action, layer_of_candidate_action, false, &new_rp_step->s);
				Q.push(subgoal);

#ifdef DEBUG_UNSUPPORTED_KNOWN_PRECONDITIONS
				SubgoalSet.insert(make_pair(p, candidate_action));
#endif


#ifdef DEBUG_EXTRACT
				TAB(3); cout<<"New subgoal added: ";
				subgoal.print();
				cout<<endl<<endl;

				TAB(3); cout<<"Current Q'top: ";
				Q.top().print();
				cout<<endl<<endl;
#endif

			}

			for (int i=0;i<gef_conn[ef].num_poss_PC;i++) {
				int p = gef_conn[ef].poss_PC[i];

				if (RelaxedPlan::ignore_poss_del_in_rp && is_known_in_state(p, current))
					continue;
				SubGoal subgoal(p, candidate_action, layer_of_candidate_action, true, &new_rp_step->s);
				Q.push(subgoal);

#ifdef DEBUG_UNSUPPORTED_KNOWN_PRECONDITIONS
				SubgoalSet.insert(make_pair(p, candidate_action));
#endif


#ifdef DEBUG_EXTRACT
				TAB(3); cout<<"New subgoal added: ";
				subgoal.print();
				cout<<endl<<endl;

				TAB(3); cout<<"Current Q'top: ";
				Q.top().print();
				cout<<endl<<endl;
#endif
			}

#ifdef DEBUG_UNSUPPORTED_KNOWN_PRECONDITIONS
				cout<<"Unsupported known precondition set: ";
				print_unsupported_known_precondition_set();
				cout<<endl<<endl;

				cout<<"Subgoal set: ";
				print_subgoal_set();
				cout<<endl<<endl;

				// Check if all unsupported known preconditions present in the queue
				for (std::set<std::pair<int,int> >::const_iterator itr = unsupported_known_precondition_set.begin();
						itr != unsupported_known_precondition_set.end(); itr++) {
					assert(SubgoalSet.find(make_pair(itr->first, itr->second)) != SubgoalSet.end());
				}
#endif
		}

	} // out of Q-loop

#ifdef DEBUG_EXTRACT
	cout<<"Out Q-loop "<<__LINE__<<endl<<endl;
	cout<<"Num unsupported known preconditions: "<<num_unsupported_known_preconditions<<endl<<endl;
	cout<<"current_robustness_for_heuristics: "<<current_robustness_for_heuristics<<endl<<endl;
	cout<<"current_robustness: "<<compute_robustness()<<endl<<endl;
	cout<<"RP length: "<<rp.size()<<endl<<endl;
	cout<<"End extract_01..."<<__LINE__<<endl<<endl;
#endif

#ifdef DEBUG_UNSUPPORTED_KNOWN_PRECONDITIONS
	cout<<"Unsupported known precondition set: ";
	for (set<pair<int,int> >::const_iterator itr = unsupported_known_precondition_set.begin();
			itr != unsupported_known_precondition_set.end(); itr++) {
		cout<<"("<<itr->first<<", "<<itr->second<<") ";
	}
	cout<<endl<<endl;
#endif


#ifndef NDEBUG
	assert(!unsupported_known_precondition_exists());
#endif

	// Compute the robustness after the relaxed plan is built
	current_robustness = compute_robustness();

	// This must be satisfied, since a trivial relaxed plan includes all "best actions", with which
	// all known preconditions are supported
	assert(num_unsupported_known_preconditions == 0);

	// This must be true, because when all known preconditions are supported, clause sets from RPG no longer need
	//assert(current_robustness_for_heuristics == current_robustness);

	result.first = rp.size() - 1;	// Ignore the goal step
	result.second = current_robustness;

	// Check if the relaxed plan returned has enough robustness
	if (current_robustness <= robustness_threshold) {
		return false;
	}

	return true;
}

bool RelaxedPlan::extract(std::pair<int, double>& result) {
	bool ret = false;
	switch(RelaxedPlan::rp_types) {
	case PURE_FF_RP:
		ret = extract_pure_ff_heuristic(result);
		break;
	case ROBUST_FF_RP:
		ret = extract_robust_ff_heuristic(result);
		break;
	case INCREMENTAL_ROBUSTNESS_RP:
		ret = extract_incremental_robustness_rp(result);
		break;
	case LOCALLY_INCREMENTAL_ROBUSTNESS_RP:
		ret = extract_locally_incremental_robustness_rp(result);
		break;
	case GREEDY_ROBUSTNESS_RP:
		ret = extract_greedy_robustness_rp(result);
		break;
	case ALL_MOST_ROBUST_SUPPORTING_ACTIONS_RP:
		ret = extract_rp_with_all_most_robust_supporting_actions(result);
		break;
	}
	RelaxedPlan::num_rp_calls++;
	if (ret)
		RelaxedPlan::num_successful_rp_calls++;
	return ret;
}

//void RelaxedPlan::extract(pair<int, double>& result) {
//
//	// The queue to store all unsupported chosen actions
//	UNSUPPORTED_ACTION_QUEUE Q;
//
//	// Known and possible preconditions that need to be supported
//	vector<int> unsup_pres;
//	vector<int> unsup_poss_pres;
//
//	// The level at which all goals appear. So this is also the layer at which GOAL_ACTION is put
//	int n = P.size() - 1;
//	num_chosen_actions.reserve(n+1);
//	for (int i=0;i<=n;i++) num_chosen_actions[i] = 0;
//
//	// The first and last fact layer
//	const FactLayer& first_fact_layer = *(P[0]);
//	const FactLayer& last_fact_layer = *(P[n]);
//
//	//
//	// Create the step containing goal action of the relaxed plan
//	//
//	RP_STEP *goal_step = new RP_STEP;
//	goal_step->a = GOAL_ACTION;
//	goal_step->layer = n;
//	for (int i=0;i<current->num_F;i++) {
//		int f = current->F[i];
//		goal_step->s[f] = 1;	// facts in the current state are assumed to be supported by the INIT_ACTION
//	}
//
//	for (int i=0;i<goals->num_F; i++) {
//		int g = goals->F[i];
//
//		// If "g" is already in the current state, then we take the clauses for the fact at the current state
//		if (in_rp_state(g, goal_step->s)) {
//			assert(first_fact_layer.find(g) != first_fact_layer.end());
//			if (first_fact_layer.at(g).best_clauses.size())
//				(goal_step->pre_clauses)[g] = new ClauseSet(first_fact_layer.at(g).best_clauses);
//		}
//		// Otherwise, take the most "optimistic" clause set (got from the RPG construction)
//		else {
//			assert(last_fact_layer.find(g) != last_fact_layer.end());
//			if (last_fact_layer.at(g).best_clauses.size())
//				(goal_step->pre_clauses)[g] = new ClauseSet(last_fact_layer.at(g).best_clauses);
//		}
//	}
//
//	// Add the goal step to the relaxed plan, and increase the counter
//	rp.push_back(goal_step);
//	num_chosen_actions[n] = 1;
//
//	// Initialize Q with the goal action
//	UnsupportedAction goal_action;
//	goal_action.a = GOAL_ACTION;
//	goal_action.l = n;
//	goal_action.s_ptr = &goal_step->s;
//	Q.push(goal_action);
//
//	// Extracting actions in the relaxed planning graph to support actions in Q
//	while (!Q.empty()) {
//		int op = Q.top().a;
//		int l_op = Q.top().l;
//		const RP_STATE *state_before_op = Q.top().s_ptr;
//		Q.pop();
//
//#ifndef NDEBUG
//		// This action must be found in the relaxed plan
//		list<RP_STEP*>::iterator itr = rp.begin();
//		for (; itr != rp.end(); itr++)
//			if ((*itr)->a == op)
//				break;
//		assert(itr != rp.end());
//#endif
//		// Make sure these are cleared before being filled
//		unsup_pres.clear();
//		unsup_poss_pres.clear();
//
//		if (op == GOAL_ACTION) {
//			for (int i = 0; i < goals->num_F; i++) {
//				int g = goals->F[i];
//
//				// If "g" has been in the state before "op", we don't need to support it
//				// NO GOOD!!! (goals appearing in the current state must be able to improved)
//				if (state_before_op->find(g) != state_before_op->end())
//					continue;
//				unsup_pres.push_back(g);
//			}
//		}
//		else {
//			for (int i = 0; i < gop_conn[op].num_E; i++) {
//				int ef = gop_conn[op].E[i];
//				for (int j = 0; j < gef_conn[ef].num_PC; j++) {
//					int g = gef_conn[ef].PC[j];
//					// If "g" has been in the state before "op", we don't need to support it
//					if (state_before_op->find(g) != state_before_op->end())
//						continue;
//					unsup_pres.push_back(g);
//				}
//				for (int j = 0; j < gef_conn[ef].num_poss_PC; j++) {
//					int g = gef_conn[ef].poss_PC[j];
//					// If "g" has been in the state before "op", we don't need to support it
//					if (state_before_op->find(g) != state_before_op->end())
//						continue;
//					unsup_poss_pres.push_back(g);
//				}
//			}
//		}
//
//		FactLayer& current_fact_layer = *(P[l_op]);	// All known preconditions of "a" belong to this fact layer. Possible preconditions may NOT be!
//
//		// For each unsupported precondition, we add an action to support it.
//		for (int i = 0; i < unsup_pres.size(); i++) {
//			int g = unsup_pres[i];
//
//			// The first layer in the RPG that "g" appears
//			int first_layer = current_fact_layer[g].first_layer;
//
//			// Consider *all* actions from "first_layer" to "l_a - 1" which (possibly) support "g"
//			// OPTIONS: only consider actions with the best robustness at each layer
//			// THIS IS "HEAVY" STEP!!!!
//			double best_robustness = -1;
//			int best_supporting_action;
//			int layer_of_best_supporting_action;
//
//			int last_layer = (op != GOAL_ACTION? l_op : l_op - 1);
//			for (int l = first_layer; l <= last_layer; l++) {
//
//				ActionLayer& al = *(A[l]);
//
//				for (int j = 0; j < gft_conn[g].num_A; j++) {
//					int supporting_action = gft_conn[g].A[j];
//
//					// First, this action must be present in the action layer "l"
//					if (al.find(supporting_action) == al.end()) continue;
//
//					// Second, it has not been selected
//					if (al[supporting_action].in_rp) continue;
//
//					// Now evaluate this action...
//					double r = evaluate_candidate_action(supporting_action, l);
//					if (best_robustness < r) {
//						best_robustness = r;
//						best_supporting_action = supporting_action;
//						layer_of_best_supporting_action = l;
//
//						// The robustness of {plan prefix + relaxed plan} is the last value of the evaluation
//						// So we just record it
//						result.second = best_robustness;
//					}
//				}
//
//				for (int j = 0; j < gft_conn[g].num_poss_A; j++) {
//					int possibly_supporting_action = gft_conn[g].poss_A[j];
//
//					// First, this action must be present in the action layer "l"
//					if (al.find(possibly_supporting_action) == al.end()) continue;
//
//					// Second, it has not been selected
//					if (al[possibly_supporting_action].in_rp) continue;
//
//					// Now evaluate this action...
//					double r = evaluate_candidate_action(possibly_supporting_action, l);
//					if (best_robustness < r) {
//						best_robustness = r;
//						best_supporting_action = possibly_supporting_action;
//						layer_of_best_supporting_action = l;
//						// The robustness of {plan prefix + relaxed plan} is the last value of the evaluation
//						// So we just record it
//						result.second = best_robustness;
//					}
//				}
//			}
//
//			// Add the best supporting action into the current relaxed plan...
//			insert_action_into_relaxed_plan(best_supporting_action, layer_of_best_supporting_action);
//
//			// Mark this action node as being in the rp
//			(*(A[layer_of_best_supporting_action]))[best_supporting_action].in_rp = true;
//
//			// If the action is at the first action layer, collect its add and possible add effects
//			// for the purpose of using helpful actions
//			if (layer_of_best_supporting_action == 0) {
//				for (int j = 0; j < gop_conn[best_supporting_action].num_E; j++) {
//					int ef = gop_conn[best_supporting_action].E[j];
//					for (int k=0;k<gef_conn[ef].num_A;k++) {
//						int p = gef_conn[ef].A[k];
//						possibly_supported_facts_at_1st_fact_layer.insert(p);
//					}
//					for (int k=0;k<gef_conn[ef].num_poss_A;k++) {
//						int p = gef_conn[ef].poss_A[k];
//						possibly_supported_facts_at_1st_fact_layer.insert(p);
//					}
//				}
//			}
//		}
//	}
//
//	result.first = rp.size();
//}

void RelaxedPlan::get_FF_helpful_actions(std::vector<int>& helpful_actions) const {

	for (int op=0;op<gnum_op_conn;op++) {
		if (!applicable_action(op, current))
			continue;

		// We check if it (possibly) adds any propositions in the first fact layer that
		// has been selected during the relaxed plan extraction
		bool helpful = false;
		for (int ft = 0; ft < gnum_ft_conn && !helpful; ft++) {
			if (!known_and_possible_adds_of_actions_in_first_layer[ft])
				continue;
			if (is_add(ft, op) || is_poss_add(ft, op)) {
				helpful_actions.push_back(op);
				helpful = true;
			}
		}
	}
}

// Estimate the robustness of the plan prefix + the current relaxed plan
double RelaxedPlan::compute_robustness() {

//#define DEBUG_ESTIMATE_ROBUSTNESS

#ifdef DEBUG_ESTIMATE_ROBUSTNESS
	cout<<"Begin estimate_robustness..."<<endl<<endl;
#endif

	// Check if any know precondition is not in the corresponding state
	// In that case, return 0
	if (num_unsupported_known_preconditions > 0) {

#ifdef DEBUG_ESTIMATE_ROBUSTNESS
		cout<<"End estimate_robustness... Num unsupported known preconditions: "<<num_unsupported_known_preconditions<<endl<<endl;
#endif

		return 0;
	}

	// Now collect the clauses of the plan prefix + relaxed plan
	ClauseSet all_clauses;
	e->get_clauses(all_clauses);

	for (RELAXED_PLAN::iterator rp_itr = rp.begin(); rp_itr != rp.end(); rp_itr++) {

		int this_op = (*rp_itr)->a;
		int layer = (*rp_itr)->layer;

		if (this_op != GOAL_ACTION) {

			assert(gop_conn[this_op].num_E == 1);
			int ef = gop_conn[this_op].E[0];

			// Consider known preconditions of "this_op"
			for (int i=0;i<gef_conn[ef].num_PC;i++) {
				int p = gef_conn[ef].PC[i];

#ifndef NDEBUG
				// We're here only when all known preconditions present in their rp_states
				assert(in_rp_state(p, (*rp_itr)->s));
#endif
				//			// If this known precondition is not in the state before the action
				//			if (!in_rp_state(p, (*rp_itr)->s) && RelaxedPlan::clauses_from_rpg_for_false_preconditions) {
				//				all_clauses.add_clauses((*P[layer]).at(p).best_clauses);
				//				continue;
				//			}

				if ((*rp_itr)->pre_clauses.find(p) == (*rp_itr)->pre_clauses.end() ||
						(*rp_itr)->pre_clauses.at(p)->size() <= 0)
					continue;

				// Collect this clause set
				const ClauseSet& cs = *(*rp_itr)->pre_clauses.at(p);
				all_clauses.add_clauses(cs);
			}

			// Consider possible preconditions of "this_op"
			for (int i=0;i<gef_conn[ef].num_poss_PC;i++) {
				int p = gef_conn[ef].poss_PC[i];

				if ((*rp_itr)->poss_pre_clauses.find(p) == (*rp_itr)->poss_pre_clauses.end() ||
						(*rp_itr)->poss_pre_clauses.at(p)->size() <= 0)
					continue;

				// Clause set for this possible precondition
				const ClauseSet& cs = *(*rp_itr)->poss_pre_clauses.at(p);
				all_clauses.add_clauses(cs);
			}
		}
		else {
			RELAXED_PLAN::iterator goal_step_itr = rp.end();
			goal_step_itr--;
			assert((*goal_step_itr)->a == GOAL_ACTION);
			for (int i=0;i<goals->num_F;i++) {
				int g = goals->F[i];

				assert(in_rp_state(g, (*goal_step_itr)->s));

				if ((*goal_step_itr)->pre_clauses.find(g) != (*goal_step_itr)->pre_clauses.end() &&
						(*goal_step_itr)->pre_clauses.at(g)->size())
					all_clauses.add_clauses(*((*goal_step_itr)->pre_clauses.at(g)));
			}
		}
	}

	if (RelaxedPlan::use_lower_bound_in_rp) {
#ifdef DEBUG_ESTIMATE_ROBUSTNESS
		cout<<"End estimate_robustness... Clauses: "<<all_clauses<<__LINE__<<endl<<endl;
#endif
		return all_clauses.lower_wmc();
	}
	else if (RelaxedPlan::use_upper_bound_in_rp) {
#ifdef DEBUG_ESTIMATE_ROBUSTNESS
		cout<<"End estimate_robustness... Clauses: "<<all_clauses<<__LINE__<<endl<<endl;
#endif
		return all_clauses.upper_wmc();
	}
	// else

	CACHET_OUTPUT o;
	all_clauses.wmc(o);

#ifdef DEBUG_ESTIMATE_ROBUSTNESS
	cout<<"End estimate_robustness... Clauses: "<<all_clauses<<__LINE__<<endl<<endl;
#endif

	wmc_time += o.time;

	return o.prob;
}

// As the above function: Estimate both lower bound, upper bound of the plan prefix + the current relaxed plan
// Except: the upper bound is compared against the robustness threshold, and if it is better then the exact robustness will also be computed
// (otherwise, it will be set < 0)
// Return false if there exists unsupported preconditions in the current relaxed plan
bool RelaxedPlan::compute_robustness(double& lower, double& upper, double& exact) {
#ifdef DEBUG_ESTIMATE_ROBUSTNESS
	cout<<"Begin estimate_robustness..."<<endl<<endl;
#endif

	// Check if any know precondition is not in the corresponding state
	// In that case, return 0
	if (num_unsupported_known_preconditions > 0) {

#ifdef DEBUG_ESTIMATE_ROBUSTNESS
		cout<<"End estimate_robustness... Num unsupported known preconditions: "<<num_unsupported_known_preconditions<<endl<<endl;
#endif

		return 0;
	}

	// Now collect the clauses of the plan prefix + relaxed plan
	ClauseSet all_clauses;
	e->get_clauses(all_clauses);

	for (RELAXED_PLAN::iterator rp_itr = rp.begin(); rp_itr != rp.end(); rp_itr++) {

		int this_op = (*rp_itr)->a;
		int layer = (*rp_itr)->layer;

		if (this_op != GOAL_ACTION) {

			assert(gop_conn[this_op].num_E == 1);
			int ef = gop_conn[this_op].E[0];

			// Consider known preconditions of "this_op"
			for (int i=0;i<gef_conn[ef].num_PC;i++) {
				int p = gef_conn[ef].PC[i];

#ifndef NDEBUG
				// We're here only when all known preconditions present in their rp_states
				assert(in_rp_state(p, (*rp_itr)->s));
#endif
				//			// If this known precondition is not in the state before the action
				//			if (!in_rp_state(p, (*rp_itr)->s) && RelaxedPlan::clauses_from_rpg_for_false_preconditions) {
				//				all_clauses.add_clauses((*P[layer]).at(p).best_clauses);
				//				continue;
				//			}

				if ((*rp_itr)->pre_clauses.find(p) == (*rp_itr)->pre_clauses.end() ||
						(*rp_itr)->pre_clauses.at(p)->size() <= 0)
					continue;

				// Collect this clause set
				const ClauseSet& cs = *(*rp_itr)->pre_clauses.at(p);
				all_clauses.add_clauses(cs);
			}

			// Consider possible preconditions of "this_op"
			for (int i=0;i<gef_conn[ef].num_poss_PC;i++) {
				int p = gef_conn[ef].poss_PC[i];

				if ((*rp_itr)->poss_pre_clauses.find(p) == (*rp_itr)->poss_pre_clauses.end() ||
						(*rp_itr)->poss_pre_clauses.at(p)->size() <= 0)
					continue;

				// Clause set for this possible precondition
				const ClauseSet& cs = *(*rp_itr)->poss_pre_clauses.at(p);
				all_clauses.add_clauses(cs);
			}
		}
		else {
			RELAXED_PLAN::iterator goal_step_itr = rp.end();
			goal_step_itr--;
			assert((*goal_step_itr)->a == GOAL_ACTION);
			for (int i=0;i<goals->num_F;i++) {
				int g = goals->F[i];
				assert(in_rp_state(g, (*goal_step_itr)->s));
				if ((*goal_step_itr)->pre_clauses.find(g) != (*goal_step_itr)->pre_clauses.end() &&
						(*goal_step_itr)->pre_clauses.at(g)->size())
					all_clauses.add_clauses(*((*goal_step_itr)->pre_clauses.at(g)));
			}
		}
	}

	lower = all_clauses.lower_wmc();
	upper = all_clauses.upper_wmc();

	// We don't always compute the exact robustness, to save time!
	if (RelaxedPlan::use_robustness_threshold && upper > robustness_threshold) {
		CACHET_OUTPUT o;
		all_clauses.wmc(o);
		exact = o.prob;
		wmc_time += o.time;
	}
	else
		exact = -1.0;
	return true;
}

void RelaxedPlan::print_subgoals(SubGoalQueue Q) {

}

// Collect all clauses associated with rp_steps in [begin, end)
// Note that for preconditions not supported in the rp_states, we (optionally) use clause sets
// constructed in the RPG.
// Modified: "clause_set_collection" (initially may not be empty) will be added with these potential clauses
void RelaxedPlan::collect_rp_step_clauses_for_heuristics(RELAXED_PLAN::iterator begin, RELAXED_PLAN::iterator end, ClauseSet& clauses) const{

	if (clauses.size())
		clauses.clear();

	for (RELAXED_PLAN::iterator itr = begin; itr != end; itr++) {

		int op = (*itr)->a;
		int op_l = (*itr)->layer;
		const FactLayer& current_fact_layer = *(P[op_l]);

		if (op != GOAL_ACTION) {
			assert(op >= 0 && op < gnum_op_conn);
			assert(gop_conn[op].num_E == 1);
			int ef = gop_conn[op].E[0];

			// Known preconditions
			for (int i=0;i<gef_conn[ef].num_PC;i++) {
				int p = gef_conn[ef].PC[i];

				// CASE 1: If "p" is present in the state before "op",
				// then take the clause set associated with it
				if (in_rp_state(p, (*itr)->s)) { // (*itr)->s.find(p) != (*itr)->s.end()) {

					if ((*itr)->pre_clauses.find(p) != (*itr)->pre_clauses.end() &&
							(*itr)->pre_clauses.at(p)->size()) {

						clauses.add_clauses(*((*itr)->pre_clauses.at(p)));
					}
				}

				// CASE 2: Otherwise, use the clause set established during RPG construction
				else {

					if (RelaxedPlan::clauses_from_rpg_for_false_preconditions) {

						// "p" must be present in the fact layer of the RPG
						assert(current_fact_layer.find(p) != current_fact_layer.end());

						if (current_fact_layer.at(p).best_clauses.size())
							clauses.add_clauses(current_fact_layer.at(p).best_clauses);
					}
				}
			}

			// Possible preconditions
			for (int i=0;i<gef_conn[ef].num_poss_PC;i++) {
				int p = gef_conn[ef].poss_PC[i];
				int bvar = get_bool_var(p, op, POSS_PRE);

				// CASE 1: If "p" is present in the state before "op"
				// Take the clause set associated with "p"
				if (in_rp_state(p, (*itr)->s)) { //  (*itr)->s.find(p) != (*itr)->s.end()) {
					if ((*itr)->poss_pre_clauses.find(p) != (*itr)->poss_pre_clauses.end() &&
						(*itr)->poss_pre_clauses.at(p)->size())

					// Note: this already includes the negative boolean var for this possible precondition (see the insertion of rp_step part)
					clauses.add_clauses(*((*itr)->poss_pre_clauses.at(p)));
				}

				// CASE 2: "p" is not in the state before "op", but
				// present in the fact layer.
				// We take the clause set established during the RPG construction
				else if (current_fact_layer.find(p) != current_fact_layer.end()) {

					if (RelaxedPlan::clauses_from_rpg_for_false_preconditions) {

						const ClauseSet& cs = current_fact_layer.at(p).best_clauses;

						// These clauses are needed only if the possible precondition is realized
						ClauseSet temp_cs;
						if (cs.size()) {
							for (ClauseSet::const_iterator itr = cs.cbegin(); itr != cs.cend(); itr++) {
								Clause c = *itr;
								c.add_literal(-bvar);
								temp_cs.add_clause(c);
							}
						}
						else {
							Clause c;
							c.add_literal(-bvar);
							temp_cs.add_clause(c);
						}
						clauses.add_clauses(temp_cs);
					}

				}

				// CASE 3: "p" is not in the fact layer of the RPG, so for sure it has empty clause set
				else {

//					ClauseSet temp_cs;
//					Clause c;
//					c.add_literal(-bvar);
//					temp_cs.add_clause(c);
//					clauses.add_clauses(temp_cs);

					assert((*itr)->poss_pre_clauses.find(p) != (*itr)->poss_pre_clauses.end() &&
						(*itr)->poss_pre_clauses.at(p)->size());

					clauses.add_clauses(*((*itr)->poss_pre_clauses.at(p))); // this clause establishes in the insertion phase
				}
			}
		}

		// Collect clause sets for goals
		else {
			for (int i=0;i<goals->num_F;i++) {
				int g = goals->F[i];

				if (in_rp_state(g, (*itr)->s)) {

					if ((*itr)->pre_clauses.find(g) != (*itr)->pre_clauses.end() &&
							(*itr)->pre_clauses.at(g)->size()) {

						clauses.add_clauses(*((*itr)->pre_clauses.at(g)));
					}
				}
				else if (RelaxedPlan::clauses_from_rpg_for_false_preconditions) {

					// "p" must be present in the last fact layer of the RPG
					assert(current_fact_layer.find(g) != current_fact_layer.end());

					if (current_fact_layer.at(g).best_clauses.size())
						clauses.add_clauses(current_fact_layer.at(g).best_clauses);
				}
			}
		}
	}

}

// Collect clauses for a new RP_STEP. Optionally, preconditions not present in rp_states
// are associated with clause sets constructed in the RPG
void RelaxedPlan::construct_rp_step_clauses_for_heuristics(RELAXED_PLAN::iterator rp_step_itr, ClauseSet& clauses) {

	clauses.clear();

	int action = (*rp_step_itr)->a;
	const FactLayer& current_fact_layer = *(P[(*rp_step_itr)->layer]);
	assert(gop_conn[action].num_E == 1);
	int ef = gop_conn[action].E[0];

	// Note that both known and possible preconditions may not be present in the state before "action"

	// Known preconditions
	// Two cases: it is or is not present in the state before "action"
	for (int j=0;j<gef_conn[ef].num_PC;j++) {
		int p = gef_conn[ef].PC[j];

		// CASE 1: If "p" is present in the state before "action", then we can construct correctness constraints for it
		if (in_rp_state(p, (*rp_step_itr)->s)) { //  (*rp_step_itr)->s.find(p) != (*rp_step_itr)->s.end()) {
			ClauseSet cs;
			bool success = supporting_constraints(p, rp_step_itr, cs);
			assert(success);

			if (cs.size()) {
				clauses.add_clauses(cs);
			}
		}
		// CASE 2: Otherwise, use the clause set established during RPG construction
		else if (RelaxedPlan::clauses_from_rpg_for_false_preconditions) {
			// "p" must be present in the fact layer of the RPG
			assert(current_fact_layer.find(p) != current_fact_layer.end());

			if (current_fact_layer.at(p).best_clauses.size())
				clauses.add_clauses(current_fact_layer.at(p).best_clauses);
		}
	}

	// Possible preconditions
	// Unlike known preconditions, there 3 cases for possible preconditions
	// (1) in the state before "action"
	// (2) not in the state, but in the fact layer of the RPG
	// (3) not in the state or the fact layer

	for (int j=0;j<gef_conn[ef].num_poss_PC;j++) {
		int p = gef_conn[ef].poss_PC[j];
		int bvar = get_bool_var(p, action, POSS_PRE);

		// CASE 1: If "p" is present in the state before "action", then we can construct correctness constraints for it
		if (in_rp_state(p, (*rp_step_itr)->s)) { //(*rp_step_itr)->s.find(p) != (*rp_step_itr)->s.end()) {
			ClauseSet cs;
			bool success = supporting_constraints(p, rp_step_itr, cs);
			assert(success);

			// These clauses are needed only if the possible precondition is realized
			ClauseSet temp_cs;
			if (cs.size()) {
				for (ClauseSet::const_iterator itr = cs.cbegin(); itr != cs.cend(); itr++) {
					Clause c = *itr;
					c.add_literal(-bvar);
					temp_cs.add_clause(c);
				}
			}
			else {
				Clause c;
				c.add_literal(-bvar);
				temp_cs.add_clause(c);
			}
			clauses.add_clauses(temp_cs);

		}

		// CASE 2: "p" is not in the state before "action", but
		// present in the fact layer. We take the clause set established during the RPG construction
		else if (current_fact_layer.find(p) != current_fact_layer.end()) {
			if (RelaxedPlan::clauses_from_rpg_for_false_preconditions) {

				const ClauseSet& cs = current_fact_layer.at(p).best_clauses;

				// These clauses are needed only if the possible precondition is realized
				ClauseSet temp_cs;
				if (cs.size()) {
					for (ClauseSet::const_iterator itr = cs.cbegin(); itr != cs.cend(); itr++) {
						Clause c = *itr;
						c.add_literal(-bvar);
						temp_cs.add_clause(c);
					}
				}
				else {
					Clause c;
					c.add_literal(-bvar);
					temp_cs.add_clause(c);
				}
				clauses.add_clauses(temp_cs);
			}
		}

		// CASE 3: "p" is not in the fact layer of the RPG, so for sure it has empty clause set
		else {
			ClauseSet temp_cs;
			Clause c;
			c.add_literal(-bvar);
			temp_cs.add_clause(c);
			clauses.add_clauses(temp_cs);
		}
	}

}

// Update rp_states before actions after a new step RP_STEP is removed
void RelaxedPlan::update_rp_states_for_step_removal(RELAXED_PLAN::iterator& new_itr) {
	int action = (*new_itr)->a;
	assert(action != GOAL_ACTION);
	assert(gop_conn[action].num_E == 1);
	int ef = gop_conn[action].E[0];

	// Known add effect
	for (int j = 0; j < gef_conn[ef].num_A; j++) {
		int p = gef_conn[ef].A[j];

		// Consider all actions after "action"
		// Including GOAL_ACTION
		RELAXED_PLAN::iterator itr = new_itr;
		itr++;
		while (itr != rp.end()) {
			RP_STEP& step = **itr;
			assert(step.s.find(p) != step.s.end());
			assert(step.s[p].first >= 1);
			step.s[p].first--;
			itr++;
		}
	}

	// Possible add effect
	for (int j = 0; j < gef_conn[ef].num_poss_A; j++) {
		int p = gef_conn[ef].poss_A[j];
		// Consider all actions after "action"
		RELAXED_PLAN::iterator itr = new_itr;
		itr++;
		while (itr != rp.end()) {
			RP_STEP& step = **itr;
			assert(step.s.find(p) != step.s.end());
			assert(step.s[p].second >= 1);
			step.s[p].second--;
			itr++;
		}
	}
}

// Evaluate a candidate "action", which is at "layer" of the RPG, wrt the plan prefix and the current relaxed plan.
// Return the POTENTIAL robustness of plan prefix + relaxed plan + this new action
double RelaxedPlan::evaluate_candidate_action(int action, int layer) {

	assert(action >= 0 && action < gnum_op_conn);
	ClauseSet new_clauses_for_heuristics;
	e->get_clauses(new_clauses_for_heuristics);

	double r;

	// Current fact layer of the RPG
	const FactLayer& current_fact_layer = *(P[layer]);

	// The new step to be evaluated
	RP_STEP *new_step = new RP_STEP;
	new_step->a = action;
	new_step->layer = layer;

	// Number of steps/actions before the "layer" (in the current partial relaxed plan)
	int count = 0;
	for (int l=0;l<layer;l++)
		count += num_chosen_actions[l];

	// We now insert the new step, and keep the iterator
	RELAXED_PLAN::iterator new_itr;
	if (count == 0) {
		rp.push_front(new_step);
		new_itr = rp.begin();
		assert(*new_itr == new_step);
	}
	else {
		RELAXED_PLAN::iterator itr = rp.begin();
		for (int i=0;i<count;i++)
			itr++;
		new_itr = rp.insert(itr, new_step);
		assert(*new_itr == new_step);
	}

	// Collect clauses from rp_steps before "new_itr"
	ClauseSet clauses_before;
	collect_rp_step_clauses_for_heuristics(rp.begin(), new_itr, clauses_before);

	new_clauses_for_heuristics.add_clauses(clauses_before);

	// Update the step's state
	update_rp_state(new_itr, false);	// Don't have to compute the number of newly unsupported known preconditions!

	// Update states before actions that are after the new step
	if (RelaxedPlan::candidate_actions_affect_current_actions)
		update_rp_states_after(new_itr, false); // Don't have to compute the number of newly supported known preconditions!

	// Construct clauses for known and possible preconditions of "action"
	ClauseSet clauses_this_step;
	if (RelaxedPlan::current_actions_affect_candidate_action) {
		construct_rp_step_clauses_for_heuristics(new_itr, clauses_this_step);
		new_clauses_for_heuristics.add_clauses(clauses_this_step);
	}
	else {
		// If we ignore the affect of current actions on the candidate actions,
		// we will take the clause set associated with it in the RPG
		cout<<"Option not implemented. File "<<__FILE__<<", line "<<__LINE__<<endl;
		exit(1);
	}

	// Collect clauses for known and possible preconditions of actions "a" after "action" in the relaxed plan
	if (RelaxedPlan::candidate_actions_affect_current_actions) {
		RELAXED_PLAN::iterator after_new_itr = new_itr;
		after_new_itr++;
		ClauseSet clauses_after;
		collect_rp_step_clauses_for_heuristics(after_new_itr, rp.end(), clauses_after);
		new_clauses_for_heuristics.add_clauses(clauses_after);
	}

	// Now we evaluate the set of all clauses
	if (RelaxedPlan::use_lower_bound_in_rp)
		r = new_clauses_for_heuristics.lower_wmc();
	else if (RelaxedPlan::use_upper_bound_in_rp)
		r = new_clauses_for_heuristics.upper_wmc();
	else {
		CACHET_OUTPUT o;
		new_clauses_for_heuristics.wmc(o);
		r = o.prob;
		wmc_time += o.time;
	}

	// NOW REMOVING THE NEW STEP

	// Remove contribution of add effect and possible add effect of "action" to the states after it
	if (RelaxedPlan::candidate_actions_affect_current_actions)
		update_rp_states_for_step_removal(new_itr);

	// Release memory for clause sets of (possible) preconditions of the new action
	for (PRE_2_CLAUSES::iterator itr3 = (*new_itr)->pre_clauses.begin(); itr3 != (*new_itr)->pre_clauses.end(); itr3++) {
		if (itr3->second) {
			delete itr3->second;
			itr3->second = 0;
		}
	}

	for (POSS_PRE_2_CLAUSES::iterator itr3 = (*new_itr)->poss_pre_clauses.begin(); itr3 != (*new_itr)->poss_pre_clauses.end(); itr3++) {
		if (itr3->second) {
			delete itr3->second;
			itr3->second = 0;
		}
	}

	// Remove the iterator to the new step
	assert(*new_itr == new_step);
	delete *new_itr;
	rp.erase(new_itr);

	return r;

}

//// Evaluate a candidate "action", which is at "layer" of the RPG, wrt the current relaxed plan.
//double RelaxedPlan::evaluate_candidate_action(int action, int layer) {
//
//	double r;
//
//	// Current fact layer of the RPG
//	const FactLayer& current_fact_layer = *(P[layer]);
//
//	// The new step to be inserted
//	RP_STEP *new_step = new RP_STEP;
//
//	// Number of steps/actions before the "layer" (in the current partial relaxed plan)
//	int count = 0;
//	for (int l=0;l<layer;l++)
//		count += num_chosen_actions[l];
//
//	// We now insert the new step, and keep the iterator
//	RELAXED_PLAN::iterator new_itr;
//	if (count == 0) {
//		rp.push_front(new_step);
//		new_itr = rp.begin();
//	}
//	else {
//		RELAXED_PLAN::iterator itr = rp.begin();
//		for (int i=0;i<count;i++)
//			itr++;
//		new_itr = rp.insert(itr, new_step);
//	}
//
//	// All clauses for later evaluation
//	ClauseSet all_clauses;
//	e->get_clauses(all_clauses);
//
//	// Collect clause sets for (possible) preconditions of actions before "action" in the relaxed plan
//	for (RELAXED_PLAN::iterator itr = rp.begin(); itr != new_itr; itr++) {
//
//		// Known preconditions
//		for (PRE_2_CLAUSES::iterator itr2 = (*itr)->pre_clauses.begin(); itr2 != (*itr)->pre_clauses.end(); itr2++) {
//			if (itr2->second)
//				all_clauses.add_clauses(*(itr2->second));
//		}
//
//		// Possible preconditions
//		for (POSS_PRE_2_CLAUSES::iterator itr2 = (*itr)->poss_pre_clauses.begin(); itr2 != (*itr)->poss_pre_clauses.end(); itr2++) {
//			if (itr2->second)
//				all_clauses.add_clauses(*(itr2->second));
//		}
//	}
//
//	// Update the step's action
//	new_step->a = action;
//
//	// Update the step's state
//	if (new_itr == rp.begin()) {
//		for (int i=0;i<current->num_F; i++)
//			new_step->s[current->F[i]] = 1;
//	}
//	else {
//		RELAXED_PLAN::iterator itr = new_itr;
//		itr--;
//		int prev_action = (*itr)->a;
//		const RP_STATE& prev_state = (*itr)->s;
//		for (RP_STATE::const_iterator i = prev_state.begin(); i != prev_state.end(); i++) {
//			new_step->s[i->first] = i->second;
//		}
//		// Add known and possible effects of "prev_action" into "s", if they were not present
//		for (int i = 0; i < gop_conn[prev_action].num_E; i++) {
//			int ef = gop_conn[prev_action].E[i];
//			for (int j = 0; j < gef_conn[ef].num_A; j++)
//				if (new_step->s.find(gef_conn[ef].A[j]) != new_step->s.end())
//					++(new_step->s[gef_conn[ef].A[j]]);
//				else
//					new_step->s[gef_conn[ef].A[j]] = 1;
//
//			for (int j = 0; j < gef_conn[ef].num_poss_A; j++)
//				if (new_step->s.find(gef_conn[ef].poss_A[j]) != new_step->s.end())
//					++(new_step->s[gef_conn[ef].poss_A[j]]);
//				else
//					new_step->s[gef_conn[ef].poss_A[j]] = 1;
//		}
//	}
//
//	// Update states before actions that are after the new step
//	for (int i = 0; i < gop_conn[action].num_E; i++) {
//		int ef = gop_conn[action].E[i];
//
//		// Know add effect
//		for (int j = 0; j < gef_conn[ef].num_A; j++) {
//			int p = gef_conn[ef].A[j];
//
//			// Consider all actions after "action"
//			RELAXED_PLAN::iterator itr = new_itr;
//			itr++;
//			while (itr != rp.end()) {
//				RP_STEP& step = **itr;
//				if (step.s.find(p) != step.s.end()) step.s[p]++;
//				else step.s[p] = 1;
//
//				itr++;
//			}
//		}
//
//		// Possible add effect
//		for (int j = 0; j < gef_conn[ef].num_poss_A; j++) {
//			int p = gef_conn[ef].poss_A[j];
//
//			// Consider all actions after "action"
//			RELAXED_PLAN::iterator itr = new_itr;
//			itr++;
//			while (itr != rp.end()) {
//				RP_STEP& step = **itr;
//				if (step.s.find(p) != step.s.end()) step.s[p]++;
//				else step.s[p] = 1;
//
//				itr++;
//			}
//		}
//
//	}
//
//	// Collect clauses for known and possible preconditions of "action"
//	// Note that both known and possible preconditions may not be present in the state before "action"
//	for (int i=0;i<gop_conn[action].num_E;i++) {
//		int ef = gop_conn[action].E[i];
//
//		// Known preconditions
//		// Two cases: it is or is not present in the state before "action"
//		for (int j=0;j<gef_conn[ef].num_PC;j++) {
//			int p = gef_conn[ef].PC[j];
//
//			// CASE 1: If "p" is present in the state before "action", then we can construct correctness constraints for it
//			if (new_step->s.find(p) != new_step->s.end()) {
//				ClauseSet cs;
//				bool success = supporting_constraints(p, new_itr, cs);
//				assert(success);
//
//				if (cs.size()) {
//					all_clauses.add_clauses(cs);
//				}
//			}
//			// CASE 2: Otherwise, use the clause set established during RPG construction
//			else {
//				// "p" must be present in the fact layer of the RPG
//				assert(current_fact_layer.find(p) != current_fact_layer.end());
//
//				if (current_fact_layer.at(p).best_clauses.size())
//					all_clauses.add_clauses(current_fact_layer.at(p).best_clauses);
//			}
//		}
//
//		// Possible preconditions
//		// Unlike known preconditions, there 3 cases for possible preconditions
//		// (1) in the state before "action"
//		// (2) not in the state, but in the fact layer of the RPG
//		// (3) not in the state or the fact layer
//
//		for (int j=0;j<gef_conn[ef].num_poss_PC;j++) {
//			int p = gef_conn[ef].poss_PC[j];
//			int bvar = get_bool_var(p, action, POSS_PRE);
//
//			// CASE 1: If "p" is present in the state before "action", then we can construct correctness constraints for it
//			if (new_step->s.find(p) != new_step->s.end()) {
//				ClauseSet cs;
//				bool success = supporting_constraints(p, new_itr, cs);
//				assert(success);
//
//				// These clauses are needed only if the possible precondition is realized
//				ClauseSet temp_cs;
//				if (cs.size()) {
//					for (ClauseSet::const_iterator itr = cs.cbegin(); itr != cs.cend(); itr++) {
//						Clause c = *itr;
//						c.add_literal(-bvar);
//						temp_cs.add_clause(c);
//					}
//				}
//				else {
//					Clause c;
//					c.add_literal(-bvar);
//					temp_cs.add_clause(c);
//				}
//				all_clauses.add_clauses(temp_cs);
//
//			}
//
//			// CASE 2: "p" is not in the state before "action", but
//			// present in the fact layer. We take the clause set established during the RPG construction
//			else if (current_fact_layer.find(p) != current_fact_layer.end()) {
//				const ClauseSet& cs = current_fact_layer.at(p).best_clauses;
//
//				// These clauses are needed only if the possible precondition is realized
//				ClauseSet temp_cs;
//				if (cs.size()) {
//					for (ClauseSet::const_iterator itr = cs.cbegin(); itr != cs.cend(); itr++) {
//						Clause c = *itr;
//						c.add_literal(-bvar);
//						temp_cs.add_clause(c);
//					}
//				}
//				else {
//					Clause c;
//					c.add_literal(-bvar);
//					temp_cs.add_clause(c);
//				}
//				all_clauses.add_clauses(temp_cs);
//
//			}
//
//			// CASE 3: "p" is not in the fact layer of the RPG, so for sure it has empty clause set
//			else {
//				ClauseSet temp_cs;
//				Clause c;
//				c.add_literal(-bvar);
//				temp_cs.add_clause(c);
//				all_clauses.add_clauses(temp_cs);
//			}
//		}
//	}
//
//	/*
//	 * Collect clauses for known and possible preconditions of actions "a" after "action" in the relaxed plan
//	 */
//	RELAXED_PLAN::iterator itr = new_itr;
//	itr++;
//	while (itr != rp.end()) {
//
//		RP_STEP& step = **itr;
//		int a = step.a;
//		if (a == GOAL_ACTION) break;
//
//		const FactLayer& fact_layer = *(P[step.layer]);
//
//
//		for (int j = 0; j < gop_conn[a].num_E; j++) {
//			int ef = gop_conn[a].E[j];
//
//			////////////////////////////
//			// Known preconditions
//			for (int k = 0; k < gef_conn[ef].num_PC; k++) {
//				int p = gef_conn[ef].PC[k];
//
//				// If the precondition is not affected, or possibly affected, at all by the new action
//				// then we simply take its associated clause set
//				if (!is_add(p, action) && !is_poss_add(p, action) &&
//					(!RelaxedPlan::ignore_poss_del_in_rp || !is_poss_del(p, action))) {
//
//					if (step.pre_clauses[p]) {
//						all_clauses.add_clauses(*(step.pre_clauses[p]));
//					}
//				}
//				else {
//					// TWO CASES: (1) it is or (2) is not present in the state before "a"
//					// CASE 1
//					if (step.s.find(p) != step.s.end()) {
//
//						ClauseSet cs;
//						bool success = supporting_constraints(p, itr, cs);
//						assert(success);
//
//						if (cs.size()) {
//							all_clauses.add_clauses(cs);
//						}
//					}
//
//					// CASE 2
//					else {
//						// Since "p" is known precondition of "a", it must be present in the fact layer
//						assert(fact_layer.find(p) != fact_layer.end());
//
//						if (fact_layer.at(p).best_clauses.size()) {
//							//all_clauses.add(get_predicate(p), *(step.pre_clauses[p]));
//							all_clauses.add_clauses(fact_layer.at(p).best_clauses);
//						}
//					}
//				}
//			}
//
//			////////////////////////////
//			// Possible preconditions
//			for (int k = 0; k < gef_conn[ef].num_poss_PC; k++) {
//				int p = gef_conn[ef].poss_PC[k];
//				int bvar = get_bool_var(p, action, POSS_PRE);
//
//				if (!is_add(p, action) && !is_poss_add(p, action) &&
//					(!RelaxedPlan::ignore_poss_del_in_rp || !is_poss_del(p, action))) {
//
//					if (step.poss_pre_clauses[p]) {
//						//clause_sets_for_rp.push_back((*itr)->poss_pre_clauses[p]);
//						all_clauses.add_clauses(*(step.poss_pre_clauses[p]));
//					}
//				}
//				// If "p" is added or possibly added by "action", all states before actions between "action" and "a" might change
//				else {
//
//					// CASE 1: "p" is in the state before "a"
//					if (step.s.find(p) != step.s.end()) {
//						ClauseSet cs;
//						bool success = supporting_constraints(p, itr, cs);
//						assert(success);
//
//						// These clauses are needed only if the possible precondition is realized
//						if (cs.size()) {
//							ClauseSet temp_cs;
//							for (ClauseSet::const_iterator itr2 = cs.cbegin(); itr2 != cs.cend(); itr2++) {
//								Clause c = *itr2;
//								c.add_literal(-bvar);
//								temp_cs.add_clause(c);
//							}
////							(*itr)->temp_poss_pre_clauses[p] = new ClauseSet(temp_cs);
////							clause_sets_for_rp.push_back((*itr)->temp_poss_pre_clauses[p]);
//							all_clauses.add_clauses(temp_cs);
//						}
//					}
//
//					// CASE 2: "p" is not in the state before "a", but in the fact layer
//					else if (fact_layer.find(p) != fact_layer.end()) {
//						const ClauseSet& cs = fact_layer.at(p).best_clauses;
//
//						// These clauses are needed only if the possible precondition is realized
//						ClauseSet temp_cs;
//						if (cs.size()) {
//							for (ClauseSet::const_iterator itr2 = cs.cbegin(); itr2 != cs.cend(); itr2++) {
//								Clause c = *itr2;
//								c.add_literal(-bvar);
//								temp_cs.add_clause(c);
//							}
//						}
//						else {
//							Clause c;
//							c.add_literal(-bvar);
//							temp_cs.add_clause(c);
//						}
////						(*itr)->temp_poss_pre_clauses[p] = new ClauseSet(temp_cs);
////						clause_sets_for_rp.push_back((*itr)->temp_poss_pre_clauses[p]);
//						all_clauses.add_clauses(temp_cs);
//					}
//
//					// CASE 3: "p" is not in the state before "a", or in the fact layer before it
//					else {
//						ClauseSet temp_cs;
//						Clause c;
//						c.add_literal(-bvar);
//						temp_cs.add_clause(c);
////						(*itr)->temp_poss_pre_clauses[p] = new ClauseSet(temp_cs);
////						clause_sets_for_rp.push_back((*itr)->temp_poss_pre_clauses[p]);
//						all_clauses.add_clauses(temp_cs);
//					}
//				}
//			}
//		}
//
//		// Next action after "action"
//		itr++;
//	}
//
//	// Now we evaluate the set of all clauses
//	r = all_clauses.upper_wmc();
//
//	// Delete memory for clause sets of (possible) preconditions of the new action
//	for (PRE_2_CLAUSES::iterator itr3 = (*new_itr)->pre_clauses.begin(); itr3 != (*new_itr)->pre_clauses.end(); itr3++) {
//		if (itr3->second) {
//			delete itr3->second;
//			itr3->second = 0;
//		}
//	}
//
//	// Update states before actions that are after the new step
//	// Remove contribution of add effect and possible add effect of "action" to the states after it
//	for (int i = 0; i < gop_conn[action].num_E; i++) {
//		int ef = gop_conn[action].E[i];
//
//		// Know add effect
//		for (int j = 0; j < gef_conn[ef].num_A; j++) {
//			int p = gef_conn[ef].A[j];
//			// Consider all actions after "action"
//			RELAXED_PLAN::iterator itr = new_itr;
//			itr++;
//			while (itr != rp.end()) {
//				RP_STEP& step = **itr;
//				assert(step.s.find(p) != step.s.end());
//				assert(step.s[p] >= 1);
//				step.s[p]--;
//				itr++;
//			}
//		}
//
//		// Possible add effect
//		for (int j = 0; j < gef_conn[ef].num_poss_A; j++) {
//			int p = gef_conn[ef].poss_A[j];
//			// Consider all actions after "action"
//			RELAXED_PLAN::iterator itr = new_itr;
//			itr++;
//			while (itr != rp.end()) {
//				RP_STEP& step = **itr;
//				assert(step.s.find(p) != step.s.end());
//				assert(step.s[p] >= 1);
//				step.s[p]--;
//				itr++;
//			}
//		}
//	}
//
//	// Remove the iterator to the new step
//	rp.erase(new_itr);
//
//	// Delete memory for the new step
//	delete new_step;
//
//	return r;
//}


// Update state before an RP_STEP
int RelaxedPlan::update_rp_state(const RELAXED_PLAN::iterator& rp_step_itr, bool compute_unsupported_known_preconditions) {

	assert((*rp_step_itr)->a != GOAL_ACTION && ((*rp_step_itr)->a >= 0 && (*rp_step_itr)->a < gnum_op_conn));

	(*rp_step_itr)->s.clear();

	if (rp_step_itr == rp.begin()) {
		for (int i=0;i<current->num_known_F; i++)
			(*rp_step_itr)->s[current->known_F[i]] = std::make_pair(1,0);

		for (int i=0;i<current->num_F; i++)
			if ((*rp_step_itr)->s.find(current->F[i]) == (*rp_step_itr)->s.end())
				(*rp_step_itr)->s[current->F[i]] = std::make_pair(0,1);
	}
	else {
		RELAXED_PLAN::iterator itr = rp_step_itr;
		itr--;
		int prev_action = (*itr)->a;
		assert(prev_action != GOAL_ACTION && (prev_action >= 0 && prev_action < gnum_op_conn));

		const RP_STATE& prev_state = (*itr)->s;

		// Copy all facts from the previous state
		for (RP_STATE::const_iterator i = prev_state.begin(); i != prev_state.end(); i++) {
			if ((i->second).first || (i->second).second)
				(*rp_step_itr)->s[i->first] = i->second;
		}

		// Add known and possible add effects of "prev_action" into "s", if they were not present
		assert(gop_conn[prev_action].num_E == 1);
		int ef = gop_conn[prev_action].E[0];

		for (int j = 0; j < gef_conn[ef].num_A; j++)
			if (in_rp_state(gef_conn[ef].A[j], (*rp_step_itr)->s)) {
				if (known_in_rp_state(gef_conn[ef].A[j], (*rp_step_itr)->s))
					++((*rp_step_itr)->s[gef_conn[ef].A[j]]).first;
				else
					((*rp_step_itr)->s[gef_conn[ef].A[j]]).first = 1;
			}
			else
				(*rp_step_itr)->s[gef_conn[ef].A[j]] = std::make_pair(1,0);

		for (int j = 0; j < gef_conn[ef].num_poss_A; j++)
			if (in_rp_state(gef_conn[ef].poss_A[j], (*rp_step_itr)->s))
				++((*rp_step_itr)->s[gef_conn[ef].poss_A[j]]).second;
			else
				(*rp_step_itr)->s[gef_conn[ef].poss_A[j]] = std::make_pair(0,1);
	}

	if (compute_unsupported_known_preconditions) {
		int count = 0;
		int action = (*rp_step_itr)->a;
		assert(gop_conn[action].num_E == 1);
		int ef = gop_conn[action].E[0];

		for (int i=0;i<gef_conn[ef].num_PC;i++) {
			int p = gef_conn[ef].PC[i];
			if (!in_rp_state(p, (*rp_step_itr)->s)) {
				count++;

#ifdef DEBUG_UNSUPPORTED_KNOWN_PRECONDITIONS
				cout<<"Add new unsupported known precondition: "<<p<<", action: "<<action<<endl<<endl;
				unsupported_known_precondition_set.insert(make_pair(p, action));
#endif
			}
		}

		return count;

#ifdef DEBUG_UNSUPPORTED_KNOWN_PRECONDITIONS
		cout<<"Unsupported known precondition set: ";
		print_unsupported_known_precondition_set();
		cout<<endl<<endl;
#endif

	}

	return 0;
}

int RelaxedPlan::update_rp_states_after(const RELAXED_PLAN::iterator& rp_step_itr, bool compute_new_supported_preconditions) {
	int count = 0;
	int action = (*rp_step_itr)->a;
	assert(action != GOAL_ACTION && (action >= 0 && action < gnum_op_conn));	// We will never call this function for GOAL_ACTION
	assert(gop_conn[action].num_E == 1);
	int ef = gop_conn[action].E[0];

	// Known add effect
	for (int j = 0; j < gef_conn[ef].num_A; j++) {
		int p = gef_conn[ef].A[j];

		// Consider all actions after "action"
		// This include the step for GOAL_ACTION
		RELAXED_PLAN::iterator itr = rp_step_itr;
		itr++;
		while (itr != rp.end()) {
			RP_STEP& step = **itr;

			// If this proposition is already in the rp_state before this action,
			// then increase its count
			if (in_rp_state(p, step.s)) step.s[p].first++;
			// Otherwise, "p" is not in the rp_state. Initialize its count to make it belong to the state
			else {
				step.s[p] = std::make_pair(1,0);

				// Update the count if required
				if (compute_new_supported_preconditions) {
					if (step.a == GOAL_ACTION) {

						// If "p" is a goal, and since it changes from NOT in the rp_state before to belonging to it
						// we need to count
						if (is_in_state(p, goals)) {

							count++;

#ifdef DEBUG_UNSUPPORTED_KNOWN_PRECONDITIONS
							cout<<"Remove unsupported goal: "<<p<<endl<<endl;
							unsupported_known_precondition_set.erase(make_pair(p, GOAL_ACTION));
#endif

						}
					}
					else if (is_pre(p, step.a)) {

						count++;

#ifdef DEBUG_UNSUPPORTED_KNOWN_PRECONDITIONS
						cout<<"Remove unsupported goal: "<<p<<endl<<endl;
						unsupported_known_precondition_set.erase(make_pair(p, step.a));
#endif

					}
				}
			}

			itr++;
		}
	}

	// Possible add effect
	for (int j = 0; j < gef_conn[ef].num_poss_A; j++) {
		int p = gef_conn[ef].poss_A[j];

		// Consider all actions after "action"
		RELAXED_PLAN::iterator itr = rp_step_itr;
		itr++;
		while (itr != rp.end()) {
			RP_STEP& step = **itr;
			if (in_rp_state(p, step.s)) step.s[p].second++;
			else {
				step.s[p] = std::make_pair(0,1);

				// Update the count if required
				if (compute_new_supported_preconditions) {

					if (step.a == GOAL_ACTION) {
						// If "p" is a goal, and since it was NOT in the rp_state before, we decrease the
						// count
						if (is_in_state(p, goals)) {
							count++;

#ifdef DEBUG_UNSUPPORTED_KNOWN_PRECONDITIONS
							cout<<"Remove unsupported goal: "<<p<<endl<<endl;
							unsupported_known_precondition_set.erase(make_pair(p, GOAL_ACTION));
#endif

						}
					}
					else if (is_pre(p, step.a)) {
						count++;

#ifdef DEBUG_UNSUPPORTED_KNOWN_PRECONDITIONS
						cout<<"Remove unsupported precondition: "<<p<<endl<<endl;
						unsupported_known_precondition_set.erase(make_pair(p, step.a));
#endif

					}
				}
			}

			itr++;
		}
	}

	return count;

}

void RelaxedPlan::update_rp_step_clauses(const RELAXED_PLAN::iterator& rp_step_itr) {

	int action = (*rp_step_itr)->a;

	assert(action != GOAL_ACTION);	// We never call this function with GOAL_ACTION

	// Current fact layer of the RPG
	const FactLayer& current_fact_layer = *(P[(*rp_step_itr)->layer]);

	// Update clauses for known and possible preconditions of "action"
	assert(gop_conn[action].num_E == 1);
	int ef = gop_conn[action].E[0];

	// Known preconditions
	// Two cases: it is or is not present in the state before "action"
	for (int j=0;j<gef_conn[ef].num_PC;j++) {
		int p = gef_conn[ef].PC[j];

		// If "p" is present in the state before "action", then we can construct correctness constraints for it
		// So this clause set encodes how current actions before "action" in the plan prefix and the relaxed plan affect it.
		if (in_rp_state(p, (*rp_step_itr)->s)) { //  (*rp_step_itr)->s.find(p) != (*rp_step_itr)->s.end()) {
			ClauseSet cs;
			bool success = supporting_constraints(p, rp_step_itr, cs);
			assert(success);

			if (cs.size()) {
				(*rp_step_itr)->pre_clauses[p] = new ClauseSet(cs);
			}
		}
		// Otherwise, do nothing.
		// Later on in heuristics evaluation, we will use clauses from the RPG to this known precondition
		else {

//			if (RelaxedPlan::clauses_from_rpg_for_false_preconditions) {
//				// "p" must be present in the fact layer of the RPG
//				assert(current_fact_layer.find(p) != current_fact_layer.end());
//
//				if (current_fact_layer.at(p).best_clauses.size()) {
//					(*rp_step_itr)->pre_clauses[p] = new ClauseSet(current_fact_layer.at(p).best_clauses);
//				}
//			}

		}
	}

	// Possible preconditions
	// Unlike known preconditions, there 3 cases for possible preconditions
	// (1) in the state before "action"
	// (2) not in the state, but in the fact layer of the RPG
	// (3) not in the state or the fact layer

	for (int j=0;j<gef_conn[ef].num_poss_PC;j++) {
		int p = gef_conn[ef].poss_PC[j];
		int bvar = get_bool_var(p, action, POSS_PRE);

		// CASE 1: If "p" is present in the state before "action", then we can construct correctness constraints for it
		if (in_rp_state(p, (*rp_step_itr)->s)) {  //   (*rp_step_itr)->s.find(p) != (*rp_step_itr)->s.end()) {
			ClauseSet cs;
			bool success = supporting_constraints(p, rp_step_itr, cs);
			assert(success);

			// These clauses are needed only if the possible precondition is realized
			ClauseSet temp_cs;
			if (cs.size()) {
				for (ClauseSet::const_iterator itr = cs.cbegin(); itr != cs.cend(); itr++) {
					Clause c = *itr;
					c.add_literal(-bvar);
					temp_cs.add_clause(c);
				}
			}
			else {
				Clause c;
				c.add_literal(-bvar);
				temp_cs.add_clause(c);
			}

			(*rp_step_itr)->poss_pre_clauses[p] = new ClauseSet(temp_cs);	// "temp_cs" always non-empty
		}

		// CASE 2: "p" is not in the state before "action", this possible precondition
		// must be false. Note again that in heuristics evaluation, we also take clauses
		// from the RPG into consideration
		else {
			ClauseSet temp_cs;
			Clause c;
			c.add_literal(-bvar);
			temp_cs.add_clause(c);
			(*rp_step_itr)->poss_pre_clauses[p] = new ClauseSet(temp_cs);
		}
	}
}

// Update all clauses for steps after a particular step
void RelaxedPlan::update_rp_step_clauses_after(const RELAXED_PLAN::iterator& rp_step_itr) {
	int action = (*rp_step_itr)->a;
	RELAXED_PLAN::iterator itr = rp_step_itr;

	// Move to the next step
	itr++;

	while (itr != rp.end()) {
		RP_STEP& step = **itr;
		int a = step.a;

		const FactLayer& fact_layer = *(P[step.layer]);

		if (a != GOAL_ACTION) {

			assert(gop_conn[a].num_E == 1);

			int ef = gop_conn[a].E[0];

			////////////////////////////
			// Known preconditions
			for (int k = 0; k < gef_conn[ef].num_PC; k++) {
				int p = gef_conn[ef].PC[k];

				// If the precondition is not affected, or possibly affected, at all by the new action
				// then the clause set won't change
				if (!is_add(p, action) && !is_poss_add(p, action) &&
						(RelaxedPlan::ignore_poss_del_in_rp || !is_poss_del(p, action))) {
					continue;
				}
				else {

					// TWO CASES: (1) it is or (2) is not present in the state before "a".
					// The clause sets can only change in the first case!
					// CASE 1
					if (in_rp_state(p, step.s)) { //step.s.find(p) != step.s.end()) {

						// Recompute the clause set for this known precondition
						ClauseSet cs;
						bool success = supporting_constraints(p, itr, cs);
						assert(success);

						if (cs.size()) {

							// Release memory
							if (step.pre_clauses.find(p) != step.pre_clauses.end() && step.pre_clauses[p]) {
								delete step.pre_clauses[p];
							}

							// Update with the new clause set
							step.pre_clauses[p] = new ClauseSet(cs);
						}
					}
					// CASE 2
					else {
					}
				}
			}

			////////////////////////////
			// Possible preconditions
			for (int k = 0; k < gef_conn[ef].num_poss_PC; k++) {
				int p = gef_conn[ef].poss_PC[k];
				int bvar = get_bool_var(p, a, POSS_PRE);

				// If "p" is not affected by the new action, do nothing
				if (!is_add(p, action) && !is_poss_add(p, action) &&
						(RelaxedPlan::ignore_poss_del_in_rp || !is_poss_del(p, action))) {
					continue;
				}
				// If "p" is added or possibly added by "action", the clause sets may change
				else {
					// If "p" is in the state before "a"
					if (in_rp_state(p, step.s)) { //step.s.find(p) != step.s.end()) {

						// Recompute the clause set
						ClauseSet cs;
						bool success = supporting_constraints(p, itr, cs);
						assert(success);

						// These clauses are needed only if the possible precondition is realized
						ClauseSet temp_cs;
						if (cs.size()) {
							for (ClauseSet::const_iterator itr2 = cs.cbegin(); itr2 != cs.cend(); itr2++) {
								Clause c = *itr2;
								c.add_literal(-bvar);
								temp_cs.add_clause(c);
							}
						}
						else {
							Clause c;
							c.add_literal(-bvar);
							temp_cs.add_clause(c);

						}

						if (step.poss_pre_clauses.find(p) != step.poss_pre_clauses.end() && step.poss_pre_clauses[p])
							delete step.poss_pre_clauses[p];

						step.poss_pre_clauses[p] = new ClauseSet(temp_cs);
					}
					// Otherwise, the clause sets won't change
					else {

					}
				}
			}
		}

		// Now updating clause set for GOAL_ACTION
		else {

			for (int i=0;i<goals->num_F;i++) {
				int g = goals->F[i];

				// If this goal is not an add or possible add effect of "action", its clause set won't change
				if (!is_add(g, action) && !is_poss_add(g, action) &&
						(RelaxedPlan::ignore_poss_del_in_rp || !is_poss_del(g, action))) {
					continue;
				}

				if (in_rp_state(g, step.s)) { // step.s.find(g) != step.s.end()) {
					// Recompute the clause set for this known precondition
					ClauseSet cs;
					bool success = supporting_constraints(g, itr, cs);
					assert(success);

					if (cs.size()) {

						// Release memory
						if (step.pre_clauses.find(g) != step.pre_clauses.end() && step.pre_clauses[g])
							delete step.pre_clauses[g];

						// Update with the new clause set
						step.pre_clauses[g] = new ClauseSet(cs);
					}
				}
			}
		}

		// Next action after "action"
		itr++;
	}
}

// Insert "action" at "layer" of the RPG into the current relaxed plan
// Note that we order it in front of all chosen actions at the same layer
//boost::shared_ptr<RelaxedPlan::RP_STEP> RelaxedPlan::insert_action_into_relaxed_plan(int action, int layer) {
RelaxedPlan::RP_STEP *RelaxedPlan::insert_action_into_relaxed_plan(int action, int layer) {
	assert(action >= 0 && action < gnum_op_conn);

	double r;

	// Current fact layer of the RPG (the layer at which the "action" belongs to)
	const FactLayer& current_fact_layer = *(P[layer]);

	// The new step to be inserted
	//boost::shared_ptr<RP_STEP> new_step(new RP_STEP);
	RP_STEP *new_step = new RP_STEP;
	assert(new_step);

	// Update the step's action and layer
	new_step->a = action;
	new_step->layer = layer;

	// Number of steps/actions before the "layer" (in the current partial relaxed plan)
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
		for (int i=0;i<count;i++)
			itr++;
		new_itr = rp.insert(itr, new_step);
	}

	// Increase the number of actions chosen at this layer
	num_chosen_actions[layer]++;

	// Mark this action node in the RPG
	A[layer]->at(action).in_rp = true;

	// Update the step's state
	int newly_unsupported_known_preconditions = update_rp_state(new_itr);
	num_unsupported_known_preconditions += newly_unsupported_known_preconditions;

	// Update states before actions that are after the new step
	// Must be careful with GOAL_ACTION!!!
	int newly_supported_known_preconditions = update_rp_states_after(new_itr);
	num_unsupported_known_preconditions -= newly_supported_known_preconditions;

	// Update clauses for known and possible preconditions of "action"
	update_rp_step_clauses(new_itr);

	// Update clauses for known and possible preconditions of actions after "action"
	// Must be careful with GOAL_ACTION!!!
	update_rp_step_clauses_after(new_itr);

	// If the action is at the first action layer, collect its add and possible add effects
	// for the purpose of using helpful actions
	if (layer == 0) {
		assert(gop_conn[action].num_E == 1);
		int ef = gop_conn[action].E[0];

		for (int k=0;k<gef_conn[ef].num_A;k++) {
			int p = gef_conn[ef].A[k];
			known_and_possible_adds_of_actions_in_first_layer[p] = true;
		}
		for (int k=0;k<gef_conn[ef].num_poss_A;k++) {
			int p = gef_conn[ef].poss_A[k];
			known_and_possible_adds_of_actions_in_first_layer[p] = true;
		}
	}

	assert(*new_itr == new_step);
	return new_step;
}

void RelaxedPlan::get_confirmed_step_or_level(int p, const RELAXED_PLAN::iterator& the_step_itr,
							pair<int, RELAXED_PLAN::iterator>& output) {
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

bool RelaxedPlan::supporting_constraints(int p, const RELAXED_PLAN::iterator& the_step_itr, ClauseSet& clauses) {
#ifndef NDEBUG
	// Make sure "p" is in the state pointed to by "the_step_itr"
	assert((*the_step_itr)->s.find(p) != (*the_step_itr)->s.end());
#endif

	// Get the position (in the plan prefix or the relaxed plan) where the value of "p" is confirmed
	pair<int, RELAXED_PLAN::iterator> confirmed_pos;
	get_confirmed_step_or_level(p, the_step_itr, confirmed_pos);

	// TWO CASES
	// (1) the value of "p" is confirmed at some point in the plan prefix
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
					c.add_literal(bvar);
				}
			}

			// part of the relaxed plan prefix
			RELAXED_PLAN::iterator itr = rp.begin();
			while (itr != the_step_itr) {
				int a = (*itr)->a;
				if (is_poss_add(p, a)) {
					int bvar = get_bool_var(p, a, POSS_ADD);
					c.add_literal(bvar);
				}
				itr++;
			}
			clauses.add_clause(c);
		}

		// Now protection constraints, due to possible delete effects in the plan prefix and in the relaxed plan
		// Note: possible delete effect in the relaxed plan is considered only if "RelaxedPlan::ignore_poss_del_in_rp"
		// is set to FALSE
		if (RelaxedPlan::ignore_poss_del_in_rp) {
			for(int i=confirmed_pos.first;i<actions.size();i++) {
				if (is_poss_del(p, actions[i])) {
					Clause c;
					int bvar = get_bool_var(p, actions[i], POSS_DEL);
					c.add_literal(-bvar);

					// Actions in plan prefix and relaxed plan are considered

					// First, part of the plan prefix
					for (int j=i+1;j<actions.size();j++)
						if (is_poss_add(p, actions[j])) {
							int bvar = get_bool_var(p, actions[j], POSS_ADD);
							c.add_literal(bvar);
						}

					// Second, part of the relaxed plan
					RELAXED_PLAN::iterator itr = rp.begin();
					while (itr != the_step_itr) {
						int a = (*itr)->a;
						if (is_poss_add(p, a)) {
							int bvar = get_bool_var(p, a, POSS_ADD);
							c.add_literal(bvar);
						}
						itr++;
					}
					clauses.add_clause(c);
				}
			}
		}
		else {
			cout<<"Option not implemented. File "<<__FILE__<<", line "<<__LINE__<<endl;
			exit(1);
		}
	}

	// (2) it is confirmed at some step of the current relaxed plan
	else {
		// If "p" is confirmed false (and in that case, only at the first step of the relaxed plan),
		// we first need establishment constraints
		if (confirmed_pos.second == rp.begin() && P[0]->find(p) == P[0]->end()) {
			Clause c;
			RELAXED_PLAN::iterator itr = confirmed_pos.second;
			while (itr != the_step_itr) {
				if (is_poss_add(p, (*itr)->a)) {
					int bvar = get_bool_var(p, (*itr)->a, POSS_ADD);
					c.add_literal(bvar);
				}
				itr++;
			}
			clauses.add_clause(c);
		}

		// Now protection constraints
		// Only if "RelaxedPlan::ignore_poss_del_in_rp" is set to FALSE
		if (!RelaxedPlan::ignore_poss_del_in_rp) {
			cout<<"Option not implemented. File "<<__FILE__<<", line "<<__LINE__<<endl;
			exit(1);

			RELAXED_PLAN::iterator itr = confirmed_pos.second;
			while (itr != the_step_itr) {
				if (is_poss_del(p, (*itr)->a)) {
					Clause c;
					int bvar = get_bool_var(p, (*itr)->a, POSS_DEL);
					c.add_literal(-bvar);

					RELAXED_PLAN::iterator itr2 = itr;
					itr2++;
					while (itr2 != the_step_itr) {
						if (is_poss_add(p, (*itr2)->a)) {
							bvar = get_bool_var(p, (*itr)->a, POSS_ADD);
							c.add_literal(bvar);
						}
					}
					clauses.add_clause(c);
				}
				itr++;
			}
		}
	}
	return true;
}

void RelaxedPlan::initialize_fact_layer() {
	assert(P.size() == 0);

	int n = e->get_actions().size();	// Plan prefix length
	FactLayer *first_fact_layer = new FactLayer;
	assert(first_fact_layer);

	for (int i=0;i<current->num_F;i++) {
		int ft = current->F[i];
		FactNode node;
		e->supporting_constraints(ft, n, node.best_clauses);

//		// WE DON'T NEED THIS PART
//		// Clause sets of all actions in the plan prefix
//		ClauseSet all_clauses;
//		e->get_clauses(all_clauses);
//		all_clauses.add_clauses(node.best_clauses);
//
//		// Estimating robustness
//		if (RelaxedPlan::use_lower_bound_in_rp)
//			node.best_robustness = all_clauses.lower_wmc();
//		else
//			node.best_robustness = all_clauses.upper_wmc();

		// The supporting action is the last one of the plan prefix, or the INIT_ACTION
		if (n == 0)
			node.best_supporting_action = INIT_ACTION;	// If the plan prefix if empty, the INIT_ACTION
		else
			node.best_supporting_action = e->get_actions()[n-1];

		node.in_rp = true;		// Is this correct?
		node.first_layer = 0;
		(*first_fact_layer)[ft] = node;

		// Mark this fact as being present in the RPG
		(*facts_in_rpg)[ft] = true;
	}

	P.push_back(first_fact_layer);
}

void RelaxedPlan::initialize_ff_fact_layer() {
	assert(P.size() == 0);

	int n = e->get_actions().size();	// Plan prefix length
	FactLayer *first_fact_layer = new FactLayer;
	assert(first_fact_layer);

	for (int i=0;i<current->num_F;i++) {
		int ft = current->F[i];
		FactNode node;

		// The supporting action is the last one of the plan prefix, or the INIT_ACTION
		if (n == 0)
			node.best_supporting_action = INIT_ACTION;	// If the plan prefix if empty, the INIT_ACTION
		else
			node.best_supporting_action = e->get_actions()[n-1];

		node.in_rp = true;		// Is this correct?
		node.first_layer = 0;
		(*first_fact_layer)[ft] = node;

		// Mark this fact as being present in the RPG
		(*facts_in_rpg)[ft] = true;
	}

	P.push_back(first_fact_layer);
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

// Check if an action is applicable wrt a fact layer
bool RelaxedPlan::applicable_action_in_rpg(int op, int l) {
	assert(op >=0 && op < gnum_op_conn);
	assert(gop_conn[op].num_E == 1);
	int ef = gop_conn[op].E[0];

	for (int i=0;i<gef_conn[ef].num_PC;i++) {
		int p = gef_conn[ef].PC[i];
		if (!fact_present(p, l))
			return false;
	}
	return true;
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
	const FactLayer& current_fact_layer = *(P[n]);
	ActionLayer * new_action_layer = new ActionLayer;
	if (!new_action_layer) {
		cerr<<"Not enough memory! "<<__FILE__<<": "<<__LINE__<<endl;
		exit(1);
	}

	for (int op = 0; op < gnum_op_conn; op++) {

		assert(gop_conn[op].num_E == 1);
		int ef = gop_conn[op].E[0];

		// Ignore actions that are not applicable. Only need to check those not in the relaxed planning graph.
		if (!(*actions_in_rpg)[op] && !applicable_action_in_rpg(op, n))
			continue;

		// Information node for this action
		ActionNode node;

		// First, clause sets for known preconditions
		for (int i=0;i<gef_conn[ef].num_PC;i++) {
			int ft = gef_conn[ef].PC[i];
			// "ft" must be in the fact layer, due to the applicability condition
			node.clauses.add_clauses(current_fact_layer.at(ft).best_clauses);
		}

		// Second, clause sets for possible preconditions
		for (int i=0;i<gef_conn[ef].num_poss_PC;i++) {
			int ft = gef_conn[ef].poss_PC[i];
			int bvar = get_bool_var(ft, op, POSS_PRE);

			// TWO CASES
			// (1) this fact is in the current fact layer
			if (current_fact_layer.find(ft) != current_fact_layer.end()) {
				const ClauseSet& ft_cs = current_fact_layer.at(ft).best_clauses;
				ClauseSet temp_ft_cs;
				for (ClauseSet::const_iterator itr = ft_cs.cbegin(); itr != ft_cs.cend(); itr++) {
					Clause c = *itr;
					c.add_literal(-bvar);
					temp_ft_cs.add_clause(c);
				}
				node.clauses.add_clauses(temp_ft_cs);
			}
			// (2) it is not: then it means that this proposition is for sure to be FALSE
			// Thus, if this action were selected, it must not realized this proposition as its precondition
			else {
				ClauseSet cs;
				Clause c;
				c.add_literal(-bvar);
				cs.add_clause(c);
				node.clauses.add_clauses(cs);
			}
		}

		// Get the clause set for the current plan
		ClauseSet cs;
		e->get_clauses(cs);

		// Merge it with the clause set for this action
		cs.add_clauses(node.clauses);

		// Evaluate them all
		if (RelaxedPlan::use_lower_bound_in_rp)
			node.robustness = cs.lower_wmc();
		else if (RelaxedPlan::use_upper_bound_in_rp)
			node.robustness = cs.upper_wmc();
		else {
			// Exact computation
			CACHET_OUTPUT o;
			cs.wmc(o);
			node.robustness = o.prob;
			wmc_time += o.time;
		}

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
		assert(P[n]->find(ft) != P[n]->end());
	}
#endif

	A.push_back(new_action_layer);
	return true;
}

bool RelaxedPlan::grow_ff_action_layer() {
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
	const FactLayer& current_fact_layer = *(P[n]);
	ActionLayer * new_action_layer = new ActionLayer;
	if (!new_action_layer) {
		cerr<<"Not enough memory! "<<__FILE__<<": "<<__LINE__<<endl;
		exit(1);
	}

	for (int op = 0; op < gnum_op_conn; op++) {

		assert(gop_conn[op].num_E == 1);
		int ef = gop_conn[op].E[0];

		// Ignore actions that are not applicable. Only need to check those not in the relaxed planning graph.
		if (!(*actions_in_rpg)[op] && !applicable_action_in_rpg(op, n))
			continue;

		// Information node for this action
		ActionNode node;
		node.in_rp = false;

		// Add this node into the layer action and its clause set into the action layer
		(*new_action_layer)[op] = node;	// Almost empty action node!!!

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
		assert(P[n]->find(ft) != P[n]->end());
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
	if (!new_fact_layer) {
		cerr<<"Not enough memory! "<<__FILE__<<": "<<__LINE__<<endl;
		exit(1);
	}
	for (int ft = 0; ft < gnum_ft_conn; ft++) {

		bool will_be_added;	// Whether this fact will be added into the RPG at this iteration
		int best_supporting_action;
		ClauseSet best_clauses;
		double best_robustness;

		// If this fact has been present already, establish NOOP action
		if (current_fact_layer.find(ft) != current_fact_layer.end()) {
			will_be_added = true;
			best_clauses = current_fact_layer[ft].best_clauses;
			best_robustness = current_fact_layer[ft].best_robustness;
			best_supporting_action = NOOP;
		}
		else {
			will_be_added = false;
			best_robustness = 0;
		}

		// Collect non-NOOP actions certainly/possibly supporting this fact
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

		// Now find the best supporter for this fact (including NOOP)
		if (certainly_supporting_actions.size() || possibly_supporting_actions.size()) {
			will_be_added = true;

			// Among actions certainly adding this fact, find the one with the highest robustness
			for (int i = 0; i < certainly_supporting_actions.size(); i++) {
				int op = certainly_supporting_actions[i];

				RelaxedPlan::num_better_supporting_action_checks_in_rpg++;

				if (current_action_layer[op].robustness > best_robustness) {
					best_robustness = current_action_layer[op].robustness;
					best_supporting_action = op;
					best_clauses = current_action_layer[op].clauses;

					RelaxedPlan::num_better_supporting_actions_found_in_rpg++;
				}
			}

			// Similarly, find the best actions possibly adding this fact
			for (int i = 0; i < possibly_supporting_actions.size(); i++) {
				int op = possibly_supporting_actions[i];
				int bvar = get_bool_var(ft, op, POSS_ADD);
				ClauseSet cs = current_action_layer[op].clauses;
				Clause c;
				c.add_literal(bvar);	// A clause with a single literal
				cs.add_clause(c);		// CAREFUL: this very like removes many clauses that are superset of "c"

				ClauseSet tmp;
				e->get_clauses(tmp);
				cs.add_clauses(tmp);

				double r;
				if (RelaxedPlan::use_lower_bound_in_rp)
					r = cs.lower_wmc();
				else if (RelaxedPlan::use_upper_bound_in_rp)
					r = cs.upper_wmc();
				else {
					CACHET_OUTPUT o;
					cs.wmc(o);
					r = o.prob;
					wmc_time += o.time;
				}

				RelaxedPlan::num_better_supporting_action_checks_in_rpg++;

				if (r > best_robustness) {
					best_robustness = r;
					best_supporting_action = op;
					best_clauses = cs;

					RelaxedPlan::num_better_supporting_actions_found_in_rpg++;
				}
			}
		}


		// Create new fact node for the newly added fact
		if (will_be_added) {

#ifndef NDEBUG
			if (best_supporting_action == NOOP) {
				assert(fact_present(ft, n));
			}
#endif


			FactNode node;
			node.best_clauses = best_clauses;
			node.best_robustness = best_robustness;
			node.best_supporting_action = best_supporting_action;	// Including NOOP
			node.in_rp = false;
			// Update its first layer if necessary
			if (!(*facts_in_rpg)[ft])
				node.first_layer = n+1;
			else {
				node.first_layer = current_fact_layer[ft].first_layer;
			}

			// Insert the new node into the new layer
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


bool RelaxedPlan::grow_ff_fact_layer() {
	// We cannot grow fact layers if:
	// + There's no action layer. Call initialization function instead.
	// + The number of fact layers > action layers
	if (A.size() == 0 || P.size() > A.size())
		return false;

	int n = A.size() - 1;
	ActionLayer& current_action_layer = *(A[n]);
	FactLayer& current_fact_layer = *(P[n]);
	FactLayer *new_fact_layer = new FactLayer;
	if (!new_fact_layer) {
		cerr<<"Not enough memory! "<<__FILE__<<": "<<__LINE__<<endl;
		exit(1);
	}
	for (int ft = 0; ft < gnum_ft_conn; ft++) {

		bool will_be_added = false;	// Whether this fact will be added into the RPG at this iteration

		// If this fact has been present already, establish NOOP action
		if (current_fact_layer.find(ft) != current_fact_layer.end()) {
			will_be_added = true;
		}
		else {
			for (int i = 0; i < gft_conn[ft].num_A; i++) {
				int ef = gft_conn[ft].A[i];
				int op = gef_conn[ef].op;
				if (action_present(op, n)) {
					will_be_added = true;
					break;
				}
			}

			if (!will_be_added) {
				for (int i = 0; i < gft_conn[ft].num_poss_A; i++) {
					int ef = gft_conn[ft].poss_A[i];
					int op = gef_conn[ef].op;
					if (action_present(op, n)) {
						will_be_added = true;
						break;
					}
				}
			}
		}

		// Create new fact node for the newly added fact
		if (will_be_added) {

			FactNode node;
			node.in_rp = false;
			// Update its first layer if necessary
			if (!(*facts_in_rpg)[ft])
				node.first_layer = n+1;
			else {
				node.first_layer = current_fact_layer[ft].first_layer;
			}

			// Insert the new node into the new layer
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

// Estimate probability of goals in the RPG
double RelaxedPlan::goals_prob_in_rpg() {
	int n = P.size()-1;
	const FactLayer& current_fact_layer = *(P[n]);

	// Collect all clause sets for goals
	ClauseSet all_goal_clauses;
	for (int i=0;i<goals->num_F;i++) {
		int g = goals->F[i];
		if (current_fact_layer.find(g) == current_fact_layer.end())
			return 0;
		all_goal_clauses.add_clauses(current_fact_layer.at(g).best_clauses);
	}

	// Clause sets of all actions in the plan prefix
	ClauseSet all_clauses;
	e->get_clauses(all_clauses);
	all_clauses.add_clauses(all_goal_clauses);

	double r;
	if (RelaxedPlan::use_lower_bound_in_rp) {
		r = all_clauses.lower_wmc();
	}
	else if (RelaxedPlan::use_upper_bound_in_rp) {
		r = all_clauses.upper_wmc();
	}
	else {
		CACHET_OUTPUT o;
		all_clauses.wmc(o);
		r = o.prob;
		wmc_time += o.time;
	}
	return r;
}

bool RelaxedPlan::same_fact_layers(FactLayer& factlayer_1, FactLayer& factlayer_2) {

	if (factlayer_1.size() != factlayer_2.size()) return false;

	// Consider each fact "ft"
	for (int ft = 0; ft < gnum_ft_conn; ft++) {
		bool found_1 = (factlayer_1.find(ft) != factlayer_1.end());	// If this fact is in the first layer
		bool found_2 = (factlayer_2.find(ft) != factlayer_2.end());	// If it is in the second layer
		if ((found_1 && !found_2) || (!found_1 && found_2))	// If it is in only one layer, the two layers are different
			return false;

		// If we're here, "ft" could be in both, or neither of the two fact layers

		// For the first case, we check if the sets of clauses present are the same
		if (found_1 && found_2) {
			if (!(factlayer_1[ft].best_clauses == factlayer_2[ft].best_clauses)) return false;
		}
	}

	return true;
}

bool RelaxedPlan::same_ff_fact_layers(FactLayer& factlayer_1, FactLayer& factlayer_2) {

	if (factlayer_1.size() != factlayer_2.size()) return false;

	// Consider each fact "ft"
	for (int ft = 0; ft < gnum_ft_conn; ft++) {
		bool found_1 = (factlayer_1.find(ft) != factlayer_1.end());	// If this fact is in the first layer
		bool found_2 = (factlayer_2.find(ft) != factlayer_2.end());	// If it is in the second layer
		if ((found_1 && !found_2) || (!found_1 && found_2))	// If it is in only one layer, the two layers are different
			return false;
	}

	return true;
}

// Check if a proposition is in a RP-STATE
bool RelaxedPlan::in_rp_state(int p, const RP_STATE& s) const {
	if (s.find(p) != s.end() && (s.at(p).first || s.at(p).second))
		return true;
	return false;
}

// Check if a proposition is in a RP-STATE
bool RelaxedPlan::known_in_rp_state(int p, const RP_STATE& s) const {
	if (s.find(p) != s.end() && s.at(p).first)
		return true;
	return false;
}

RelaxedPlan::SubGoal::SubGoal(int g, int op, int l, bool possible_precondition, RP_STATE* s_ptr) {
	this->g = g;
	this->op = op;
	this->l = l;
	this->possible_precondition = possible_precondition;
	this->state_before_op = s_ptr;
}


// Return TRUE if "g1 < g2". The queue's top is the largest element.
bool RelaxedPlan::SubGoalComparison::operator() (const SubGoal& g1, const SubGoal& g2) const {
	// If the two subgoals are at two different layers, the queue prefers to pop the
	// one at earlier layer.
	if (g1.l != g2.l)
		return (g1.l > g2.l);	// So subgoals at earlier layers are more preferred
	else {
		// If they are at the same layer, prefer known precondition to possible precondition
		if (g1.possible_precondition && !g2.possible_precondition)
			return true;
		else if (!g1.possible_precondition && g2.possible_precondition)
			return false;
		else {
			// If they are both known or both unknown preconditions,
			// compare first their known supporters: those with higher
			// number of known supporters go first
			if (gft_conn[g1.g].num_A != gft_conn[g2.g].num_A)
				return (gft_conn[g1.g].num_A < gft_conn[g2.g].num_A);
			// then their possible add supporters
			else if (gft_conn[g1.g].num_poss_A != gft_conn[g2.g].num_poss_A)
				return (gft_conn[g1.g].num_poss_A < gft_conn[g2.g].num_poss_A);
			else
				return true;
		}
	}
}











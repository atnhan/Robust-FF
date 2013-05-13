/*
 * StochasticLocalSearch.cpp
 *
 *  Created on: Apr 1, 2013
 *      Author: tanguye1
 */

#include "StochasticLocalSearch.h"
#include "assert.h"
#include <vector>
#include <ctime>
#include "Helpful.h"
#include "StripsEncoding.h"
#include <climits>

using namespace std;

StochasticLocalSearch::StochasticLocalSearch(State *init, State *goals, int max_restarts, int max_steps, double noise):
	Search(init, goals) {
	assert(max_restarts > 0);
	assert(max_steps > 0);
	assert(noise >= 0.0);
	this->max_restarts = max_restarts;
	this->max_steps = max_steps;
	this->noise = noise;
}

StochasticLocalSearch::~StochasticLocalSearch() {

}

bool StochasticLocalSearch::run() {
	// Set up seed for the generator
	generator.seed(static_cast<unsigned int>(std::time(0)));		// Initialize seed using the current time

	// The two distribution for random number generation
	boost::random::uniform_int_distribution<> int_dist;		// The range can change
	boost::random::uniform_real_distribution<> real_dist;	// By default: [0, 1)

	best_plan.actions.clear();
	best_plan.robustness = 0;

	CACHET_OUTPUT r;

	// Number of plans found
	int num_plans = 0;

	// We try to find better plans in at most "max_restarts" restarts from the initial state
	for (int i=0;i<max_restarts;i++) {

		// When we're here: we have a current best robustness for the current best plan
		// Restart from the initial state
		StripsEncoding *e = new StripsEncoding(init);

//		//
//		cout<<"RESTART: i = "<<i<<endl;
//		//

		for (int j=0;j<max_steps;j++) {

//			//
//			cout<<"\nMAX STEPS: j = "<<j<<endl;
//
//			cout<<"\nCURRENT STATE:"<<endl;
//			print_state(*e->get_last_state());
//			cout<<endl;
//			//

			bool better_plan_found = false;

			// (1) Get applicable actions
			vector<int> applicable_actions;

			// Build the relaxed plan from the current state, and get the set of applicable actions
			double robustness_threshold;
			if (RelaxedPlan::use_robustness_threshold)
				robustness_threshold = best_plan.robustness;
			else
				robustness_threshold = 0;

			RelaxedPlan rp(e, init, goals, robustness_threshold);
			pair<int, double> rp_info;
			// If we cannot build the relaxed plan, it means that no plan exists from this state
			// Restart from the initial state
			if (!rp.extract_01(rp_info))
				break;

			get_applicable_actions(e->get_last_state(), applicable_actions, &rp, Search::FF_helpful_actions);

			// If there is no applicable action, we're done with this iteration
			// Restart from the initial state
			if (applicable_actions.size() <= 0)
				break;

//			//
//			cout<<"\nAPPLICABLE ACTIONS:"<<endl;
//			for (int k=0;k<applicable_actions.size();k++) {
//				print_op_name(applicable_actions[k]);
//				cout<<endl;
//			}
//			//

#ifndef NDEBUG
		const State& s0 = *e->get_last_state();
#endif

			// (2) For each applicable action, quickly check if we have a better plan with it
			for (int k=0;k<applicable_actions.size();k++) {

				assert(same_state(s0, *e->get_last_state()));

//				//
//				cout<<"\n>>>>> k = "<<k<<": ";
//				print_op_name(applicable_actions[k]);
//				cout<<endl;
//
//				cout<<"\nState:"<<endl;
//				print_state(*e->get_last_state());
//				//

				// If the estimated robustness of "plan prefix + this action" wrt the goals is
				// not less than the current best robustness, then we check its exact robustness
				double lower, upper;
				e->append(applicable_actions[k]);
				ClauseSet all_clauses;
				e->get_clauses(all_clauses);

				// Check if the goals present in the last state, and get its clause set
				ClauseSet goal_clauses;
				bool goals_present = e->check_goals(goals, goal_clauses);

				// We no longer need the appended action
				e->remove_last();

				// If there exists a goal not present in the last state, then this extended plan
				// prefix cannot be a solution (since the robustness must be 0)
				// Remove the action just appended, and continue with the next candidate action.
				if (!goals_present) {
					continue;
				}

				// Add goal clauses into the set of all clauses
				all_clauses.add_clauses(goal_clauses);

				// Evaluate its lower and upper bound
				lower = all_clauses.lower_wmc();
				upper = all_clauses.upper_wmc();

				// CHECKING CONDITION 1: If the lower bound is already better than the current robustness
				// then a new plan found!
				if (lower >= best_plan.robustness) {

					all_clauses.wmc(r);
					assert(lower <= r.prob);

					// Extend the plan prefix
					e->extend_plan_prefix(applicable_actions[k]);

					// Record the new best plan
					best_plan.actions = e->get_actions();	// Note: plan_prefix = all actions!
					best_plan.robustness = r.prob;

					better_plan_found = true;

					// PRINT OUT THIS PLAN
					cout<<"PLAN "<<num_plans++<<endl;
					cout<<"Number of actions:"<<best_plan.actions.size()<<endl;
					for (int i=0;i<best_plan.actions.size();i++) {
						int op = best_plan.actions[i];
						print_op_name(op);
						cout<<endl;
					}
					cout<<"Robustness: "<<best_plan.robustness<<endl;

					break;	// out of considering other applicable actions
				}

				// CHECKING CONDITION 2: If the upper bound is better than the current robustness,
				// then we also need to check the exact robustness
				if (upper >= best_plan.robustness) {

					all_clauses.wmc(r);
					assert(upper >= r.prob);

					// Only accept new plan if its exact robustness is better than the current value
					if (r.prob > best_plan.robustness) {
						// Extend the plan prefix
						e->extend_plan_prefix(applicable_actions[k]);

						// Record the new best plan
						best_plan.actions = e->get_actions();	// Note: plan_prefix = all actions!
						best_plan.robustness = r.prob;

						better_plan_found = true;

						// PRINT OUT THIS PLAN
						cout<<"PLAN "<<num_plans++<<endl;
						cout<<"Number of actions:"<<best_plan.actions.size()<<endl;
						for (int i=0;i<best_plan.actions.size();i++) {
							int op = best_plan.actions[i];
							print_op_name(op);
							cout<<endl;
						}
						cout<<"Robustness: "<<best_plan.robustness<<endl;

						break;	// out of considering other applicable actions
					}
				}
			}

			// If we find a better plan, we are done for this improvement iteration
			// We will restart from the initial state for finding even better plan
			if (better_plan_found)
				break; // out of "max_steps" loop

			// (3) If we're here, none of the candidate actions gives better plan
			// Evaluate the neighbor actions to make local move
			int best_action;
			double best_robustness = 0;
			int best_length = INT_MAX;
			for (int k=0;k<applicable_actions.size();k++) {

				// Append the action
				e->append(applicable_actions[k]);

				// Extract the relaxed plan
				double robustness_threshold;
				if (RelaxedPlan::use_robustness_threshold)
					robustness_threshold = best_plan.robustness;
				else
					robustness_threshold = 0;

				RelaxedPlan rp2(e, e->get_last_state(), goals, robustness_threshold);
				pair<int, double> rp2_info;
				rp2.extract_01(rp2_info);

				// Record the best action
//				if (best_robustness < rp2_info.second) {
//					best_action = applicable_actions[k];
//					best_robustness = rp2_info.second;
//				}

				if (best_length > rp2_info.first) {
					best_action = applicable_actions[k];
					best_length = rp2_info.first;
				}

				// Remove this action to consider the next one
				e->remove_last();
			}

//			//
//			cout<<"\n\nBEST ACTION: ";
//			print_op_name(best_action);
//			cout<<endl;
//			//

			// (4) Toss the coin to make local move
			// if "random < noise" then move to a random neighbor
			// otherwise, move to the best one
			int next_action;
			double random = real_dist(generator);

//			//
//			cout<<endl<<"Random: "<<random<<endl;
//			//

			if (random < noise) {
				// Now we randomly select on neighbor
				int selected_neighbor = int_dist(generator,
						boost::random::uniform_int_distribution<>::param_type(0, applicable_actions.size()-1));
				next_action = applicable_actions[selected_neighbor];
			}
			else {
				next_action = best_action;
			}

//			//
//			cout<<"*** NEXT ACTION: ";
//			print_op_name(next_action);
//			cout<<endl;
//			//

			// (5) Extend the plan prefix with the next action
			e->extend_plan_prefix(next_action);

//			//
//			cout<<"NEW STATE:"<<endl;
//			print_state(*e->get_last_state());
//			cout<<endl;
//			cout<<"END OF j = "<<j<<endl;
//			//
		}

		// Release memory
		delete e;
	}

	if (best_plan.actions.size() > 0 && best_plan.robustness > 0) {
		cout<<"SOLUTION PLAN:"<<best_plan.actions.size()<<endl;
		for (int i=0;i<best_plan.actions.size();i++) {
			int op = best_plan.actions[i];
			print_op_name(op);
			cout<<endl;
		}
		cout<<"ROBUSTNESS: "<<best_plan.robustness<<endl;
		return true;
	}
	else {
		cout<<"PLAN NOT FOUND!"<<endl;
	}

	return false;
}







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
#include <cmath>
#include <algorithm>
#include <fstream>
#include "Helpful.h"
#include "StripsEncoding.h"

using namespace std;

extern int gnum_possible_annotations;

// Default search parameters
bool StochasticLocalSearch::FF_helpful_actions = true;
int StochasticLocalSearch::max_restarts = 5;
int StochasticLocalSearch::initial_depth_bound = 10;
int StochasticLocalSearch::max_iterations = 5;
int StochasticLocalSearch::probes_at_depth = 60;
int StochasticLocalSearch::neighborhood_size = 5;
int StochasticLocalSearch::fail_bound = 32;
double StochasticLocalSearch::max_heuristic_bias = 1.5;
double StochasticLocalSearch::min_heuristic_bias = 0.5;

StochasticLocalSearch::StochasticLocalSearch(State *init, State *goals, double desired_robustness):
	Search(init, goals, desired_robustness) {

}

StochasticLocalSearch::~StochasticLocalSearch() {

}

// Sample a set of "n" actions from a given state
bool StochasticLocalSearch::sample_next_actions(StripsEncoding* e, double robustness_threshold, int n, std::vector<int>& actions, int tab) {

//#define DEBUG_SAMPLE_ACTION
#ifdef DEBUG_SAMPLE_ACTION
	TAB(tab); cout<<"Begin sampling action ";
	if (StochasticLocalSearch::FF_helpful_actions)
		cout<<"(FF-helpful only)..."<<endl;
	else
		cout<<"(all applicable actions)..."<<endl;
	TAB(tab); cout<<n<<" actions"<<endl<<endl;
#endif

	const State *current_state = e->get_last_state();

	// The set of candidate applicable actions (either FF helpful or all applicable actions)
	vector<int> candidate_applicable_actions;

	// Add search time
	clock.stop();
	timer.search_time += clock.time();
	clock.restart();
	//

	if (StochasticLocalSearch::FF_helpful_actions) {
		// Extract relaxed plan for the current state, in order to get helpful action
		RelaxedPlan rp(e, current_state, goals, robustness_threshold);
		pair<int, double> rp_info;
		if (!rp.extract(rp_info)) {

#ifdef DEBUG_SAMPLE_ACTION
			TAB(tab); cout<<"End sampling action... FAILED!"<<endl<<endl;
#endif

			// Add RP time
			clock.stop();
			timer.rp_time += clock.time();
			clock.restart();

			return false;
		}

		// Add RP time
		clock.stop();
		timer.rp_time += clock.time();
		clock.restart();

		// Add WMC time
		timer.robustness_computation_time += rp.get_wmc_time();

		// Get FF helpful actions
		get_FF_helpful_actions(current_state, candidate_applicable_actions, &rp);

		// Note: this set may well be empty!
		if (candidate_applicable_actions.size() == 0)
			return false;

	}
	// Get all applicable actions (including non FF-helpful)
	else {
		for (int op = 0; op < gnum_op_conn; op++) {
			if (applicable_action(op, current_state))
				candidate_applicable_actions.push_back(op);
		}
	}

#ifdef DEBUG_SAMPLE_ACTION
	TAB(tab+1); cout<<"Candidate actions: ";
	for (int i=0;i<candidate_applicable_actions.size();i++)
		cout<<candidate_applicable_actions[i]<<" ";
	cout<<endl<<endl;
#endif

	// Now sample "n" actions
	if (n < candidate_applicable_actions.size()) {
		vector<int> indices;
		sample_k(n, candidate_applicable_actions.size(), indices);
		for (int i=0;i<indices.size();i++) {
			actions.push_back(candidate_applicable_actions[indices[i]]);
		}
	}
	else {
		actions = candidate_applicable_actions;
	}

#ifdef DEBUG_SAMPLE_ACTION
	TAB(tab+1); cout<<"Sampled actions: ";
	for (int i=0;i<actions.size();i++)
		cout<<actions[i]<<" ";
	cout<<endl<<endl;
	TAB(tab); cout<<"End sampling action."<<endl<<endl;
#endif

	return true;
}

// Sample a next state of a given state
bool StochasticLocalSearch::sample_next_state(StripsEncoding* e, double current_robustness, int h, double heuristic_bias, NEIGHBOR& selected_neighbor, int tab) {
//#define DEBUG_SAMPLE_STATE
#ifdef DEBUG_SAMPLE_STATE
	TAB(tab); cout<<"Begin sample next state..."<<endl<<endl;
#endif

	// Set of sampled actions
	vector<int> sampled_applicable_actions;
	sample_next_actions(e, current_robustness, neighborhood_size, sampled_applicable_actions, tab + 1);

	// In case one of the sampled action leads to a better state
	bool better_state_found = false;

	// Set of heuristic values if applying each action
	vector<NEIGHBOR> neighbors;
	double sum_weights = 0.0;
	for (int i = 0; i < sampled_applicable_actions.size() && !better_state_found; i++) {
		int a = sampled_applicable_actions[i];
		assert(a >= 0 && a < gnum_op_conn);

		// Add search time
		clock.stop();
		timer.search_time += clock.time();
		clock.restart();

		e->append(a);

		// Add encoding time
		clock.stop();
		timer.clause_set_construction_time += clock.time();
		clock.restart();
		//

		// Extract relaxed plan
		RelaxedPlan rp(e, e->get_last_state(), goals, current_robustness);
		pair<int, double> rp_info;

		// If a relaxed plan with (lower/upper/exact) robustness > current_robustness,
		// record the action and relaxed plan length
		if (rp.extract(rp_info)) {

			// Add RP time
			clock.stop();
			timer.rp_time += clock.time();
			clock.restart();
			//

			assert(rp_info.second > current_robustness);

			if (rp_info.first < h) {
				selected_neighbor.action = a;
				selected_neighbor.h = rp_info.first;
				selected_neighbor.robustness = rp_info.second;
				better_state_found = true;
			}
			else {
				NEIGHBOR neighbor;
				neighbor.action = a;
				neighbor.h = rp_info.first;
				neighbor.robustness = rp_info.second;
				neighbor.h_weight = pow(1.0 / neighbor.h, heuristic_bias);
				sum_weights += neighbor.h_weight;

				// Add this neighbor
				neighbors.push_back(neighbor);

#ifdef DEBUG_SAMPLE_STATE
				TAB(tab + 1); cout<<"New neighbor: "; neighbor.print(); cout<<endl<<endl;
#endif
			}
		}

		// Add RP time
		clock.stop();
		timer.rp_time += clock.time();
		clock.restart();

		// Add WMC time
		timer.robustness_computation_time += rp.get_wmc_time();

		// Retract this action
		e->remove_last();

		// Add encoding time
		clock.stop();
		timer.clause_set_construction_time += clock.time();
		clock.restart();
		//

	}

	// If a better state found, immediately return
	if (better_state_found)
		return true;

	if (neighbors.size() <= 0)
		return false;

#ifdef DEBUG_SAMPLE_STATE
	TAB(tab); cout<<"Sum weight: "<<sum_weights<<endl<<endl;
#endif

	// Compute the probability for each neighbor
	for (int i=0;i<neighbors.size();i++) {
		neighbors[i].prob = neighbors[i].h_weight / sum_weights;
	}

	// Sort the neighbors in increasing of their probability
	std::sort(neighbors.begin(), neighbors.end(), neighbor_comparison_obj);

#ifdef DEBUG_SAMPLE_STATE
	TAB(tab); cout<<"Sorted neighbors:"<<endl;
	for (int i=0;i<neighbors.size();i++) {
		TAB(tab+1); cout<<i<<": "; neighbors[i].print(); cout<<endl<<endl;
	}
#endif

	// Roulette wheel selection
	double random = real_dist(generator);
	int k = 0;
	double sum = 0;
	for (; k < neighbors.size(); k++) {
		sum += neighbors[k].prob;
		if (sum > random)
			break;
	}

#ifdef DEBUG_SAMPLE_STATE
	TAB(tab); cout<<"Roulette wheel selects: "<<endl;
	TAB(tab+1); cout<<"Random: "<<random<<endl;
	TAB(tab+1); cout<<"Neighbor: "<<k<<", sum prob: "<<sum<<endl<<endl;
#endif


	// Return the selected neighbor
	selected_neighbor = neighbors[k];
	return true;
}

// Using local search to find a better state than the current one (i.e., having heuristic value < "h")
bool StochasticLocalSearch::local_search_for_a_better_state(StripsEncoding* e,
		double current_robustness, int h, int& next_h, double& next_robustness, int& fail_count, int tab) {

#ifdef DEBUG_LOCAL_SEARCH
	TAB(tab); cout<<"Begin local search..."<<endl;
	TAB(tab); cout<<"Robustness threshold: "<<current_robustness<<endl;
#endif

	State *S_min = e->get_last_state();
	bool better_state_found = false;

#ifdef DEBUG_LOCAL_SEARCH
	TAB(tab+2); cout<<"State: "; print_state_with_fact_indices(*S_min); cout<<endl<<endl;
	TAB(tab+2); cout<<"Current h: "<<h<<endl<<endl;
#endif

	int depth_bound = initial_depth_bound;
	for (int i=0; i < max_iterations && !better_state_found && fail_count < fail_bound; i++) {

		// In the first half of the iterations, we only consider FF helpful actions
		// In the second half, all actions will be considered
		// This option will be used in sampling the next actions
		//StochasticLocalSearch::FF_helpful_actions = (i <= max_iterations/2) ? true : false;

#ifdef DEBUG_LOCAL_SEARCH
		TAB(tab+3); cout<<"Iteration "<<i+1<<endl<<endl;
#endif
		// Execute "probes_at_depth" probes of "depth"
		for (int probes = 1; probes <= probes_at_depth && !better_state_found && fail_count < fail_bound; probes++) {

#ifdef DEBUG_LOCAL_SEARCH
			TAB(tab+4); cout<<"Probe "<<probes<<endl<<endl;
#endif

			State *S = S_min;
			int new_actions_count = 0;

#ifdef DEBUG_LOCAL_SEARCH
			TAB(tab+4); cout<<"State: "; print_state_with_fact_indices(*S); cout<<endl<<endl;
			TAB(tab+4); cout<<"Current h: "<<h<<endl<<endl;
#endif

			// "depth_bound" is the maximal length of a sequence of sampled actions
			double heuristic_bias = max_heuristic_bias;
			double heuristic_bias_length = max_heuristic_bias - min_heuristic_bias;
			for (int depth = 1; depth <= depth_bound && !better_state_found; depth++) {

				// Update the heuristic bias (so that it decreases linearly from max to min
				// when the length of the probe goes from 1 to depth_bound
				heuristic_bias -= (heuristic_bias_length / depth_bound);

				// Sample the next state. Since we have a threshold for robustness,
				// only states from which we can extract a "valid" relaxed plan are considered
				// (the returned value of the following function)
				NEIGHBOR selected_neighbor;
				if (sample_next_state(e, current_robustness, h, heuristic_bias, selected_neighbor, tab + 4)) {

					// Update search time
					clock.stop();
					timer.search_time += clock.time();
					clock.restart();
					//

					// Append the next action
					e->append(selected_neighbor.action);

					// Update clause set construction time
					clock.stop();
					timer.clause_set_construction_time += clock.time();
					clock.restart();
					//

					new_actions_count++;

#ifdef DEBUG_LOCAL_SEARCH
					TAB(tab+5); cout<<"Sampled action: "<<selected_neighbor.action<<endl<<endl;
					TAB(tab+5); cout<<"Next state: "; print_state_with_fact_indices(*e->get_last_state()); cout<<endl<<endl;
					TAB(tab+5); cout<<"h from here: "<<selected_neighbor.h<<endl<<endl;
#endif

					// A BETTER STATE IS FOUND!!!
					if (selected_neighbor.h < h) {
						next_h = selected_neighbor.h;
						next_robustness = selected_neighbor.robustness;

						// Update search time
						clock.stop();
						timer.search_time += clock.time();
						clock.restart();
						//

						e->advance_plan_prefix();

						// Update clause set construction time
						clock.stop();
						timer.clause_set_construction_time += clock.time();
						clock.restart();
						//

						// Mark that a better state is found so that the fail count won't be updated
						better_state_found = true;

#ifdef DEBUG_LOCAL_SEARCH
						TAB(tab+5); cout<<"Better state found!"<<endl<<endl;
#endif

						break;	// out of building the current probe
					}
				}
				// If we cannot sample a state from which a relaxed plan (with >= a robustness threshold) exists,
				// then we roll back and start a new probe.
				else {

					// Update search time
					clock.stop();
					timer.search_time += clock.time();
					clock.restart();
					//

					for (int j = 0; j < new_actions_count; j++) {
						e->remove_last();
					}

					// Update clause set construction time
					clock.stop();
					timer.clause_set_construction_time += clock.time();
					clock.restart();
					//

#ifdef DEBUG_LOCAL_SEARCH
					TAB(tab+5); cout<<"No sampled action found;"<<endl<<endl;
					TAB(tab+5); cout<<"Roll back "<<new_actions_count<<" actions."<<endl<<endl;
#endif

					break;	// out of building the current probe
				}
			}

			// Increment fail count if no better state is found
			if (!better_state_found) {
				fail_count++;
			}
		}

		// Double depth bound
		depth_bound *= 2;
	}

	return better_state_found;
}

bool StochasticLocalSearch::run(FILE *log) {

//#define DEBUG_SLS_RUN
#ifdef DEBUG_SLS_RUN
	cout<<"BEGIN StochasticLocalSearch::run()..."<<endl<<endl;
#endif

	// Start the clock
	clock.restart();

	// Set up seed for the generator
	generator.seed(static_cast<unsigned int>(std::time(0)));		// Initialize seed using the current time

	// Reset best plan information
	best_plan.actions.clear();
	best_plan.robustness = 0;

	// To store output by Cachet
	CACHET_OUTPUT r;

	// Number of plans found
	int num_plans = 0;

#ifdef DEBUG_SLS_RUN
		cout<<"Best plan: "<<best_plan.actions.size()<<" actions. Robustness: "<<best_plan.robustness<<endl<<endl;

		cout<<"INIT STATE: ";
		print_state_with_fact_indices(*init);
		cout<<endl<<endl;
#endif

	// Add search time
	clock.stop();
	timer.search_time += clock.time();
	clock.restart();

	// Find the relaxed plan from the initial state
	StripsEncoding *e_0 = new StripsEncoding(init);

	// Add encoding time
	clock.stop();
	timer.clause_set_construction_time += clock.time();
	clock.restart();
	//

	RelaxedPlan::num_rp_calls = RelaxedPlan::num_successful_rp_calls = 0;

	RelaxedPlan rp_0(e_0, init, goals, initial_robustness_threshold);
	pair<int, double> rp_0_info;
	if (!rp_0.extract(rp_0_info)) {

#ifdef DEBUG_SLS_RUN
		cout<<"Relaxed plan extraction fails! Line: "<<__LINE__<<endl<<endl;
		cout<<"END StochasticLocalSearch::run()..."<<endl<<endl;
#endif

		// RP time
		clock.stop();
		timer.rp_time += clock.time();

		return false;		// No relaxed plan found.
	}

	// RP time
	clock.stop();
	timer.rp_time += clock.time();
	clock.restart();

#ifdef DEBUG_SLS_RUN
	cout<<"Relaxed plan: "<<rp_0_info.first<<" actions. Robustness (plan prefix + relaxed plan): "<<rp_0_info.second<<endl<<endl;

	cout<<"Now searching for a plan with more than "<<best_plan.robustness<<" robustness..."<<endl<<endl;
#endif

	// We try to find better plans in at most "max_restarts" restarts from the initial state
	int restarts = 0;
	while (restarts < max_restarts) {

#ifdef DEBUG_SLS_RUN
		cout<<"Restart from initial state: restarts = "<<restarts<<endl<<endl;
#endif

		// This is where we restart from the initial state
		clock.stop();
		StripsEncoding *e = new StripsEncoding(init);
		timer.clause_set_construction_time += clock.time();
		clock.restart();

		// Reset the number of fails. Each fail corresponds to a complete probe without finding a better state
		int fail_count = 0;

		// Current heuristic
		int h = rp_0_info.first;

#ifdef DEBUG_SLS_RUN
		TAB(2); cout<<"Current h = "<<h<<endl<<endl;
#endif

		while (true) {

			int next_h;
			double next_robustness;
			double current_robustness = (best_plan.robustness < initial_robustness_threshold) ? initial_robustness_threshold : best_plan.robustness;
			bool better_state_found = local_search_for_a_better_state(e, current_robustness, h, next_h, next_robustness, fail_count);

			// Two cases to restart the local search from the initial state
			// (1) fails count reached
			// (2) cannot escape the plateau
			if (fail_count >= fail_bound) {
				fail_count = 0;
				break;
			}

			if (!better_state_found) {
				break;
			}

			// HERE, A BETTER STATE HAS BEEN FOUND!!!

			// Check if the plan prefix has better robustness than the current best plan (i.e., the relaxed plan is empty)
			// The local search succeeds
			if (next_h == 0) {

				// Add search time
				clock.stop();
				timer.search_time += clock.time();

				// Update the current best plan
				best_plan.actions = e->get_actions();
				best_plan.robustness = next_robustness;
				best_plan.id = plans.size() + 1;

				// Save this new plan to the set of all plans
				plans.push_back(best_plan);

				// Update the log
				fprintf(log, "Plan %d found.\n", plans.size());

				// Update the analysis file
				update_experiment_analysis_file(best_plan);

				// Save the plan to its individual plan file
				stringstream ss;
				ss<<best_plan.id;
				string this_plan_file = string(gcmd_line.path) + string(gcmd_line.ops_file_name) + string("@") +
						string(gcmd_line.fct_file_name) + string(".") + ss.str() + string(".SOL");
				ofstream f;
				f.open(this_plan_file.c_str());
				f<<best_plan;
				f.close();

				// Restart the clock
				clock.restart();

				// Reset counting on relaxed plan constructions
				RelaxedPlan::num_rp_calls = RelaxedPlan::num_successful_rp_calls = 0;

				// Reset counting on checking better supporting actions in RPG
				RelaxedPlan::num_better_supporting_action_checks_in_rpg = 0;
				RelaxedPlan::num_better_supporting_actions_found_in_rpg = 0;

				// Reset counting on checking the increase of relaxed plan robustness
				RelaxedPlan::num_rp_robustness_increasing_checks = 0;
				RelaxedPlan::num_rp_robustness_increasing_check_success = 0;

#ifdef DEBUG_SLS_RUN
				TAB(2); cout<<"BETTER PLAN FOUND!"<<endl;
#endif

				// Restart from the initial state
				break;

			}
			// Otherwise, update the current heuristics
			else {

#ifdef DEBUG_SLS_RUN
				TAB(2); cout<<"Next h = "<<next_h<<endl<<endl;
#endif

				h = next_h;
			}
		}

		// Restart from the initial state
		restarts++;

		// Update fail bound if needs
		if (restarts % 3 == 0)
			fail_bound *= 2;

		// Release memory
		delete e;
	}

	// Add search time
	clock.stop();
	timer.search_time += clock.time();

	// WRITE TO FILE FOR EXPERIMENT ANALYSIS
	update_experiment_analysis_file_for_complete_run();

#ifdef DEBUG_SLS_RUN
	cout<<endl<<"===== "<<plans.size()<<" PLANS ====="<<endl<<endl;
	for (int i=0;i<plans.size(); i++) {
		cout<<"PLAN "<<i<<endl;
		for (int j=0;j<plans[i].actions.size();j++) {
			int op = plans[i].actions[j];
			cout<<j<<": ";
			print_op_name(op);
			cout<<endl;
		}
		cout<<"ROBUSTNESS: "<<plans[i].robustness<<endl;
	}
#endif

	delete e_0;

	return true;
}

// Sample k distinct integers from 0 to n-1
void StochasticLocalSearch::sample_k(int k, int n, vector<int>& result) {
	assert(k < n);
	vector<int> a;
	for (int i=0;i<n;i++) a.push_back(i);
	for (int i=0;i<k;i++) {
		int t = int_dist(generator, boost::random::uniform_int_distribution<>::param_type(0, a.size()-1));
		result.push_back(a[t]);
		a.erase(a.begin() + t);
	}
}

// Update the experiment analysis file
void StochasticLocalSearch::update_experiment_analysis_file_for_complete_run() {

#define tab		"\t"

	string stat_file = string(gcmd_line.path) + string(gcmd_line.experiment_analysis_file_for_complete_run);

	// Check if the file exists, if not then write the header
	ifstream f0(stat_file.c_str());
	bool file_exists = f0.good();
	f0.close();

	ofstream f;
	f.open(stat_file.c_str(), ios::out | ios::app);

	if (!file_exists) {
		f<<"#SEARCH PARAMETERS:"<<endl;
		f<<"#Max restart:\t"<<max_restarts<<endl;
		f<<"#Max iterations:\t"<<max_iterations<<endl;
		f<<"#Initial depth bound:\t"<<initial_depth_bound<<endl;
		f<<"#Probes at depth:\t"<<probes_at_depth<<endl;
		f<<"#Neighborhood size:\t"<<neighborhood_size<<endl;
		f<<"#Initial fail bound:\t"<<fail_bound<<endl;
		f<<"#Min heuristic bias:\t"<<min_heuristic_bias<<endl;
		f<<"#Max heuristic bias:\t"<<max_heuristic_bias<<endl;
		f<<"#"<<endl;
		f<<"#RELAXED PLAN PARAMETERS:"<<endl;
		f<<"#ignore_poss_del_in_rp:\t"<<RelaxedPlan::ignore_poss_del_in_rp<<endl;
		f<<"#use_lower_bound_in_rp:\t"<<RelaxedPlan::use_lower_bound_in_rp<<endl;
		f<<"#use_upper_bound_in_rp:\t"<<RelaxedPlan::use_upper_bound_in_rp<<endl;
		f<<"#use_robustness_threshold:\t"<<RelaxedPlan::use_robustness_threshold<<endl;
		f<<"#clauses_from_rpg_for_false_preconditions:\t"<<RelaxedPlan::clauses_from_rpg_for_false_preconditions<<endl;
		f<<"#current_actions_affect_candidate_action:\t"<<RelaxedPlan::current_actions_affect_candidate_action<<endl;
		f<<"#candidate_actions_affect_current_actions:\t"<<RelaxedPlan::candidate_actions_affect_current_actions<<endl;
		f<<"#"<<endl;
		f<<"#Domain"<<tab
		 <<"Problem"<<tab
		 <<"Incompleteness_amount"<<tab
		 <<"Num_plans"<<tab
		 <<"Best-plan-length"<<tab
		 <<"Best-plan-robustness"<<tab
		 <<"Total-time"<<tab
		 <<"Search-time"<<tab
		 <<"RP-time"<<tab
		 <<"clause-time"<<tab
		 <<"WMC-time"<<endl;
	}

	// Domain
	f<<gcmd_line.ops_file_name<<tab;

	// Problem
	f<<gcmd_line.fct_file_name<<tab;

	// The total number of possible preconditions and effects
	f<<gnum_possible_annotations<<tab;

	// The number of plans found
	f<<plans.size()<<tab;

	// Best plan's length
	f<<best_plan.actions.size()<<tab;

	// Best plan's robustness
	f<<best_plan.robustness<<tab;

	// Total time
	f<<timer.total()<<tab;

	// Search time
	f<<timer.search_time<<tab;

	// Time to extract relaxed plans
	f<<timer.rp_time<<tab;

	// Time to construct clause sets
	f<<timer.clause_set_construction_time<<tab;

	// Time to estimate and exactly compute robustness
	f<<timer.robustness_computation_time<<tab;

	f<<endl;

	f.close();

}

void StochasticLocalSearch::update_experiment_analysis_file(const Plan& p) {
#define tab		"\t"
#define sep		","

	string stat_file = string(gcmd_line.path) + string(gcmd_line.experiment_analysis_file);

	// Check if the file exists, if not then write the header
	ifstream f0(stat_file.c_str());
	bool file_exists = f0.good();
	f0.close();

	ofstream f;
	f.open(stat_file.c_str(), ios::out | ios::app);

	if (!file_exists) {
		f<<"#SEARCH PARAMETERS:"<<endl;
		f<<"#Initial robustness threshold:\t"<<initial_robustness_threshold<<endl;
		f<<"#Max restart:\t"<<max_restarts<<endl;
		f<<"#Max iterations:\t"<<max_iterations<<endl;
		f<<"#Initial depth bound:\t"<<initial_depth_bound<<endl;
		f<<"#Probes at depth:\t"<<probes_at_depth<<endl;
		f<<"#Neighborhood size:\t"<<neighborhood_size<<endl;
		f<<"#Initial fail bound:\t"<<fail_bound<<endl;
		f<<"#Min heuristic bias:\t"<<min_heuristic_bias<<endl;
		f<<"#Max heuristic bias:\t"<<max_heuristic_bias<<endl;
		f<<"#"<<endl;
		f<<"#RELAXED PLAN PARAMETERS:"<<endl;
		f<<"#Relaxed plan type: ";
		switch (RelaxedPlan::rp_types) {
		case RelaxedPlan::PURE_FF_RP:
			f<<"PURE_FF_RP";
			break;
		case RelaxedPlan::ROBUST_FF_RP:
			f<<"ROBUST_FF_RP";
			break;
		case RelaxedPlan::INCREMENTAL_ROBUSTNESS_RP:
			f<<"INCREMENTAL_ROBUSTNESS_RP";
			break;
		case RelaxedPlan::LOCALLY_INCREMENTAL_ROBUSTNESS_RP:
			f<<"LOCALLY_INCREMENTAL_ROBUSTNESS_RP";
			break;
		case RelaxedPlan::GREEDY_ROBUSTNESS_RP:
			f<<"GREEDY_ROBUSTNESS_RP";
			break;
		case RelaxedPlan::ALL_MOST_ROBUST_SUPPORTING_ACTIONS_RP:
			f<<"ALL_MOST_ROBUST_SUPPORTING_ACTIONS_RP";
			break;
		}
		f<<endl;
		f<<"#ignore_poss_del_in_rp:\t"<<RelaxedPlan::ignore_poss_del_in_rp<<endl;
		f<<"#use_lower_bound_in_rp:\t"<<RelaxedPlan::use_lower_bound_in_rp<<endl;
		f<<"#use_upper_bound_in_rp:\t"<<RelaxedPlan::use_upper_bound_in_rp<<endl;
		f<<"#use_robustness_threshold:\t"<<RelaxedPlan::use_robustness_threshold<<endl;
		f<<"#clauses_from_rpg_for_false_preconditions:\t"<<RelaxedPlan::clauses_from_rpg_for_false_preconditions<<endl;
		f<<"#current_actions_affect_candidate_action:\t"<<RelaxedPlan::current_actions_affect_candidate_action<<endl;
		f<<"#candidate_actions_affect_current_actions:\t"<<RelaxedPlan::candidate_actions_affect_current_actions<<endl;
		f<<"#"<<endl;
		f<<"domain"<<sep
		 <<"problem"<<sep
		 <<"inc_amount"<<sep
		 <<"plan_id"<<sep
		 <<"plan_length"<<sep
		 <<"plan_robustness"<<sep
		 <<"total_time"<<sep
		 <<"search_time"<<sep
		 <<"rp_time"<<sep
		 <<"clause_time"<<sep
		 <<"wmc_time"<<sep
		 <<"rp_success_rate"<<sep
		 <<"rpg_better_supporting_actions_rate"<<sep
		 <<"rp_increasing_robustness_rate"<<endl;
	}

	// Domain
	f<<gcmd_line.ops_file_name<<sep;

	// Problem
	f<<gcmd_line.fct_file_name<<sep;

	// The total number of possible preconditions and effects
	f<<gnum_possible_annotations<<sep;

	// The plan ID
	f<<p.id<<sep;

	// The plan's length
	f<<p.actions.size()<<sep;

	// The plan's robustness
	f<<p.robustness<<sep;

	// Total time
	f<<timer.total()<<sep;

	// Search time
	f<<timer.search_time<<sep;

	// Time to extract relaxed plans
	f<<timer.rp_time<<sep;

	// Time to construct clause sets
	f<<timer.clause_set_construction_time<<sep;

	// Time to compute robustness (WMC time)
	f<<timer.robustness_computation_time<<sep;

	// Successful rate of relaxed plans
	f<<double(RelaxedPlan::num_successful_rp_calls) / RelaxedPlan::num_rp_calls<<sep;

	// Better supporting actions rate in building RPG
	f<<double(RelaxedPlan::num_better_supporting_actions_found_in_rpg) / RelaxedPlan::num_better_supporting_action_checks_in_rpg<<sep;

	// Increasing relaxed plan robustness rate
	f<<double(RelaxedPlan::num_rp_robustness_increasing_check_success) / RelaxedPlan::num_rp_robustness_increasing_checks;

	f<<endl;

	f.close();

}

//bool StochasticLocalSearch::run() {
//	// Set up seed for the generator
//	generator.seed(static_cast<unsigned int>(std::time(0)));		// Initialize seed using the current time
//
//	// The two distribution for random number generation
//	boost::random::uniform_int_distribution<> int_dist;		// The range can change
//	boost::random::uniform_real_distribution<> real_dist;	// By default: [0, 1)
//
//	best_plan.actions.clear();
//	best_plan.robustness = 0;
//
//	CACHET_OUTPUT r;
//
//	// Number of plans found
//	int num_plans = 0;
//
//	// We try to find better plans in at most "max_restarts" restarts from the initial state
//	for (int i=0;i<max_restarts;i++) {
//
//		// When we're here: we have a current best robustness for the current best plan
//		// Restart from the initial state
//		StripsEncoding *e = new StripsEncoding(init);
//
////		//
////		cout<<"RESTART: i = "<<i<<endl;
////		//
//
//		for (int j=0;j<max_steps;j++) {
//
////			//
////			cout<<"\nMAX STEPS: j = "<<j<<endl;
////
////			cout<<"\nCURRENT STATE:"<<endl;
////			print_state(*e->get_last_state());
////			cout<<endl;
////			//
//
//			bool better_plan_found = false;
//
//			// (1) Get applicable actions
//			vector<int> applicable_actions;
//
//			// Build the relaxed plan from the current state, and get the set of applicable actions
//			double robustness_threshold;
//			if (RelaxedPlan::use_robustness_threshold)
//				robustness_threshold = best_plan.robustness;
//			else
//				robustness_threshold = 0;
//
//			RelaxedPlan rp(e, init, goals, robustness_threshold);
//			pair<int, double> rp_info;
//			// If we cannot build the relaxed plan, it means that no plan exists from this state
//			// Restart from the initial state
//			if (!rp.extract(rp_info))
//				break;
//
//			get_applicable_actions(e->get_last_state(), applicable_actions, &rp, Search::FF_helpful_actions);
//
//			// If there is no applicable action, we're done with this iteration
//			// Restart from the initial state
//			if (applicable_actions.size() <= 0)
//				break;
//
////			//
////			cout<<"\nAPPLICABLE ACTIONS:"<<endl;
////			for (int k=0;k<applicable_actions.size();k++) {
////				print_op_name(applicable_actions[k]);
////				cout<<endl;
////			}
////			//
//
//#ifndef NDEBUG
//		const State& s0 = *e->get_last_state();
//#endif
//
//			// (2) For each applicable action, quickly check if we have a better plan with it
//			for (int k=0;k<applicable_actions.size();k++) {
//
//				assert(same_state(s0, *e->get_last_state()));
//
////				//
////				cout<<"\n>>>>> k = "<<k<<": ";
////				print_op_name(applicable_actions[k]);
////				cout<<endl;
////
////				cout<<"\nState:"<<endl;
////				print_state(*e->get_last_state());
////				//
//
//				// If the estimated robustness of "plan prefix + this action" wrt the goals is
//				// not less than the current best robustness, then we check its exact robustness
//				double lower, upper;
//				e->append(applicable_actions[k]);
//				ClauseSet all_clauses;
//				e->get_clauses(all_clauses);
//
//				// Check if the goals present in the last state, and get its clause set
//				ClauseSet goal_clauses;
//				bool goals_present = e->check_goals(goals, goal_clauses);
//
//				// We no longer need the appended action
//				e->remove_last();
//
//				// If there exists a goal not present in the last state, then this extended plan
//				// prefix cannot be a solution (since the robustness must be 0)
//				// Remove the action just appended, and continue with the next candidate action.
//				if (!goals_present) {
//					continue;
//				}
//
//				// Add goal clauses into the set of all clauses
//				all_clauses.add_clauses(goal_clauses);
//
//				// Evaluate its lower and upper bound
//				lower = all_clauses.lower_wmc();
//				upper = all_clauses.upper_wmc();
//
//				// CHECKING CONDITION 1: If the lower bound is already better than the current robustness
//				// then a new plan found!
//				if (lower >= best_plan.robustness) {
//
//					all_clauses.wmc(r);
//					assert(lower <= r.prob);
//
//					// Extend the plan prefix
//					e->extend_plan_prefix(applicable_actions[k]);
//
//					// Record the new best plan
//					best_plan.actions = e->get_actions();	// Note: plan_prefix = all actions!
//					best_plan.robustness = r.prob;
//
//					better_plan_found = true;
//
//					// PRINT OUT THIS PLAN
//					cout<<"PLAN "<<num_plans++<<endl;
//					cout<<"Number of actions:"<<best_plan.actions.size()<<endl;
//					for (int i=0;i<best_plan.actions.size();i++) {
//						int op = best_plan.actions[i];
//						print_op_name(op);
//						cout<<endl;
//					}
//					cout<<"Robustness: "<<best_plan.robustness<<endl;
//
//					break;	// out of considering other applicable actions
//				}
//
//				// CHECKING CONDITION 2: If the upper bound is better than the current robustness,
//				// then we also need to check the exact robustness
//				if (upper >= best_plan.robustness) {
//
//					all_clauses.wmc(r);
//					assert(upper >= r.prob);
//
//					// Only accept new plan if its exact robustness is better than the current value
//					if (r.prob > best_plan.robustness) {
//						// Extend the plan prefix
//						e->extend_plan_prefix(applicable_actions[k]);
//
//						// Record the new best plan
//						best_plan.actions = e->get_actions();	// Note: plan_prefix = all actions!
//						best_plan.robustness = r.prob;
//
//						better_plan_found = true;
//
//						// PRINT OUT THIS PLAN
//						cout<<"PLAN "<<num_plans++<<endl;
//						cout<<"Number of actions:"<<best_plan.actions.size()<<endl;
//						for (int i=0;i<best_plan.actions.size();i++) {
//							int op = best_plan.actions[i];
//							print_op_name(op);
//							cout<<endl;
//						}
//						cout<<"Robustness: "<<best_plan.robustness<<endl;
//
//						break;	// out of considering other applicable actions
//					}
//				}
//			}
//
//			// If we find a better plan, we are done for this improvement iteration
//			// We will restart from the initial state for finding even better plan
//			if (better_plan_found)
//				break; // out of "max_steps" loop
//
//			// (3) If we're here, none of the candidate actions gives better plan
//			// Evaluate the neighbor actions to make local move
//			int best_action;
//			double best_robustness = 0;
//			int best_length = INT_MAX;
//			for (int k=0;k<applicable_actions.size();k++) {
//
//				// Append the action
//				e->append(applicable_actions[k]);
//
//				// Extract the relaxed plan
//				double robustness_threshold;
//				if (RelaxedPlan::use_robustness_threshold)
//					robustness_threshold = best_plan.robustness;
//				else
//					robustness_threshold = 0;
//
//				RelaxedPlan rp2(e, e->get_last_state(), goals, robustness_threshold);
//				pair<int, double> rp2_info;
//				rp2.extract(rp2_info);
//
//				// Record the best action
////				if (best_robustness < rp2_info.second) {
////					best_action = applicable_actions[k];
////					best_robustness = rp2_info.second;
////				}
//
//				if (best_length > rp2_info.first) {
//					best_action = applicable_actions[k];
//					best_length = rp2_info.first;
//				}
//
//				// Remove this action to consider the next one
//				e->remove_last();
//			}
//
////			//
////			cout<<"\n\nBEST ACTION: ";
////			print_op_name(best_action);
////			cout<<endl;
////			//
//
//			// (4) Toss the coin to make local move
//			// if "random < noise" then move to a random neighbor
//			// otherwise, move to the best one
//			int next_action;
//			double random = real_dist(generator);
//
////			//
////			cout<<endl<<"Random: "<<random<<endl;
////			//
//
//			if (random < noise) {
//				// Now we randomly select on neighbor
//				int selected_neighbor = int_dist(generator,
//						boost::random::uniform_int_distribution<>::param_type(0, applicable_actions.size()-1));
//				next_action = applicable_actions[selected_neighbor];
//			}
//			else {
//				next_action = best_action;
//			}
//
////			//
////			cout<<"*** NEXT ACTION: ";
////			print_op_name(next_action);
////			cout<<endl;
////			//
//
//			// (5) Extend the plan prefix with the next action
//			e->extend_plan_prefix(next_action);
//
////			//
////			cout<<"NEW STATE:"<<endl;
////			print_state(*e->get_last_state());
////			cout<<endl;
////			cout<<"END OF j = "<<j<<endl;
////			//
//		}
//
//		// Release memory
//		delete e;
//	}
//
//	if (best_plan.actions.size() > 0 && best_plan.robustness > 0) {
//		cout<<"SOLUTION PLAN:"<<endl;
//		for (int i=0;i<best_plan.actions.size();i++) {
//			int op = best_plan.actions[i];
//			print_op_name(op);
//			cout<<endl;
//		}
//		cout<<"ROBUSTNESS: "<<best_plan.robustness<<endl;
//		return true;
//	}
//	else {
//		cout<<"PLAN NOT FOUND!"<<endl;
//	}
//
//	return false;
//}







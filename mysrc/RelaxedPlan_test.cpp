/*
 * RelaxedPlan_test.cpp
 *
 *  Created on: Feb 23, 2013
 *      Author: tanguye1
 */

#include "RelaxedPlan.h"
#include "Helpful.h"
#include <string>
#include <iostream>
#include <fstream>
#include <sstream>
using namespace std;

void test_relaxed_plan(string partial_sol_file, State *initial_state, State* goal_state) {

	string file_path = string(gcmd_line.path) + partial_sol_file;

	ifstream f(file_path.c_str(),ifstream::in );
	if (!f.good()) {
		printf("Error opening solution file: %s! File %s, line %d.\n",file_path.c_str(),__FILE__,__LINE__);
		exit(1);
	}

	char buff[MAX_LENGTH];
	string domain, problem, sol_file;
	vector<string> plan_actions;

	// Store the solution file name
	sol_file = file_path;

	// Domain file
	f.getline(buff,MAX_LENGTH);
	istringstream s1(string(buff),istringstream::in);
	s1>>domain;

	// Problem file
	f.getline(buff,MAX_LENGTH);
	istringstream s2(string(buff),istringstream::in);
	s2>>problem;

	// Number of plan actions
	int num_actions;
	f.getline(buff,MAX_LENGTH);
	istringstream s5(string(buff),istringstream::in);
	s5>>num_actions;

	// Plan actions
	for(int i=0;i<num_actions;i++) {
		string the_action;
		f.getline(buff,MAX_LENGTH);
		plan_actions.push_back(buff);
	}

	// Close the solution file
	f.close();

	StripsEncoding e(initial_state);
	for (int i=0;i<plan_actions.size();i++) {
		int op = find_action(plan_actions[i]);
		if (!e.append(op)) {
			cout<<"Action "<<plan_actions[i]<<" not found! File "<<__FILE__<<", line "<<__LINE__<<endl;
			exit(1);
		}
	}
	const vector<int> actions = e.get_actions();
	const vector<State*> states = e.get_states();
	cout<<"Initial State:"<<endl;
	print_state(states[0]);
	cout<<endl<<endl;
	for (int i=0;i<actions.size();i++) {
		cout<<"Action "<<actions[i]<<": ";
		print_op_name(actions[i]);
		cout<<endl;
		cout<<"State ["<<i+1<<"]:"<<endl;
		print_state(states[i+1]);
		cout<<endl<<endl;
	}

	// Creating relaxed plan
	cout<<"==== RELAXED PLAN ===="<<endl;
	RelaxedPlan rp(&e, goal_state);
	cout<<"Current state:"<<endl;
	const State& current = rp.get_current_state();
	print_state(current);

	rp.initialize_fact_layer();
}

void print_fact_layer(RelaxedPlan::FactLayer& fact_layer) {
	for (int ft = 0; ft < gnum_ft_conn; ft++) {
		if (fact_layer.find(ft) == fact_layer.end())
			continue;

		RelaxedPlan::FactNode& node = fact_layer[ft];

	}
}

void print_fact_node(RelaxedPlan::FactNode& node) {
	cout<<"Clauses: ";
	for (ClauseSet::const_iterator itr = node.clauses.begin(); itr != node.clauses.end(); itr++) {

	}
}







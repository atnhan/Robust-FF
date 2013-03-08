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
	int test = 2;
	if (test == 1) {
		cout<<"==== RELAXED PLAN ===="<<endl;
		RelaxedPlan rp(&e, goal_state);
		cout<<endl<<"Current state:"<<endl;
		const State& current = rp.get_current_state();
		print_state(current);

		cout<<endl;
		cout<<"Encoding: "<<e.get_clauses()<<endl;

		rp.initialize_fact_layer();
		cout<<endl<<"FIRST FACT LAYER:"<<endl<<endl;
		print_fact_layer(*(rp.P[0]));

		rp.grow_action_layer();
		cout<<endl<<"FIRST ACTION LAYER:"<<endl<<endl;
		print_action_layer(*(rp.A[0]));

		rp.grow_fact_layer();
		cout<<endl<<"SECOND FACT LAYER:"<<endl<<endl;
		print_fact_layer(*(rp.P[1]));
	}
	else if (test == 2) {
		cout<<"==== RELAXED PLAN ===="<<endl;
		RelaxedPlan rp(&e, goal_state);
		cout<<endl<<"Current state:"<<endl;
		const State& current = rp.get_current_state();
		print_state(current);

		cout<<endl;
		cout<<"Encoding: "<<e.get_clauses()<<endl;

		rp.build_relaxed_planning_graph(10);
		print_relaxed_planning_graph(rp);
	}
}

void print_fact_layer(RelaxedPlan::FactLayer& fact_layer) {
	for (int ft = 0; ft < gnum_ft_conn; ft++) {
		if (fact_layer.find(ft) == fact_layer.end())
			continue;
		cout<<"--- ["<<ft<<"]";
		print_ft_name(ft);
		cout<<endl;
		RelaxedPlan::FactNode& node = fact_layer[ft];
		print_fact_node(node);
		cout<<endl<<endl;
	}
}

void print_action_layer(RelaxedPlan::ActionLayer& action_layer) {
	for (int op = 0; op <gnum_op_conn; op++) {
		if (action_layer.find(op) == action_layer.end())
			continue;
		cout<<"--- ["<<op<<"]";
		print_op_name(op);
		cout<<endl;
		cout<<"Known PCs: ";
		for (int i = 0; i < gop_conn[op].num_E; i++) {
			int ef = gop_conn[op].E[i];
			for (int j=0;j<gef_conn[ef].num_PC;j++) {
				cout<<gef_conn[ef].PC[j]<<" ";
			}
		}
		cout<<endl;

		RelaxedPlan::ActionNode& node = action_layer[op];
		print_action_node(node);
		cout<<endl<<endl;
	}
}

void print_relaxed_planning_graph(RelaxedPlan& rp) {
	if (rp.P.size() <= 0) {
		cout<<"EMPTY RGP"<<endl;
		return;
	}

	int i = 0;
	bool new_layer = true;
	while (new_layer) {
		new_layer = false;
		if (i < rp.P.size()) {
			cout<<"==== FACT LAYER "<<i<<"===="<<endl;
			print_fact_layer(*(rp.P[i]));
			new_layer = true;
		}
		if (i < rp.A.size()) {
			cout<<"==== ACTION LAYER "<<i<<"===="<<endl;
			print_action_layer(*(rp.A[i]));
			new_layer = true;
		}
		i++;
	}
}

void print_fact_node(RelaxedPlan::FactNode& node) {
	cout<<"Clauses: ";
	cout<<node.clauses<<endl;
	cout<<"Robustness: "<<node.robustness<<endl;
	cout<<"Supporting action: "<<node.best_supporting_action;
}

void print_action_node(RelaxedPlan::ActionNode& node) {
	cout<<"Clauses: ";
	cout<<node.clauses<<endl;
	cout<<"Robustness: ";
	cout<<node.robustness;
}





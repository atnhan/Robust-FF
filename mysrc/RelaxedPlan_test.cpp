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
#include <assert.h>
using namespace std;

#define SEP		"\t"

void test_relaxed_planning_graph(string partial_sol_file, State *initial_state, State* goal_state) {

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
		e.append(op);
	}
	const vector<int>& actions = e.get_actions();
	const vector<State*>& states = e.get_states();
	cout<<"Initial State:"<<endl;
	print_state(*states[0]);
	cout<<endl<<endl;
	for (int i=0;i<actions.size();i++) {
		cout<<"Action "<<actions[i]<<": ";
		print_op_name(actions[i]);
		cout<<endl;
		cout<<"State ["<<i+1<<"]:"<<endl;
		print_state(*states[i+1]);
		cout<<endl<<endl;
	}

	// Creating relaxed planning graph
	int test = 2;
	if (test == 1) {
		cout<<"==== RELAXED PLAN ===="<<endl;
		RelaxedPlan rp(&e, e.get_last_state(), goal_state);
		cout<<endl<<"Current state:"<<endl;
		const State& current = rp.get_current_state();
		print_state(current);

		cout<<endl;
		ClauseSet clauses;
		e.get_clauses(clauses);
		cout<<clauses<<endl;

		int style = 1;
		rp.initialize_fact_layer();
		cout<<endl<<"FIRST FACT LAYER:"<<endl<<endl;
		print_fact_layer(*(rp.P[0]), style);

		rp.grow_action_layer();
		cout<<endl<<"FIRST ACTION LAYER:"<<endl<<endl;
		print_action_layer(*(rp.A[0]), style);

		rp.grow_fact_layer();
		cout<<endl<<"SECOND FACT LAYER:"<<endl<<endl;
		print_fact_layer(*(rp.P[1]), style);
	}
	else if (test == 2) {
		cout<<"==== RELAXED PLAN ===="<<endl;
		RelaxedPlan rp(&e, e.get_last_state(), goal_state);
		cout<<endl<<"Current state:"<<endl;
		const State& current = rp.get_current_state();
		print_state(current);

		cout<<endl;
		ClauseSet clauses;
		e.get_clauses(clauses);
		cout<<clauses<<endl;

		rp.build_relaxed_planning_graph();
		print_relaxed_planning_graph(rp, 2, 3);
	}
	else if (test == 3) {
		cout<<"==== RELAXED PLAN ===="<<endl;
		RelaxedPlan rp(&e, e.get_last_state(), goal_state);
		cout<<endl<<"Current state:"<<endl;
		const State& current = rp.get_current_state();
		print_state(current);

		cout<<endl;
		ClauseSet clauses;
		e.get_clauses(clauses);
		cout<<clauses<<endl;

		rp.build_relaxed_planning_graph();
		print_action_through_layers(rp, 1037, 3);
	}
}

void test_relaxed_plan(std::string partial_sol_file, State *initial_state, State* goal_state) {

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
		e.append(op);
	}
	const vector<int>& actions = e.get_actions();
	const vector<State*>& states = e.get_states();
	cout<<"Initial State:"<<endl;
	print_state(*states[0]);
	cout<<endl<<endl;
	for (int i=0;i<actions.size();i++) {
		cout<<"Action "<<actions[i]<<": ";
		print_op_name(actions[i]);
		cout<<endl;
		cout<<"State ["<<i+1<<"]:"<<endl;
		print_state(*states[i+1]);
		cout<<endl<<endl;
	}

	RelaxedPlan rp(&e, e.get_last_state(), goal_state);
	cout<<endl<<"Current state:"<<endl;
	const State& current = rp.get_current_state();
	print_state(current);

	cout<<endl;
	cout<<"CLAUSES: "<<endl;
	ClauseSet clauses;
	e.get_clauses(clauses);
	cout<<clauses<<endl;

	cout<<"GOALS:"<<endl;
	print_state(*goal_state);
	cout<<endl;

	cout<<"Extracting relaxed plan..."<<endl;
	pair<int, double> rp_info;
	rp.extract(rp_info);
	cout<<"Done. Relaxed plan length: "<<rp_info.first<<", estimated robustness: "<<rp_info.second<<endl;

	cout<<"******* RP-STEPS ********"<<endl<<endl;
	for (int i=0;i<rp.length()-1;i++) {
		print_rp_step(rp, i);
		cout<<endl<<endl;
	}
}

void print_rp_step(RelaxedPlan& r, int step) {
	assert(step >= 0 && step < r.length()-1);

	RelaxedPlan::RELAXED_PLAN::const_iterator itr = r.rp.begin();
	int i = 0;
	for (; i < step; i++) itr++;

	cout<<">> STEP "<<step<<endl;
	cout<<"STATE:"<<endl;
	for (int ft = 0; ft < gnum_ft_conn; ft++) {
		if ((*itr)->s.find(ft) == (*itr)->s.end())
			continue;
		cout<<"F"<<ft<<" ";
	}
	cout<<endl;
	cout<<"ACTION: "<<(*itr)->a<<endl;
	assert(gop_conn[(*itr)->a].num_E == 1);
	int ef = gop_conn[(*itr)->a].E[0];

	for (int j=0;j<gef_conn[ef].num_PC;j++) {
		int p = gef_conn[ef].PC[j];
		cout<<" Precond: "<<p;
		if (!r.in_rp_state(p, (*itr)->s))
			cout<<" *";
		if ((*itr)->pre_clauses.find(p) != (*itr)->pre_clauses.end())
			cout<<" Clauses: "<<(*((*itr)->pre_clauses[p]));
		cout<<endl;
	}

	for (int j=0;j<gef_conn[ef].num_poss_PC;j++) {
		int p = gef_conn[ef].poss_PC[j];
		cout<<" Poss precond: "<<p;
		if ((*itr)->poss_pre_clauses.find(p) != (*itr)->poss_pre_clauses.end())
			cout<<" Clauses: "<<(*((*itr)->poss_pre_clauses[p]));
		cout<<endl;
	}

	for (int j=0;j<gef_conn[ef].num_A;j++) {
		int p = gef_conn[ef].A[j];
		cout<<" Add: "<<p;
		cout<<endl;
	}

	for (int j=0;j<gef_conn[ef].num_poss_A;j++) {
		int p = gef_conn[ef].poss_A[j];
		cout<<" Poss add: "<<p;
		cout<<endl;
	}

	for (int j=0;j<gef_conn[ef].num_D;j++) {
		int p = gef_conn[ef].D[j];
		cout<<" Del: "<<p;
		cout<<endl;
	}

	for (int j=0;j<gef_conn[ef].num_poss_D;j++) {
		int p = gef_conn[ef].poss_D[j];
		cout<<" Poss del: "<<p;
		cout<<endl;
	}
}

void print_fact_layer(RelaxedPlan::FactLayer& fact_layer, int style) {
	int count = 0;
	int L = 10;
	switch (style) {
	case 0:
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
		break;
	case 1:
		for (int ft = 0; ft < gnum_ft_conn; ft++) {
			if (fact_layer.find(ft) == fact_layer.end())
				continue;
			cout<<ft<<SEP;
			if (++count % L == 0) cout<<endl;
		}
		cout<<endl;
		break;
	case 2:
		for (int ft = 0; ft < gnum_ft_conn; ft++) {
			if (fact_layer.find(ft) == fact_layer.end())
				continue;
			cout<<"F"<<ft;
			RelaxedPlan::FactNode& node = fact_layer[ft];
			cout<<"["<<node.best_supporting_action<<","<<node.best_robustness<<"]";
			cout<<node.best_clauses<<SEP;
			if (++count % L == 0) cout<<endl;
		}
		cout<<endl;
		break;

	default:
		cout<<"Options not found!"<<endl;
		exit(1);
	}

}

void print_action_layer(RelaxedPlan::ActionLayer& action_layer, int style) {
	int count = 0;
	int L = 10;
	switch (style) {
	case 0:
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

		break;
	case 1:

		for (int op = 0; op <gnum_op_conn; op++) {
			if (action_layer.find(op) == action_layer.end())
				continue;
			cout<<op<<"=> ";
			cout<<"PCs: ";
			for (int i = 0; i < gop_conn[op].num_E; i++) {
				int ef = gop_conn[op].E[i];
				for (int j=0;j<gef_conn[ef].num_PC;j++) {
					cout<<gef_conn[ef].PC[j]<<" ";
				}
			}
			cout<<";;;; Poss-PCs: ";
			for (int i = 0; i < gop_conn[op].num_E; i++) {
				int ef = gop_conn[op].E[i];
				for (int j=0;j<gef_conn[ef].num_poss_PC;j++) {
					cout<<gef_conn[ef].poss_PC[j]<<" ";
				}
			}
			cout<<endl;
		}
		break;
	case 2:
		for (int op = 0; op <gnum_op_conn; op++) {
			if (action_layer.find(op) == action_layer.end())
				continue;
			cout<<"A"<<op<<"["<<action_layer[op].robustness<<"]";
			cout<<"{";
			for (int i = 0; i < gop_conn[op].num_E; i++) {
				int ef = gop_conn[op].E[i];
				for (int j=0;j<gef_conn[ef].num_PC;j++) {
					cout<<gef_conn[ef].PC[j];
					if (j < gef_conn[ef].num_PC - 1) cout<<" ";
				}
			}
			cout<<"} ";
			cout<<"*{";
			for (int i = 0; i < gop_conn[op].num_E; i++) {
				int ef = gop_conn[op].E[i];
				for (int j=0;j<gef_conn[ef].num_poss_PC;j++) {
					cout<<gef_conn[ef].poss_PC[j];
					if (j < gef_conn[ef].num_poss_PC - 1) cout<<" ";
				}
			}
			cout<<"} ";

			if (++count % L == 0) cout<<endl;
		}
		break;

	case 3:

		for (int op = 0; op <gnum_op_conn; op++) {
			if (action_layer.find(op) == action_layer.end())
				continue;
			cout<<"A"<<op<<"["<<action_layer[op].robustness<<"]";
			cout<<action_layer[op].clauses<<SEP;
			if (++count % L == 0) cout<<endl;
		}
		break;

	default:
		cout<<"Options not found!"<<endl;
		exit(1);
	}
}

void print_relaxed_planning_graph(RelaxedPlan& rp, int style_f, int style_a) {
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
			print_fact_layer(*(rp.P[i]), style_f);
			cout<<endl;
			new_layer = true;
		}
		if (i < rp.A.size()) {
			cout<<"==== ACTION LAYER "<<i<<"===="<<endl;
			print_action_layer(*(rp.A[i]), style_a);
			cout<<endl<<endl;
			new_layer = true;
		}
		i++;
	}
	cout<<endl<<endl<<"END OF RPG. LENGTH: "<<rp.P.size()<<endl;
}

void print_fact_node(RelaxedPlan::FactNode& node) {
	cout<<"Clauses: ";
	cout<<node.best_clauses<<endl;
	cout<<"Robustness: "<<node.best_robustness<<endl;
	cout<<"Supporting action: "<<node.best_supporting_action;
}

void print_action_node(RelaxedPlan::ActionNode& node) {
	cout<<"Clauses: ";
	cout<<node.clauses<<endl;
	cout<<"Robustness: ";
	cout<<node.robustness;
}

void print_action_through_layers(RelaxedPlan& rp, int op, int num_layers) {
	assert(num_layers <= rp.P.size());
	assert(op >= 0 && op < gnum_op_conn);

	for (int i = 0; i < num_layers; i++) {
		cout<<"==========  LAYER "<<i<<"=========="<<endl;
		if (rp.A[i]->find(op) == rp.A[i]->end())
			continue;
		cout<<"PCs: ";
		for (int j=0;j<gop_conn[op].num_E;j++) {
			int ef = gop_conn[op].E[j];
			for (int k=0;k<gef_conn[ef].num_PC;k++) {
				int ft = gef_conn[ef].PC[k];
				cout<<ft<<"["<<(*(rp.P[i]))[ft].best_supporting_action<<":";
				if ((*(rp.P[i]))[ft].best_supporting_action != NOOP)
					print_op_name((*(rp.P[i]))[ft].best_supporting_action);
				cout<<","<<(*(rp.P[i]))[ft].best_robustness<<"]"<<(*(rp.P[i]))[ft].best_clauses<<SEP;
			}
		}
		cout<<endl;
		cout<<"Poss-PCs: ";
		for (int j=0;j<gop_conn[op].num_E;j++) {
			int ef = gop_conn[op].E[j];
			for (int k=0;k<gef_conn[ef].num_poss_PC;k++) {
				int ft = gef_conn[ef].poss_PC[k];
				cout<<ft<<"["<<(*(rp.P[i]))[ft].best_supporting_action<<":";
				if ((*(rp.P[i]))[ft].best_supporting_action != NOOP)
					print_op_name((*(rp.P[i]))[ft].best_supporting_action);
				cout<<","<<(*(rp.P[i]))[ft].best_robustness<<"]"<<(*(rp.P[i]))[ft].best_clauses<<SEP;
			}
		}
		cout<<endl;
		cout<<"Action: "<<(*(rp.A[i]))[op].clauses<<"["<<(*(rp.A[i]))[op].robustness<<"]";
		cout<<endl<<endl;
	}
}





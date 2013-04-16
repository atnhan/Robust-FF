/*
 * StripsEncoding_test.cpp
 *
 *  Created on: Feb 14, 2013
 *      Author: tuan
 */

#include "StripsEncoding.h"
#include "Helpful.h"
#include <string>
#include <fstream>
#include <sstream>
#include <iostream>
using namespace std;

/*
 * Read a solution file, and compute its robustness
 */
void evaluate_plan_robustness(string filename, State *initial_state, State* goal_state) {
	string file_path = string(gcmd_line.path) + filename;

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

	/***********************************************/
	StripsEncoding e(initial_state);
	for (int i=0;i<plan_actions.size();i++) {
		int op = find_action(plan_actions[i]);
		e.append(op);
	}

//	const ClauseSet& cs = e.get_clauses();
//	cout<<"Clause Set:"<<endl;
//	cout<<e.get_clauses()<<endl;

//	const vector<int> actions = e.get_actions();
//	const vector<State*> states = e.get_states();
//	cout<<"Initial State:"<<endl;
//	print_state(states[0]);
//	cout<<endl<<endl;
//	for (int i=0;i<actions.size();i++) {
//		cout<<"Action "<<actions[i]<<": ";
//		print_op_name(actions[i]);
//		cout<<endl;
//		cout<<"State ["<<i+1<<"]:"<<endl;
//		print_state(states[i+1]);
//		cout<<endl<<endl;
//	}
//
//	if (goal_state) {
//
//	}

//	int satresult;
//	double sat_prob;
//	double rtime;
//	e.evaluate_robustness(satresult,sat_prob, rtime);



}

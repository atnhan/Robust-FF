/*
 * StripsEncoding.cpp
 *
 *  Created on: Feb 7, 2013
 *      Author: tuan
 */

#include "StripsEncoding.h"
#include "Helpful.h"
#include <iostream>
#include <assert.h>
#include <string.h>
using namespace std;

extern int result_to_dest( State *dest, State *source, int op );
extern void source_to_dest( State *dest, State *source );
extern void make_state( State *S, int n );
extern int gnum_possible_annotations;

StripsEncoding::StripsEncoding(State *init) {
	State *init_state = (State*) calloc(1, sizeof(State));
	make_state(init_state, gnum_ft_conn);
	init_state->max_F = gnum_ft_conn;
	source_to_dest(init_state, init);
	this->states.push_back(init_state);
	this->goal = 0;
	this->clauses = new ClauseSet;
}

StripsEncoding::~StripsEncoding() {
	for (int i=0;i<this->states.size();i++)
		if (this->states[i])
			free(this->states[i]);

	if (goal)
		free(goal);

	if (clauses)
		delete clauses;
}

bool StripsEncoding::append(int action) {
	if (action < 0 || action >= gnum_op_conn) {
		return false;
	}
	State *current_state = this->states[this->actions.size()];
	State *resulting_state = (State*) calloc(1, sizeof(State));
	make_state(resulting_state, gnum_ft_conn);
	resulting_state->max_F = gnum_ft_conn;
	result_to_dest(resulting_state, current_state, action);
	this->states.push_back(resulting_state);
	this->actions.push_back(action);

	// Update the clause set
	int level = this->actions.size()-1;
	assert(gop_conn[action].num_E == 1);
	int n_ef = gop_conn[action].E[0];
	// First, for known preconditions
	for (int i=0;i<gef_conn[n_ef].num_PC;i++) {
		int ft = gef_conn[n_ef].PC[i];
		ClauseSet cs;
		bool success = supporting_constraints(ft, level, cs);
		assert(success);
		// Update the clause set by adding new clauses
		add_clauses(cs);
	}

	// Second, for possible preconditions
	for (int i = 0; i < gef_conn[n_ef].num_poss_PC;i++) {
		int ft = gef_conn[n_ef].poss_PC[i];
		int bvar = get_bool_var(ft, action, POSS_PRE);
		assert(bvar > 0);

		if (is_in_state(ft, this->states[level])) {
			ClauseSet cs;
			bool success = supporting_constraints(ft, level, cs);
			assert(success);

			// Since "ft" is possible precondition, the above constraints need only when it is realized
			// as a precondition of the action.
			ClauseSet temp_cs;
			for (ClauseSet::const_iterator itr = cs.cbegin(); itr != cs.cend(); itr++) {
				Clause c = *itr;
				c.insert(-bvar);
				temp_cs.add_clause(c);
			}

			// Update the clause set by adding new ones
			add_clauses(temp_cs);
		}
		else {	// If "ft" is false at the level, we don't need to find supporting clauses (i.e., they are empty)
				// This action is applicable only when this possible precondition is not realized
			Clause c;
			c.insert(-bvar);
			clauses->add_clause(c);
		}
	}

	return true;
}

int StripsEncoding::get_confirmed_level(int ft,int level)
{
	int n = actions.size();
	if (level < 0 && level > n)
	{
		return -1;
	}

	// If "ft" is after the last action, then we only check if it is add or delete effect of the last action
	if (level == n) {
		if (is_add(ft, actions[n-1]) || is_del(ft, actions[n-1]))
			return n;
	}
	// Otherwise, we also check if it is precondition of the action at some previous level
	for (int l = level; l >= 1; l--) {
		// Check if it is a precondition
		if (is_pre(ft, actions[l])) return l;
		// Check if it is add or delete effect
		if (is_add(ft, actions[l-1]) || is_del(ft, actions[l-1])) return l;
	}

	// All fact values are "confirmed at the initial state
	return 0;
}

// Construct the set of clauses for TRUE truth value of a fact at a level
bool StripsEncoding::supporting_constraints(int ft, int level, ClauseSet& clauses) {
	if (clauses.size())
		clauses.clear();

	if (level < 0 || level > this->actions.size())
		return false;

	if (!is_in_state(ft, this->states[level]))	// There's no way to satisfy this fact
		return false;

	int confirmed_level = get_confirmed_level(ft, level);

	// If "ft" is false at the confirmed level, we first need "establishment constraints"
	if (!is_in_state(ft, this->states[confirmed_level])) {
		Clause c;
		int k;
		for (k = confirmed_level; k < level; k++)
			if (is_poss_add(ft, this->actions[k])) {
				int bvar = get_bool_var(ft, this->actions[k], POSS_ADD);
				c.insert(bvar);
			}
		clauses.add_clause(c);
	}

	// Protection constraints
	for (int k = confirmed_level; k < level; k++)
		if (is_poss_del(ft, this->actions[k])) {
			Clause c;
			int bvar = get_bool_var(ft, this->actions[k], POSS_DEL);
			c.insert(-bvar);

			for (int j=k+1;j<level;j++) {
				if (is_poss_add(ft, this->actions[j])) {
					bvar = get_bool_var(ft, this->actions[j], POSS_ADD);
					c.insert(bvar);
				}
			}
			clauses.add_clause(c);
		}

	return true;

}

// Add new clause or clause set
void StripsEncoding::add_clauses(const ClauseSet& cs) {
	clauses->add_clauses(cs);
}

void StripsEncoding::add_clause(const Clause& c) {
	clauses->add_clause(c);
}

bool StripsEncoding::check_goals(State *goals, ClauseSet& cs) {
	if (!goals) {
		cs.clear();
		return false;
	}
	int n = this->actions.size();
	State *current_state = this->states[n];
	for (int i=0;i<goals->num_F;i++) {
		int ft = goals->F[i];
		if (!is_in_state(ft, current_state)) {
			cs.clear();
			return false;
		}
		ClauseSet ft_clauses;
		bool success = supporting_constraints(ft, n, ft_clauses);
		assert(success);
		// Add new clauses into "cs"
		cs.add_clauses(ft_clauses);
	}

	// NOTE: if "cs" is empty here, it means that goals are certainly satisfied?
	return true;
}

bool StripsEncoding::evaluate_robustness(int& satresult, double& sat_prob, double& rtime, State *goals) {
	if (clauses->size() == 0) {
		satresult = 0;
		sat_prob = 0;
		rtime = 0;
		return false;
	}
	float r = 0;

	// Check the model counting file
	FILE *f;
	if ((f = fopen ("./cachet", "r")) == NULL)
	{
		printf("Model counting software not found! File %s, line %d.\n",__FILE__,__LINE__);
		exit(1);
	}
	fclose(f);

	string filename = "CNF.txt";

	write_cnf_file(filename.c_str(), goals);	// Current implementation: ignore weights.
	string cmd = "./cachet " + filename + " -q";

	// Calling the model counting and write the answer to "A" file
	system(cmd.c_str());

	// Read the answer file to get the resulting information
	read_weighted_model_counting_answer_file(satresult,sat_prob,rtime);

	// The clause must be satisfiable
	if (satresult != 2)
	{
		//printf("The clause set must not be unsatisfiable! File %s, line %d.\n",__FILE__,__LINE__);
		//exit(1);
		return false;
	}

	if (gcmd_line.display_info == 1000)
	{
		printf("\nMODEL COUNTING RESULT:\n");
		printf(">> CNF file: %s\n",filename.c_str());
		printf(">> Number of boolean variables: %d\n",gnum_possible_annotations);
		printf(">> Robustness value: %f\n",sat_prob);
		printf(">> Counting time: %lf\n",rtime);
		fflush(stdout);
	}
	return true;
}

bool StripsEncoding::write_cnf_file(const char* filename, State *goals, std::vector<float> *weights) {
	FILE *CNF;
	if ( (CNF = fopen(filename,"w")) == NULL )
	{
		printf("Can not open CNF file! File %s, line %d.\n",__FILE__,__LINE__);
		exit(1);
	}

	ClauseSet goal_clauses;
	if (goals) {
		bool success = check_goals(goals, goal_clauses);
		if (!success) {
			fclose(CNF);
			return false;
		}
	}

	fprintf(CNF, "c clauses representing causal proof of plan correctness\n");
	fprintf(CNF, "p cnf %d %d\n", gnum_possible_annotations, clauses->size() + goal_clauses.size());

	// Write the non-uniform weights of (positive) literals
	if (weights)
	{
		if (weights->size() != gnum_possible_annotations + 1)
		{
			printf("ERROR: Weigh vector is wrong! File %s, line %d.\n",__FILE__,__LINE__);
			exit(1);
		}

		for (int i=1;i<=weights->size()-1;i++)
		{
			// We don't need to write the weight 0.5 into the CNF file
			if ((*weights)[i] == 0.5)
				continue;
			fprintf(CNF,"w %d %f\n",i,(*weights)[i]);
		}
	}

	// Write each clause to the file, ending with 0
	for (ClauseSet::const_iterator itr = clauses->cbegin(); itr != clauses->cend(); itr++) {
		const Clause& c = *itr;
		for (Clause::const_iterator itr2 = c.begin(); itr2 != c.end(); itr2++) {
			assert(*itr2 > 0);
			fprintf(CNF, "%d ", *itr2);
		}
		fprintf(CNF, "0\n");
	}

//	for (int i = 0; i < clauses->size(); i++) {
//		Clause *c = clauses[i];
//		if (!c) continue;
//		for (Clause::const_iterator itr = c->begin(); itr != c->end(); itr++) {
//			assert(*itr > 0);
//			fprintf(CNF,"%d ",*itr);
//		}
//		fprintf(CNF,"0\n");
//	}

	for (ClauseSet::const_iterator itr = goal_clauses.cbegin(); itr != goal_clauses.cend(); itr++) {
		const Clause& c = *itr;
		for (Clause::const_iterator itr2 = c.begin(); itr2 != c.end(); itr2++) {
			assert(*itr2 > 0);
			fprintf(CNF, "%d ", *itr2);
		}
		fprintf(CNF, "0\n");
	}

	fclose(CNF);
	return true;
}

void StripsEncoding::read_weighted_model_counting_answer_file(int& satresult,double& sat_prob, double& rtime) {
	FILE *A;
	if ( (A = fopen("A","r")) == NULL )
	{
		printf("Can't open Catchet's answer file! File %s, line %d.\n",__FILE__,__LINE__);
		exit( 1 );
	}
	fscanf(A,"%d %lf %lf\n", &satresult, &sat_prob, &rtime);
	fclose(A);
}


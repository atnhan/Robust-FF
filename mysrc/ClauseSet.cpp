/*
 * ClauseSet.cpp
 *
 *  Created on: Feb 25, 2013
 *      Author: tanguye1
 */

#include "ClauseSet.h"
#include <stdio.h>
#include <assert.h>
#include <string>
#include <fstream>
#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/classification.hpp>
using namespace std;

extern string gproblem_file;

ClauseSet::ClauseSet() {

}

ClauseSet::ClauseSet(const ClauseSet& cs) {
	this->clauses = cs.clauses;
}

ClauseSet::~ClauseSet() {
	clauses.clear();
}

// Assignment operator
ClauseSet& ClauseSet::operator=(const ClauseSet& cs) {
	this->clauses = cs.clauses;
	return *this;
}

void ClauseSet::add_clause(const Clause& c) {
	if (c.size() == 0)
		return;

	// If "c" is superset of any other sets, ignore it
	for (ClauseSet::const_iterator itr = this->cbegin(); itr != this->cend(); itr++)
		if (itr->subset(c))
			return;

	// If any clause "c'" is a superset of "c", we remove "c'"
	ClauseSet::iterator itr = this->begin();
	while (itr != this->end()) {
		if (c.subset(*itr)) {
			itr = clauses.erase(itr);
		}
		else
			itr++;
	}

	clauses.insert(c);
}

void ClauseSet::add_clauses(const ClauseSet& cs) {
	for (ClauseSet::const_iterator itr = cs.clauses.begin(); itr != cs.clauses.end(); itr++) {
		if (itr->size()) {
			add_clause(*itr);
		}
	}
}

void ClauseSet::wmc(CACHET_OUTPUT& r) const {
	if (clauses.size() <= 0) {
		r.prob = 1;
		r.time = 0;
		return;
	}

	// Check the model counting file
	FILE *f;
	if ((f = fopen ("./cachet-wmc", "r")) == NULL)
	{
		printf("Model counting software not found! File %s, line %d.\n",__FILE__,__LINE__);
		exit(1);
	}
	fclose(f);

	// CNF and result file
	string cnf_file = gproblem_file + string(".cnf");
	string result_file = gproblem_file + string(".cnf") + string(".cachet");

	write_cnf_file(cnf_file.c_str());
	string cmd = "./cachet-wmc " + cnf_file + " > " + result_file;

	// Calling the model counting and write the answer to "A" file
	system(cmd.c_str());

	// Read the answer file to get the resulting information
	read_wmc_answer_file(result_file, r);
}

double ClauseSet::lower_wmc() const {
	// A simple lower bound: product of individual probability
	double lower = 1;
	for (ClauseSet::const_iterator itr = clauses.begin(); itr != clauses.end(); itr++) {
		lower *= itr->prob();
	}
	return lower;
}

double ClauseSet::upper_wmc() const{
	// A simple upper bound: min of individual probability
	// Better bound: product of upper bound of all connected components
	double upper = 1;
	double prob;
	for (ClauseSet::const_iterator itr = clauses.begin(); itr != clauses.end(); itr++) {
		prob = itr->prob();
		if (upper > prob)
			upper = prob;
	}
	return upper;
}

ostream& operator<<(ostream& os, const ClauseSet& cs) {
	for (ClauseSet::const_iterator itr = cs.cbegin(); itr != cs.cend(); itr++) {
		const Clause& c = *itr;
		os<<c<<" ";
	}
	return os;
}

// Write to a file
void ClauseSet::write_cnf_file(const char* filename) const {
	bool uniform = true;
	for (int p=1;p<=Clause::num_bool_vars();p++) {
		if (Clause::weight(p) != 0.5) {
			uniform = false;
			break;
		}
	}

	FILE *CNF;
	if ( (CNF = fopen(filename,"w")) == NULL )
	{
		printf("Can not open CNF file! File %s, line %d.\n",__FILE__,__LINE__);
		exit(1);
	}

	fprintf(CNF, "c clauses representing causal proof of plan correctness\n");
	fprintf(CNF, "p cnf %d %d\n", Clause::num_bool_vars(), clauses.size());

	// Write the non-uniform weights of (positive) literals
	if (!uniform)
	{
		for (int p=1;p<=Clause::num_bool_vars();p++)
		{
			// We don't need to write the weight 0.5 into the CNF file
			if (Clause::weight(p) == 0.5)
				continue;
			fprintf(CNF,"w %d %f\n",p,Clause::weight(p));
		}
	}

	// Write each clause to the file, ending with 0
	for (ClauseSet::const_iterator itr = clauses.begin(); itr != clauses.end(); itr++) {
		const Clause& c = *itr;
		for (Clause::const_iterator itr2 = c.cbegin(); itr2 != c.cend(); itr2++) {
			fprintf(CNF, "%d ", *itr2);
		}
		fprintf(CNF, "0\n");
	}
	fclose(CNF);
}

void ClauseSet::read_wmc_answer_file(std::string result_file, CACHET_OUTPUT& r) const {

	ifstream f(result_file.c_str());

	if (!f.is_open())
	{
		printf("Can't open Catchet's answer file! File %s, line %d.\n",__FILE__,__LINE__);
		exit( 1 );
	}

	const string vars_str("Number of Variables");
	const string clauses_str("Original Num Clauses");
	const string time_str("Total Run Time");
	const string prob_str("Satisfying probability");
	const string solutions_str("Number of solutions");

	// Read the file
	const int SIZE = 256;
	char s[SIZE];
	while (f.getline(s, SIZE)) {
		string line(s);
		vector<string> strs;
		boost::split(strs, line, boost::is_any_of("\t"));

		if (strs[0] == vars_str) {
			r.vars = atoi(strs[strs.size()-1].c_str());

			//
			//cout<<"Vars: "<<r.vars<<endl;
		}
		else if (strs[0] == clauses_str) {
			r.clauses = atoi(strs[strs.size()-1].c_str());

			//
			//cout<<"Clauses: "<<r.clauses<<endl;
		}
		else if (strs[0] == time_str) {
			r.time = atof(strs[strs.size()-1].c_str());

			//
			//cout<<"Time: "<<r.time<<endl;
		}
		else if (strs[0] == prob_str) {
			r.prob = atof(strs[strs.size()-1].c_str());

			//
			//cout<<"Probability: "<<r.prob<<endl;
		}
		else if (strs[0] == solutions_str) {
			r.solutions = atoi(strs[strs.size()-1].c_str());

			//
			//cout<<"Solutions: "<<r.solutions<<endl;
		}
	}
	f.close();
}

bool operator==(ClauseSet const& cs1, ClauseSet const& cs2) {
	return (cs1.clauses == cs2.clauses);
}















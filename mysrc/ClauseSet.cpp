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

bool ClauseSet::UPPER_WMC = true;

ClauseSet::ClauseSet() {
	max_component_id = 0;
}

ClauseSet::ClauseSet(const ClauseSet& cs) {
	*this = cs;
}

ClauseSet::~ClauseSet() {

}

// Assignment operator
ClauseSet& ClauseSet::operator=(const ClauseSet& cs) {
	this->clauses = cs.clauses;
	this->clause_components = cs.clause_components;
	this->max_component_id = cs.max_component_id;
	return *this;
}

void ClauseSet::add_clause(const Clause& c) {
	if (c.size() == 0 || clauses.find(c) != clauses.end())
		return;

	// If "c" is a superset of any other sets, ignore it
	for (const_iterator itr = cbegin(); itr != cend(); itr++) {
		if (itr->subset(c))
			return;
	}

	if (ClauseSet::UPPER_WMC) {
		// Remove any clause "c'" that is a superset of "c"
		// Also record components that must be merged
		boost::unordered_set<int> components_to_be_merged;
		ClauseComponentMap::const_iterator itr = clause_components.cbegin();
		while (itr != clause_components.cend()) {
			if (c.subset(itr->first)) {
				// Erase this superset clause from the clause set
				clauses.erase(itr->first);

				// Erase this clause from the mapping, and move to the next one
				itr = clause_components.erase(itr);
			}
			else {
				// If "c" is not subset of this clause, but they share common literals
				// then all clauses in the same components with this clause (including itself)
				// will be put into the same component with "c"
				if (c.share_literals(itr->first))
					components_to_be_merged.insert(itr->second);

				// Move to the next node
				itr++;
			}
		}

		// Add the new clause "c", and reorganize the connected components
		if (components_to_be_merged.size() == 0) {
			clauses.insert(c);
			clause_components[c] = ++max_component_id;
		}
		else {
			int component = *(components_to_be_merged.begin());
			for (ClauseComponentMap::const_iterator itr = clause_components.cbegin(); itr != clause_components.cend(); itr++) {
				if (components_to_be_merged.find(itr->second) != components_to_be_merged.end()) {
					clause_components[itr->first] = component;
				}
			}
			clauses.insert(c);
			clause_components[c] = component;
		}
	}
	else {
		// Remove any clause "c'" that is a superset of "c"
		const_iterator itr = cbegin();
		while (itr != cend()) {
			if (c.subset(*itr))
				clauses.erase(itr);
			else
				itr++;
		}

		// Add the new clause "c"
		clauses.insert(c);
	}
}

void ClauseSet::add_clauses(const ClauseSet& cs) {
	for (const_iterator itr = cs.cbegin(); itr != cs.cend(); itr++) {
		if (itr->size()) {
			add_clause(*itr);
		}
	}
}

void ClauseSet::clear() {
	clauses.clear();

	if (ClauseSet::UPPER_WMC) {
		clause_components.clear();
		max_component_id = 0;
	}
}

void ClauseSet::wmc(CACHET_OUTPUT& r) const {
	if (size() <= 0) {
		r.prob = 1;
		r.time = 0;
		return;
	}

	// Check the model counting file
	FILE *f;
	string wmc_file = string(gcmd_line.path_to_wmc) + string("cachet-wmc");
	if ((f = fopen (wmc_file.c_str(), "r")) == NULL)
	{
		printf("Model counting software not found! File %s, line %d.\n",__FILE__,__LINE__);
		exit(1);
	}
	fclose(f);

	// CNF and result file

	string cnf_file = string(gcmd_line.ops_file_name) + string("__") + string(gcmd_line.fct_file_name) + string(".cnf");
	string result_file = cnf_file + string(".cachet");

	write_cnf_file(cnf_file.c_str());
	string cmd = wmc_file + " " + cnf_file + " > " + result_file;

	// Calling the model counting and write the answer to "A" file
	system(cmd.c_str());

	// Read the answer file to get the resulting information
	read_wmc_answer_file(result_file, r);

	// Remove the files
	remove(cnf_file.c_str());
	remove(result_file.c_str());
}

double ClauseSet::lower_wmc() const {
	// A simple lower bound: product of individual probability
	double lower = 1;
	for (const_iterator itr = cbegin(); itr != cend(); itr++) {
		lower *= itr->prob();
	}
	return lower;
}

// An upper bound for the probability:
// (1) for each connected component, get the minimal probability of clauses
// (2) take the product of these mins
double ClauseSet::upper_wmc() const{

	if (!ClauseSet::UPPER_WMC) {
		return 1;
	}

	// Empty clause set
	if (size() == 0)
		return 1;

	// First, compute the minimal probability of clauses in each connected components
	boost::unordered_map<int, double> min_probs;
	for (ClauseComponentMap::const_iterator itr = clause_components.cbegin(); itr != clause_components.cend(); itr++) {
		if (min_probs.find(itr->second) == min_probs.end()) {
			min_probs[itr->second] = itr->first.prob();
		}
		else if (min_probs[itr->second] > itr->first.prob()) {
			min_probs[itr->second] = itr->first.prob();
		}
	}

	// Second, take the product of these mins
	double r = 1.0;
	for (boost::unordered_map<int, double>::const_iterator itr = min_probs.cbegin();
			itr != min_probs.cend(); itr++) {
		r *= itr->second;
	}

	return r;

}

int ClauseSet::compare_lower_wmc(const ClauseSet& cs1, const ClauseSet& cs2) const {
	ClauseSet clauses_1(*this);
	clauses_1.add_clauses(cs1);
	ClauseSet clauses_2(*this);
	clauses_2.add_clauses(cs2);

	double prob1 = clauses_1.lower_wmc();
	double prob2 = clauses_2.lower_wmc();
	if (prob1 < prob2) return -1;
	else if (prob1 > prob2) return 1;
	else return 0;
}

int ClauseSet::compare_upper_wmc(const ClauseSet& cs1, const ClauseSet& cs2) const {
	ClauseSet clauses_1(*this);
	clauses_1.add_clauses(cs1);
	ClauseSet clauses_2(*this);
	clauses_2.add_clauses(cs2);

	double prob1 = clauses_1.upper_wmc();
	double prob2 = clauses_2.upper_wmc();
	if (prob1 < prob2) return -1;
	else if (prob1 > prob2) return 1;
	else return 0;
}

int ClauseSet::compare_wmc(const ClauseSet& cs1, const ClauseSet& cs2) const {
	ClauseSet clauses_1(*this);
	clauses_1.add_clauses(cs1);
	ClauseSet clauses_2(*this);
	clauses_2.add_clauses(cs2);

	CACHET_OUTPUT o1, o2;
	clauses_1.wmc(o1);
	clauses_2.wmc(o2);
	double prob1 = o1.prob;
	double prob2 = o2.prob;
	if (prob1 < prob2) return -1;
	else if (prob1 > prob2) return 1;
	else return 0;
}

ostream& operator<<(ostream& os, const ClauseSet& cs) {
	for (ClauseSet::const_iterator itr = cs.cbegin(); itr != cs.cend(); itr++) {
		os<<*itr<<" ";
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
		printf("Can not open CNF file %s! File %s, line %d.\n", filename, __FILE__,__LINE__);
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
	for (ClauseSet::const_iterator itr = cbegin(); itr != cend(); itr++) {
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

// Print out components
void ClauseSet::print_components() {
	if (ClauseSet::UPPER_WMC) {
		for (ClauseComponentMap::const_iterator itr = clause_components.cbegin(); itr != clause_components.cend(); itr++) {
			cout<<itr->first<<": "<<itr->second<<endl;
		}
	}
}










/*
 * PredicateClauseSet.cpp
 *
 *  Created on: Apr 23, 2013
 *      Author: tanguye1
 */

#include "PredicateClauseSet.h"
#include <iostream>
using namespace std;

extern Token gpredicates[MAX_PREDICATES];
extern int gnum_predicates;

PredicateClauseSet::PredicateClauseSet() {
}

PredicateClauseSet::~PredicateClauseSet() {

}

// Assignment operator
PredicateClauseSet& PredicateClauseSet::operator=(const PredicateClauseSet& s) {
	clauses = s.clauses;
	return *this;
}

// Add a clause set for a particular predicate
void PredicateClauseSet::add(int predicate, const ClauseSet& cs) {
	assert(predicate >= 0 && predicate < gnum_predicates);
	if (clauses.find(predicate) == clauses.end()) {
		clauses[predicate] = cs;
	}
	else {
		clauses[predicate].add_clauses(cs);
	}
}

// Merge another clause sets for predicates into the current one
void PredicateClauseSet::merge(const PredicateClauseSet& cs) {
	for (PREDICATE_CLAUSES::const_iterator itr = cs.clauses.cbegin(); itr != cs.clauses.cend(); itr++) {
		add(itr->first, itr->second);
	}
}

// Number of clauses
int PredicateClauseSet::get_num_clauses(int predicate) {
	if (clauses.find(predicate) == clauses.end())
		return 0;
	return clauses.at(predicate).size();
}


int PredicateClauseSet::get_num_clauses() {
	int count=0;
	for (PREDICATE_CLAUSES::const_iterator itr = clauses.cbegin(); itr != clauses.cend(); itr++) {
		count += (itr->second).size();
	}
	return count;
}

void PredicateClauseSet::wmc(CACHET_OUTPUT& r) const {
	// Collect all clauses of all predicates
	ClauseSet all_clauses;
	for (PREDICATE_CLAUSES::const_iterator itr = clauses.cbegin(); itr != clauses.cend(); itr++) {
		all_clauses.add_clauses(itr->second);
	}
	all_clauses.wmc(r);
}

double PredicateClauseSet::lower_wmc() const {
	double lower = 1;
	for (PREDICATE_CLAUSES::const_iterator itr = clauses.cbegin(); itr != clauses.cend(); itr++) {
		const ClauseSet& cs = itr->second;
		lower *= cs.lower_wmc();
	}
	return lower;
}

double PredicateClauseSet::upper_wmc() const{
	double upper = 1;
	for (PREDICATE_CLAUSES::const_iterator itr = clauses.cbegin(); itr != clauses.cend(); itr++) {
		const ClauseSet& cs = itr->second;
		upper *= cs.upper_wmc();
	}
	return upper;
}

ostream& operator<<(std::ostream& os, const PredicateClauseSet& cs) {
	for (PredicateClauseSet::PREDICATE_CLAUSES::const_iterator itr = cs.clauses.cbegin(); itr != cs.clauses.cend(); itr++) {
		cout<<itr->first<<": "<<gpredicates[itr->first]<<" ";
		cout<<itr->second<<endl;
	}
	return os;
}

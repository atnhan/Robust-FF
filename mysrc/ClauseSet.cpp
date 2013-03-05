/*
 * ClauseSet.cpp
 *
 *  Created on: Feb 25, 2013
 *      Author: tanguye1
 */

#include "ClauseSet.h"
#include <assert.h>
using namespace std;

vector<double> ClauseSet::weights;

ClauseSet::ClauseSet() {

}

ClauseSet::~ClauseSet() {
	clauses.clear();
}

void ClauseSet::add_clause(const Clause& c) {
	if (c.size() == 0)
		return;
	clauses.insert(c);
}

void ClauseSet::add_clauses(const ClauseSet& cs) {
	for (ClauseSet::const_iterator itr = cs.clauses.begin(); itr != cs.clauses.end(); itr++) {
		if (itr->size())
			clauses.insert(*itr);
	}
}

double ClauseSet::estimate_robustness(const ClauseSet& additionals) {
	double r = 1;
	ClauseSet cs;
	const ClauseSet& original_clauses = *this;
	cs.add_clauses(original_clauses);
	cs.add_clauses(additionals);

	for (ClauseSet::const_iterator itr = cs.cbegin(); itr != cs.cend(); itr++) {
		const Clause& c = *itr;
		r *= true_prob(c);
	}

	return r;
}

double ClauseSet::true_prob(const Clause& c) {
	double false_prob = 1;
	for (Clause::const_iterator itr = c.begin(); itr != c.end(); itr++) {
		int l = *itr;
		assert(l > 0 && l < weights.size());
		if (l > 0) {
			false_prob *= (1 - weights[l]);
		}
		else {
			false_prob *= (1 - weights[-l]);
		}
	}

	return 1 - false_prob;
}

ostream& operator<<(ostream& os, const ClauseSet& cs) {
	for (ClauseSet::const_iterator itr = cs.cbegin(); itr != cs.cend(); itr++) {
		const Clause& c = *itr;
		os<<"{";
		for (Clause::const_iterator itr2 = c.begin(); itr2 != c.end(); itr2++) {
			os<<*itr2<<", ";
		}
		os<<"} ";
	}
	return os;
}

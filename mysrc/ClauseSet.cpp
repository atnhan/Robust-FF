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

bool ClauseSet::is_subset(const Clause& c1, const Clause& c2) {
	if (c1.size()==0) return true;
	if (c1.size() > c2.size()) return false;
	for (Clause::const_iterator itr = c1.begin(); itr != c1.end(); itr++) {
		if (c2.find(*itr) == c2.end())
			return false;
	}
	return true;
}

void ClauseSet::add_clause(const Clause& c) {
	if (c.size() == 0)
		return;
	// If "c" is superset of any other sets, ignore it
	for (ClauseSet::const_iterator itr = this->cbegin(); itr != this->cend(); itr++)
		if (ClauseSet::is_subset(*itr, c)) // *itr <= c
			return;
	// If any clause "c'" is a subset of "c", we remove it "c'"
	ClauseSet::iterator itr = this->begin();
	ClauseSet::iterator itr2;
	while (itr != this->end()) {
		itr2 = itr;
		itr++;
		if (ClauseSet::is_subset(c, *itr2)) {
			clauses.erase(itr2);
		}
	}
	clauses.insert(c);
}

void ClauseSet::add_clauses(const ClauseSet& cs) {
	for (ClauseSet::const_iterator itr = cs.clauses.begin(); itr != cs.clauses.end(); itr++) {
		if (itr->size())
			add_clause(*itr);
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

double estimate_robustness(const std::vector<ClauseSet*> clause_sets) {
	double r = 0;

	return r;
}

double ClauseSet::true_prob(const Clause& c) {
	double false_prob = 1;
	for (Clause::const_iterator itr = c.begin(); itr != c.end(); itr++) {
		int l = *itr;
		int p = l > 0? l : -l;
		assert(p > 0 && p <= weights.size());
		if (l > 0) {
			false_prob *= (1 - weights[p-1]);
		}
		else {
			false_prob *= weights[p-1];
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

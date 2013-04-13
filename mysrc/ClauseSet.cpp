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
boost::unordered_map<Clause, double> ClauseSet::clause_probability;

ClauseSet::ClauseSet() {

}

ClauseSet::ClauseSet(const ClauseSet& cs) {
	this->clauses = cs.clauses;
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
		if (itr->size()) {
			add_clause(*itr);
		}
	}
}

double ClauseSet::lower_bound() {
	// A simple lower bound: product of individual probability
	double lower = 1;
	for (ClauseSet::const_iterator itr = clauses.begin(); itr != clauses.end(); itr++) {
		lower *= prob(*itr);
	}
	return lower;
}

double ClauseSet::upper_bound() {
	// A simple upper bound: min of individual probability
	// Better bound: product of upper bound of all connected components
	double upper = 1;
	for (ClauseSet::const_iterator itr = clauses.begin(); itr != clauses.end(); itr++) {
		if (upper > prob(*itr))
			upper = prob(*itr);
	}
	return upper;
}

double ClauseSet::estimate_robustness(const ClauseSet& additionals) const {
	if (this->size() + additionals.size() <= 0)
		return 1;

	ClauseSet cs;
	const ClauseSet& original_clauses = *this;
	cs.add_clauses(original_clauses);
	cs.add_clauses(additionals);

	double r = 1;
	for (ClauseSet::const_iterator itr = cs.cbegin(); itr != cs.cend(); itr++) {
		const Clause& c = *itr;
		r *= prob(c);
	}

	return r;
}

double ClauseSet::estimate_robustness(const std::vector<const ClauseSet*>& additional_clause_sets) const {
	ClauseSet cs;
	for (vector<const ClauseSet*>::const_iterator itr = additional_clause_sets.begin(); itr != additional_clause_sets.end(); itr++) {
		if (*itr && (*itr)->size())
			cs.add_clauses(**itr);
	}
	return this->estimate_robustness(cs);
}

double ClauseSet::prob(const Clause& c) const {

	if (ClauseSet::clause_probability.find(c) != ClauseSet::clause_probability.end()) {
		return ClauseSet::clause_probability[c];
	}

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

	ClauseSet::clause_probability[c] = 1 - false_prob;
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

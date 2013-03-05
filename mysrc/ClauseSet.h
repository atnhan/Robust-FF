/*
 * ClauseSet.h
 *
 *  Created on: Feb 25, 2013
 *      Author: tanguye1
 */

#ifndef CLAUSESET_H_
#define CLAUSESET_H_

#include <set>
#include <vector>
#include <iostream>

typedef std::set<int> Clause;		// Note: allocating memory using "calloc" causes core dumped error.

class ClauseSet {

	std::set<Clause> clauses;

	// Probability that a clause is true
	double true_prob(const Clause& c);

public:
	// Weights for boolean variables
	static std::vector<double> weights;	// The size is equal to the number of variables

	ClauseSet();
	virtual ~ClauseSet();

	// Add clauses into the set
	void add_clause(const Clause& c);
	void add_clauses(const ClauseSet& cs);

	// Get size
	int size() {
		return clauses.size();
	}

	// Empty the set
	void clear() {
		clauses.clear();
	}

	// Iterators
	typedef std::set<Clause>::const_iterator const_iterator;
	typedef std::set<Clause>::iterator iterator;

	const_iterator cbegin() const {
		return clauses.begin();
	}

	const_iterator cend() const {
		return clauses.end();
	}

	iterator begin() {
		return clauses.begin();
	}

	iterator end() {
		return clauses.end();
	}

	// Compute approximate robustness value
	double estimate_robustness(const ClauseSet& cs);

	// Print out the clause set
	friend std::ostream& operator<<(std::ostream& os, const ClauseSet& cs);
};

#endif /* CLAUSESET_H_ */

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
#include <boost/unordered_map.hpp>

typedef std::set<int> Clause;		// Note: allocating memory using "calloc" causes core dumped error.

class ClauseSet {

	std::set<Clause> clauses;

	/*
	 * Helper functions for processing clauses
	 */

	// If c1 is subset of c2
	static bool is_subset(const Clause& c1, const Clause& c2);

	// Probability that a clause is true
	double prob(const Clause& c) const;

	// Probability that a clause "c2" is true, given that "c1" is true
	double cond_prob(const Clause& c1, const Clause& c2) const;

public:
	// Weights for boolean variables
	static std::vector<double> weights;	// The size is equal to the number of variables

	// Look-up table for individual clause's robustness
	static boost::unordered_map<Clause, double> clause_probability;

	ClauseSet();
	ClauseSet(const ClauseSet& clauses);
	virtual ~ClauseSet();

	// Add clauses into the set
	void add_clause(const Clause& c);				// OPTIMIZATION POSSIBLE!!!
	void add_clauses(const ClauseSet& cs);

	// Get size
	int size() const {
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

	friend bool operator==(const ClauseSet& cs1, const ClauseSet& cs2) {
		return (cs1.clauses == cs2.clauses);
	}

	// Compute approximate robustness value
	double estimate_robustness(const ClauseSet& cs) const;
	double estimate_robustness(const std::vector<const ClauseSet*>& clause_sets) const;

	// Compute its lower and upper bound
	double lower_bound();
	double upper_bound();


	// Print out the clause set
	friend std::ostream& operator<<(std::ostream& os, const ClauseSet& cs);
};

#endif /* CLAUSESET_H_ */

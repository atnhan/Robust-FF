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
#include "Clause.h"
#include <boost/unordered_set.hpp>

class ClauseSet {

	boost::unordered_set<Clause, boost::hash<Clause> > clauses;

	// Read the output file of Cachet
	void read_wmc_answer_file(int& satresult,double& sat_prob, double& rtime) const;

public:

	ClauseSet();
	ClauseSet(const ClauseSet& clauses);
	virtual ~ClauseSet();

	// Add clauses into the set
	void add_clause(const Clause& c);				// OPTIMIZATION POSSIBLE!!!
	void add_clauses(const ClauseSet& cs);

	// Assignment operator
	ClauseSet& operator=(const ClauseSet& cs) {
		this->clauses = cs.clauses;
		return *this;
	}

	// Get size
	int size() const {
		return clauses.size();
	}

	// Empty the set
	void clear() {
		clauses.clear();
	}

	// Iterators
	typedef boost::unordered_set<Clause>::const_iterator const_iterator;
	typedef boost::unordered_set<Clause>::iterator iterator;

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

	// Compute its exact WMC and the two lower and upper bounds
	void wmc(int& satresult, double& satprob, double& rtime) const;
	double lower_wmc() const;
	double upper_wmc() const;

	// Write to a file
	void write_cnf_file(const char* filename) const;

	// Print out the clause set
	friend std::ostream& operator<<(std::ostream& os, const ClauseSet& cs);

	friend bool operator==(ClauseSet const& cs1, ClauseSet const& cs2);

};

#endif /* CLAUSESET_H_ */

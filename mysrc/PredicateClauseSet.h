/*
 * PredicateClauseSet.h
 *
 *  Created on: Apr 23, 2013
 *      Author: tanguye1
 */

#ifndef PREDICATECLAUSESET_H_
#define PREDICATECLAUSESET_H_

#include "ClauseSet.h"
#include "../ff.h"

class PredicateClauseSet {

	typedef int PREDICATE;
	typedef boost::unordered_map<PREDICATE, ClauseSet> PREDICATE_CLAUSES;
	PREDICATE_CLAUSES clauses;

public:
	PredicateClauseSet();
	virtual ~PredicateClauseSet();

	// Assignment operator
	PredicateClauseSet& operator=(const PredicateClauseSet& clauses);

	// Clear all
	void clear() { clauses.clear(); }

	// Add a clause set for a particular predicate
	void add(int predicate, const ClauseSet& cs);

	// Merge another clause sets for predicates into the current one
	void merge(const PredicateClauseSet& cs);

	// Number of clauses
	int get_num_clauses(int predicate);
	int get_num_clauses();

	// Compute its exact WMC and the two lower and upper bounds
	void wmc(CACHET_OUTPUT& r) const;
	double lower_wmc() const;
	double upper_wmc() const;

	// Print out
	friend std::ostream& operator<<(std::ostream& os, const PredicateClauseSet& cs);

};

#endif /* PREDICATECLAUSESET_H_ */

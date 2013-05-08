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
#include <boost/unordered_map.hpp>
#include <boost/unordered_set.hpp>

// The information output by Cachet that we want to have
struct CACHET_OUTPUT {
	int vars;			// number of variables
	int clauses;		// number of clauses
	double time;		// running time
	double prob;		// satisfying probability
	int solutions;		// number of solutionRP_STEP

	void print() {
		std::cout<<"Number of variables: "<<vars<<std::endl;
		std::cout<<"Number of clauses: "<<clauses<<std::endl;
		std::cout<<"Running time: "<<time<<std::endl;
		std::cout<<"Probability: "<<prob<<std::endl;
		std::cout<<"Number of solutions: "<<solutions<<std::endl;
	}
};

class ClauseSet {

	// The set of clauses
	boost::unordered_set<Clause, boost::hash<Clause> > clauses;

	// Each clause is associated with a component id.
	typedef int COMPONENT_ID;
	typedef boost::unordered_map<Clause, COMPONENT_ID, boost::hash<Clause> > ClauseComponentMap;
	ClauseComponentMap clause_components;

	// The maximal component of the current set of clauses
	int max_component_id;

	// Read the output file of Cachet
	void read_wmc_answer_file(std::string result_file, CACHET_OUTPUT& r) const;

	// Print out components
	void print_components();

public:

	ClauseSet();
	ClauseSet(const ClauseSet& clauses);
	virtual ~ClauseSet();

	// Assignment operator
	ClauseSet& operator=(const ClauseSet& cs);

	// Add clauses into the set
	void add_clause(const Clause& c);				// OPTIMIZATION POSSIBLE!!!
	void add_clauses(const ClauseSet& cs);

	// Get size
	int size() const {
		return clauses.size();
	}

	// Empty the set
	void clear();

	// Iterators
	typedef boost::unordered_set<Clause, boost::hash<Clause> >::const_iterator const_iterator;

	const_iterator cbegin() const {
		return clauses.begin();
	}

	const_iterator cend() const {
		return clauses.end();
	}

	// Compute its exact WMC and the two lower and upper bounds
	void wmc(CACHET_OUTPUT& r) const;
	double lower_wmc() const;
	double upper_wmc() const;

	// Compare two clause sets "cs1" and "cs2" w.r.t the current "cs" (i.e., "this->clauses")
	// according to lower, upper or exact WMC
	// Return:
	// Negative if the probability of (cs AND cs1) < that of (cs AND cs1)
	// Positive if the probability of (cs AND cs1) > that of (cs AND cs1)
	// Zero otherwise
	int compare_lower_wmc(const ClauseSet& cs1, const ClauseSet& cs2) const;
	int compare_upper_wmc(const ClauseSet& cs1, const ClauseSet& cs2) const;
	int compare_wmc(const ClauseSet& cs1, const ClauseSet& cs2) const;

	// Write to a file
	void write_cnf_file(const char* filename) const;

	// Print out the clause set
	friend std::ostream& operator<<(std::ostream& os, const ClauseSet& cs);

	// Equality comparison
	friend bool operator==(ClauseSet const& cs1, ClauseSet const& cs2);

	/*
	 * TEST
	 */
	friend void test_estimate_robustness();
};

#endif /* CLAUSESET_H_ */

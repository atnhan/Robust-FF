/*
 * Clause.h
 *
 *  Created on: Apr 13, 2013
 *      Author: tanguye1
 */

#ifndef CLAUSE_H_
#define CLAUSE_H_
#include <assert.h>
#include <boost/unordered_map.hpp>
#include <boost/functional/hash.hpp>

class Clause {

	// All literals, in ascending order
	std::vector<int> clause;

	// Look-up table for individual clause's robustness
	static boost::unordered_map<Clause, double> clause_prob;

	// Weights for boolean variables
	static std::vector<double> weights;	// The size is equal to the number of variables

	// The first object creation
	static bool first_clause;

public:

	Clause();
	virtual ~Clause();

	// Get the weight of a boolean variable
	static double weight(int pro);

	// Get the number of boolean variables
	static int num_bool_vars() { return Clause::weights.size(); }

	// Set the weights
	static void set_weights(std::vector<double> weights);

	// Equality
	friend bool operator==(Clause const& c1, Clause const& c2);
	friend bool operator!=(Clause const& c1, Clause const& c2);

	// Get literal at a position
	int lit(int pos) const;

	// Add a new literal
	void add_literal(int l);

	// Check if a literal is in the set
	bool contain(int l) const;

	// Test if a "subset" of another clause
	bool subset(const Clause& c) const;

	// Number of literals
	int size() const {
		return clause.size();
	}

	// Iterators
	typedef std::vector<int>::const_iterator const_iterator;

	const_iterator cbegin() const {
		return clause.begin();
	}

	const_iterator cend() const {
		return clause.end();
	}

	// Probability
	double prob() const;

	// Print out
	friend std::ostream& operator<<(std::ostream& os, const Clause& cs);

	// Hash value. Note that the order of literals does matter, and since we sort literals,
	// two same clauses must have the same hash value
	friend std::size_t hash_value(Clause const& c);

};

#endif /* CLAUSE_H_ */

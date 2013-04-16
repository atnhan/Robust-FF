/*
 * Clause.cpp
 *
 *  Created on: Apr 13, 2013
 *      Author: tanguye1
 */

#include "Clause.h"
#include <iostream>
#include <algorithm>
using namespace std;

boost::unordered_map<Clause, double> Clause::clause_prob;
std::vector<double> Clause::weights;
bool Clause::first_clause = false;

Clause::Clause() {

	// Check if the weight vector has been initialized
	if (!Clause::first_clause) {
		if (Clause::weights.size() <= 0) {
			cout<<"Clause weights uninitialized! File "<<__FILE__<<", line "<<__LINE__<<endl;
			exit(1);
		}
		first_clause = true;
	}
}

Clause::~Clause() {

}

// Add a new literal
void Clause::add_literal(int l) {
	assert(abs(l) >= 1 && abs(l) <= Clause::num_bool_vars());
	// Ignore if the literal has already been present
	if (contain(l)) return;

	// Add the literal, and sort
	clause.push_back(l);
	sort(clause.begin(), clause.end());
}

// Get literal at a position
int Clause::lit(int pos) const {
	assert(pos >= 0 && pos < size());
	return clause[pos];
}

double Clause::weight(int p) {
	assert(p >= 1 && p <= Clause::weights.size());
	return Clause::weights[p-1];
}

// Set the weights
void Clause::set_weights(std::vector<double> weights) {
	Clause::weights = weights;
}

// Equality
bool operator==(Clause const& c1, Clause const& c2) {
	if (c1.size() != c2.size()) return false;
	for (int i=0;i<c1.size();i++)
		if (c1.lit(i) != c2.lit(i))
			return false;
	return true;

}

bool operator!=(Clause const& c1, Clause const& c2) {
	return (!(c1 == c2));
}

// Check if a literal is in the set
bool Clause::contain(int l) const {
	if (clause.size() <= 0)
		return false;
	return (std::binary_search(clause.begin(), clause.end(), l));
}


// Test if a "subset" of another clause
bool Clause::subset(const Clause& c) const {
	if (clause.size()==0) return true;
	if (clause.size() > c.size()) return false;

	for (Clause::const_iterator itr = cbegin(); itr != cend(); itr++) {
		if (!c.contain(*itr))
			return false;
	}
	return true;
}

// Probability
double Clause::prob() const {
	const Clause& c = *this;
	if (Clause::clause_prob.find(c) != Clause::clause_prob.end()) {
		return Clause::clause_prob[c];
	}

	double false_prob = 1;
	for (int i=0;i<size();i++) {
		int l = lit(i);
		assert(l != 0);
		int p = l > 0? l : -l;
		assert(p > 0 && p <= weights.size());
		if (l > 0) {
			false_prob *= (1 - weights[p-1]);
		}
		else {
			false_prob *= weights[p-1];
		}
	}

	Clause::clause_prob[c] = 1 - false_prob;
	return 1 - false_prob;
}

ostream& operator<<(std::ostream& os, const Clause& cs) {
	os<<"{";
	for (Clause::const_iterator itr = cs.cbegin(); itr != cs.cend(); itr++) {
		os<<*itr<<", ";
	}
	os<<"}";
	return os;
}

std::size_t hash_value(Clause const& c)
{
	std::size_t seed = 0;
	for (int i=0;i<c.size();i++) {
		boost::hash_combine(seed, c.lit(i));
	}
	return seed;
}

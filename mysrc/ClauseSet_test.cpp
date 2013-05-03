/*
 * ClauseSet_test.cpp
 *
 *  Created on: Feb 25, 2013
 *      Author: tanguye1
 */

#include "ClauseSet.h"
#include <iostream>
#include <vector>
#include <ctime>
#include <boost/random/uniform_int_distribution.hpp>
#include <boost/random/uniform_real_distribution.hpp>
#include <boost/random/mersenne_twister.hpp>
using namespace std;

void test_adding_removing_clauses() {
	vector<double> weights(3, 0.5);
	Clause::set_weights(weights);
	ClauseSet cs;

	Clause c1; c1.add_literal(-1); c1.add_literal(2);
	Clause c2; c2.add_literal(-3);
	Clause c3; c3.add_literal(-1); c3.add_literal(2);
	Clause c4; c4.add_literal(-3); c4.add_literal(2);
	cs.add_clause(c1);
	cs.add_clause(c2);
	cs.add_clause(c3);
	cs.add_clause(c4);
	cout<<"Clauses (cs): "<<cs<<endl;

	ClauseSet cs2 = cs;
	cout<<"Clauses (cs2): "<<cs2<<endl;

}

// Randomly generate clause set
void random_clauses_00(int num_vars, int num_clauses, int max_clause_size, vector<double>& weights, ClauseSet& cs) {
	assert(max_clause_size <= num_vars);

	cout<<"====== BEGIN RANDOM_CLAUSES_00 ======="<<endl;

	const double pos_lit_prob = 0.5;	// probability for positive literals
	const double lit_sel_prob = 0.5;	// probability for selecting a literal into a clause

	boost::mt19937 generator;
	generator.seed(static_cast<unsigned int>(std::time(0)));		// Initialize seed using the current time

	// The two distribution for random number generation
	boost::random::uniform_int_distribution<> int_dist;		// The range can change
	boost::random::uniform_real_distribution<> real_dist;	// By default: [0, 1)

	// Weight vector
	weights.clear();
	for (int i=0;i<num_vars;i++) {
		//weights.push_back(real_dist(generator));
		weights.push_back(0.5);
	}
	Clause::set_weights(weights);

	// Literals
	vector<int> literals;
	for (int p=1;p<=num_vars;p++) {
		if (real_dist(generator) <= 0.5)
			literals.push_back(p);
		else
			literals.push_back(-p);
	}

	cout<<"Literals: ";
	for (int i=0;i<num_vars;i++)
		cout<<literals[i]<<" ";
	cout<<endl<<endl;
	cout<<"Weights: ";
	for (int i=0;i<num_vars;i++)
		cout<<weights[i]<<" ";
	cout<<endl<<endl;

	// Clause set
	for (int i=0;i<num_clauses;i++) {
		Clause c;
		int clause_size = int_dist(generator,
								boost::random::uniform_int_distribution<>::param_type(1, max_clause_size));
		for (int i=0;i<clause_size;i++) {
			// Uniformly select one literal to add
			int p = int_dist(generator,
					boost::random::uniform_int_distribution<>::param_type(0, num_vars-1));
			int l = literals[p];
			c.add_literal(l);
		}
		cout<<"Adding clause: "<<c<<endl<<endl;
		cs.add_clause(c);
	}

	cout<<"Final clauses:"<<endl;
	cout<<cs<<endl<<endl;

	cout<<"====== END RANDOM_CLAUSES_00 ======="<<endl<<endl;

}

void test_estimate_robustness() {
//	vector<double> weights(5, 0.5);
//	Clause::set_weights(weights);
//	Clause c1; c1.add_literal(-1); c1.add_literal(2);
//	Clause c2; c2.add_literal(-3);
//	Clause c3; c3.add_literal(-1); c3.add_literal(4);
//	Clause c4; c4.add_literal(-3); c4.add_literal(-5);
//	ClauseSet cs;
//	cs.add_clause(c1);
//	cs.add_clause(c2);
//	cs.add_clause(c3);
//	cs.add_clause(c4);

	// RANDOMLY GENERATE CLAUSES
	int num_vars = 20;
	int num_clauses = 5;
	int max_clause_size = 5;
	vector<double> weights;
	ClauseSet cs;
	random_clauses_00(num_vars, num_clauses, max_clause_size, weights, cs);

	cout<<"CLAUSE COMPONENTS"<<endl;
	cs.print_components();
	cout<<endl;

	cout<<"CACHET OUTPUT"<<endl;
	CACHET_OUTPUT o;
	cs.wmc(o);
	o.print();
	cout<<endl;

	cout<<"LOWER BOUND: "<<cs.lower_wmc()<<endl;
	cout<<"UPPER BOUND: "<<cs.upper_wmc()<<endl;

	cout<<"MIN OF ALL CLAUSE PROB: ";
	double r = 1;
	for (ClauseSet::ClauseComponentMap::const_iterator itr = cs.clause_components.cbegin();
			itr != cs.clause_components.cend(); itr++) {
		if (r > itr->first.prob())
			r = itr->first.prob();
	}
	cout<<r<<endl;

}

/*
 * ClauseSet_test.cpp
 *
 *  Created on: Feb 25, 2013
 *      Author: tanguye1
 */

#include "ClauseSet.h"
#include <iostream>
#include <vector>
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
	cout<<"Clauses: "<<cs<<endl;
}

void test_estimate_robustness() {
	// {-1}, {-3, -1, 2} {-1, 2} {-3, 2}
	ClauseSet cs;
	Clause c1; c1.add_literal(-1); c1.add_literal(2);
	//Clause c2; c2.insert(-3);
//	Clause c3; c3.insert(-1); c3.insert(2);
//	Clause c4; c4.insert(-3); c4.insert(2);
	cs.add_clause(c1);
//	cs.add_clause(c2);
//	cs.add_clause(c3);
//	cs.add_clause(c4);
	cout<<"Clauses: "<<cs<<endl;
	ClauseSet emptyset;
//	cout<<"Robustness: "<<cs.estimate_robustness(emptyset)<<endl;
}

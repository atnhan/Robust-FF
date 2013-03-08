/*
 * ClauseSet_test.cpp
 *
 *  Created on: Feb 25, 2013
 *      Author: tanguye1
 */

#include "ClauseSet.h"
#include <iostream>
using namespace std;

void test_estimate_robustness() {
	// {-1}, {-3, -1, 2} {-1, 2} {-3, 2}
	ClauseSet cs;
	Clause c1; c1.insert(-1);
	Clause c2; c2.insert(-3); c2.insert(-1); c2.insert(2);
	Clause c3; c3.insert(-1); c3.insert(2);
	Clause c4; c4.insert(-3); c4.insert(2);
	cs.add_clause(c1);
	cs.add_clause(c2);
	cs.add_clause(c3);
	cs.add_clause(c4);
	cout<<"Clauses: "<<cs<<endl;
	ClauseSet emptyset;
	cout<<"Robustness: "<<cs.estimate_robustness(emptyset)<<endl;
}

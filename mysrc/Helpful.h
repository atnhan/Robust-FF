/*
 * Helpful.h
 *
 *  Created on: Feb 10, 2013
 *      Author: tuan
 */

#ifndef HELPFUL_H_
#define HELPFUL_H_

#include "../ff.h"
#include "../output.h"
#include <iostream>
#include <string>

enum AnnotationType {POSS_PRE, POSS_ADD, POSS_DEL};

// Check if a fact is a (possible) precondition or effect of an action
bool is_poss_pre(int ft, int action);
bool is_poss_add(int ft, int action);
bool is_poss_del(int ft, int action);
bool is_pre(int ft, int action);
bool is_add(int ft, int action);
bool is_del(int ft, int action);

// Check if a fact is in a state. Faster implementation may be needed.
bool is_in_state(int ft, const State *s);

// Check if a fact is certainly known in a state
bool is_known_in_state(int ft, const State *s);

// Find the index of an action from its name. Action name is assumed to be in upper case,
// parameters are separated by white spaces: "DEBARK PERSON4 PLANE2 CITY1"
int find_action(std::string action_name);

// Print state
void print_state(const State& s);

// Print state with fact indices only
void print_state_with_fact_indices(const State& s);

// Get boolean variable for a possible precondition and effect
// Valid boolean variable must be POSITIVE
int get_bool_var(int ft, int action, AnnotationType t);

// Get predicate of a proposition
int get_predicate(int pro);

// Read WMC output file
void read_wmc_answer_file(int& satresult,double& sat_prob, double& rtime);

// Check if an action is applicable in a state
// (all its known preconditions must present in the state)
bool applicable_action(int action, const State* s);

// Check if two states are the same
bool same_state(const State& s1, const State& s2);

// Print TAB
void TAB(int n);

#endif /* HELPFUL_H_ */

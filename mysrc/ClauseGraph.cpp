/*
 * ClauseGraph.cpp
 *
 *  Created on: Apr 28, 2013
 *      Author: tanguye1
 */

#include "ClauseGraph.h"
#include <vector>
using namespace std;

ClauseGraph::ClauseGraph() {

}

ClauseGraph::~ClauseGraph() {

}

// Add a clause
void ClauseGraph::add_clause(const Clause& c) {

	// If this clause presents, do nothing
	if (this->include(c))
		return;

	// Get three information:
	// (1) the set of connected components that must be merged,
	// (2) the set of clauses that are superset of "c" (and thus to be removed)
	// (3) the set of clauses, which are neither subset nor superset of "c", sharing common literals with "c" (thus will be in
	// the adjacency list of "c"
	vector<Clause> clauses_to_be_removed;
	vector<component_iterator> components_to_be_merged;
	AdjacencyClauseList clauses_with_common_literals;
	for (component_iterator com_itr = begin(); com_itr != end(); com_itr++) {
		// Consider each component
		Component& com = *com_itr;
		bool to_be_merged = false;
		// For each clause in that component
		for (Component::const_iterator itr = com.begin(); itr != com.end(); itr++) {
			Clause& this_clause = *itr;

			// if "c" is a superset of "this_clause", do nothing
			if (this_clause.subset(c))
				return;

			// if "c" is subset of "this_clause", we will remove "this_clause"
			if (c.subset(this_clause))
				clauses_to_be_removed.push_back(this_clause);
			// otherwise, if they share literals, then they will be connected in the graph after "c" is added
			else if (c.share_literals(this_clause)) {

				// Note: we make sure that clauses to be removed won't be in this list
				clauses_with_common_literals.insert(this_clause);

				// the corresponding component will also be merged
				if (!to_be_merged) {
					components_to_be_merged.push_back(com_itr);
					to_be_merged = true;
				}
			}
		}
	}

	/*
	 * Updating the graph
	 */
	// Add adjacency list for the new clause "c"
	boost::unordered_map<Clause, AdjacencyClauseList>::value_type v(c, clauses_with_common_literals);
	clause_adj.insert(v);

	// Add "c" into adjacency list of clauses sharing common literals with "c"
	for (AdjacencyClauseList::const_iterator c_itr = clauses_with_common_literals.begin();
			c_itr != clauses_with_common_literals.end(); c_itr++) {
		const Clause& connected_clause = *c_itr;
		assert(clause_adj.find(connected_clause) != clause_adj.end());
		AdjacencyClauseList& l = *(clause_adj.find(connected_clause));
		l.insert(c);
	}

	// Remove clauses that are superset of "c"

}

// Remove a clause from the graph, and update the set of connected components
void ClauseGraph::remove_clause(const Clause& c) {

	// If the clause is not present, do nothing
	if (!this->include(c))
		return;

	// (1) Remove the clause from the graph data structure
	// Find the iterator to this clause
	ClauseGraph::clause_iterator c_itr = clause_adj.find(c);

	// Erase this
	clause_adj.erase(c_itr);

	// Erase this clause from adjacency list of all other clauses
	for (ClauseGraph::clause_iterator itr = begin(); itr != end(); itr++) {
		AdjacencyClauseList& l = itr->second;
		if (l.find(c) != l.end())
			continue;

		// Remove "c" from "l"
		l.erase(c);
	}

	// (2) Remove the clause from its component
	ClauseGraph::component_iterator com_itr = begin();
	while (com_itr != end()) {
		Component& com = *com_itr;
		if (com.find(c) == com.end()) {
			com_itr++;
			continue;
		}

		// Inefficient step:
	}
}








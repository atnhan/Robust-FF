/*
 * ClauseGraph.h
 *
 *  Created on: Apr 28, 2013
 *      Author: tanguye1
 */

#ifndef CLAUSEGRAPH_H_
#define CLAUSEGRAPH_H_
#include "Clause.h"
#include <boost/unordered_set.hpp>
#include <boost/unordered_map.hpp>
#include <list>
using namespace std;

class ClauseGraph {

	// Adjacency list representation
	typedef boost::unordered_set<Clause, boost::hash<Clause> > AdjacencyClauseList;
	boost::unordered_map<Clause, AdjacencyClauseList> clause_adj;	// a set of clauses sharing literals with a given one

	// The set of connected components, updated when a new clause is added into the graph,
	// which may cause some removal as well.
	typedef boost::unordered_set<Clause, boost::hash<Clause> > Component;
	std::list<Component> clause_connected_components;

public:
	ClauseGraph();
	virtual ~ClauseGraph();

	// Iterators
	typedef boost::unordered_map<Clause, AdjacencyClauseList>::const_iterator clause_const_iterator;
	typedef boost::unordered_map<Clause, AdjacencyClauseList>::iterator clause_iterator;
	typedef std::list<Component>::const_iterator component_const_iterator;
	typedef std::list<Component>::iterator component_iterator;

	clause_const_iterator cbegin() { return clause_adj.cbegin(); }

	clause_const_iterator cend() { return clause_adj.cend(); }

	clause_iterator begin() { return clause_adj.begin(); }

	clause_iterator end() { return clause_adj.end(); }

	component_const_iterator cbegin() { return clause_connected_components.begin(); }

	component_const_iterator cend() { return clause_connected_components.end(); }

	component_iterator begin() { return clause_connected_components.begin(); }

	component_iterator end() { return clause_connected_components.end(); }

	// Test if a clause is in included the graph
	bool include(const Clause& c) {
		return (clause_adj.find(c) != clause_adj.end());
	}

	// Add a clause, and update the set of connected components
	void add_clause(const Clause& c);

	// Remove a clause from the graph, and update the set of connected components
	void remove_clause(const Clause& c);
};

#endif /* CLAUSEGRAPH_H_ */

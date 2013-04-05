/*
 * Search.h
 *
 *  Created on: Mar 30, 2013
 *      Author: tanguye1
 */

#ifndef SEARCH_H_
#define SEARCH_H_
#include "../ff.h"
#include <vector>
#include "StripsEncoding.h"

class Search {
protected:
	StripsEncoding *e;
	const State *init;
	const State *goals;
	std::vector<int> actions;

public:
	Search(State* init, State* goals);
	virtual ~Search();

	virtual void run() = 0;
};

#endif /* SEARCH_H_ */

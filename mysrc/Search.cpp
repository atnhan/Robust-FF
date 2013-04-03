/*
 * Search.cpp
 *
 *  Created on: Mar 30, 2013
 *      Author: tanguye1
 */

#include "Search.h"

Search::Search(State* init, State* goals) {
	e = new StripsEncoding(init);
	this->goals = goals;
}

Search::~Search() {
	if (e) {
		delete e;
		e = 0;
	}
	if (goals) {
		delete goals;
		goals = 0;
	}
	if (init) {
		delete init;
		init = 0;
	}
}



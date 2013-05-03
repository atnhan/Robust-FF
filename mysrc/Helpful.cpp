/*
 * Helpful.cpp
 *
 *  Created on: Feb 10, 2013
 *      Author: tuan
 */

#include "Helpful.h"
#include <iostream>
#include <assert.h>
using namespace std;

bool is_poss_pre(int ft, int action) {
	if (ft < 0 || ft >= gnum_ft_conn) return false;
	if (action < 0 || action >= gnum_op_conn) return false;

	for (int i = 0; i < gft_conn[ft].num_poss_PC; i++) {
		int ef = gft_conn[ft].poss_PC[i];
		if (gef_conn[ef].removed) {
			continue;
		}
		if (action == gef_conn[ef].op) return true;
	}
	return false;
}

bool is_poss_add(int ft, int action) {
	if (ft < 0 || ft >= gnum_ft_conn) return false;
	if (action < 0 || action >= gnum_op_conn) return false;

	for (int i = 0; i < gft_conn[ft].num_poss_A; i++) {
		int ef = gft_conn[ft].poss_A[i];
		if (gef_conn[ef].removed) {
			continue;
		}
		if (action == gef_conn[ef].op) return true;
	}
	return false;
}


bool is_poss_del(int ft, int action) {
	if (ft < 0 || ft >= gnum_ft_conn) return false;
	if (action < 0 || action >= gnum_op_conn) return false;

	for (int i = 0; i < gft_conn[ft].num_poss_D; i++) {
		int ef = gft_conn[ft].poss_D[i];
		if (gef_conn[ef].removed) {
			continue;
		}
		if (action == gef_conn[ef].op) return true;
	}
	return false;
}
bool is_pre(int ft, int action) {
	if (ft < 0 || ft >= gnum_ft_conn) return false;
	if (action < 0 || action >= gnum_op_conn) return false;

	for (int i = 0; i < gft_conn[ft].num_PC; i++) {
		int ef = gft_conn[ft].PC[i];
		if (gef_conn[ef].removed) {
			continue;
		}
		if (action == gef_conn[ef].op) return true;
	}
	return false;
}

bool is_add(int ft, int action) {
	if (ft < 0 || ft >= gnum_ft_conn) return false;
	if (action < 0 || action >= gnum_op_conn) return false;

	for (int i = 0; i < gft_conn[ft].num_A; i++) {
		int ef = gft_conn[ft].A[i];
		if (gef_conn[ef].removed) {
			continue;
		}
		if (action == gef_conn[ef].op) return true;
	}
	return false;
}

bool is_del(int ft, int action) {
	if (ft < 0 || ft >= gnum_ft_conn) return false;
	if (action < 0 || action >= gnum_op_conn) return false;

	for (int i = 0; i < gft_conn[ft].num_D; i++) {
		int ef = gft_conn[ft].D[i];
		if (gef_conn[ef].removed) {
			continue;
		}
		if (action == gef_conn[ef].op) return true;
	}
	return false;
}

bool is_in_state(int ft, const State *s) {
	assert(s);
	for (int i = 0; i < s->num_F; i++)
		if (ft == s->F[i])
			return true;
	return false;
}

int find_action(string action_name) {
	string s;
	for (int op = 0; op < gnum_op_conn; op++) {
		Action *a = gop_conn[op].action;
		if ( !a->norm_operator && !a->pseudo_action )
			return -1;
		else {
			s = a->name;
			for (int i = 0; i < a->num_name_vars; i++ ) {
				s += string(" ") + gconstants[a->name_inst_table[i]];
			}
			if (s == action_name)
				return op;
		}
	}
	return -1;
}

void print_state(const State& s) {
	for (int i=0;i<s.num_F;i++) {
		int ft = s.F[i];
		print_ft_name(ft);
		if (i < s.num_F-1)
			cout<<endl;
	}
}

// Check if two states are the same
bool same_state(const State& s1, const State& s2) {
	if (s1.num_F != s2.num_F)
		return false;
	for (int i=0; i<s1.num_F;i++)
		if (!is_in_state(s1.F[i], &s2))
			return false;
	return true;
}

int get_bool_var(int ft, int action, AnnotationType t) {

	// Optimization (to be added): if "ft" is not possibly added or deleted by any action, then return.

	int res = 0;	// Note: valid boolean variable must be POSITIVE
	if (action < 0 || action >= gnum_op_conn)
		return res;
	if (gop_conn[action].num_E > 1) {
		cout<<"Error! File %s "<<__FILE__<<", line "<<__LINE__<<endl;
		exit(1);
	}

	int n_ef = gop_conn[action].E[0];
	int n;
	int *possibles;
	int *annotations;
	switch (t) {
	case POSS_PRE:
		n = gef_conn[n_ef].num_poss_PC;
		possibles = gef_conn[n_ef].poss_PC;
		annotations = gef_conn[n_ef].poss_PC_annotation_id;
		break;
	case POSS_ADD:
		n = gef_conn[n_ef].num_poss_A;
		possibles = gef_conn[n_ef].poss_A;
		annotations = gef_conn[n_ef].poss_A_annotation_id;
		break;
	case POSS_DEL:
		n = gef_conn[n_ef].num_poss_D;
		possibles = gef_conn[n_ef].poss_D;
		annotations = gef_conn[n_ef].poss_D_annotation_id;
		break;
	}

	for (int i=0;i<n;i++) {
		if (possibles[i] == ft) {
			res = annotations[i];
			break;
		}
	}

	assert(res > 0);
	return res;
}

// Get predicate of a proposition
int get_predicate(int pro) {
	assert(pro >=0 && pro < gnum_relevant_facts);
	return grelevant_facts[pro].predicate;
}

bool applicable_action(int action, const State* s) {
	assert(s);
	assert(action >=0 && action < gnum_op_conn);
	assert(gop_conn[action].num_E == 1);
	int ef = gop_conn[action].E[0];

	// Only check if all known preconditions present in the state
	for (int j=0;j<gef_conn[ef].num_PC;j++) {
		int p = gef_conn[ef].PC[j];
		if (!is_in_state(p, s)) {
			return false;
		}
	}

	return true;
}






/*
 * THIS SOURCE CODE IS SUPPLIED  ``AS IS'' WITHOUT WARRANTY OF ANY KIND, 
 * AND ITS AUTHOR AND THE JOURNAL OF ARTIFICIAL INTELLIGENCE RESEARCH 
 * (JAIR) AND JAIR'S PUBLISHERS AND DISTRIBUTORS, DISCLAIM ANY AND ALL 
 * WARRANTIES, INCLUDING BUT NOT LIMITED TO ANY IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE, AND
 * ANY WARRANTIES OR NON INFRINGEMENT.  THE USER ASSUMES ALL LIABILITY AND
 * RESPONSIBILITY FOR USE OF THIS SOURCE CODE, AND NEITHER THE AUTHOR NOR
 * JAIR, NOR JAIR'S PUBLISHERS AND DISTRIBUTORS, WILL BE LIABLE FOR 
 * DAMAGES OF ANY KIND RESULTING FROM ITS USE.  Without limiting the 
 * generality of the foregoing, neither the author, nor JAIR, nor JAIR's
 * publishers and distributors, warrant that the Source Code will be 
 * error-free, will operate without interruption, or will meet the needs 
 * of the user.
 */






/*********************************************************************
 * File: main.c
 * Description: The main routine for the FastForward Planner.
 *
 * Author: Joerg Hoffmann 2000
 * 
 *********************************************************************/ 








#include "ff.h"

#include "memory.h"
#include "output.h"

#include "parse.h"

#include "inst_pre.h"
#include "inst_easy.h"
#include "inst_hard.h"
#include "inst_final.h"

#include "orderings.h"

#include "relax.h"
#include "search.h"


/*
 * TUAN (begin)
 */
#include <string.h>
#include <string>
#include "mysrc/StripsEncoding.h"
#include "mysrc/RelaxedPlan.h"
#include "mysrc/ClauseSet.h"
#include "mysrc/StochasticLocalSearch.h"

using namespace std;

extern void test_evaluate_plan_robustness(std::string filename, State *initial_state, State *goal_state);
//extern void test_relaxed_planning_graph(string partial_sol_file, State *initial_state, State* goal_state);
extern void test_relaxed_plan(std::string partial_sol_file, State *initial_state, State* goal_state);
extern void test_estimate_robustness();
extern void test_adding_removing_clauses();
extern int gnum_possible_annotations;

// Problem file
string gproblem_file;

// The log file
//tlib::Log glog;


double ginitial_robustness_threshold = 0;

bool gannotations_at_grounded_level = false;

/*
 * TUAN (end)
 */








/*
 *  ----------------------------- GLOBAL VARIABLES ----------------------------
 */












/*******************
 * GENERAL HELPERS *
 *******************/








/* used to time the different stages of the planner
 */
float gtempl_time = 0, greach_time = 0, grelev_time = 0, gconn_time = 0;
float gsearch_time = 0;


/* the command line inputs
 */
struct _command_line gcmd_line;

/* number of states that got heuristically evaluated
 */
int gevaluated_states = 0;

/* maximal depth of breadth first search
 */
int gmax_search_depth = 0;





/***********
 * PARSING *
 ***********/







/* used for pddl parsing, flex only allows global variables
 */
int gbracket_count;
char *gproblem_name;

/* The current input line number
 */
int lineno = 1;

/* The current input filename
 */
char *gact_filename;

/* The pddl domain name
 */
char *gdomain_name = NULL;

/* loaded, uninstantiated operators
 */
PlOperator *gloaded_ops = NULL;

/* stores initials as fact_list 
 */
PlNode *gorig_initial_facts = NULL;

/* not yet preprocessed goal facts
 */
PlNode *gorig_goal_facts = NULL;

/* axioms as in UCPOP before being changed to ops
 */
PlOperator *gloaded_axioms = NULL;

/* the types, as defined in the domain file
 */
TypedList *gparse_types = NULL;

/* the constants, as defined in domain file
 */
TypedList *gparse_constants = NULL;

/* the predicates and their arg types, as defined in the domain file
 */
TypedListList *gparse_predicates = NULL;

/* the objects, declared in the problem file
 */
TypedList *gparse_objects = NULL;


/* connection to instantiation ( except ops, goal, initial )
 */

/* all typed objects 
 */
FactList *gorig_constant_list = NULL;

/* the predicates and their types
 */
FactList *gpredicates_and_types = NULL;












/*****************
 * INSTANTIATING *
 *****************/









/* global arrays of constant names,
 *               type names (with their constants),
 *               predicate names,
 *               predicate aritys,
 *               defined types of predicate args
 */
Token gconstants[MAX_CONSTANTS];
int gnum_constants = 0;
Token gtype_names[MAX_TYPES];
int gtype_consts[MAX_TYPES][MAX_TYPE];
Bool gis_member[MAX_CONSTANTS][MAX_TYPES];
int gmember_nr[MAX_CONSTANTS][MAX_TYPES];/* nr of object within a type */
int gtype_size[MAX_TYPES];
int gnum_types = 0;
Token gpredicates[MAX_PREDICATES];
int garity[MAX_PREDICATES];
int gpredicates_args_type[MAX_PREDICATES][MAX_ARITY];
int gnum_predicates = 0;





/* the domain in integer (Fact) representation
 */
Operator_pointer goperators[MAX_OPERATORS];
int gnum_operators = 0;
Fact *gfull_initial;
int gnum_full_initial = 0;
WffNode *ggoal = NULL;




/* stores inertia - information: is any occurence of the predicate
 * added / deleted in the uninstantiated ops ?
 */
Bool gis_added[MAX_PREDICATES];
Bool gis_deleted[MAX_PREDICATES];

/*
 * TUAN (begin)
 */
Bool gis_poss_added[MAX_PREDICATES];
Bool gis_poss_deleted[MAX_PREDICATES];
/*
 * TUAN (end)
 */

/* splitted initial state:
 * initial non static facts,
 * initial static facts, divided into predicates
 * (will be two dimensional array, allocated directly before need)
 */
Facts *ginitial = NULL;
int gnum_initial = 0;
Fact **ginitial_predicate;
int *gnum_initial_predicate;



/* the type numbers corresponding to any unary inertia
 */
int gtype_to_predicate[MAX_PREDICATES];
int gpredicate_to_type[MAX_TYPES];

/* (ordered) numbers of types that new type is intersection of
 */
TypeArray gintersected_types[MAX_TYPES];
int gnum_intersected_types[MAX_TYPES];



/* splitted domain: hard n easy ops
 */
Operator_pointer *ghard_operators;
int gnum_hard_operators;
NormOperator_pointer *geasy_operators;
int gnum_easy_operators;



/* so called Templates for easy ops: possible inertia constrained
 * instantiation constants
 */
EasyTemplate *geasy_templates;
int gnum_easy_templates;



/* first step for hard ops: create mixed operators, with conjunctive
 * precondition and arbitrary effects
 */
MixedOperator *ghard_mixed_operators;
int gnum_hard_mixed_operators;



/* hard ''templates'' : pseudo actions
 */
PseudoAction_pointer *ghard_templates;
int gnum_hard_templates;



/* store the final "relevant facts"
 */
Fact grelevant_facts[MAX_RELEVANT_FACTS];
int gnum_relevant_facts = 0;
int gnum_pp_facts = 0;



/* the final actions and problem representation
 */
Action *gactions;
int gnum_actions;
State ginitial_state;
State ggoal_state;









/**********************
 * CONNECTIVITY GRAPH *
 **********************/







/* one ops (actions) array ...
 */
OpConn *gop_conn;
int gnum_op_conn;



/* one effects array ...
 */
EfConn *gef_conn;
int gnum_ef_conn;



/* one facts array.
 */
FtConn *gft_conn;
int gnum_ft_conn;









/*******************
 * SEARCHING NEEDS *
 *******************/








/* the goal state, divided into subsets
 */
State *ggoal_agenda;
int gnum_goal_agenda;



/* byproduct of fixpoint: applicable actions
 */
int *gA;
int gnum_A;



/* communication from extract 1.P. to search engines:
 * 1P action choice
 */
int *gH;
int gnum_H;



/* the effects that are considered true in relaxed plan
 */
int *gin_plan_E;
int gnum_in_plan_E;



/* always stores (current) serial plan
 */
int gplan_ops[MAX_PLAN_LENGTH];
int gnum_plan_ops = 0;



/* stores the states that the current plan goes through
 * ( for knowing where new agenda entry starts from )
 */
State gplan_states[MAX_PLAN_LENGTH + 1];








/*
 *  ----------------------------- HEADERS FOR PARSING ----------------------------
 * ( fns defined in the scan-* files )
 */







void get_fct_file_name( char *filename );
void load_ops_file( char *filename );
void load_fct_file( char *filename );











/*
 *  ----------------------------- MAIN ROUTINE ----------------------------
 */




struct tms lstart, lend;







int main( int argc, char *argv[] )
{

	/* resulting name for ops file
	 */
	char ops_file[MAX_LENGTH] = "";
	/* same for fct file
	 */
	char fct_file[MAX_LENGTH] = "";

	struct tms start, end;

	times ( &lstart );


	/* command line treatment
	 */
	if ( argc == 1 || ( argc == 2 && *++argv[0] == '?' ) ) {
		ff_usage();
		exit( 1 );
	}
	if ( !process_command_line( argc, argv ) ) {
		ff_usage();
		exit( 1 );
	}


	/* make file names
	 */

	/* one input name missing
	 */
	if ( !gcmd_line.ops_file_name ||
			!gcmd_line.fct_file_name ) {
		fprintf(stdout, "\nff: two input files needed\n\n");
		ff_usage();
		exit( 1 );
	}
	/* add path info, complete file names will be stored in
	 * ops_file and fct_file
	 */
	sprintf(ops_file, "%s%s", gcmd_line.path, gcmd_line.ops_file_name);
	sprintf(fct_file, "%s%s", gcmd_line.path, gcmd_line.fct_file_name);

	/*
	 * TUAN (begin)
	 */
	string logfile(gcmd_line.path_to_experiment_result_files);
	logfile += string(gcmd_line.log_file);
	FILE *log = fopen(logfile.c_str(), "a");
	if (!log) {
		printf("Cannot open log file: %s\n", logfile.c_str());
		exit(1);
	}
	fprintf(log, "========================================================\n");
	fprintf(log, "DOMAIN: %s\n\n", gcmd_line.ops_file_name);
	fprintf(log, "PROBLEM: %s\n\n", gcmd_line.fct_file_name);
	/*
	 * TUAN (end)
	 */


	/* parse the input files
	 */

	/* start parse & instantiation timing
	 */
	times( &start );
	/* domain file (ops)
	 */
	if ( gcmd_line.display_info >= 1 ) {
		printf("\nff: parsing domain file");
	}
	/* it is important for the pddl language to define the domain before
	 * reading the problem
	 */
	load_ops_file( ops_file );





	/* problem file (facts)
	 */
	if ( gcmd_line.display_info >= 1 ) {
		printf(" ... done.\nff: parsing problem file");
	}
	load_fct_file( fct_file );
	if ( gcmd_line.display_info >= 1 ) {
		printf(" ... done.\n\n");
	}

	/* This is needed to get all types.
	 */
	build_orig_constant_list();

	/* last step of parsing: see if it's an ADL domain!
	 */
	if ( !make_adl_domain() ) {
		printf("\nff: this is not an ADL problem!");
		printf("\n    can't be handled by this version.\n\n");
		exit( 1 );
	}


	/* now instantiate operators;
	 */


	/**************************
	 * first do PREPROCESSING *
	 **************************/


	/* start by collecting all strings and thereby encoding
	 * the domain in integers.
	 */
	encode_domain_in_integers();		// TUAN: "goperators" created inside.

	// TUAN: OK until now

	/* inertia preprocessing, first step:
	 *   - collect inertia information
	 *   - split initial state into
	 *        _ arrays for individual predicates
	 *        - arrays for all static relations
	 *        - array containing non - static relations
	 */
	do_inertia_preprocessing_step_1();

	/* normalize all PL1 formulae in domain description:
	 * (goal, preconds and effect conditions)
	 *   - simplify formula
	 *   - expand quantifiers
	 *   - NOTs down
	 */
	normalize_all_wffs();					/* TUAN: Currently we don't need to do this for possible preconditions and effects*/

	/* translate negative preconds: introduce symmetric new predicate
	 * NOT-p(..) (e.g., not-in(?ob) in briefcaseworld)
	 */
	translate_negative_preconds();			/* TUAN: Currently we don't have negative (possible) preconditions */


	/* split domain in easy (disjunction of conjunctive preconds)
	 * and hard (non DNF preconds) part, to apply
	 * different instantiation algorithms
	 */
	split_domain();			/* TUAN: "goperators" are partitioned into two types: hard ("ghard_operators") and easy "geasy_operators".
							With STRIPS domains, all actions are classified as "easy"*/


	/***********************************************
	 * PREPROCESSING FINISHED                      *
	 *                                             *
	 * NOW MULTIPLY PARAMETERS IN EFFECTIVE MANNER *
	 ***********************************************/

	build_easy_action_templates();		/* TUAN: For STRIPS, all actions will be processed here, ...*/
	build_hard_action_templates();		/* ... not here */

	times( &end );
	TIME( gtempl_time );

	times( &start );

	/* perform reachability analysis in terms of relaxed
	 * fixpoint
	 */
	perform_reachability_analysis();		/*TUAN: possibly positive relevant facts are created*/

	// TUAN: OK until now

	times( &end );
	TIME( greach_time );

	times( &start );

	/* collect the relevant facts and build final domain
	 * and problem representations.
	 */
	collect_relevant_facts();				/*TUAN: possibly negative relevant facts created; Is it true: relevant facts do not include those in initial state?
											initial state, goals, instantiated actions created.*/

	// TUAN: OK until now

	times( &end );
	TIME( grelev_time );

	times( &start );

	/* now build globally accessible connectivity graph
	 */
	build_connectivity_graph();

	times( &end );
	TIME( gconn_time );

	/******************************************************************************************************************
	 * TUAN: Begin search for robust plans
	 ******************************************************************************************************************/
	gproblem_file = string(gcmd_line.fct_file_name);

	// If the annotations are supposed to be at the grounded level, then we reinitialize the boolean variables
	// associated with possible preconditions and effects.
	if (true || gannotations_at_grounded_level) {
		gnum_possible_annotations = 0;
		for (int i = 0; i < gnum_ef_conn; i++ ) {
			if ( gef_conn[i].removed ) continue;

			for (int j = 0; j < gef_conn[i].num_poss_PC; j++ )
				gef_conn[i].poss_PC_annotation_id[j] = ++gnum_possible_annotations;

			for (int j = 0; j < gef_conn[i].num_poss_A; j++ )
				gef_conn[i].poss_A_annotation_id[j] = ++gnum_possible_annotations;

			for (int j = 0; j < gef_conn[i].num_poss_D; j++ )
				gef_conn[i].poss_D_annotation_id[j] = ++gnum_possible_annotations;
		}
	}

	// Weights of annotations. Uniform just for now!
	// The weights have not been updated directly from the parser
	vector<double> weights(gnum_possible_annotations, 0.5);
	Clause::set_weights(weights);

	// Start the search
	StochasticLocalSearch search(&ginitial_state, &ggoal_state, ginitial_robustness_threshold);

	if (search.run(log)) {
		fprintf(log, "\n>> SUCCESSFUL!\n\n");
	}
	else {
		fprintf(log, "\n>> FAILED!\n\n");
	}
	fclose(log);

	return 0;
}











/*
 *  ----------------------------- HELPING FUNCTIONS ----------------------------
 */












void output_planner_info( void )

{

	printf( "\n\ntime spent: %7.2f seconds instantiating %d easy, %d hard action templates",
			gtempl_time, gnum_easy_templates, gnum_hard_mixed_operators );
	printf( "\n            %7.2f seconds reachability analysis, yielding %d facts and %d actions",
			greach_time, gnum_pp_facts, gnum_actions );
	printf( "\n            %7.2f seconds creating final representation with %d relevant facts",
			grelev_time, gnum_relevant_facts );
	printf( "\n            %7.2f seconds building connectivity graph",
			gconn_time );
	printf( "\n            %7.2f seconds searching, evaluating %d states, to a max depth of %d",
			gsearch_time, gevaluated_states, gmax_search_depth );
	printf( "\n            %7.2f seconds total time",
			gtempl_time + greach_time + grelev_time + gconn_time + gsearch_time );

	printf("\n\n");

	exit( 0 );

	print_official_result();

}


FILE *out;

void print_official_result( void )

{

	int i;
	char name[MAX_LENGTH];

	sprintf( name, "%s.soln", gcmd_line.fct_file_name );

	if ( (out = fopen( name, "w")) == NULL ) {
		printf("\n\nCan't open official output file!\n\n");
		return;
	}

	times( &lend );
	fprintf(out, "Time %d\n",
			(int) ((lend.tms_utime - lstart.tms_utime + lend.tms_stime - lstart.tms_stime) * 10.0));

	for ( i = 0; i < gnum_plan_ops; i++ ) {
		print_official_op_name( gplan_ops[i] );
		fprintf(out, "\n");
	}

	fclose( out );

}



void print_official_op_name( int index )

{

	int i;
	Action *a = gop_conn[index].action;

	if ( a->norm_operator ||
			a->pseudo_action ) {
		fprintf(out, "(%s", a->name );
		for ( i = 0; i < a->num_name_vars; i++ ) {
			fprintf(out, " %s", gconstants[a->name_inst_table[i]]);
		}
		fprintf(out, ")");
	}

}



void ff_usage( void )

{

	printf("\nusage of ff:\n");

	printf("\nOPTIONS   DESCRIPTIONS\n\n");
	printf("-p <str>    path for operator and fact file\n");
	printf("-o <str>    operator file name\n");
	printf("-f <str>    fact file name\n\n");
	printf("-i <num>    run-time information level( preset: 1 )\n");
	printf("      0     only times\n");
	printf("      1     problem name, planning process infos\n");
	printf("    101     parsed problem data\n");
	printf("    102     cleaned up ADL problem\n");
	printf("    103     collected string tables\n");
	printf("    104     encoded domain\n");
	printf("    105     predicates inertia info\n");
	printf("    106     splitted initial state\n");
	printf("    107     domain with Wff s normalized\n");
	printf("    108     domain with NOT conds translated\n");
	printf("    109     splitted domain\n");
	printf("    110     cleaned up easy domain\n");
	printf("    111     unaries encoded easy domain\n");
	printf("    112     effects multiplied easy domain\n");
	printf("    113     inertia removed easy domain\n");
	printf("    114     easy action templates\n");
	printf("    115     cleaned up hard domain representation\n");
	printf("    116     mixed hard domain representation\n");
	printf("    117     final hard domain representation\n");
	printf("    118     reachability analysis results\n");
	printf("    119     facts selected as relevant\n");
	printf("    120     final domain and problem representations\n");
	printf("    121     connectivity graph\n");
	printf("    122     fixpoint result on each evaluated state\n");
	printf("    123     1P extracted on each evaluated state\n");
	printf("    124     H set collected for each evaluated state\n");
	printf("    125     False sets of goals <GAM>\n");
	printf("    126     detected ordering constraints leq_h <GAM>\n");
	printf("    127     the Goal Agenda <GAM>\n");



	/*   printf("    109     reachability analysis results\n"); */
	/*   printf("    110     final domain representation\n"); */
	/*   printf("    111     connectivity graph\n"); */
	/*   printf("    112     False sets of goals <GAM>\n"); */
	/*   printf("    113     detected ordering constraints leq_h <GAM>\n"); */
	/*   printf("    114     the Goal Agenda <GAM>\n"); */
	/*   printf("    115     fixpoint result on each evaluated state <1Ph>\n"); */
	/*   printf("    116     1P extracted on each evaluated state <1Ph>\n"); */
	/*   printf("    117     H set collected for each evaluated state <1Ph>\n"); */

	printf("\n-d <num>    switch on debugging\n\n");

}



Bool process_command_line( int argc, char *argv[] )
{

	char option;

	gcmd_line.display_info = 1;
	gcmd_line.debug = 0;

	memset(gcmd_line.ops_file_name, 0, MAX_LENGTH);
	memset(gcmd_line.fct_file_name, 0, MAX_LENGTH);
	memset(gcmd_line.path, 0, MAX_LENGTH);

	while ( --argc && ++argv ) {

		if ( *argv[0] != '-') {
			return FALSE;
		}

		if (strlen(*argv) == 2) {
			option = *++argv[0];
			switch ( option ) {
			default:
				if ( --argc && ++argv ) {
					switch ( option ) {
					case 'p':
						strncpy( gcmd_line.path, *argv, MAX_LENGTH );
						break;
					case 'o':
						strncpy( gcmd_line.ops_file_name, *argv, MAX_LENGTH );
						break;
					case 'f':
						strncpy( gcmd_line.fct_file_name, *argv, MAX_LENGTH );
						break;
					case 'i':
						sscanf( *argv, "%d", &gcmd_line.display_info );
						break;
					case 'd':
						sscanf( *argv, "%d", &gcmd_line.debug );
						break;

					/*
					 * TUAN (begin)
					 */
					case 'w':
						strncpy(gcmd_line.experiment_analysis_file_for_complete_run, *argv, MAX_LENGTH );
						break;

					case 'u':
						strncpy(gcmd_line.experiment_analysis_file, *argv, MAX_LENGTH );
						break;

					case 'l':
						strncpy(gcmd_line.log_file, *argv, MAX_LENGTH );
						break;

					/*
					 * TUAN (end)
					 */

					default:
						printf( "\nff: unknown option: %c entered\n\n", option );
						return FALSE;
					}
				} else {
					return FALSE;
				}
				break;
			}
		}
		else {
			char str_option[80];
			strcpy(str_option,*argv);

			// Robustness evaluation
			if (strcmp(str_option,"-r") == 0) {
				gcmd_line.robustness_evaluation = true;
				continue;
			}

			// If the annotations are interpreted as at the grounded level
			// By default, it must be at schema level
			if (strcmp(str_option,"-annotations_at_grounded_level") == 0) {
				gannotations_at_grounded_level = true;
				continue;
			}

			/*
			 * OPTIONS RELATED TO RELAXED PLAN EXTRACTION
			 */

			// ignore_poss_del_in_rp;
			// Default: true
			if (strcmp(str_option,"-poss_del_in_rp") == 0) {
				RelaxedPlan::ignore_poss_del_in_rp = false;
				continue;
			}

			// use_lower_bound_in_rp;
			// Default: false
			if (strcmp(str_option,"-lower_bound_in_rp") == 0) {

				if (RelaxedPlan::use_upper_bound_in_rp) {
					printf("Upper and lower bounds cannot be both used.\n\n");
					return FALSE;
				}

				RelaxedPlan::use_lower_bound_in_rp = true;
				continue;
			}

			// use_upper_bound_in_rp;
			// Default: false
			if (strcmp(str_option,"-upper_bound_in_rp") == 0) {

				if (RelaxedPlan::use_lower_bound_in_rp) {
					printf("Upper and lower bounds cannot be both used.\n\n");
					return FALSE;
				}

				RelaxedPlan::use_upper_bound_in_rp = true;
				continue;
			}

			// candidate_actions_affect_current_actions;
			// Default: true
			if (strcmp(str_option,"-candidate_actions_not_affect_current_actions") == 0) {

				RelaxedPlan::candidate_actions_affect_current_actions = false;
				continue;
			}

			// current_actions_affect_candidate_action
			// Default: true
			if (strcmp(str_option,"-current_actions_not_affect_candidate_action") == 0) {

				RelaxedPlan::current_actions_affect_candidate_action = false;
				continue;
			}

			// clauses_from_rpg_for_false_preconditions;
			// Default: true
			if (strcmp(str_option,"-no_clauses_from_rpg_for_false_preconditions") == 0) {

				RelaxedPlan::clauses_from_rpg_for_false_preconditions = false;
				continue;
			}

			// rp_types
			// Default: INCREMENTAL_ROBUSTNESS_RP
			if (strcmp(str_option,"-pure_ff_rp") == 0) {

				RelaxedPlan::rp_types = RelaxedPlan::PURE_FF_RP;
				continue;
			}

			if (strcmp(str_option,"-annotations_free_ff_rp") == 0) {

				RelaxedPlan::rp_types = RelaxedPlan::ANNOTATIONS_FREE_FF_RP;
				continue;
			}

			if (strcmp(str_option,"-robust_ff_rp") == 0) {

				RelaxedPlan::rp_types = RelaxedPlan::ROBUST_FF_RP;
				continue;
			}

			if (strcmp(str_option,"-all_most_robust_supporting_actions") == 0) {

				RelaxedPlan::rp_types = RelaxedPlan::ALL_MOST_ROBUST_SUPPORTING_ACTIONS_RP;
				continue;
			}

			if (strcmp(str_option,"-locally_incremental_robustness") == 0) {

				RelaxedPlan::rp_types = RelaxedPlan::LOCALLY_INCREMENTAL_ROBUSTNESS_RP;
				continue;
			}

			if (strcmp(str_option,"-greedy_robustness_rp") == 0) {

				RelaxedPlan::rp_types = RelaxedPlan::GREEDY_ROBUSTNESS_RP;
				continue;
			}

			if (--argc && ++argv) {
				if (strcmp(str_option,"-s") == 0) {
					strcpy(gcmd_line.solution_file,*argv);
				}

				if (strcmp(str_option,"-path_to_experiment_result_files") == 0) {
					strncpy( gcmd_line.path_to_experiment_result_files, *argv, MAX_LENGTH );
				}

				if (strcmp(str_option,"-path_to_plan_files") == 0) {
					strncpy( gcmd_line.path_to_plan_files, *argv, MAX_LENGTH );
				}

				if (strcmp(str_option,"-wmc_file") == 0) {
					strncpy( gcmd_line.wmc_file, *argv, MAX_LENGTH );
				}

				/*
				 * OPTIONS RELATED TO SEARCH
				 */

				if (strcmp(str_option,"-max_restarts") == 0) {
					sscanf(*argv, "%d", &StochasticLocalSearch::max_restarts);
				}

				if (strcmp(str_option,"-max_iterations") == 0) {
					sscanf(*argv, "%d", &StochasticLocalSearch::max_iterations);
				}

				if (strcmp(str_option,"-initial_depth_bound") == 0) {
					sscanf(*argv, "%d", &StochasticLocalSearch::initial_depth_bound);
				}

				if (strcmp(str_option,"-probes_at_depth") == 0) {
					sscanf(*argv, "%d", &StochasticLocalSearch::probes_at_depth);
				}

				if (strcmp(str_option,"-neighborhood_size") == 0) {
					sscanf(*argv, "%d", &StochasticLocalSearch::neighborhood_size);
				}

				if (strcmp(str_option,"-fail_bound") == 0) {
					sscanf(*argv, "%d", &StochasticLocalSearch::fail_bound);
				}

				if (strcmp(str_option,"-initial_robustness_threshold") == 0) {
					sscanf(*argv, "%lf", &ginitial_robustness_threshold);
				}


			}
		}
	}

	return TRUE;

}


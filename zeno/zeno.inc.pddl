(define (domain zeno-travel)
(:requirements :typing)
(:types aircraft person city flevel dummy_type - object)
(:predicates (at ?x - (either person aircraft) ?c - city)
             (in ?p - person ?a - aircraft)
	     (fuel-level ?a - aircraft ?l - flevel)
	     (next ?l1 ?l2 - flevel)
	     (dummy_pred ?v - dummy_type)
	     )


(:action board
 :parameters (?p - person ?a - aircraft ?c - city ?v - dummy_type)
 
 :precondition (and (at ?p ?c)
                 (at ?a ?c)
                 )
                 
 :possprecondition (and ) 
                 
 :effect (and (not (at ?p ?c))
              (in ?p ?a))
              
 :posseffect (and (not (dummy_pred ?v)))
)

(:action debark
 :parameters (?p - person ?a - aircraft ?c - city ?v - dummy_type)

 :precondition (and (in ?p ?a)
                 (at ?a ?c))
                 
 :possprecondition (and )
  
 :effect (and (not (in ?p ?a))
              (at ?p ?c)
             )

 :posseffect (and (dummy_pred ?v))              
)

(:action fly 
 :parameters (?a - aircraft ?c1 ?c2 - city ?l1 ?l2 - flevel ?v - dummy_type)
 
 :precondition (and (at ?a ?c1)
 					(fuel-level ?a ?l1)
                    (next ?l2 ?l1))
 
 :possprecondition (and (dummy_pred ?v))
 
 :effect (and (not (at ?a ?c1))
              (at ?a ?c2)
              (not (fuel-level ?a ?l1))
              (fuel-level ?a ?l2))
 :posseffect (and )              
)
                                  
(:action zoom
 :parameters (?a - aircraft ?c1 ?c2 - city ?l1 ?l2 ?l3 - flevel)

 :precondition (and (at ?a ?c1)
                 (fuel-level ?a ?l1)
		 (next ?l2 ?l1)
		 (next ?l3 ?l2)
		)
		
 :possprecondition (and )
 
 :effect (and (not (at ?a ?c1))
              (at ?a ?c2)
              (not (fuel-level ?a ?l1))
              (fuel-level ?a ?l3)
	)
 :posseffect (and )
) 

(:action refuel
 :parameters (?a - aircraft ?c - city ?l - flevel ?l1 - flevel)

 :precondition (and (fuel-level ?a ?l)
                 (next ?l ?l1)
                 (at ?a ?c))
                 
 :possprecondition (and )
 
 :effect (and (fuel-level ?a ?l1) (not (fuel-level ?a ?l)))
 
 :posseffect (and ) 
)


)

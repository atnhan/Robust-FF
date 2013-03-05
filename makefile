#!/bin/sh
#
# Makefile for FF v 1.0
#


####### FLAGS

TYPE	= 
ADDONS	= 

CC      = g++

#CFLAGS	= -O6 -Wall -g -ansi $(TYPE) $(ADDONS) 

# Debug
CFLAGS	= -g3 -Wall -ansi $(TYPE) $(ADDONS)

LIBS    = -lm

MY_LIBS = mysrc/mysrc.a

SUB_DIRS =	mysrc

####### Files


PDDL_PARSER_OBJ = scan-fct_pddl.tab.o \
	scan-ops_pddl.tab.o 


SOURCES 	= main.cpp \
	memory.c \
	output.c \
	parse.c \
	inst_pre.c \
	inst_easy.c \
	inst_hard.c \
	inst_final.c \
	orderings.c \
	relax.c \
	search.c

OBJECTS 	= $(SOURCES:.c=.o)

####### Implicit rules

.SUFFIXES:

.SUFFIXES: .c .o

.c.o:; $(CC) -c $(CFLAGS) $<

####### Build rules


ff: COMPILE_SUB $(OBJECTS) $(PDDL_PARSER_OBJ)
	$(CC) -o ff $(OBJECTS) $(PDDL_PARSER_OBJ) $(MY_LIBS) $(LIBS) $(CFLAGS)
COMPILE_SUB:
	@ for i in $(SUB_DIRS); do $(MAKE) -C $$i; done

# misc
clean:
	rm -f *.o *.bak *~ *% core *_pure_p9_c0_400.o.warnings \
        \#*\# $(RES_PARSER_SRC) $(PDDL_PARSER_SRC)

veryclean: clean
	rm -f ff H* J* K* L* O* graph.* *.symbex gmon.out \
	$(PDDL_PARSER_SRC) \
	lex.fct_pddl.c lex.ops_pddl.c lex.probname.c \
	*.output

depend:
	makedepend -- $(SOURCES) $(PDDL_PARSER_SRC)

lint:
	lclint -booltype Bool $(SOURCES) 2> output.lint

# DO NOT DELETE

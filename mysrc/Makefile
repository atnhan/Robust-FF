#####################
# Makefile for LPG  #
#####################


####### FLAGS

#CFLAGS = -g -DNDEBUG

# For debug
#CC	= g++ -g3

# For release
CC	= g++ -O6 -DNDEBUG

####### COMPILE

BOOST_HEADER = /usr/local/boost_1_53_0

PATH_HEADERS = -I$(BOOST_HEADER) -I$(SRC_HEADER)

SRC_HEADER = .

SOURCES =	Clause.cpp ClauseSet.cpp RelaxedPlan.cpp \
			StripsEncoding.cpp Helpful.cpp ClauseSet_test.cpp \
			Search.cpp StochasticLocalSearch.cpp

OBJECTS = $(SOURCES:.cpp=.o)

find_header = $(wildcard $(dir)/*.h)

HEADERS := $(foreach dir,.,$(find_header))

####### Implicit rules

.SUFFIXES:

.SUFFIXES: .cpp .o

.cpp.o:; $(CC) $(PATH_HEADERS) -c $(CFLAGS) $<


####### Build rules

mysrc.a: 	$(OBJECTS)
		$(AR) rc $@ $^

$(OBJECTS): $(HEADERS)

clean: 	
	rm -f *.o *~ *.a .#* 
	
#######

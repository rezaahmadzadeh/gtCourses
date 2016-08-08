GEN_SRC := rvm.cpp 
GEN_OBJ := $(patsubst %.cpp,%.o, $(GEN_SRC))
CC := g++
FLAGS := -Wall -g -static 

.PHONY: all clean

all: abort basic multi multi-abort truncate

%.o: %.cpp %.c
	$(CC) -Wall -DHAVE_INTTYPES_H -DHAVE_NETINET_IN_H -c -g $< -o $@

abort: abort.o $(GEN_OBJ)
	$(CC) $^ -o $@ $(FLAGS) 

basic: basic.o $(GEN_OBJ)
	$(CC) $^ -o $@ $(FLAGS) 

multi: multi.o $(GEN_OBJ)
	$(CC) $^ -o $@ $(FLAGS) 

multi-abort: multi-abort.o $(GEN_OBJ)
	$(CC) $^ -o $@ $(FLAGS) 

truncate: truncate.o $(GEN_OBJ)
	$(CC) $^ -o $@ $(FLAGS) 

clean:
	$(RM) *.o rvm abort basic multi multi-abort truncate


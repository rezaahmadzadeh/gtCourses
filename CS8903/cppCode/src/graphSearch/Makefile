ROOT_DIR:=/home/abenbihi/gtCourses/CS8903/
GRAPH_DIR:=$(ROOT_DIR)cppCode/src/graphSearch/
MISC_DIR:= $(ROOT_DIR)cppCode/src/misc/

SRC:= test.cpp $(GRAPH_DIR)GraphSearch.cpp $(MISC_DIR)misc.cpp
OBJ:= $(patsubst %.cpp, %.o, $(SRC))
CXX:= g++
CXXFLAGS:= -Wall -g  

INC:= -I/usr/include/boost\
    -I$(ROOT_DIR)cppCode/inc\
    -I$(ROOT_DIR)cppCode/src/ikfast\
    -I$(GRAPH_DIR)\
    -I$(MISC_DIR)

LIB:= -lstdc++ -lrt -L/usr/lib/lapack/ -llapack

.PHONY: all clean

all: test

%.o: %.cpp 
	$(CXX) $(CXXFLAGS) $(INC) -c $< -o $@ 

test: $(OBJ) 
	$(CXX) $^ $(INC) -o $@ $(LIB) $(CXXFLAGS)

cleanData:
	$(RM) data/*.csv data/*.txt

clean:
	$(RM) *.o *~ test

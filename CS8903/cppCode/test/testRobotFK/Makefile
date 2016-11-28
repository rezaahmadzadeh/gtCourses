ROOT_DIR:=/home/abenbihi/gtCourses/CS8903/
SRC:= robot.cpp
OBJ:= $(patsubst %.cpp, %.o, $(SRC))
CXX:= g++
CXXFLAGS:= -Wall -g  

INC:= -I/usr/include/boost -I$(ROOT_DIR)cppCode/inc -I$(ROOT_DIR)cppCode/src/ikfast

LIB:= -lstdc++ -lrt -L/usr/lib/lapack/ -llapack

.PHONY: all clean cleanData

all: robot

%.o: %.cpp 
	$(CXX) $(CXXFLAGS) $(INC) -c $< -o $@ 

robot: $(OBJ) 
	$(CXX) $(INC) $^ -o $@ $(LIB) $(CXXFLAGS)

cleanData:
	$(RM) data/*.csv data/*.txt

clean:
	$(RM) *.o *~ robot 


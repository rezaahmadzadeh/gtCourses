
SRC:= robot.cpp
OBJ:= $(patsubst %.cpp, %.o, $(SRC))
CXX:= g++
CXXFLAGS:= -Wall -g  

INC:= -I/usr/include/boost

LIB:= -lstdc++ -lrt -L/usr/lib/lapack/ -llapack

.PHONY: all clean

all: robot

%.o: %.cpp 
	$(CXX) $(CXXFLAGS) -c $< -o $@ 

robot: $(OBJ) 
	$(CXX) $^ -o $@ $(LIB) $(CXXFLAGS)

clean:
	$(RM) *.o *.~ robot test_4_3/*.csv


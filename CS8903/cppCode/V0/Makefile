
SRC:= ikDataSampleCSV.cpp
OBJ:= $(patsubst %.cpp, %.o, $(SRC))
CXX:= g++
CXXFLAGS:= -Wall -g  

INC:= -I/iusr/include/boost 

LIB:= -lstdc++ -llapack -lrt -L/usr/lib/x86_64-linux-gnu/

.PHONY: all clean

all: ikDataSampleCSV

%.o: %.cpp 
	$(CXX) $(CXXFLAGS) -c $< -o $@ 

ikDataSampleCSV: $(OBJ) 
	$(CXX) $^ -o $@ $(LIB) $(CXXFLAGS)

clean:
	$(RM) *.o *.~ *.csv


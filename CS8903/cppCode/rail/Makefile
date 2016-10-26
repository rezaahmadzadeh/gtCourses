SRC := robot.cpp
OBJ := $(patsubst %.cpp, %.o,$(SRC))
CXX := g++
CXXFLAGS := -Wall -g -static

.PHONY: all clean

all: robot

%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

robot: $(OBJ)
	$(CXX) $^ -o $@ $(CXXFLAGS)

clean:
	$(RM) *.o *.~ robot

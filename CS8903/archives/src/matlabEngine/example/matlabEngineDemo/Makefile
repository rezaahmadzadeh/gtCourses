MATLABROOT := /usr/local/MATLAB/R2016a/
SRC:= engdemo.cpp
OBJ:= $(patsubst %.cpp,%.o, $(SRC))
CXX:= g++
CXXFLAGS:= -I$(MATLABROOT)extern/include
LDLIBS:= -L$(MATLABROOT)extern/lib -L$(MATLABROOT)bin/glnxa64 -leng -lmx

.PHONY: clean

all: engdemo

engdemo:
	$(CXX) engdemo.cpp -o engdemo $(CXXFLAGS) $(LDLIBS) 

clean:
	$(RM) engdemo *.~ *.o

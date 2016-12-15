#####################
# TODO WARNING !!!!!
# Run the following command before compiling: LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/MATLAB/R2016a/bin/glnxa64
# Run the following command before running: export PATH=$PATH:/usr/local/MATLAB/R2016a/bin
#####################

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
	$(RM) engdemo *~ *.o

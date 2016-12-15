# Makefile for the icosahedron assignmnet
# George F. Riley, ECE6122 Fall 2015

all: icosahedron

icosahedron:	icosahedron.o
	$(CXX) -g -o icosahedron icosahedron.o -L/usr/lib64 -lglut -lGLU -lGL

icosahedron.o:	icosahedron.cc
	$(CXX) -c -g icosahedron.cc

clean:
	@rm -f icosahedron icosahedron.o

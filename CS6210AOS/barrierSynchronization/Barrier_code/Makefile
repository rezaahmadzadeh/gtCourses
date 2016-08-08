#OpenMP Flags Etc.
OMPFLAGS = -g -Wall -fopenmp -DLEVEL1_DCACHE_LINESIZE=`getconf LEVEL1_DCACHE_LINESIZE`
OMPLIBS = -lgomp -lm
CC = gcc
CPPFLAGS = -g -Wall $(OMPFLAGS)
LDFLAGS = -g -Wall
LDLIBS = $(OMPLIBS) -lm

#MPI Flags Etc (may need to customize)
#MPICH = /usr/lib64/openmpi/1.4-gcc
MPICH = /usr/lib/openmpi
MPIFLAGS = -g -Wall -I$(MPICH)/include
MPILIBS =
#MPICC = /opt/openmpi-1.4.3-gcc44/bin/mpicc
MPICC = mpicc

all: stour mcs tournament SRB SRT

stour: stour.c
	$(MPICC) $(OMPFLAGS) -o $@ $^ $(OMPLIBS)

tournament: tournament.c
	$(MPICC) -o $@ $(OMPFLAGS) $^ $(OMPLIBS)

mcs: mcs.c
	$(MPICC) -o $@ $(OMPFLAGS) $^ $(OMPLIBS)

SRB: SRB.o
	$(CC) $(LDFLAGS) -o $@ $^ $(OMPLIBS)

SRT: SRT.o
	$(CC) $(LDFLAGS) -o $@ $^ $(OMPLIBS)

clean:
	rm -rf *.o stour mcs tournament SRB SRT

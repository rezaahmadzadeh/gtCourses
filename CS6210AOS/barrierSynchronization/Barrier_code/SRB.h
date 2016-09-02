#include <omp.h>
#include <stdlib.h>
#include <stdio.h>
#include <sys/time.h>
#include <unistd.h>
#include <string.h>

#define NUM_BARRIERS 100
#define DELAY_RATE 10
#define EB_DELAY 0.0000001
#define GB_DELAY 0.0000001
#define LB_DELAY 0.0000001

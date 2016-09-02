#include <omp.h>
#include <stdlib.h>
#include <stdio.h>
#include<math.h>
#include <sys/time.h>
#include <unistd.h>
#include <string.h>

#define FANIN 2
#define NUM_BARRIERS 25
#define DELAY_RATE 10
#define EB_DELAY 0.1
#define GB_DELAY 0.1
#define LB_DELAY 0.1

int  NUM_THREADS;

typedef struct node {
    int id;
    int k;
    int count;
    int locksense;
    struct node* parent;
} Node;

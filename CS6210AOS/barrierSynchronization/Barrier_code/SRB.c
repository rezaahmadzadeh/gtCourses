#include <omp.h>
#include <stdlib.h>
#include <stdio.h>
#include <sys/time.h>
#include <unistd.h>
#include <string.h>
#include "SRB.h"


/***** TEST 1 : average time for barrier achievement of default barrier *****/
void SRbarrier1 (int* count, int* sense, int* local_sense) {
    *local_sense = !(*local_sense);
#pragma omp critical 
    {
        *count = *count - 1;
        if(*count == 0) {
            *count = omp_get_num_threads();
            *sense = *local_sense;
        }
    }
    while (*sense != *local_sense) {};
}

/***** TEST 2 : average contention for accessing the count i.e critical part *****/
void SRbarrier2 (int* count, int* sense, int* local_sense,double* local_arrival_contention_time) {

    double t_start,t_end;
    *local_sense = !(*local_sense);
    t_start = omp_get_wtime();

#pragma omp critical 
    {
        t_end = omp_get_wtime();
        *count = *count - 1;
        if(*count == 0) {
            //int tid = omp_get_thread_num();
            //printf("I am %d and I am the last one to arrive\n",tid);
            *count = omp_get_num_threads();
            *sense = *local_sense;
        }
    }

    *local_arrival_contention_time = *local_arrival_contention_time + t_end - t_start;
    while (*sense != *local_sense) {};

}

/***** TEST 3 :  measure the part of synchronization instructions in the overhead *****/
void SRbarrier3 (int* count, int* sense, int* local_sense,double* local_cc_time) {
    double t_start,t_end;
    *local_sense = !(*local_sense);
#pragma omp critical 
    {
        t_start = omp_get_wtime();
        *count = *count - 1;
        if(*count == 0) {
            *count = omp_get_num_threads();
            *sense = *local_sense;
        }
        t_end = omp_get_wtime();
    }
    *local_cc_time = *local_cc_time + t_end - t_start;
    while (*sense != *local_sense) {};
}

/***** TEST 4 :  measure the contention of sense spinning *****/
void SRbarrier4 (int* count, int* sense, int* local_sense,unsigned int* local_spin_count) {
    *local_sense = !(*local_sense);
#pragma omp critical 
    {
        *count = *count - 1;
        if(*count == 0) {
            *count = omp_get_num_threads();
            *sense = *local_sense;
        }
    }
    while (*sense != *local_sense) {
        *local_spin_count = *local_spin_count +1 ;
    };
}


/***** TEST 5 :  measure the contention of sense spinning with exponential backoff*****/
void SRbarrier5 (int* count, int* sense, int* local_sense,unsigned int* local_spin_count) {
    *local_sense = !(*local_sense);
#pragma omp critical 
    {
        *count = *count - 1;
        if(*count == 0) {
            *count = omp_get_num_threads();
            *sense = *local_sense;
        }
    }
    int delay = EB_DELAY;
    while (*sense != *local_sense) {
        (*local_spin_count)++;
        usleep(delay);
        delay = delay*delay;
    };
}


/***** TEST 6 :  measure the contention of sense spinning with geometrical backoff GB*****/
void SRbarrier6 (int* count, int* sense, int* local_sense,unsigned int* local_spin_count) {
    *local_sense = !(*local_sense);
#pragma omp critical 
    {
        *count = *count - 1;
        if(*count == 0) {
            *count = omp_get_num_threads();
            *sense = *local_sense;
        }
    }
    int delay = GB_DELAY;
    while (*sense != *local_sense) {
        (*local_spin_count)++;
        usleep(delay);
        delay = delay*DELAY_RATE;
    };
}


/***** TEST 7 :  measure the contention of sense spinning with linear backoff LB*****/
void SRbarrier7 (int* count, int* sense, int* local_sense,unsigned int* local_spin_count) {
    *local_sense = !(*local_sense);
#pragma omp critical 
    {
        *count = *count - 1;
        if(*count == 0) {
            *count = omp_get_num_threads();
            *sense = *local_sense;
        }
    }
    int delay = LB_DELAY;
    while (*sense != *local_sense) {
        (*local_spin_count)++;
        usleep(delay);
        delay = delay + DELAY_RATE;
    };
}




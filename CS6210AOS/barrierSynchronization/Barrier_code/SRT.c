#include <omp.h>
#include <stdlib.h>
#include <stdio.h>
#include<math.h>
#include <sys/time.h>
#include <unistd.h>
#include <string.h>
#include "SRT.h"

/***** TEST 1 : average time for barrier achievement of default barrier *****/
void combining_barrier_aux1(Node* mynode, int* sense,int tree_thread_num,int left_over) {
    int tid = omp_get_thread_num();
    int equality = -1;
#pragma omp critical 
    {
        mynode->count = mynode->count -1;
        equality = (mynode->count == 0); 
    }
    if(equality) {         
        if(tid <= tree_thread_num) {
            if (mynode->parent != NULL) {
                combining_barrier_aux1(mynode->parent,sense, tree_thread_num, left_over);}
            if(mynode->id==0) {
                mynode->count = left_over+FANIN;}
            else {
                mynode->count = FANIN;}
        }
        if(tid > tree_thread_num) {
            mynode->count = left_over+FANIN;}
        mynode->locksense = !(mynode->locksense);
    }

    while(mynode->locksense != *sense);
}

void combining_barrier1(Node* mynode, int* sense,int tree_thread_num,int left_over) {
    combining_barrier_aux1(mynode, sense, tree_thread_num, left_over);
    *sense = !(*sense); 
}


/***** TEST 2 : average contention for accessing the count i.e critical part *****/
void combining_barrier_aux2(Node* mynode, int* sense,int tree_thread_num,int left_over,double* local_arrival_contention_time) {
    int tid = omp_get_thread_num();
    int equality = -1;
double t_start,t_end;
t_start = omp_get_wtime();
#pragma omp critical 
    {
        t_end = omp_get_wtime();
        mynode->count = mynode->count -1;
        equality = (mynode->count == 0); 
    }
    if(equality) { 
        if(tid <= tree_thread_num) {
            if (mynode->parent != NULL) {
                combining_barrier_aux2(mynode->parent,sense, tree_thread_num, left_over, local_arrival_contention_time);}
            if(mynode->id==0) {
                mynode->count = left_over+FANIN;}
            else {
                mynode->count = FANIN;}
        }
        if(tid > tree_thread_num) {
            mynode->count = left_over+FANIN;}
        mynode->locksense = !(mynode->locksense);
    }
*local_arrival_contention_time = *local_arrival_contention_time + t_end - t_start;
    while(mynode->locksense != *sense);
}
void combining_barrier2(Node* mynode, int* sense,int tree_thread_num,int left_over,double* local_arrival_contention_time) {
    combining_barrier_aux2(mynode, sense, tree_thread_num, left_over, local_arrival_contention_time);
    *sense = !(*sense); 
}


/***** TEST 3 :  measure the part of synchronization instructions in the overhead *****/
void combining_barrier_aux3(Node* mynode, int* sense,int tree_thread_num,int left_over,double* local_cc_time) {
    int tid = omp_get_thread_num();
    int equality = -1;
    double t_start,t_end;
#pragma omp critical 
    {
        t_start = omp_get_wtime();
        mynode->count = mynode->count -1;
        equality = (mynode->count == 0); 
        t_end = omp_get_wtime();
    }
    if(equality) {
        if(tid <= tree_thread_num) {
            if (mynode->parent != NULL) {
                combining_barrier_aux3(mynode->parent,sense, tree_thread_num, left_over, local_cc_time);}
            if(mynode->id==0) {
                mynode->count = left_over+FANIN;}
            else {
                mynode->count = FANIN;}
        }
        if(tid > tree_thread_num) {
            mynode->count = left_over+FANIN;}
        mynode->locksense = !(mynode->locksense);
    }
    *local_cc_time = *local_cc_time + t_end - t_start;
    while(mynode->locksense != *sense);
}

void combining_barrier3(Node* mynode, int* sense,int tree_thread_num,int left_over,double* local_cc_time) {
    combining_barrier_aux3(mynode, sense, tree_thread_num, left_over,local_cc_time);
    *sense = !(*sense); 
}


/***** TEST 4 :  measure the contention of sense spinning for each spin data structure*****/
void combining_barrier_aux4(Node* mynode, int* sense,int tree_thread_num,int left_over,unsigned int local_spin_count[]) {
    int tid = omp_get_thread_num();
    int equality = -1;
#pragma omp critical 
    {
        mynode->count = mynode->count -1;
        equality = (mynode->count == 0); 
    }
    if(equality) { 
        if(tid <= tree_thread_num) {
            if (mynode->parent != NULL) {
                combining_barrier_aux4(mynode->parent,sense, tree_thread_num, left_over,  local_spin_count);}
            if(mynode->id==0) {
                mynode->count = left_over+FANIN;}
            else {
                mynode->count = FANIN;}
        }
        if(tid > tree_thread_num) {
            mynode->count = left_over+FANIN;}
        mynode->locksense = !(mynode->locksense);
    }
    while(mynode->locksense != *sense){
        local_spin_count[mynode->id] = local_spin_count[mynode->id]  +1 ; 
    }
}
void combining_barrier4(Node* mynode, int* sense,int tree_thread_num,int left_over,unsigned int local_spin_count[]) {
    combining_barrier_aux4(mynode, sense, tree_thread_num, left_over,  local_spin_count);
    *sense = !(*sense); 
}


/***** TEST 5 :  measure the consequences of dynamically located spin location*****/
void combining_barrier_aux5(Node* mynode, int* sense,int tree_thread_num,int left_over,double spin_times[NUM_BARRIERS][NUM_THREADS],int i) {
    int tid = omp_get_thread_num();
    int equality = -1;
    double t_start,t_end;
#pragma omp critical 
    {
        mynode->count = mynode->count -1;
        equality = (mynode->count == 0); 
    }
    if(equality) { 
        if(tid <= tree_thread_num) {
            if (mynode->parent != NULL) {
                combining_barrier_aux5(mynode->parent,sense, tree_thread_num, left_over,spin_times,i);}
            if(mynode->id==0) {
                mynode->count = left_over+FANIN;}
            else {
                mynode->count = FANIN;}
        }
        if(tid > tree_thread_num) {
            mynode->count = left_over+FANIN;}
        mynode->locksense = !(mynode->locksense);
    }
    t_start = omp_get_wtime();
    while(mynode->locksense != *sense);
    t_end = omp_get_wtime();
    spin_times[i][tid] = spin_times[i][tid] + t_end - t_start;
}

void combining_barrier5(Node* mynode, int* sense,int tree_thread_num,int left_over,double spin_times[NUM_BARRIERS][NUM_THREADS], int i) {
    combining_barrier_aux5(mynode, sense, tree_thread_num, left_over,  spin_times,i);
    *sense = !(*sense); 
}

/***** TEST 6 :  measure the contention of sense spinning for each spin data structure zith exponential delay EB*****/
void combining_barrier_aux6(Node* mynode, int* sense,int tree_thread_num,int left_over,unsigned int local_spin_count[]) {
    int tid = omp_get_thread_num();
    int equality = -1;
#pragma omp critical 
    {
        mynode->count = mynode->count -1;
        equality = (mynode->count == 0);     
    }
    if(equality) { 
        if(tid <= tree_thread_num) {
            if (mynode->parent != NULL) {
                combining_barrier_aux6(mynode->parent,sense, tree_thread_num, left_over,  local_spin_count);}
            if(mynode->id==0) {
                mynode->count = left_over+FANIN;}
            else {
                mynode->count = FANIN;}
        }
        if(tid > tree_thread_num) {
            mynode->count = left_over+FANIN;}
        mynode->locksense = !(mynode->locksense);
    }
    int delay = EB_DELAY;
    while(mynode->locksense != *sense){
        local_spin_count[mynode->id] = local_spin_count[mynode->id]  +1 ; 
        usleep(delay);
        delay = delay*delay;
    }
}

void combining_barrier6(Node* mynode, int* sense,int tree_thread_num,int left_over,unsigned int local_spin_count[]) {
    combining_barrier_aux6(mynode, sense, tree_thread_num, left_over,  local_spin_count);
    *sense = !(*sense); 
}


/***** TEST 7 :  measure the consequences of dynamically located spin location with EB*****/
void combining_barrier_aux7(Node* mynode, int* sense,int tree_thread_num,int left_over,double spin_times[NUM_BARRIERS][NUM_THREADS],int i) {
    int tid = omp_get_thread_num();
    int equality = -1;
    double t_start,t_end;
#pragma omp critical 
    {
        mynode->count = mynode->count -1;
        equality = (mynode->count == 0); 
    }

    if(equality) { 
        if(tid <= tree_thread_num) {
            if (mynode->parent != NULL) {
                combining_barrier_aux7(mynode->parent,sense, tree_thread_num, left_over,spin_times,i);}
            if(mynode->id==0) {
                mynode->count = left_over+FANIN;}
            else {
                mynode->count = FANIN;}
        }
        if(tid > tree_thread_num) {
            mynode->count = left_over+FANIN;}
        mynode->locksense = !(mynode->locksense);
    }
    int delay = EB_DELAY;
    t_start = omp_get_wtime();
    while(mynode->locksense != *sense) {
    usleep(delay);
    delay = delay*delay;
    }
    t_end = omp_get_wtime();
    spin_times[i][tid] = spin_times[i][tid] + t_end - t_start;
}

void combining_barrier7(Node* mynode, int* sense,int tree_thread_num,int left_over,double spin_times[NUM_BARRIERS][NUM_THREADS], int i) {
    combining_barrier_aux7(mynode, sense, tree_thread_num, left_over,  spin_times,i);
    *sense = !(*sense); 
}


/***** TEST 8 :  measure the contention of sense spinning for each spin data structure zith exponential delay GB*****/
void combining_barrier_aux8(Node* mynode, int* sense,int tree_thread_num,int left_over,unsigned int local_spin_count[]) {
    int tid = omp_get_thread_num();
    int equality = -1;
#pragma omp critical 
    {
        mynode->count = mynode->count -1;
        equality = (mynode->count == 0);   
    }
    if(equality) { 
        if(tid <= tree_thread_num) {
            if (mynode->parent != NULL) {
                combining_barrier_aux8(mynode->parent,sense, tree_thread_num, left_over,  local_spin_count);}
            if(mynode->id==0) {
                mynode->count = left_over+FANIN;}
            else {
                mynode->count = FANIN;}
        }
        if(tid > tree_thread_num) {
            mynode->count = left_over+FANIN;}
        mynode->locksense = !(mynode->locksense);
    }
    int delay = GB_DELAY;
    while(mynode->locksense != *sense){
        local_spin_count[mynode->id] = local_spin_count[mynode->id]  +1 ; 
        usleep(delay);
        delay = delay+delay;
    }
}
void combining_barrier8(Node* mynode, int* sense,int tree_thread_num,int left_over,unsigned int local_spin_count[]) {
    combining_barrier_aux8(mynode, sense, tree_thread_num, left_over,  local_spin_count);
    *sense = !(*sense); 
}


/***** TEST 9 :  measure the consequences of dynamically located spin location with GB*****/
void combining_barrier_aux9(Node* mynode, int* sense,int tree_thread_num,int left_over,double spin_times[NUM_BARRIERS][NUM_THREADS],int i) {
    int tid = omp_get_thread_num();
    int equality = -1;
    double t_start,t_end;
#pragma omp critical 
    {
        mynode->count = mynode->count -1;
        equality = (mynode->count == 0); 
    }

    if(equality) { 
        if(tid <= tree_thread_num) {
            if (mynode->parent != NULL) {
                combining_barrier_aux9(mynode->parent,sense, tree_thread_num, left_over,spin_times,i);}
            if(mynode->id==0) {
                mynode->count = left_over+FANIN;}
            else {
                mynode->count = FANIN;}
        }
        if(tid > tree_thread_num) {
            mynode->count = left_over+FANIN;}
        mynode->locksense = !(mynode->locksense);
    }
    int delay = GB_DELAY;
    t_start = omp_get_wtime();
    while(mynode->locksense != *sense) {
    usleep(delay);
    delay = delay+delay;
    }
    t_end = omp_get_wtime();
    spin_times[i][tid] = spin_times[i][tid] + t_end - t_start;
}

void combining_barrier9(Node* mynode, int* sense,int tree_thread_num,int left_over,double spin_times[NUM_BARRIERS][NUM_THREADS], int i) {
    combining_barrier_aux9(mynode, sense, tree_thread_num, left_over,  spin_times,i);
    *sense = !(*sense); 
}


/***** TEST 10 :  measure the contention of sense spinning for each spin data structure zith exponential delay LB*****/
void combining_barrier_aux10(Node* mynode, int* sense,int tree_thread_num,int left_over,unsigned int local_spin_count[]) {
    int tid = omp_get_thread_num();
    int equality = -1;
#pragma omp critical 
    {
        mynode->count = mynode->count -1;
        equality = (mynode->count == 0); 
    }
    if(equality) {
        if(tid <= tree_thread_num) {
            if (mynode->parent != NULL) {
                combining_barrier_aux10(mynode->parent,sense, tree_thread_num, left_over,  local_spin_count);}
            if(mynode->id==0) {
                mynode->count = left_over+FANIN;}
            else {
                mynode->count = FANIN;}
        }
        if(tid > tree_thread_num) {
            mynode->count = left_over+FANIN;}
        mynode->locksense = !(mynode->locksense);
    }
    int delay = LB_DELAY;
    while(mynode->locksense != *sense){
        local_spin_count[mynode->id] = local_spin_count[mynode->id]  +1 ; 
        usleep(delay);
        delay = delay+delay;
    }
}

void combining_barrier10(Node* mynode, int* sense,int tree_thread_num,int left_over,unsigned int local_spin_count[]) {
    combining_barrier_aux10(mynode, sense, tree_thread_num, left_over,  local_spin_count);
    *sense = !(*sense); 
}


/***** TEST 11 :  measure the consequences of dynamically located spin location with LB*****/
void combining_barrier_aux11(Node* mynode, int* sense,int tree_thread_num,int left_over,double spin_times[NUM_BARRIERS][NUM_THREADS],int i) {
    int tid = omp_get_thread_num();
    int equality = -1;
    double t_start,t_end;
#pragma omp critical 
    {
        mynode->count = mynode->count -1;
        equality = (mynode->count == 0); 
    }
    if(equality) {
        if(tid <= tree_thread_num) {
            if (mynode->parent != NULL) {
                combining_barrier_aux11(mynode->parent,sense, tree_thread_num, left_over,spin_times,i);}
            if(mynode->id==0) {
                mynode->count = left_over+FANIN;}
            else {
                mynode->count = FANIN;}
        }
        if(tid > tree_thread_num) {
            mynode->count = left_over+FANIN;}
        mynode->locksense = !(mynode->locksense);
    }
    int delay = LB_DELAY;
    t_start = omp_get_wtime();
    while(mynode->locksense != *sense) {
    usleep(delay);
    delay = delay+delay;
    }
    t_end = omp_get_wtime();
    spin_times[i][tid] = spin_times[i][tid] + t_end - t_start;
    
}

void combining_barrier11(Node* mynode, int* sense,int tree_thread_num,int left_over,double spin_times[NUM_BARRIERS][NUM_THREADS], int i) {
    combining_barrier_aux11(mynode, sense, tree_thread_num, left_over,  spin_times,i);
    *sense = !(*sense); 
}

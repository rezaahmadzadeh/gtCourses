#include <omp.h>
#include <stdlib.h>
#include <stdio.h>
#include<math.h>
#include <sys/time.h>
#include <unistd.h>
#include <string.h>

int  NUM_THREADS;
#define FANIN 2
#define NUM_BARRIERS 25
#define DELAY_RATE 10
#define EB_DELAY 0.1
#define GB_DELAY 0.1
#define LB_DELAY 0.1

int square (int a) {
    return a*a;
}

typedef struct node {
    int id;
    int k;
    int count;
    int locksense;
    struct node* parent;
} Node;


void init_tree(Node tree[],int size_tree, int left_over) {
    int i = 0;
    tree[0].id = i;
    tree[0].k = left_over+FANIN;
    tree[0].count = left_over+2;
    tree[0].locksense = 1;
    tree[0].parent = NULL;
    for (i=1;i<size_tree;i++) {
        tree[i].id = i;
        tree[i].k = FANIN;
        tree[i].count = FANIN;
        tree[i].locksense = 1;
        tree[i].parent = &tree[(i-1)/FANIN];
    }
}

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
/*************************************************************************************************/
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
/*************************************************************************************************/
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
/*************************************************************************************************/
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
/*****************************************************************************************************/
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
/*************************************************************************************************/
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
/*************************************************************************************************/
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
/*************************************************************************************************/
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
/*************************************************************************************************/
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
/***********************************************************************************************************************/
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
/*************************************************************************************************/
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
/*****************************************************************************************************/

int main (int argv, char**argc) {


    /***********************/
    /*** TEST PREPARATION***/
    /*
       int gi;
       FILE *fp1;
       fp1 = fopen("test_tree1.csv","w+");
       fprintf(fp1,"Num_barriers,%d\n",NUM_BARRIERS);
       fprintf(fp1,"Num_thread,GAT,GACT,GCCT\n");

       FILE *fp4;
       fp4 = fopen("test_tree4.csv","w+");
       fprintf(fp4,"GSC : spin count on each node\n");
       for(gi=0;gi<NUM_BARRIERS;gi++){
       fprintf(fp4,"%d,",gi);}
       fprintf(fp4,"\n");

       FILE *fp5;
       fp5 = fopen("test_tree5.csv","w+");
       fprintf(fp5,"Time spent on spinning by thread j on barrier i\n");
       for(gi=0;gi<NUM_THREADS;gi++) {
       fprintf(fp5,"%d,",gi);}
       fprintf(fp5,"\n");

       FILE *fp6;
       fp6 = fopen("test_tree6.csv","w+");

       fprintf(fp6,"EB GSC : spin count on each node\n");
       for(gi=0;gi<NUM_BARRIERS;gi++){
       fprintf(fp6,"%d,",gi);}
       fprintf(fp6,"\n");

       FILE *fp7;
       fp7 = fopen("test_tree7.csv","w+");
       fprintf(fp7,"EBTime spent on spinning by thread j on barrier i\n");
       for(gi=0;gi<NUM_THREADS;gi++) {
       fprintf(fp7,"%d,",gi);}
       fprintf(fp7,"\n");

       FILE *fp8;
       fp8 = fopen("test_tree8.csv","w+");
       fprintf(fp8,"GB GSC : spin count on each node\n");
       for(gi=0;gi<NUM_BARRIERS;gi++){
       fprintf(fp8,"%d,",gi);}
       fprintf(fp8,"\n");

       FILE *fp9;
       fp9 = fopen("test_tree9.csv","w+");
       fprintf(fp9,"GBTime spent on spinning by thread j on barrier i\n");
       for(gi=0;gi<NUM_THREADS;gi++) {
       fprintf(fp9,"%d,",gi);}
       fprintf(fp9,"\n");

       FILE *fp10;
       fp10 = fopen("test_tree10.csv","w+");
       fprintf(fp10,"LB GSC : spin count on each node\n");
       for(gi=0;gi<NUM_BARRIERS;gi++){
       fprintf(fp10,"%d,",gi);}
       fprintf(fp10,"\n");

       FILE *fp11;
       fp11 = fopen("test_tree11.csv","w+");
       for(gi=0;gi<NUM_THREADS;gi++) {
       fprintf(fp11,"%d,",gi);}
       fprintf(fp11,"\n");
       */

    NUM_THREADS=2;
    for(NUM_THREADS=2;NUM_THREADS<=8;NUM_THREADS++) {
        //fprintf(fp1,"%d,",NUM_THREADS);

        omp_set_num_threads(NUM_THREADS);
        int count = NUM_THREADS;
        int h = (int)(log(NUM_THREADS)/log(2));
        int tree_thread_num = pow(2,h)-1;
        int size_tree = (int) pow(2,h)-1;
        int left_over = NUM_THREADS - tree_thread_num-1;
        Node tree[size_tree]; 
        init_tree(tree,size_tree,left_over);
        int tid;
        Node* mynode;
        int sense=0;

        /**************************************************************************************/
        /***** TEST 0 : experimental proof of the correctness of the barrier*****/
        /**************************************************************************************/
#pragma omp parallel shared(tree) private(sense,tid,mynode) 
        { 
            sense = 0;
            tid = omp_get_thread_num();
            int tree_tid = tid + pow(2,h) - 1;
            int parent = (tree_tid -1)/2;
            if(tid >tree_thread_num) {
                parent = 0; }
            mynode = &tree[parent];
            int i=0;
            printf("Hello world from thread %d BEFORE barrier mynode->count is %d\n",tid,mynode->count);
            for(i=0;i<NUM_BARRIERS;i++) {
                combining_barrier1(mynode,&sense, tree_thread_num,left_over);
                printf("Hello World from thread = %d AFTER%d barrier.\n ",tid,i);
            }
        }

        /**************************************************************************************/
        /***** END TEST 0 : experimental proof of the correctness of the barrier *****/
        /**************************************************************************************/
        /*RESET PARAMETERS*/
        sense = 0;
        init_tree(tree,size_tree,left_over);
        /**************************************************************************************/
        /***** TEST 1 : average time for barrier achievement of default barrier *****/
        /**************************************************************************************/
        double global_achievement_time = 0;
#pragma omp parallel shared(tree) private(sense,tid,mynode) 
        { 
            sense = 0;
            tid = omp_get_thread_num();
            int tree_tid = tid + pow(2,h) - 1;
            int parent = (tree_tid -1)/2;
            if(tid >tree_thread_num) {
                parent = 0; }
            mynode = &tree[parent];
            int i=0;
            double t_start,t_end;
            double local_achievement_time = 0.0;
            t_start = omp_get_wtime();
            for(i=0;i<NUM_BARRIERS;i++) {
                combining_barrier1(mynode,&sense, tree_thread_num,left_over);}
            t_end = omp_get_wtime();
            local_achievement_time = t_end-t_start;
#pragma omp critical
            {
                global_achievement_time = global_achievement_time+ t_end-t_start;
            }
        }
        //fprintf(fp1,"%f,",global_achievement_time/NUM_BARRIERS);
        /**************************************************************************************/
        /***** END TEST 1 : average time for barrier achievement of default barrier *****/
        /**************************************************************************************/
        /*RESET PARAMETERS*/
        sense = 0;
        init_tree(tree,size_tree,left_over);
        /**************************************************************************************/
        /***** TEST 2 : average contention for accessing the count i.e critical part *****/
        /**************************************************************************************/ 
        double global_arrival_contention_time = 0.0;
#pragma omp parallel shared(tree) private(sense,tid,mynode) 
        { 
            sense = 0;
            tid = omp_get_thread_num();
            int tree_tid = tid + pow(2,h) - 1;
            int parent = (tree_tid -1)/2;
            if(tid >tree_thread_num) {
                parent = 0;}
            mynode = &tree[parent];
            int i=0;
            double local_arrival_contention_time = 0.0;
            for(i=0;i<NUM_BARRIERS;i++) {
                combining_barrier2(mynode,&sense, tree_thread_num,left_over,&local_arrival_contention_time);
            }
#pragma omp critical
            {
                global_arrival_contention_time = global_arrival_contention_time + local_arrival_contention_time;
            }

        }
        //fprintf(fp1,"%f,",global_arrival_contention_time/NUM_BARRIERS);
        /**************************************************************************************/    
        /***** END TEST 2 : average contention for accessing the count i.e critical part *****/
        /**************************************************************************************/
        /*RESET PARAMETERS*/
        sense = 0;
        init_tree(tree,size_tree,left_over);
        /**************************************************************************************/
        /***** TEST 3 :  measure the part of synchronization instructions in the overhead *****/
        /**************************************************************************************/
        double global_cc_time= 0.0;
#pragma omp parallel shared(tree) private(sense,tid,mynode) 
        { 
            sense = 0;
            tid = omp_get_thread_num();
            int tree_tid = tid + pow(2,h) - 1;
            int parent = (tree_tid -1)/2;
            if(tid >tree_thread_num) {
                parent = 0;}
            mynode = &tree[parent];
            int i=0;
            double local_cc_time = 0.0;
            for(i=0;i<NUM_BARRIERS;i++) {
                combining_barrier3(mynode,&sense, tree_thread_num,left_over,&local_cc_time );
            }
#pragma omp critical
            {
                global_cc_time = global_cc_time + local_cc_time;
            }
        }
        //fprintf(fp1,"%f\n",global_cc_time/NUM_BARRIERS);
        /**************************************************************************************/
        /***** END TEST 3 :  measure the part of synchronization instructions in the overhead *****/
        /**************************************************************************************/
        /*RESET PARAMETERS*/
        sense = 0;
        init_tree(tree,size_tree,left_over);
        /**************************************************************************************/
        /***** TEST 4 :  measure the contention of sense spinning for each spin data structure*****/
        /*************************************************************************************/
        int gsc_i=0;
        unsigned int global_spin_count[size_tree];
        for (gsc_i=0;gsc_i<size_tree;gsc_i++) {
            global_spin_count[gsc_i] = 0;}

#pragma omp parallel shared(tree) private(sense,tid,mynode) 
        { 
            sense = 0;
            tid = omp_get_thread_num();
            int tree_tid = tid + pow(2,h) - 1;
            int parent = (tree_tid -1)/2;
            if(tid >tree_thread_num) {
                parent = 0;}
            mynode = &tree[parent];
            int i=0;
            unsigned int local_spin_count[size_tree];
            for(i=0;i<NUM_BARRIERS;i++) {
                combining_barrier4(mynode,&sense, tree_thread_num,left_over,local_spin_count);
            }
#pragma omp critical
            {
                for(gsc_i=0;gsc_i<size_tree;gsc_i++){
                    global_spin_count[gsc_i] =/* global_spin_count[gsc_i] + */local_spin_count[gsc_i];
                }
            }
        }
        for(gsc_i=0;gsc_i<size_tree;gsc_i++){
            global_spin_count[gsc_i] = global_spin_count[gsc_i]/NUM_BARRIERS;
            //fprintf(fp4,"%d,",global_spin_count[gsc_i]/(NUM_BARRIERS));
        }
        //fprintf(fp4,"\n\n");
        /**************************************************************************************/   
        /***** END TEST 4 : measure the contention of sense spinning *****/
        /**************************************************************************************/
        printf("\n\n");
        /*RESET PARAMETERS*/
        sense = 0;
        init_tree(tree,size_tree,left_over);
        /**************************************************************************************/
        /***** TEST 5 :  measure the consequences of dynamically located spin location*****/
        /**************************************************************************************/
        //Measure variance on spin time with average on column sums
        //Measure global spin time in barrier with average on line sums
        double spin_times[NUM_BARRIERS][NUM_THREADS];
        int sti, stj;
        for(sti=0;sti<NUM_BARRIERS;sti++) {
            for(stj=0;stj<NUM_THREADS;stj++) {
                spin_times[sti][stj] = 0.0; }
        }
#pragma omp parallel shared(tree) private(sense,tid,mynode) 
        { 
            sense = 0;
            tid = omp_get_thread_num();
            int tree_tid = tid + pow(2,h) - 1;
            int parent = (tree_tid -1)/2;
            if(tid >tree_thread_num) {
                parent = 0;
            }
            mynode = &tree[parent];
            int i=0;
            for(i=0;i<NUM_BARRIERS;i++) {
                combining_barrier5(mynode,&sense, tree_thread_num,left_over,spin_times,i);
            }
        }/*
            for(sti=0;sti<NUM_BARRIERS;sti++) {
            fprintf(fp5,"%d,",sti);
            for(stj=0;stj<NUM_THREADS;stj++) {
            fprintf(fp5,"%f,",spin_times[sti][stj]);
            }
            fprintf(fp5,"\n");
            }
            fprintf(fp5,"\n\n");*/
        /**************************************************************************************/   
        /***** END TEST 5 : measure the consequences of dynamically located spin location *****/
        /**************************************************************************************/
        /*RESET PARAMETERS*/
        sense = 0;
        init_tree(tree,size_tree,left_over);
        /**************************************************************************************/
        /***** TEST 6 :  measure the contention of sense spinning for each spin data structure zith exponential delay EB*****/
        /**************************************************************************************/
        int EB_gsc_i=0;
        unsigned int EB_global_spin_count[size_tree];
        for (EB_gsc_i=0;EB_gsc_i<size_tree;EB_gsc_i++) {
            EB_global_spin_count[EB_gsc_i] = 0;
        }
#pragma omp parallel shared(tree) private(sense,tid,mynode) 
        { 
            sense = 0;
            tid = omp_get_thread_num();
            int tree_tid = tid + pow(2,h) - 1;
            int parent = (tree_tid -1)/2;
            if(tid >tree_thread_num) {
                parent = 0;
            }
            mynode = &tree[parent];
            int i=0;
            unsigned int EB_local_spin_count[size_tree];
            for(i=0;i<NUM_BARRIERS;i++) {
                combining_barrier6(mynode,&sense, tree_thread_num,left_over,EB_local_spin_count);
            }
#pragma omp critical
            {
                for(gsc_i=0;gsc_i<size_tree;gsc_i++){
                    EB_global_spin_count[gsc_i] = EB_global_spin_count[gsc_i] + EB_local_spin_count[gsc_i];
                }
            }
        }
        for(EB_gsc_i=0;EB_gsc_i<size_tree;EB_gsc_i++){

            EB_global_spin_count[EB_gsc_i] = EB_global_spin_count[EB_gsc_i]/NUM_BARRIERS;}
        //fprintf(fp6,"%d,",EB_global_spin_count[EB_gsc_i]/(NUM_BARRIERS));
        //fprintf(fp6,"\n\n");
        /**************************************************************************************/   
        /***** END TEST 6 : measure the contention of sense spinning with EB *****/
        /**************************************************************************************/
        /*RESET PARAMETERS*/
        sense = 0;
        init_tree(tree,size_tree,left_over);
        /**************************************************************************************/
        /***** TEST 7 :  measure the consequences of dynamically located spin location with EB*****/
        /*************************************************************************************/
        double EB_spin_times[NUM_BARRIERS][NUM_THREADS];
        int EB_sti, EB_stj;
        for(EB_sti=0;EB_sti<NUM_BARRIERS;EB_sti++) {
            for(EB_stj=0;EB_stj<NUM_THREADS;EB_stj++) {
                EB_spin_times[EB_sti][EB_stj] = 0.0;}
        }
#pragma omp parallel shared(tree) private(sense,tid,mynode) 
        { 
            sense = 0;
            tid = omp_get_thread_num();
            int tree_tid = tid + pow(2,h) - 1;
            int parent = (tree_tid -1)/2;
            if(tid >tree_thread_num) {
                parent = 0;
            }
            mynode = &tree[parent];
            int i=0;
            for(i=0;i<NUM_BARRIERS;i++) {
                combining_barrier7(mynode,&sense, tree_thread_num,left_over,EB_spin_times,i);
            }
        }/*
            for(sti=0;sti<NUM_BARRIERS;sti++) {
            fprintf(fp7,"%d,",sti);
            for(stj=0;stj<NUM_THREADS;stj++) {
            fprintf(fp7,"%f,",spin_times[sti][stj]);
            printf("%f",spin_times[sti][stj]);
            }
            fprintf(fp7,"\n");
            }
            fprintf(fp7,"\n");
            fprintf(fp7,"\n");*/
        /**************************************************************************************/   
        /***** END TEST 7 :measure the consequences of dynamically located spin location with EB*****/
        /**************************************************************************************/
        /*RESET PARAMETERS*/
        sense = 0;
        init_tree(tree,size_tree,left_over);
        /**************************************************************************************/
        /***** TEST 8 :  measure the contention of sense spinning for each spin data structure zith exponential delay GB*****/
        /**************************************************************************************/
        int GB_gsc_i;
        unsigned int GB_global_spin_count[size_tree];
        for (GB_gsc_i=0;GB_gsc_i<size_tree;GB_gsc_i++) {
            GB_global_spin_count[GB_gsc_i] = 0;}
#pragma omp parallel shared(tree) private(sense,tid,mynode) 
        { 
            sense = 0;
            tid = omp_get_thread_num();
            int tree_tid = tid + pow(2,h) - 1;
            int parent = (tree_tid -1)/2;
            if(tid >tree_thread_num) {
                parent = 0;}
            mynode = &tree[parent];
            int i=0;
            unsigned int GB_local_spin_count[size_tree];
            for(i=0;i<NUM_BARRIERS;i++) {
                combining_barrier8(mynode,&sense, tree_thread_num,left_over,GB_local_spin_count);
            }
#pragma omp critical
            {
                for(GB_gsc_i=0;GB_gsc_i<size_tree;GB_gsc_i++){
                    GB_global_spin_count[GB_gsc_i] = GB_global_spin_count[GB_gsc_i] + GB_local_spin_count[GB_gsc_i];}
            }
        }
        for(GB_gsc_i=0;GB_gsc_i<size_tree;GB_gsc_i++){
            GB_global_spin_count[GB_gsc_i] =GB_global_spin_count[GB_gsc_i]/NUM_BARRIERS;
            //fprintf(fp8,"%d,",GB_global_spin_count[GB_gsc_i]/(NUM_BARRIERS));
        }
        //fprintf(fp8,"\n\n");
        /**************************************************************************************/   
        /***** END TEST 8 : measure the contention of sense spinning with GB *****/
        /**************************************************************************************/
        /*RESET PARAMETERS*/
        sense = 0;
        init_tree(tree,size_tree,left_over);
        /**************************************************************************************/
        /***** TEST 9 :  measure the consequences of dynamically located spin location with GB*****/
        /**************************************************************************************/
        double GB_spin_times[NUM_BARRIERS][NUM_THREADS];
        int GB_sti, GB_stj;
        for(GB_sti=0;GB_sti<NUM_BARRIERS;GB_sti++) {
            for(GB_stj=0;GB_stj<NUM_THREADS;GB_stj++) {
                GB_spin_times[GB_sti][GB_stj] = 0.0;}
        }
#pragma omp parallel shared(tree) private(sense,tid,mynode) 
        { 
            sense = 0;
            tid = omp_get_thread_num();
            int tree_tid = tid + pow(2,h) - 1;
            int parent = (tree_tid -1)/2;
            if(tid >tree_thread_num) {
                parent = 0;
            }
            mynode = &tree[parent];
            int i=0;
            for(i=0;i<NUM_BARRIERS;i++) {
                combining_barrier9(mynode,&sense, tree_thread_num,left_over,GB_spin_times,i);}
        }/*
            for(GB_sti=0;GB_sti<NUM_BARRIERS;GB_sti++) {
            fprintf(fp9,"%d,",GB_sti);
            for(GB_stj=0;GB_stj<NUM_THREADS;GB_stj++) {
            fprintf(fp9,"%f,",spin_times[GB_sti][GB_stj]);
            printf("%f",GB_spin_times[GB_sti][GB_stj]);}
            fprintf(fp9,"\n");
            }
            fprintf(fp9,"\n\n");*/
        /**************************************************************************************/   
        /***** END TEST 9 :measure the consequences of dynamically located spin location with GB*****/
        /**************************************************************************************/
        /*RESET PARAMETERS*/
        sense = 0;
        init_tree(tree,size_tree,left_over);
        /**************************************************************************************/
        /***** TEST 10 :  measure the contention of sense spinning for each spin data structure zith exponential delay LB*****/
        /**************************************************************************************/
        int LB_gsc_i;
        unsigned int LB_global_spin_count[size_tree];
        for (LB_gsc_i=0;LB_gsc_i<size_tree;LB_gsc_i++) {
            LB_global_spin_count[LB_gsc_i] = 0;
        }
#pragma omp parallel shared(tree) private(sense,tid,mynode) 
        { 
            sense = 0;
            tid = omp_get_thread_num();
            int tree_tid = tid + pow(2,h) - 1;
            int parent = (tree_tid -1)/2;
            if(tid >tree_thread_num) {
                parent = 0; }
            mynode = &tree[parent];
            int i=0;
            unsigned int LB_local_spin_count[size_tree];
            for(i=0;i<NUM_BARRIERS;i++) {
                combining_barrier10(mynode,&sense, tree_thread_num,left_over,LB_local_spin_count);
            }
#pragma omp critical
            {
                for(gsc_i=0;gsc_i<size_tree;gsc_i++){
                    LB_global_spin_count[gsc_i] = LB_global_spin_count[gsc_i] + LB_local_spin_count[gsc_i]; }
            }
        }
        for(LB_gsc_i=0;LB_gsc_i<size_tree;LB_gsc_i++){
            LB_global_spin_count[LB_gsc_i] =LB_global_spin_count[LB_gsc_i]/NUM_BARRIERS;
            //fprintf(fp10,"%d,",LB_global_spin_count[LB_gsc_i]/(NUM_BARRIERS));
        }
        //fprintf(fp10,"\n\n");
        /**************************************************************************************/   
        /***** END TEST 10 : measure the contention of sense spinning with LB *****/
        /**************************************************************************************/
        /*RESET PARAMETERS*/
        sense = 0;
        init_tree(tree,size_tree,left_over);
        /**************************************************************************************/
        /***** TEST 11 :  measure the consequences of dynamically located spin location with LB*****/
        /**************************************************************************************/
        double LB_spin_times[NUM_BARRIERS][NUM_THREADS];
        int LB_sti, LB_stj;
        for(LB_sti=0;LB_sti<NUM_BARRIERS;LB_sti++) {
            for(LB_stj=0;LB_stj<NUM_THREADS;LB_stj++) {
                LB_spin_times[LB_sti][LB_stj] = 0.0;}
        }
#pragma omp parallel shared(tree) private(sense,tid,mynode) 
        { 
            sense = 0;
            tid = omp_get_thread_num();
            int tree_tid = tid + pow(2,h) - 1;
            int parent = (tree_tid -1)/2;
            if(tid >tree_thread_num) {
                parent = 0;}
            mynode = &tree[parent];
            int i=0;
            for(i=0;i<NUM_BARRIERS;i++) {
                combining_barrier9(mynode,&sense, tree_thread_num,left_over,LB_spin_times,i);}
        }
        /*
           fprintf(fp11,"LBTime spent on spinning by thread j on barrier i\n");
           for(EB_stj=0;LB_stj<NUM_THREADS;LB_stj++) {
           fprintf(fp11,"%d,",LB_stj);
           }
           for(LB_sti=0;LB_sti<NUM_BARRIERS;LB_sti++) {
           fprintf(fp11,"%d,",LB_sti);
           for(LB_stj=0;LB_stj<NUM_THREADS;LB_stj++) {
           fprintf(fp11,"%f,",spin_times[LB_sti][LB_stj]);
           }
           fprintf(fp11,"\n");
           }
           fprintf(fp11,"\n");
           fprintf(fp11,"\n");
           */
        /**************************************************************************************/   
        /***** END TEST 11 :measure the consequences of dynamically located spin location with LB*****/
        /**************************************************************************************/
}

   // fclose(fp1);fclose(fp4);fclose(fp5);fclose(fp6);fclose(fp7);fclose(fp8);fclose(fp9);fclose(fp10);fclose(fp11);
    return 0;

}



#include <omp.h>
#include <stdlib.h>
#include <stdio.h>
#include <sys/time.h>
#include <unistd.h>
#include <string.h>
#include "SRB.h"

int main (int argc, char **argv) {
    int NUM_THREADS = 2;
/*
    FILE *fp;
    fp = fopen("sense_test.csv","w+");
    int NUM_THREADS = 2;
    fprintf(fp,"Num_barriers,%d\n",NUM_BARRIERS);
    fprintf(fp,"Num_thread,GAT,GACT,GCCT,GSP,EBGSP,EBGAT,GBGSP,GBGAT,LBGSP,LBGAT\n");
    */
    for(NUM_THREADS =2;NUM_THREADS <9;NUM_THREADS +=2) {
        omp_set_num_threads(NUM_THREADS);
        int tid;
        int count = NUM_THREADS;
        int sense = 0;
        int local_sense = 0;
        //fprintf(fp,"%d,",NUM_THREADS);
        
        /***** TEST 0 : experimental proof of the correctness of the barrier*****/
#pragma omp parallel private(local_sense,tid) shared(count, sense) 
        {
            local_sense = 0;
            tid = omp_get_thread_num();
            int i=0;
            printf("Hello World from thread = %d BEFORE barrier\n",tid);
            for(i=0;i<NUM_BARRIERS;i++) {
                SRbarrier1(&count,&sense,&local_sense);
                printf("Hello World from thread = %d AFTER%d barrier.\n ",tid,i);
            }
        }
        /***** END TEST 0 *****/


        /***RESET PARAMETERS***/
        count = NUM_THREADS;
        sense = 0;
        local_sense = 0;


        /***** TEST 1 : average time for barrier achievement of default barrier *****/
        double global_achievement_time = 0;
#pragma omp parallel private(local_sense,tid) shared(count, sense,global_achievement_time) 
        {
            local_sense = 0;
            tid = omp_get_thread_num();
            int i=0;
            double t_start,t_end;
            t_start = omp_get_wtime();
            for(i=0;i<NUM_BARRIERS;i++) {
                SRbarrier1(&count,&sense,&local_sense); }
            t_end = omp_get_wtime();
#pragma omp critical
            {
                global_achievement_time = global_achievement_time+ t_end-t_start;
            }

        }
        //fprintf(fp,"%f,",global_achievement_time/NUM_BARRIERS);
        /***** END TEST 1 *****/


        /***RESET PARAMETERS***/
        count = NUM_THREADS;
        sense = 0;
        local_sense = 0;


        /***** TEST 2 : average contention for accessing the count i.e critical part *****/
        double global_arrival_contention_time = 0.0;
#pragma omp parallel private(local_sense,tid) shared(count, sense,global_arrival_contention_time) 
        {
            local_sense = 0;
            tid = omp_get_thread_num();
            int i=0;
            double local_arrival_contention_time = 0.0;
            for(i=0;i<NUM_BARRIERS;i++) {
                SRbarrier2(&count,&sense,&local_sense,&local_arrival_contention_time);}
#pragma omp critical
            {
                global_arrival_contention_time = global_arrival_contention_time + local_arrival_contention_time;
            }
        }
        //fprintf(fp,"%f,",global_arrival_contention_time/NUM_BARRIERS);
        /***** END TEST 2 *****/


        /***RESET PARAMETERS***/
        count = NUM_THREADS;
        sense = 0;
        local_sense = 0;


        /***** TEST 3 :  measure the part of synchronization instructions in the overhead *****/
        double global_cc_time= 0.0;
#pragma omp parallel private(local_sense,tid) shared(count, sense,global_cc_time) 
        {
            local_sense = 0;
            tid = omp_get_thread_num();
            int i=0;
            double local_cc_time = 0.0;
            for(i=0;i<NUM_BARRIERS;i++) {
                SRbarrier3(&count,&sense,&local_sense,&local_cc_time);
            }
#pragma omp critical
            {
                global_cc_time = global_cc_time + local_cc_time;
            }

        }
        //fprintf(fp,"%f,",global_cc_time/NUM_BARRIERS);
        /***** END TEST 3 *****/


        /***RESET PARAMETERS***/
        count = NUM_THREADS;
        sense = 0;
        local_sense = 0;


        /***** TEST 4 :  measure the contention of sense spinning *****/
        unsigned int global_spin_count= 0;
#pragma omp parallel private(local_sense,tid) shared(count, sense,global_cc_time) 
        {
            local_sense = 0;
            tid = omp_get_thread_num();
            int i=0;
            unsigned int local_spin_count = 0;
            for(i=0;i<NUM_BARRIERS;i++) {
                SRbarrier4(&count,&sense,&local_sense,&local_spin_count);
            }
#pragma omp critical
            {
                global_spin_count = global_spin_count + local_spin_count;
            }

        }
        //fprintf(fp,"%d,",global_spin_count/(NUM_BARRIERS));
        /***** END TEST 4 *****/


        /***RESET PARAMETERS***/
        count = NUM_THREADS;
        sense = 0;
        local_sense = 0;


        /***** TEST 5 :  measure the contention of sense spinning with exponential backoff*****/
        unsigned int EB_global_spin_count= 0;
        double EB_global_achievement_time = 0.0;
#pragma omp parallel private(local_sense,tid) shared(count, sense,global_cc_time) 
        {
            local_sense = 0;
            tid = omp_get_thread_num();
            int i=0;
            unsigned int EB_local_spin_count = 0;
            double EB_t_start,EB_t_end;
            EB_t_start = omp_get_wtime();
            for(i=0;i<NUM_BARRIERS;i++) {
                SRbarrier5(&count,&sense,&local_sense,&EB_local_spin_count);
            }
            EB_t_end = omp_get_wtime();
#pragma omp critical
            {
                EB_global_spin_count = EB_global_spin_count + EB_local_spin_count;
                EB_global_achievement_time = EB_global_achievement_time + EB_t_end - EB_t_start ;
            }
        }
        //fprintf(fp,"%d,",EB_global_spin_count/(NUM_BARRIERS));
        //fprintf(fp,"%f,",EB_global_achievement_time/NUM_BARRIERS);
        /***** END TEST 5*****/


        /***RESET PARAMETERS***/
        count = NUM_THREADS;
        sense = 0;
        local_sense = 0;


        /***** TEST 6 :  measure the contention of sense spinning with geometrical backoff GB*****/
        unsigned int GB_global_spin_count= 0;
        double GB_global_achievement_time = 0.0;
#pragma omp parallel private(local_sense,tid) shared(count, sense,global_cc_time) 
        {
            local_sense = 0;
            tid = omp_get_thread_num();
            int i=0;
            unsigned int GB_local_spin_count = 0;
            double GB_t_start,GB_t_end;
            GB_t_start = omp_get_wtime();
            for(i=0;i<NUM_BARRIERS;i++) {
                SRbarrier6(&count,&sense,&local_sense,&GB_local_spin_count);
            }
            GB_t_end = omp_get_wtime();
#pragma omp critical
            {
                GB_global_spin_count = GB_global_spin_count + GB_local_spin_count;
                GB_global_achievement_time = GB_global_achievement_time + GB_t_end - GB_t_start ;
            }
        }
        //fprintf(fp,"%d,",GB_global_spin_count/(NUM_BARRIERS));
        //fprintf(fp,"%f,",GB_global_achievement_time/NUM_BARRIERS);
        /***** END TEST 6 *****/


        /***RESET PARAMETERS***/
        count = NUM_THREADS;
        sense = 0;
        local_sense = 0;


        /***** TEST 7 :  measure the contention of sense spinning with linear backoff LB*****/
        unsigned int LB_global_spin_count= 0;
        double LB_global_achievement_time = 0.0;
#pragma omp parallel private(local_sense,tid) shared(count, sense,global_cc_time) 
        {
            local_sense = 0;
            tid = omp_get_thread_num();
            int i=0;
            unsigned int LB_local_spin_count = 0;
            double LB_t_start,LB_t_end;
            LB_t_start = omp_get_wtime();
            for(i=0;i<NUM_BARRIERS;i++) {
                SRbarrier7(&count,&sense,&local_sense,&LB_local_spin_count);
            }
            LB_t_end = omp_get_wtime();
#pragma omp critical
            {
                LB_global_spin_count = LB_global_spin_count + LB_local_spin_count;
                LB_global_achievement_time = LB_global_achievement_time + LB_t_end - LB_t_start ;
            }

        }
        //fprintf(fp,"%d,",LB_global_spin_count/(NUM_BARRIERS));
        //fprintf(fp,"%f\n",LB_global_achievement_time/NUM_BARRIERS);
        /***** END TEST 7*****/
    }
    //fclose(fp);
    return 0;
}


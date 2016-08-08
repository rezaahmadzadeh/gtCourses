#include "mpi.h"
#include<stdio.h>
#include<math.h>
#include <omp.h>
#include <stdlib.h>
#include <sys/time.h>
#include <unistd.h>
#include <string.h>

#define NUM_THREADS 5
#define NUM_BARRIERS 10
//#define MP_NUM_THREADS 4
#define MP_NUM_BARRIERS 10

void SRbarrier1 (int* count, int* sense, int* local_sense) {

    *local_sense = !(*local_sense);

#pragma omp critical 
    {
        *count = *count - 1;
        if(*count == 0) {
            //int tid = omp_get_thread_num();
            //printf("I am %d and I am the last one to arrive\n",tid);
            *count = omp_get_num_threads();
            *sense = *local_sense;
        }
    }

    //spin of global data structure -> contention + latency
    while (*sense != *local_sense) {};

}
void tour(int rank, int h, int ecart[], int sender[], int numb_msg[],double* arrival_time, double* wake_up_time) { 
    int tag = 1;
    int my_msg=-1;
    int i=0;
    int j=0;
    MPI_Status mpi_result;

    double a_t_start, a_t_end, t_end;
    a_t_start = MPI_Wtime();

	//arrival phase
	for (i=0;i<h;i++) {
        int ecart_i = ecart[i] ;
        int sender_i = sender[i] ; //the sender is always the right child
		int numb_msg_i = numb_msg[i];
		for(j=0;j<numb_msg_i;j++) {
            if(rank==sender_i-ecart_i) {
                MPI_Recv(&my_msg,1,MPI_INT,sender_i,tag,MPI_COMM_WORLD,&mpi_result);
            }
			if (rank==sender_i) {
				int my_dst = sender_i-ecart_i;
				MPI_Send(&my_msg, 1, MPI_INT, my_dst, tag, MPI_COMM_WORLD);
				MPI_Recv(&my_msg, 1, MPI_INT, my_dst, tag, MPI_COMM_WORLD,&mpi_result);
			}
			sender_i = sender_i + 2*ecart_i;
		}
	}
    a_t_end = MPI_Wtime(); //measure arrival tree time performance

	//wake up phase
	for(i=h-1;i>=0; i--) {
		int ecart = (int) pow(2,i);
		int sender = 0;
		int numb_msg = (int) pow(2,h-i-1);

		for(j=0;j<numb_msg;j++) {
			if (rank == sender) {
				int my_dst = sender+ecart;
                MPI_Send(&my_msg, 1, MPI_INT, my_dst, tag, MPI_COMM_WORLD);}
            sender = sender + 2*ecart;
        }
    }

    t_end = MPI_Wtime();//measure wake up tree time performace
    *arrival_time = *arrival_time +  a_t_end - a_t_start;
    *wake_up_time = *wake_up_time + t_end - a_t_start;
}


void tree_init (int  h,int ecart[],int sender[], int numb_msg[]) {
    int i;
    for (i=0;i<=h;i++) {
        ecart[i] = (int) pow(2,i);
        sender[i] = (int) pow(2,i);
        numb_msg[i] = (int) pow(2,h-i-1);
    }
}


int main(int argc, char*argv[]) {
    int h = (int)(log(NUM_THREADS)/log(2));
    int numtasks, rank, rc;
    int ecart[h];
    int sender[h];
    int numb_msg[h];
    int i;
    

    tree_init(h,ecart,sender,numb_msg);
    rc = MPI_Init(&argc,&argv);
    if(rc != MPI_SUCCESS){
        printf("Error starting MPI Program. Terminating.\n");
        MPI_Abort(MPI_COMM_WORLD,rc);
    }

    MPI_Comm_size(MPI_COMM_WORLD,&numtasks);
    MPI_Comm_rank(MPI_COMM_WORLD,&rank);

    /*******TEST :  global_achievement_time *******/
            
    FILE *fp;  
    fp = fopen("stour_test.csv","a");   
    fprintf(fp,"\n,");
    fclose(fp);

    int MP_NUM_THREADS = 2;
    for(MP_NUM_THREADS =2;MP_NUM_THREADS <13;MP_NUM_THREADS +=2) {
        omp_set_num_threads(MP_NUM_THREADS);
        int tid;
        int count = MP_NUM_THREADS;
        int sense = 0;
        int local_sense = 0;


        double global_achievement_time = 0;
        double global_arrival_time = 0;
        double global_wake_up_time = 0;
        double local_achievement_time = 0;
        double local_arrival_time = 0;
        double local_wake_up_time = 0;
        double t_start,t_end;

        t_start = MPI_Wtime();
#pragma omp parallel private(local_sense,tid) shared(count, sense) 
        {
            local_sense = 0;
            tid = omp_get_thread_num();
            int i=0;

            //printf("Hello World from thread = %d BEFORE barrier\n",tid);


            for(i=0;i<NUM_BARRIERS;i++) {
                SRbarrier1(&count,&sense,&local_sense);
                //printf("Hello World from thread = %d AFTER%d PRAGMA barrier.\n ",tid,i);
            }
        }
        //printf("Hello world from thread %d BEFORE barrier\n",rank);

        for(i=0;i<NUM_BARRIERS;i++) {
            tour(rank,h,ecart,sender,numb_msg,&local_arrival_time, &local_wake_up_time);
            //printf("Hello world from thread %d AFTER TOUR barrier\n",rank);
        }

        t_end = MPI_Wtime();
        local_achievement_time = (local_achievement_time+ t_end-t_start)/NUM_BARRIERS;
        local_arrival_time = local_arrival_time/NUM_BARRIERS;
        local_wake_up_time = local_wake_up_time/NUM_BARRIERS;

        rc = MPI_Reduce(&local_achievement_time, &global_achievement_time, 1, MPI_DOUBLE, MPI_SUM,0, MPI_COMM_WORLD);
        rc = MPI_Reduce(&local_arrival_time, &global_arrival_time, 1, MPI_DOUBLE, MPI_SUM,0, MPI_COMM_WORLD);
        rc = MPI_Reduce(&local_wake_up_time, &global_wake_up_time, 1, MPI_DOUBLE, MPI_SUM,0, MPI_COMM_WORLD);
/*
        if (rank ==0) {
            FILE *fp;
            fp = fopen("stour_test.csv","a");
            //fprintf(fp,"Num_threads,%d,",NUM_THREADS);
            fprintf(fp,"%f,",global_achievement_time);
            //fprintf(fp,"%f,",global_arrival_time);
            //fprintf(fp,"%f\n",global_wake_up_time);
            fclose(fp);
        }
*/
    }


    MPI_Finalize();
    return 0;
}

#include "mpi.h"
#include<stdio.h>
#include<math.h>

#define NUM_THREADS 8
#define NUM_BARRIERS 5

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
    double global_achievement_time = 0;
    double global_arrival_time = 0;
    double global_wake_up_time = 0;
    double local_achievement_time = 0;
    double local_arrival_time = 0;
    double local_wake_up_time = 0;
    double t_start,t_end;
    int j=0;

    t_start = MPI_Wtime();
    //printf("Hello world from thread %d BEFORE barrier\n",rank);

    for(i=0;i<NUM_BARRIERS;i++) {
        tour(rank,h,ecart,sender,numb_msg,&local_arrival_time, &local_wake_up_time);
        //printf("Hello world from thread %d AFTER barrier\n",rank);
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
        fp = fopen("tour_test.csv","a");
        fprintf(fp,"Num_threads,%d,",NUM_THREADS);
        fprintf(fp,"%f,",global_achievement_time);
        fprintf(fp,"%f,",global_arrival_time);
        fprintf(fp,"%f\n",global_wake_up_time);
        fclose(fp);
    }
*/

    MPI_Finalize();
    return 0;
}

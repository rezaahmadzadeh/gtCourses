#include <sys/time.h>
#include "mpi.h"
#include<stdio.h>
#include<math.h>
#include <unistd.h>
#include <string.h>

#define h4 2  //height of the arrival tree
#define NUM_THREADS 5
#define NUM_BARRIERS 10

typedef struct node4 {
    int id;
    int child[4];
} Node4;

typedef struct node2 {
    int id;
    int child[2];
} Node2;

typedef struct node1 { 
    int id;
    int parent;
} Node1;

typedef struct level {
    int first;
    int last;
} Level;

void mcs (int rank, Node1 arrival_parent[],  Node1 wakeup_parent[], Node2 wakeup_child[],  Level arrival_level[], Level wakeup_level[], double* arrival_time, double* wake_up_time) {
    int i,j;
    int my_msg = -1;
    MPI_Status mpi_result;
    int tag = 1;
    int h2 = log(NUM_THREADS)/log(2);

    double a_t_start, a_t_end, t_end;
    a_t_start = MPI_Wtime();

/**ARRIVAL PHASE**/ 
    for (i=h4;i>0;i--) {

        int first_node = arrival_level[i].first; 
        int last_node = arrival_level[i].last; 

        for (j=first_node;j<last_node;j++) {

            if(rank == arrival_parent[j].parent) { 
                MPI_Recv(&my_msg,1,MPI_INT,j,tag, MPI_COMM_WORLD,&mpi_result);
            }

            if (rank==j) {
                int my_dst = arrival_parent[j].parent; 
                MPI_Send(&my_msg,1,MPI_INT,my_dst,tag,MPI_COMM_WORLD);

                int my_src = wakeup_parent[j].parent; 
                MPI_Recv(&my_msg,1,MPI_INT,my_src,tag, MPI_COMM_WORLD,&mpi_result);
            }
        }
    }
    
    a_t_end = MPI_Wtime(); //measure arrival tree time performace

    /**WAKE UP PHASE**/
    for(i=0;i<h2;i++) {
        int sender = wakeup_level[i].first;
        int last_sender = wakeup_level[i].last;
        for (j=sender;j<last_sender;j++) {

            if (rank==j) {
                int fils_gauche = wakeup_child[j].child[0];
                int fils_droit = wakeup_child[j].child[1];

                if (fils_gauche != -1) {
                    MPI_Send(&my_msg,1,MPI_INT,fils_gauche,tag,MPI_COMM_WORLD);
                }
                if (fils_droit !=-1) {
                    MPI_Send(&my_msg,1,MPI_INT,fils_droit,tag,MPI_COMM_WORLD);
                }
            }
        }
    }

t_end = MPI_Wtime();//measure wake up tree time performace
*arrival_time = *arrival_time +  a_t_end - a_t_start;
*wake_up_time = *wake_up_time + t_end - a_t_start;
}


int main (int argc, char* argv[]) {

    int numtasks, rank, rc;
    int i=0;

    /**********************
     * BUILD TREE STRUCTURES
     * *********************/

    /*arrival_parent : process j-> parent process in the arrival tree*/
    Node1 arrival_parent[NUM_THREADS];
    arrival_parent[0].id = 0;
    arrival_parent[0].parent =-1;
    for (i=1;i<NUM_THREADS;i++) {
        arrival_parent[i].id = i;
        arrival_parent[i].parent =(i-1)/4;
    }

    /*wakeup_parent :process j-> parent process in the wakeup tree*/
    Node1 wakeup_parent[NUM_THREADS];
    wakeup_parent[0].id = 0;
    wakeup_parent[0].parent =-1;
    for (i=1;i<NUM_THREADS;i++) {
        wakeup_parent[i].id = i;
        wakeup_parent[i].parent =(i-1)/2;
    }

    /*wakeup_child : process j -> left and right child process in the wakeup tree*/
    Node2 wakeup_child[NUM_THREADS];
    for(i=0;i<NUM_THREADS;i++) {
        wakeup_child[i].id = i;

        if(2*i+1 < NUM_THREADS) {
            wakeup_child[i].child[0] = 2*i+1;
        }  
        else {
            wakeup_child[i].child[0] = -1;
        }

        if(2*i+2 <NUM_THREADS) {
            wakeup_child[i].child[1] = 2*i +2;
        }
        else {
            wakeup_child[i].child[1] = -1;
        }
    }
    /*arrival level :level i ->first and last process of the level in the arrival tree*/
    Level arrival_level[h4];
    for(i=0;i<h4;i++) {
        arrival_level[i].first = ( ((int) pow(4,i)) - 1)/3;
        arrival_level[i].last = ( ((int) pow(4,i+1)) - 1)/3;
    }

    /*wakeup level :level i ->first and last process of the level in the wakeup tree*/
    int h2 = log(NUM_THREADS)/log(2);
    Level wakeup_level[h2];
    for(i=0;i<h2;i++) {
        wakeup_level[i].first =  ((int) pow(2,i)) - 1;
        wakeup_level[i].last =  ((int) pow(2,i+1)) - 1;
    }

    /****************************
     *END OF TREE CONSTRUCTION*
     **************************/


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

    for(j=0;j<NUM_BARRIERS;j++) {
        mcs (rank, arrival_parent, wakeup_parent, wakeup_child,  arrival_level, wakeup_level, &local_arrival_time, &local_wake_up_time) ;}

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
        fp = fopen("mcs_test.csv","a");
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


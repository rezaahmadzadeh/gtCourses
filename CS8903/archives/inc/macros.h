/**
 * \file macros.h
 * \brief External macros (defined by Assia)
 */
#ifndef MACROS_H
#define MACROS_H

/**
 * Root repertory of this project
 */
#define ROOT_DIR "/home/abenbihi/gtCourses/CS8903/"


/**
 * Specify the type of experimental data used (RAW_DATA or PROCESSED_DATA).
 */
#define DATA_TYPE 0

/**
 * Refers to the raw source data.
 */
#define RAW_DATA 0

/**
 * Refers to the processed data. The processing is done by Reza's matlab code.
 */
#define PROCESSED_DATA 1


#define FORMATED_DATA 2

/**
 * Refers to the repertory where the exeprimental data is stored.
 */

#if DATA_TYPE == 0
#define SOURCE_DATA_DIR "/home/abenbihi/gtCourses/CS8903/data/rawData/"
#elif DATA_TYPE == 1
#define SOURCE_DATA_DIR "/home/abenbihi/gtCourses/CS8903/data/processedData/"
#elif DATA_TYPE == 2
#define SOURCE_DATA_DIR "/home/abenbihi/gtCourses/CS8903/data/formatedData/"
#endif

/**
 * Data set number. The extension 'p' is used for processed dataset.
 */
#define NUMSET "2_5"

/**
 * Refers to the name of the data used. It is a '.txt' file starting with dataset.
 */
#define SOURCE_DATA_PATH SOURCE_DATA_DIR "dataset" NUMSET ".txt"


/*
 * Factor to convert degree to radians.
 */
#define D2R M_PI/180

/**
 * Factor to convert radians to degree.
 */
#define R2D 180/M_PI

/*
 * Factor to convert from degree to radians.
 */
#define DEG2RAD M_PI/180

/**
 * Jaco2 joint limits in radians from the Jaco2 datasheet.
 */
#define J0_MIN -10000*D2R


#define J0_MAX  10000*D2R
#define J1_MIN     50*D2R
#define J1_MAX    310*D2R
#define J2_MIN     19*D2R
#define J2_MAX    341*D2R 
#define J3_MIN -10000*D2R
#define J3_MAX  10000*D2R
#define J4_MIN -10000*D2R
#define J4_MAX  10000*D2R
#define J5_MIN -10000*D2R
#define J5_MAX  10000*D2R

/*
 * Maximum authorized joint displacement (TODO:to define)
 */
#define JOINTS_EPSILON 100.0


/**
 * Set the number of ee to solve. (For debug purpose)
 */
#define ITER_MAX 10

/**
 * Set to 1 to print debug information
 */
#define DEBUG 0


/**********************
 *** TESTS SETTINGS ***
 **********************/

/**
 * Output setting: Refers to the repertory whete to store outputs of the algo.
 */
#define TEST_DATA_DIR "./data/"

/*
 * Output setting: Refers to the file where to store all joints solutions.
 */
#define TEST_DATA_PATH_Q TEST_DATA_DIR "q_ikfast" NUMSET ".txt"

/*
 * Output setting: Refers to the file where to store all ee computed from the joints solutions.
 */
#define TEST_DATA_PATH_T TEST_DATA_DIR "t_ikfast" NUMSET ".txt"

/**
 * Joint constraints Input
 */
#define ALL_Q_FORMATED_PATH ROOT_DIR "cppCode/test/testRobotIK/testAllSolutions/data/q_ikfast" NUMSET ".txt" 

/**
 * testNN and testgraph input
 */
#define JOINT_CONSTRAINTS_Q_PATH ROOT_DIR "cppCode/test/testRobotIK/testJointConstraints/data/q_ikfast" NUMSET ".txt"








/*
 * Refers to the file where to store processed joints solutions. Joint constraints only.
 */
#define TEST_DATA_PATH_Q_PROCESSED TEST_DATA_DIR "q_ikfast_processed" NUMSET ".txt"

/*
 * Refers to the file where to store ee computed from processed joints solutions. Joint constraints only.
 */
#define TEST_DATA_PATH_T_PROCESSED TEST_DATA_DIR "t_ikfast_processed" NUMSET ".txt"

/*
 * Refers to the file where to store all joints solutions.
 */
#define TEST_DATA_PATH_Q_ALL_FOR_GRAPH TEST_DATA_DIR "q_ikfast_all_for_graph" NUMSET ".txt"

/*
 * Refers to the file where to store graph processed joints solutions.
 */
#define TEST_DATA_PATH_Q_GRAPH_PROCESSED TEST_DATA_DIR "q_ikfast_graph_processed" NUMSET ".txt"

/*
 * Refers to the file where to store ee computed from graph processed joints solutions.
 */
#define TEST_DATA_PATH_T_GRAPH_PROCESSED TEST_DATA_DIR "t_ikfast_graph_processed" NUMSET ".txt"


/**
 * Set to 1 to save all solutions in a format that helps debugging graph construction.
 */
#define SAVE_ALL_SOLUTIONS_FOR_GRAPH 0

/**
 * Set to 1 to save processed solutions. Can be combined with other savings.
 */
#define SAVE_PROCESSED_SOLUTIONS 0

/**
 * Set to 1 to perform graph search in the solutions and save the result.
 */
#define SAVE_GRAPH_SOLUTIONS 1

/**
 * Set to 1 to save all solutions. Can be combines with other savings.
 */
#define SAVE_ALL_SOLUTIONS 1



#endif

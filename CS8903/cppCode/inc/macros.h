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

/*****************
 * INPUT SETTING *
 *****************/
/**
 * Separator character in the input file.
 */
#define DELIMITER "  "

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
#define NUMSET "3"

/**
 * Refers to the name of the input data used. It is a '.txt' file starting with dataset.
 */
#define SOURCE_DATA_PATH SOURCE_DATA_DIR "dataset" NUMSET ".txt"


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
 * Input file: for joint constraints
 */
#define ALL_Q_PATH ROOT_DIR "cppCode/test/testRobotIK/testAllSolutions/data/q_ikfast" NUMSET ".txt" 

/**
 * Input file: for testNN and testgraph input
 */
#define JOINT_CONSTRAINTS_Q_PATH ROOT_DIR "cppCode/test/testRobotIK/testJointConstraints/data/q_ikfast" NUMSET ".txt"

/**
 * Input file: for smoother
 */
#define JOINT_PATH_RAW ROOT_DIR "cppCode/test/testRobotIK/testGraphSolutions/data/q_ikfast" NUMSET ".txt"



/******************
 * MISC CONSTANTS *
 * ****************/
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

/**
 * Arm initial joint configuration in radians
 */
#define J0_START 0.7780 
#define J1_START 3.9038
#define J2_START 2.2818
#define J3_START 2.3029
#define J4_START 0.0552
#define J5_START 5.4704

/**
 * Arn final joint configuration in radians
 */
#define J0_END 5.4469
#define J1_END 3.8449
#define J2_END 2.1298
#define J3_END 2.2900
#define J4_END 6.2009
#define J5_END 5.4703


/*
 * Maximum authorized joint displacement unit: radian (TODO:to define)
 */
#define EPSILON_Q 0.020



/**
 * Set the number of ee to solve. (For debug purpose)
 */
#define ITER_MAX 20

/**
 * Set to 1 to print debug information
 */
#define DEBUG 0


#endif

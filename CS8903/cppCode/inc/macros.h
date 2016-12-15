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
 * Separator character in the input file. This file contains recorded trajectories.
 */
#define DELIMITER "  "

/**
 * Specify the type of experimental data used (RAW_DATA or PROCESSED_DATA).
 */
#define DATA_TYPE 1

/**
 * Refers to the raw source data.
 */
#define RAW_DATA 0

/**
 * Refers to the processed data. The processing is done by Reza's matlab code.
 */
#define PROCESSED_DATA 1


/**
 * Refers to the repertory where the input data is stored.
 */

#if DATA_TYPE == 0
#define SOURCE_DATA_DIR ROOT_DIR "data/rawData/"
#elif DATA_TYPE == 1
#define SOURCE_DATA_DIR ROOT_DIR "data/processedData/"
#endif

/**
 * Data set number. The extension 'p' is used for processed dataset.
 */
#define NUMSET "1_4p"

/**
 * Refers to the name of the input data used. The name has '.txt' extension and starts with 'dataset'.
 */
#define SOURCE_DATA_PATH SOURCE_DATA_DIR "dataset" NUMSET ".txt"


/**********************
 *** TESTS SETTINGS ***
 **********************/

/**
 * Directory where to store outputs of the algo.
 */
#define TEST_DATA_DIR "./data/"

/**
 * File where to store joints solutions produced by ikfast.
 */
#define TEST_DATA_PATH_Q TEST_DATA_DIR "q_ikfast" NUMSET ".txt"

/**
 * File where to store all ee computed from the joints solutions.
 */
#define TEST_DATA_PATH_T TEST_DATA_DIR "t_ikfast" NUMSET ".txt"

/**
 * File where to store all joints solutions produced by ikfast. (several joints/ee pose)
 */
#define ALL_Q_PATH ROOT_DIR "cppCode/test/testRobotIK/testAll/data/q_ikfast" NUMSET ".txt" 

/**
 * File where to store all CONSTRAINED joints solutions produced by ikfast. (several joints/ee pose)
 */
#define JOINT_CONSTRAINTS_Q_PATH ROOT_DIR "cppCode/test/testRobotIK/testJointConstraints/data/q_ikfast" NUMSET ".txt"

/**
 * Input file: for smoother
 */
#define JOINT_PATH_RAW ROOT_DIR "cppCode/test/testRobotIK/testGraph/data/q_ikfast" NUMSET ".txt"

#define DISP_PATH TEST_DATA_DIR "disp" NUMSET ".txt"

/******************
 * MISC CONSTANTS *
 * ****************/
/**
 * Factor to convert degree to radians.
 */
#define D2R M_PI/180

/**
 * Factor to convert radians to degree.
 */
#define R2D 180/M_PI

/**
 * Jaco2 joint 0 minimum (radians) (source: Jaco2 datasheet).
 */
#define J0_MIN -10000*D2R

/**
 * Jaco2 joint 0 maximum (radians) (source: Jaco2 datasheet).
 */
#define J0_MAX  10000*D2R

/**
 * Jaco2 joint 1 minimum (radians) (source: Jaco2 datasheet).
 */
#define J1_MIN     50*D2R

/**
 * Jaco2 joint 1 maximum (radians) (source: Jaco2 datasheet).
 */
#define J1_MAX    310*D2R

/**
 * Jaco2 joint 2 minimum (radians) (source: Jaco2 datasheet).
 */
#define J2_MIN     19*D2R

/**
 * Jaco2 joint 2 maximum (radians) (source: Jaco2 datasheet).
 */
#define J2_MAX    341*D2R 

/**
 * Jaco2 joint 3 minimum (radians) (source: Jaco2 datasheet).
 */
#define J3_MIN -10000*D2R

/**
 * Jaco2 joint 3 maximum (radians) (source: Jaco2 datasheet).
 */
#define J3_MAX  10000*D2R

/**
 * Jaco2 joint 4 minimum (radians) (source: Jaco2 datasheet).
 */
#define J4_MIN -10000*D2R

/**
 * Jaco2 joint 4 maximum (radians) (source: Jaco2 datasheet).
 */
#define J4_MAX  10000*D2R

/**
 * Jaco2 joint 5 minimum (radians) (source: Jaco2 datasheet).
 */
#define J5_MIN -10000*D2R

/**
 * Jaco2 joint 5 maximum (radians) (source: Jaco2 datasheet).
 */
#define J5_MAX  10000*D2R


/**
 * Trajectory initial joint 0 value. (radians)
 */
#define J0_START 0.7780

/**
 * Trajectory initial joint 1 value. (radians)
 */
#define J1_START 3.9038

/**
 * Trajectory initial joint 2 value. (radians)
 */
#define J2_START 2.2818

/**
 * Trajectory initial joint 3 value. (radians)
 */
#define J3_START 2.3029

/**
 * Trajectory initial joint 4 value. (radians)
 */
#define J4_START 0.0552

/**
 * Trajectory initial joint 5 value. (radians)
 */
#define J5_START 5.4704

/**
 * Trajectory final joint 0 value. (radians)
 */
#define J0_END 5.4469

/**
 * Trajectory final joint 1 value. (radians)
 */
#define J1_END 3.8449

/**
 * Trajectory final joint 2 value. (radians)
 */
#define J2_END 2.1298

/**
 * Trajectory final joint 3 value. (radians)
 */
#define J3_END 2.2900

/**
 * Trajectory final joint 4 value. (radians)
 */
#define J4_END 6.2009

/**
 * Trajectory final joint 5 value. (radians)
 */
#define J5_END 5.4703


/**
 * Maximum authorized joint displacement unit: radian (TODO:to define)
 */
#define EPSILON_Q 0.020



/**
 * Set the number of ee to solve. (For debug purpose)
 */
#define ITER_MAX 30

/**
 * Set to 1 to print debug information
 */
#define DEBUG 0


#endif

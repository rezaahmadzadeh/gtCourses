/**
 * \file types.h
 *
 * \brief Datatypes defined by Assia.
 */

// TODO RENAME WITH CAPITALS !!!!

#ifndef TYPES_H
#define TYPES_H

#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <map>
#include <list>

/*
#define IKFAST_HAS_LIBRARY
#define IKFAST_NO_MAIN

#define IK_VERSION 56
#include "output_ikfast8.cpp"

#define IKREAL_TYPE IkReal
*/
/**
 * 
 */


/**
 * Joint configuration.
 */
typedef std::vector<double> q_t;

/**
 * 2D array index.
 */
typedef std::pair<int, int> mapIndex;

/**
 * Holds the ik solution: solutions[i][j] holds the j-th joint solution for the i-th ee. 
 */
typedef std::map<mapIndex, q_t> solutionsStorage;


struct joint_t {
    float q[6];
};

typedef std::list<joint_t> eeSolution_t;

typedef std::vector<eeSolution_t> ikSolution_t;


#endif

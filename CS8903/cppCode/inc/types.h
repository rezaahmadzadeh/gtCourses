/**
 * \file types.h
 *
 * \brief Datatypes defined by Assia.
 */


#ifndef TYPES_H
#define TYPES_H

#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <map>
#include <list>


/**
 * Joint configuration.
 */
typedef std::vector<double> q_t;

/**
 * Matrix index.
 */
typedef std::pair<int, int> mapIndex;

/**
 * Matrix of joint configuration. solutions[i][j] holds the j-th joint solution for the i-th ee. 
 */
typedef std::map<mapIndex, q_t> solutionsStorage;

#endif

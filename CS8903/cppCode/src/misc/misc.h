/**
 * \file misc.h
 * \brief Miscellaneous functions.
 */

#ifndef MISC_H
#define MISC_H

#include "types.h"
#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <vector>

/**
 * \fn double qNorm(q_t jointsComputed, q_t jointsCurrent);
 * \brief Compute manhattan norm between two joint vectors.
 * \param[in] jointsComputed
 * \param[in] jointsCurrent
 * \param[out] norm
 */
double qNorm(q_t jointsComputed, q_t jointsCurrent);

/**
 * \fn std::size_t myMin(std::vector<double> v);
 * \brief Returns the index of the minimum value of the vector.
 * \param[in] v
 * \param[out] index
*/
std::size_t myMin(std::vector<double> v);

/**
 * \fn std::size_t myMax(std::vector<double> v);
 * \brief Returns the index of the maximum value of the vector.
 * \param[in] v
 * \param[out] index
*/
std::size_t myMax(std::vector<double> v);


#endif

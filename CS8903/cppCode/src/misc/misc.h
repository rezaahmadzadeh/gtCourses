
#ifndef MISC_H
#define MISC_H

#include "types.h"
#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <vector>


double qNorm(q_t jointsComputed, q_t jointsCurrent);

std::size_t myMin(std::vector<double> v);

std::size_t myMax(std::vector<double> v);


#endif

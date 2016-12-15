/*
 * \file misc.cpp
 * \brief Miscellaneous functions.
 */

#include <stdio.h>
#include <stdlib.h>
#include "misc.h"
#include "macros.h"
#include "math.h"

// norm(x) = sum |x_i|
double qNorm(q_t jointsComputed, q_t jointsCurrent){
    double norm=0.0;

    //Manhattan norm
    for(int j=0;j<6;j++){
        norm+= fabs(jointsComputed[j]-jointsCurrent[j]); 
    }

    // Infinity norm: Leads to discontinuities for dataset 2_5
    /*
    std::vector<double> disp(6,0.0);
    std::size_t j=0;
    for (std::size_t i=0;i<6;i++){
        disp[i] = fabs(jointsComputed[j]-jointsCurrent[j]); 
    }
    norm = disp[myMax(disp)];
    */
    return norm;
}


std::size_t myMin(std::vector<double> v){
    std::size_t cardinal=v.size();
    std::size_t minIdx=0;
    double minValue=0;
    if(DEBUG){
        for(std::size_t i=0;i<cardinal;i++){
            std::cout << v[i] << "  " << std::endl;
        }
    }
    if(cardinal>1){
        minValue = v[0];
        for (std::size_t i=0;i<cardinal;i++){
            if(v[i]<minValue){
                minValue = v[i];
                minIdx=i;
            }
        }
    }
    if(DEBUG){
        std::cout << "Min found at index " << minIdx << "=" << minValue << std::endl;
    }
    return minIdx;
}

std::size_t myMax(std::vector<double> v){
    std::size_t cardinal=v.size();
    std::size_t maxIdx=0;
    double maxValue=0;
    if(DEBUG){
        for(std::size_t i=0;i<cardinal;i++){
            std::cout << v[i] << "  " << std::endl;
        }
    }
    if(cardinal>1){
        maxValue = v[0];
        for (std::size_t i=0;i<cardinal;i++){
            if(v[i]>maxValue){
                maxValue = v[i];
                maxIdx=i;
            }
        }
    }
    if(DEBUG){
        std::cout << "Max found at index " << maxIdx << "=" << maxValue << std::endl;
    }
    return maxIdx;
}

/**
 * \file robot.cpp
 * \brief Run IKfast FK on joints in input file and save the result to file.
 */

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <fstream>
#include <cstring>
#include <string>
#include <vector>
#include <time.h>

#define IKFAST_HAS_LIBRARY // Build IKFast with API functions
#define IKFAST_NO_MAIN // Don't include main() from IKFast
#define IK_VERSION 56
#include "output_ikfast8.cpp"
float SIGN(float x);
float NORM(float a, float b, float c, float d);
#define IKREAL_TYPE IkReal // for IKFast 56,61

#include "macros.h"

int main(int argc, char** argv) {

    IKREAL_TYPE eerot[9], eetrans[3]; //transformation matrix
    unsigned int num_of_joints = 6;
    IKREAL_TYPE joints[num_of_joints];
    std::vector<double> tokens(6,0.0);
    int count=0;
    int iter = 0;
    std::ifstream robot;
    std::ofstream t_fkfast;
    std::string line;
    t_fkfast.open(TEST_DATA_PATH_T, std::ios::app);
    robot.open(SOURCE_DATA_PATH);
    if(!robot.is_open()){
        std::cout << "No such file. Exit." << std::endl;
        exit(-1);
    }

    while (robot.is_open() /*&& (iter<ITER_MAX)*/) {
        // Parse
        robot >> count >> joints[0] >> joints[1] >> joints[2] >> joints[3] >> joints[4] >> joints[5] >> tokens[0] >> tokens[1] >> tokens[2] >> tokens[3] >> tokens[4] >> tokens[5];
        if(robot.eof()){
            break;
        }
        
        // Convert radians to degree (!!! ikfast takes joint values in radians !!!)
        for(std::size_t i=0;i<6;i++){
            joints[i]*=(double)D2R;
        }
        
        // FK ikfast computation
        ComputeFk(joints, eetrans, eerot);

        // Save results to file
        t_fkfast << 
            eerot[0] << "  " << eerot[1] << "  " << eerot[2] << "  " << eetrans[0] << "  " <<
            eerot[3] << "  " << eerot[4] << "  " << eerot[5] << "  " << eetrans[1] << "  "  << 
            eerot[6] << "  " << eerot[7] << "  " << eerot[8] << "  " << eetrans[2] << "\n";
        iter ++;
    }
    robot.close();
    t_fkfast.close();
    return 0;
}

float SIGN(float x) {
    return (x >= 0.0f) ? +1.0f : -1.0f;
}

float NORM(float a, float b, float c, float d) {
    return sqrt(a * a + b * b + c * c + d * d);
}


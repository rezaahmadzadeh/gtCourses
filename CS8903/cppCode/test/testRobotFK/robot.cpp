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

#include "macros.h"

#define IKFAST_HAS_LIBRARY // Build IKFast with API functions
#define IKFAST_NO_MAIN // Don't include main() from IKFast

#define IK_VERSION 56
#include "output_ikfast8.cpp"

float SIGN(float x);
float NORM(float a, float b, float c, float d);

#define IKREAL_TYPE IkReal // for IKFast 56,61


int main(int argc, char** argv) {

    double deg = M_PI/180;
    std::ifstream robot;
    robot.open(SOURCE_DATA_PATH);
    unsigned int l,c;
    std::size_t pos1, pos2;
    IKREAL_TYPE eerot[9], eetrans[3]; //transformation matrix
    unsigned int num_of_joints = 6;
    std::ofstream t_fkfast;
    int iter = 0;
    bool endOfDocument = false;

    std::string line;
    std::getline(robot, line);
    endOfDocument = robot.eof();
    if(endOfDocument){
        std::cout << "Error: the document is empty." << std::endl;
        exit(-1);
    }
    
    while (!endOfDocument /*&& (iter<ITER_MAX)*/) {
        if(DEBUG){
            //std::cout << line << std::endl;
        }
        // Parse current line in the joint file
        std::vector<std::string> tokens;
        pos1 = 0;
        while(pos1 != std::string::npos) {
            pos2 = line.find(DELIMITER, pos1+1);
            std::string t;
            if(pos1==0){
                t = line.substr(pos1, pos2-pos1-1);
            }
            else{
                t = line.substr(pos1+1, pos2-pos1-1);
            }
            if(DEBUG){
                //std::cout << "t=" << t << std::endl;
            }
            pos1 = line.find(DELIMITER,pos1+1);
            tokens.push_back(t);
        }

        // FK ikfast computation
        IKREAL_TYPE joints[num_of_joints];
        for(unsigned int i=0;i<num_of_joints;i++){
            joints[i] = atof(tokens[i+1].c_str()) * deg;
        }
        ComputeFk(joints, eetrans, eerot);

        // Save results to file
        t_fkfast.open(TEST_DATA_PATH_T, std::ios::app);
        for(l=0;l<3;l++){
            for(c=0;c<3;c++){
                t_fkfast << eerot[3*l+c] << "  ";
            }
            if(l<2){
                t_fkfast << eetrans[l] << "  ";
            }
        }
        t_fkfast << eetrans[2] << "\n";
        t_fkfast.close();
        iter ++;
        std::getline(robot, line);
        endOfDocument = robot.eof();
        if(endOfDocument){
            break;
        }
    }
    robot.close();
    return 0;
}



float SIGN(float x) {
    return (x >= 0.0f) ? +1.0f : -1.0f;
}

float NORM(float a, float b, float c, float d) {
    return sqrt(a * a + b * b + c * c + d * d);
}


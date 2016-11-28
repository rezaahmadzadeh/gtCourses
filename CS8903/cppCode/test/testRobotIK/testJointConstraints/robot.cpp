/**
 * \file robot.cpp
 * \brief Reads IKfast IK joint results, apply joint constraints and save new joint to file.
 */

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <fstream>
#include <cstring>
#include <string>
#include <vector>
#include <time.h>
#include <math.h>
#include <map>

#include "types.h"
#include "macros.h"


#define IKFAST_HAS_LIBRARY // Build IKFast with API functions
#define IKFAST_NO_MAIN // Don't include main() from IKFast

#define IK_VERSION 56
#include "output_ikfast8.cpp"

#define IKREAL_TYPE IkReal // for IKFast 56,61

float SIGN(float x);
float NORM(float a, float b, float c, float d);


int main(int argc, char** argv) {

    std::ifstream robot;
    robot.open(ALL_Q_PATH);
    unsigned int l,c;
    std::size_t pos1, pos2;
    q_t q(6,0.0);
    std::ofstream q_ikfast, t_ikfast;
    int iter = 0;
    bool endOfDocument = false;
    unsigned int num_of_joints = 6;
    IKREAL_TYPE jointsComputed[num_of_joints];
    IKREAL_TYPE eerot[9], eetrans[3];
    float jointLimitMin[] = {J0_MIN, J1_MIN, J2_MIN, J3_MIN, J4_MIN, J5_MIN};
    float jointLimitMax[] = {J0_MAX, J1_MAX, J2_MAX, J3_MAX, J4_MAX, J5_MAX};
    bool jointViolation = false;

    std::string line;
    std::getline(robot, line);
    endOfDocument = robot.eof();
    if(endOfDocument){
        exit(-1);
    }

    bool newline=false;

    while (!endOfDocument /*&& (iter<ITER_MAX)*/) {
        newline=false;
        while(line!="\0"){
            // Parsing
            q_t q;
            jointViolation=false;
            pos1 = 0;
            while(pos1 != std::string::npos) {
                pos2 = line.find(DELIMITER, pos1+1);
                std::string t;
                if(pos1==0){
                    t = line.substr(pos1, pos2-pos1); //TODO -1
                }
                else {
                    t = line.substr(pos1+2, pos2-pos1-1); //TODO +1
                }
                //std::cout << t << "  " << atof(t.c_str()) << std::endl;
                pos1 = line.find(DELIMITER,pos1+1);
                q.push_back(atof(t.c_str()));
            }

            // Apply joint constraints
            for(std::size_t j=0;j<q.size();j++){
                if(q[j]<jointLimitMin[j]){
                    if( (q[j]+2*M_PI)<jointLimitMax[j]){
                        q[j] +=2*M_PI;
                    }
                    else{
                        jointViolation=true;
                        break; //joint violation
                    }
                }
                else if(q[j]>jointLimitMax[j]){
                    if( (q[j]-2*M_PI)>jointLimitMin[j]){
                        q[j]-=2*M_PI;
                    }
                    else{
                        jointViolation=true;
                        break; //joint violation
                    }
                }
            }

            if(jointViolation){
                std::getline(robot, line);
                endOfDocument=robot.eof();
                if(endOfDocument){
                    newline=false;
                    break;
                }
                continue;
            }
            newline=true;

            // Save result to file
            q_ikfast.open(TEST_DATA_PATH_Q, std::ios::app);
            for(c=0;c<num_of_joints-1;c++){
                q_ikfast << q[c] << "  ";
                jointsComputed[c]=q[c];
            }
            q_ikfast << q[num_of_joints-1] << "\n";
            jointsComputed[num_of_joints-1] = q[num_of_joints-1];
            q_ikfast.close();

            ComputeFk(jointsComputed, eetrans, eerot);
            t_ikfast.open(TEST_DATA_PATH_T, std::ios::app);
            for(l=0;l<3;l++){
                for(c=0;c<3;c++){
                    t_ikfast << eerot[3*l+c] << "  ";
                }
                if(l<2){
                    t_ikfast << eetrans[l] << "  ";
                }
            }
            t_ikfast << eetrans[2] << "\n";
            t_ikfast.close();

            std::getline(robot, line);
            endOfDocument = robot.eof();
            if(endOfDocument){
                newline=false;
                break;
            }

        } 
        std::getline(robot, line);
        endOfDocument = robot.eof();
        if(endOfDocument){
            newline=false;
            break;
        }
        if(newline){
            q_ikfast.open(TEST_DATA_PATH_Q, std::ios::app);
            q_ikfast << "\n";
            q_ikfast.close();

            t_ikfast.open(TEST_DATA_PATH_T, std::ios::app);
            t_ikfast << "\n";
            t_ikfast.close();
        }
        iter ++;
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


/**
 * \file testJointConstraints/robot.cpp
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

    IKREAL_TYPE eerot[9], eetrans[3];
    unsigned int c;
    std::size_t pos1, pos2;
    q_t q(6,0.0);
    int iter = 0;
    bool endOfDocument = false;
    unsigned int num_of_joints = 6;
    IKREAL_TYPE jointsComputed[num_of_joints];
    double jointLimitMin[] = {J0_MIN, J1_MIN, J2_MIN, J3_MIN, J4_MIN, J5_MIN};
    double jointLimitMax[] = {J0_MAX, J1_MAX, J2_MAX, J3_MAX, J4_MAX, J5_MAX};
    bool jointViolation = false;
    bool newline=false;
    std::string line;
    std::ifstream robot;
    std::ofstream q_ikfast, t_ikfast;
    q_ikfast.open(TEST_DATA_PATH_Q, std::ios::app);
    t_ikfast.open(TEST_DATA_PATH_T, std::ios::app);
    
    robot.open(ALL_Q_PATH); // check file exists
    if(!robot.is_open()){
        std::cout << "NO such file. Exit." << std::endl;
        exit(-1);
    }

    std::getline(robot, line); //check file is not empty
    endOfDocument = robot.eof();
    if(endOfDocument){
        exit(-1);
    }

    while (!endOfDocument /*&& (iter<ITER_MAX)*/) {
        newline=false;
        // for each ee pose, constraint all joint solutions
        while(line!="\0"){ 
            jointViolation=false;
            
            // Parsing
            q_t q;
            pos1=0;
            while(pos1 != std::string::npos) {
                pos2 = line.find(DELIMITER, pos1+1);
                std::string t;
                if(pos1==0){
                    t = line.substr(pos1, pos2-pos1); 
                }
                else {
                    t = line.substr(pos1+2, pos2-pos1-1); 
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
            
            // If this joint violates limits, get the next joint solution
            if(jointViolation){
                std::getline(robot, line); //get next joint solution for same ee pose
                endOfDocument=robot.eof();
                if(endOfDocument){
                    newline=false;
                    break;
                }
                continue;
            }
            // At this point, we have a joint solution beyond limits. 
            // So when we finish processing this ee pose, we break a new line
            newline=true; 

            // Save result to file
            q_ikfast <<q[0]<<"  "<<q[1]<<"  "<<q[2]<<"  "<<q[3]<<"  "<<q[4]<<"  "<<q[5]<<"  "<<"\n";
            for(c=0;c<num_of_joints;c++){
                jointsComputed[c]=q[c];
            }
            ComputeFk(jointsComputed, eetrans, eerot);
            t_ikfast << 
                eerot[0] << "  " << eerot[1] << "  " << eerot[2] << "  " << eetrans[0] << "  " <<
                eerot[3] << "  " << eerot[4] << "  " << eerot[5] << "  " << eetrans[1] << "  "  << 
                eerot[6] << "  " << eerot[7] << "  " << eerot[8] << "  " << eetrans[2] << "\n";

            std::getline(robot, line); //get next joint solution for same ee pose
            endOfDocument = robot.eof();
            if(endOfDocument){
                newline=false;
                break;
            }
        } 
        std::getline(robot, line); //get first joint solution for new ee pose
        endOfDocument = robot.eof();
        if(endOfDocument){
            newline=false;
            break;
        }

        // Separates set of joint solutions for one ee pose by a newline.
        if(newline){ 
            q_ikfast << "\n";
            t_ikfast << "\n";
        }
        iter ++;
    }
    robot.close();
    q_ikfast.close();
    t_ikfast.close();
    return 0;
}


float SIGN(float x) {
    return (x >= 0.0f) ? +1.0f : -1.0f;
}

float NORM(float a, float b, float c, float d) {
    return sqrt(a * a + b * b + c * c + d * d);
}

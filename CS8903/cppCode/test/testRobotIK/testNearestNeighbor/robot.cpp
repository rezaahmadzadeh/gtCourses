/**
 * \file testNearestNeighbor/robot.cpp
 * \brief Reads Ikfast joint constrained solutions and map each ee pose to one unique joint configuration, the nearest one to previous joint configuration.
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
#include "misc.h"


#define IKFAST_HAS_LIBRARY // Build IKFast with API functions
#define IKFAST_NO_MAIN // Don't include main() from IKFast

#define IK_VERSION 56
#include "output_ikfast8.cpp"

#define IKREAL_TYPE IkReal // for IKFast 56,61

float SIGN(float x);
float NORM(float a, float b, float c, float d);


int main(int argc, char** argv) {

    unsigned int l,c;
    std::size_t pos1, pos2;
    q_t q(6,0.0);
    q_t qCurrent(6,0.0);
    int iter = 0;
    bool endOfDocument = false;
    unsigned int num_of_joints = 6;
    IKREAL_TYPE jointsComputed[num_of_joints];
    IKREAL_TYPE eerot[9], eetrans[3];
    std::ifstream robot;
    std::ofstream q_ikfast, t_ikfast;
    std::string line;
    robot.open(JOINT_CONSTRAINTS_Q_PATH);
    q_ikfast.open(TEST_DATA_PATH_Q, std::ios::app);
    t_ikfast.open(TEST_DATA_PATH_T, std::ios::app);
    if(!robot.is_open()){
        std::cout << "No such file. Exit." << std::endl;
        exit(-1);
    }
    std::getline(robot, line);
    endOfDocument = robot.eof();
    if(endOfDocument){
        std::Cout << "Empty file. Exit." << std::endl;
        exit(-1);
    }

    while (!endOfDocument /*&& (iter<ITER_MAX)*/) {
        // Parsing
        std::vector<q_t> allQ; //all the joint solution for the current ee
        // Store distance to previous joint configuration to all joints solutions of the current ee
        std::vector<double> qDistances;

        while(line!="\0"){ // For all joint solution of the current ee
            q_t q;
            pos1 = 0;
            while(pos1 != std::string::npos) {
                pos2 = line.find(DELIMITER, pos1+1);
                std::string t;
                if(pos1==0){
                    t = line.substr(pos1, pos2-pos1);
                }
                else {
                    t = line.substr(pos1+2, pos2-pos1-1);
                }
                pos1 = line.find(DELIMITER,pos1+1);
                q.push_back(atof(t.c_str()));
                //std::cout << t << "  " << atof(t.c_str()) << std::endl;
            }

            allQ.push_back(q);
            std::getline(robot, line); //get next joint solution for the current ee pose
            endOfDocument=robot.eof();
            if(endOfDocument){
                break;
            }
        } // at this point, we have all the q solution for one ee in allQ

        if(endOfDocument || (allQ.size()==0)){
            break;
        }

        // Choose randomly the joint solution for the first ee pose.
        if(iter==0){
            c=rand()%allQ.size();
            qCurrent = allQ[c]; 
        }

        // For all joint solution of the current ee pose
        // Compute distance between the previous joint solution and these solutions
        // Select the next nearest joint configuration.
        for(c=0;c<allQ.size();c++){
            double d = qNorm(qCurrent, allQ[c]);
            qDistances.push_back(d);
        }
        int nearestIdx = myMin(qDistances);
        qCurrent = allQ[nearestIdx];

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

        std::getline(robot, line);
        endOfDocument = robot.eof();
        if(endOfDocument){
            break;
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



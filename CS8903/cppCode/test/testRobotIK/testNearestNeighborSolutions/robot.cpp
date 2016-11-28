/**
 * \file robot.cpp
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

    std::ifstream robot;
    robot.open(JOINT_CONSTRAINTS_Q_PATH);
    unsigned int l,c;
    std::size_t pos1, pos2;
    q_t q(6,0.0);
    q_t qCurrent(6,0.0);
    std::ofstream q_ikfast, t_ikfast;
    int iter = 0;
    bool endOfDocument = false;
    unsigned int num_of_joints = 6;
    IKREAL_TYPE jointsComputed[num_of_joints];
    IKREAL_TYPE eerot[9], eetrans[3];

    std::string line;
    std::getline(robot, line);
    endOfDocument = robot.eof();
    if(endOfDocument){
        exit(-1);
    }

    while (!endOfDocument /*&& (iter<ITER_MAX)*/) {
        // Parsing
        std::vector<q_t> allQ; //store all the joint solution for the current ee
        std::vector<double> qDistances;

        while(line!="\0"){ 
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
            std::getline(robot, line);
            endOfDocument=robot.eof();
            if(endOfDocument){
                break;
            }
        } // at this point, we have all the q solution for one ee in allQ
        
        if(endOfDocument || (allQ.size()==0)){
            break;
        }
        
        // Choose randomly the first joint configuration
        if(iter==0){
            c=rand()%allQ.size();
            qCurrent = allQ[c]; //TODO choose randomly
        }
       
        // Select the next nearest joint configuration.
        for(c=0;c<allQ.size();c++){
            double d = qNorm(qCurrent, allQ[c]);
            qDistances.push_back(d);
        }
        int nearestIdx = myMin(qDistances);
        qCurrent = allQ[nearestIdx];

        // Save result to file.
        q_ikfast.open(TEST_DATA_PATH_Q, std::ios::app);
        for(c=0;c<num_of_joints-1;c++){
            q_ikfast << qCurrent[c] << "  ";
            jointsComputed[c]=qCurrent[c];
        }
        q_ikfast << qCurrent[num_of_joints-1] << "\n";
        jointsComputed[num_of_joints-1] = qCurrent[num_of_joints-1];
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
            break;
        }

        /*
        q_ikfast.open(TEST_DATA_PATH_Q, std::ios::app);
        q_ikfast << "\n";
        q_ikfast.close();

        t_ikfast.open(TEST_DATA_PATH_T, std::ios::app);
        t_ikfast << eetrans[2] << "\n";
        t_ikfast.close();
        */
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



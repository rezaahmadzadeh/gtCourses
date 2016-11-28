/**
 * \file robot.cpp
 * \brief Reads IKfast joint constrained solution file and solve the shortest path in terms of joint displacement between the first and the last ee pose.
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
#include <stack>

#include "types.h"
#include "macros.h"
#include "misc.h"
#include "GraphSearch.h"


#define IKFAST_HAS_LIBRARY // Build IKFast with API functions
#define IKFAST_NO_MAIN // Don't include main() from IKFast

#define IK_VERSION 56
#include "output_ikfast8.cpp"

#define IKREAL_TYPE IkReal // for IKFast 56,61

float SIGN(float x);
float NORM(float a, float b, float c, float d);


int main(int argc, char** argv) {

    std::ifstream robot;
    robot.open(JOINT_PATH_RAW);
    std::ofstream q_ikfast, t_ikfast;
    q_ikfast.open(TEST_DATA_PATH_Q, std::ios::app);
    t_ikfast.open(TEST_DATA_PATH_T, std::ios::app);
    int l,c;
    int iter = 0;
    bool endOfDocument = false;
    unsigned int num_of_joints = 6;
    IKREAL_TYPE jointsComputed[num_of_joints];
    IKREAL_TYPE eerot[9], eetrans[3];

    q_t qCurrent(6,0.0);
    q_t qNext(6,0.0);
    std::vector<double> disp(6,0.0);
    std::vector<double> stepSize(6,0.0);
    std::size_t i=0;

    robot >> qCurrent[0] >> qCurrent[1] >> qCurrent[2] >> qCurrent[3] >> qCurrent[4] >> qCurrent[5];
    endOfDocument = robot.eof();

    while ((!endOfDocument) /*&&  (iter<ITER_MAX)*/) {
        robot >> qNext[0] >> qNext[1] >> qNext[2] >> qNext[3] >> qNext[4] >> qNext[5];

        // If there is no more joint to read, save the last joint and exit.
        endOfDocument = robot.eof();
        if(endOfDocument){
            q_ikfast << qCurrent[0] << "  " << qCurrent[1] << "  " << qCurrent[2] << "  " << qCurrent[3] << "  " << qCurrent[4] << "  " << qCurrent[5] << "\n";
            break;
        }

        // Get maximum joint displacement and sense of displacement of each joint
        for( i=0; i<num_of_joints;i++){
            disp[i]=fabs(qCurrent[i] - qNext[i]);
        }
        double maxDisp = disp[myMax(disp)];
        std::cout << iter << " maxDisp= " << maxDisp << std::endl;

        if(maxDisp>EPSILON_Q){
            break;
        }
        //std::cout << iter << " qcurrent " << qCurrent[0] << "  " <<qCurrent[1] <<  "  " <<qCurrent[2] <<  "  " <<qCurrent[3] <<  "  " <<qCurrent[4] <<  "  " <<qCurrent[5] << std::endl;
        //std::cout << iter << " qNext " << qNext[0] <<  "  " <<qNext[1] <<  "  " <<qNext[2] <<  "  " <<qNext[3] <<  "  " <<qNext[4] <<  "  " <<qNext[5]  << std::endl;
        //std::cout << "norm=" << d << "\n" << std::endl;

        for(i=0;i<num_of_joints;i++){
            jointsComputed[i]=qCurrent[i];
        }
        q_ikfast << qCurrent[0] << "  " << qCurrent[1] << "  " << qCurrent[2] << "  " << qCurrent[3] << "  " << qCurrent[4] << "  " << qCurrent[5] << "\n";

        ComputeFk(jointsComputed, eetrans, eerot);
        for(l=0;l<3;l++){
            for(c=0;c<3;c++){
                t_ikfast << eerot[3*l+c] << "  ";
            }
            if(l<2){
                t_ikfast << eetrans[l] << "  ";
            }
        }
        t_ikfast << eetrans[2] << "\n";
        qCurrent = qNext;
        iter++;
    } 
    robot.close();
    t_ikfast.close();
    q_ikfast.close();
    return 0;
}



float SIGN(float x) {
    return (x >= 0.0f) ? +1.0f : -1.0f;
}

float NORM(float a, float b, float c, float d) {
    return sqrt(a * a + b * b + c * c + d * d);
}



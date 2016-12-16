/**
 * \file testAll/robot.cpp
 * \brief Computes IKfast IK on ee pose from input file and save resulting joints and corresponding IKfast computed ee to file.
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

    // eerot: rotation matrix eerot[2*l+c]=eerot[l,c]
    // eetrans: cartesian coordinate
    IKREAL_TYPE eerot[9], eetrans[3];
    unsigned int count; //dummy parsing variable in raw data
    double theta1, theta2, theta3;
    unsigned int num_of_joints = 6;
    unsigned int num_free_parameters = 0;
    IKREAL_TYPE jointsComputed[num_of_joints];
    q_t q(6,0);
    q_t qPrevious(6,0);
    int iter = 0;
    std::vector<double> tokens(12,0.0);
    std::ofstream q_ikfast_all, t_ikfast_all; 
    std::ifstream robot;
    q_ikfast_all.open(TEST_DATA_PATH_Q, std::ios::app);
    t_ikfast_all.open(TEST_DATA_PATH_T, std::ios::app);
    robot.open(SOURCE_DATA_PATH);
    if(!robot.is_open()){
        std::cout << "No such file. Exit." << std::endl;
        exit(-1);
    }

    while (robot.is_open() /*&& (iter<ITER_MAX)*/) {    
        // Parsing and processing current line of input file.
        switch(DATA_TYPE){
            case(RAW_DATA):
                {
                    robot >> count >> tokens[0] >> tokens[1] >> tokens[2] >> tokens[3] >> tokens[4] >> tokens[5] >> tokens[6] >> tokens[7] >> tokens[8] >> tokens[9] >> tokens[10] >> tokens[11];
                    if(robot.eof()){
                        break;
                    }
                    eetrans[0] = tokens[6]; // cartesian position
                    eetrans[1] = tokens[7];
                    eetrans[2] = tokens[8];
                    theta1=tokens[9]; // orientation in euler angles
                    theta2=tokens[10]; 
                    theta3=tokens[11];
                    break;
                }
            case(PROCESSED_DATA):
                {
                    robot >> tokens[0] >> tokens[1] >> tokens[2] >> tokens[3] >> tokens[4] >> tokens[5];
                    if(robot.eof()){
                        break;
                    }
                    eetrans[0] = tokens[0];
                    eetrans[1] = tokens[1];
                    eetrans[2] = tokens[2];
                    theta1=tokens[3]; 
                    theta2=tokens[4]; 
                    theta3=tokens[5];
                    break;
                }
        }     
        if(robot.eof()){
            break;
        }

        // Euler to rotation matrix based on wikipedia formula for the XYZ convention
        double c1=cos(theta1); double c2=cos(theta2); double c3= cos(theta3);
        double s1 = sin(theta1); double s2=sin(theta2); double s3=sin(theta3);
        eerot[0] = c2*c3;
        eerot[1] = -c2*s3;
        eerot[2] = s2;
        eerot[3] = c1*s3 + c3*s1*s2;
        eerot[4] = c1*c3 - s1*s2*s3;
        eerot[5] = -c2*s1;
        eerot[6] = s1*s3 - c1*c3*s2;
        eerot[7] = c3*s1 + c1*s2*s3;
        eerot[8] = c1*c2;   

        // Begin: IK computation
        IkSolutionList<IKREAL_TYPE> solutions;
        std::vector<IKREAL_TYPE> vfree(num_free_parameters);
        for(std::size_t i = 0; i < vfree.size(); ++i){ 
            vfree[i] = atof(argv[13+i]);
        }
        bool bSuccess = ComputeIk(eetrans, eerot, vfree.size() > 0 ? &vfree[0] : NULL, solutions);
        if( !bSuccess ) {
            fprintf(stderr,"Failed to get ik solution\n");
            continue;
        }

        // Save computed joint and computed ee pose to file. 
        unsigned int num_of_solutions = (int)solutions.GetNumSolutions();
        if(DEBUG){
            printf("Found %d ik solutions:\n", num_of_solutions ); 
        }

        std::vector<IKREAL_TYPE> solvalues(num_of_joints);
        for(std::size_t i = 0; i < num_of_solutions; ++i) {
            const IkSolutionBase<IKREAL_TYPE>& sol = solutions.GetSolution(i);
            int this_sol_free_params = (int)sol.GetFree().size(); 
            std::vector<IKREAL_TYPE> vsolfree(this_sol_free_params);
            sol.GetSolution(&solvalues[0],vsolfree.size()>0?&vsolfree[0]:NULL);

            for( std::size_t j = 0; j < solvalues.size(); ++j){
                jointsComputed[j] = solvalues[j];
            }
            //std::cout << jointsComputed[0]<<" "<< jointsComputed[1]<<" "<<jointsComputed[2]<<" "<<jointsComputed[3]<<" "<<jointsComputed[4]<<" "<<jointsComputed[5]<< std::endl; 
            // Save computed joint in q_ikfast
            q_ikfast_all << 
                jointsComputed[0] << "  " << jointsComputed[1] <<  "  " << 
                jointsComputed[2] << "  " << jointsComputed[3] <<  "  " << 
                jointsComputed[4] <<  "  " << jointsComputed[5] << "\n";

            // Save ee pose corresponding to joint with IKfast IK.
            ComputeFk(jointsComputed, eetrans, eerot);
            t_ikfast_all << 
                eerot[0] << "  " << eerot[1] << "  " << eerot[2] << "  " << eetrans[0] << "  " <<
                eerot[3] << "  " << eerot[4] << "  " << eerot[5] << "  " << eetrans[1] << "  "  << 
                eerot[6] << "  " << eerot[7] << "  " << eerot[8] << "  " << eetrans[2] << "\n";
        } 
        iter ++;
        //std::cout << iter << std::endl;

        q_ikfast_all << "\n";
        t_ikfast_all << "\n ";
    }
    q_ikfast_all.close();
    t_ikfast_all.close();
    robot.close();
    return 0;
}



float SIGN(float x) {
    return (x >= 0.0f) ? +1.0f : -1.0f;
}

float NORM(float a, float b, float c, float d) {
    return sqrt(a * a + b * b + c * c + d * d);
}





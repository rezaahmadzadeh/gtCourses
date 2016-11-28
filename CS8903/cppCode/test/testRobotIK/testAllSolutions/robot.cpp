/**
 * \file robot.cpp
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

    std::ifstream robot;
    robot.open(SOURCE_DATA_PATH);
    unsigned int l,c;
    std::size_t pos1, pos2;
    IKREAL_TYPE eerot[9], eetrans[3];
    double theta1, theta2, theta3;
    unsigned int num_of_joints = 6;
    unsigned int num_free_parameters = 0;
    IKREAL_TYPE jointsComputed[num_of_joints];
    q_t q(6,0);
    q_t qPrevious(6,0);
    std::ofstream q_ikfast_all, t_ikfast_all ;
    int iter = 0;
    bool endOfDocument = false;

    std::string line;
    std::getline(robot,line);
    endOfDocument = robot.eof();
    if(endOfDocument){
        std::cout << "Error: The document is empty." << std::endl;
        exit(-1);
    }
   
    while (!endOfDocument /*&& (iter<ITER_MAX)*/) {
        
        // Parsing and processing current line of input file.
        std::vector<std::string> tokens;
        pos1 = 0;
        while(pos1 != std::string::npos) {
            pos2 = line.find(DELIMITER, pos1+1);
            std::string t;
            if(pos1==0){
                t = line.substr(pos1, pos2-pos1-1);
            }
            else {
                t = line.substr(pos1+1, pos2-pos1-1);
            }
            //std::cout << "t=" << t << std::endl;
            pos1 = line.find(DELIMITER,pos1+1);
            tokens.push_back(t);
        }

        switch(DATA_TYPE){
            case(RAW_DATA):
                {
                    eetrans[0] = atof(tokens[7].c_str()); // cartesian position
                    eetrans[1] = atof(tokens[8].c_str());
                    eetrans[2] = atof(tokens[9].c_str());
                    theta1=atof(tokens[10].c_str()); // orientation in euler angles
                    theta2=atof(tokens[11].c_str()); 
                    theta3=atof(tokens[12].c_str());
                    break;
                }
            case(PROCESSED_DATA):
                {
                    eetrans[0] = atof(tokens[0].c_str());
                    eetrans[1] = atof(tokens[1].c_str());
                    eetrans[2] = atof(tokens[2].c_str());
                    theta1=atof(tokens[3].c_str()); 
                    theta2=atof(tokens[4].c_str()); 
                    theta3=atof(tokens[5].c_str());
                    break;
                }
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
        // End: parsing


        // Begin: IK computation
        IkSolutionList<IKREAL_TYPE> solutions;
        std::vector<IKREAL_TYPE> vfree(num_free_parameters);
        for(std::size_t i = 0; i < vfree.size(); ++i){ 
            vfree[i] = atof(argv[13+i]);
        }
        bool bSuccess = ComputeIk(eetrans, eerot, vfree.size() > 0 ? &vfree[0] : NULL, solutions);
        if( !bSuccess ) {
            fprintf(stderr,"Failed to get ik solution\n");
            std::getline(robot, line);
            endOfDocument = robot.eof();
            if(endOfDocument){
                //std::cout << "End of document" << std::endl;
                break;
            }
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
            // Save computed joint in q_ikfast
            q_ikfast_all.open(TEST_DATA_PATH_Q, std::ios::app);
            for(c=0;c<num_of_joints-1;c++){
                q_ikfast_all << jointsComputed[c] << "  ";
            }  
            q_ikfast_all << jointsComputed[num_of_joints-1] << "\n";
            q_ikfast_all.close();

            // Save ee pose corresponding to joint with IKfast IK.
            ComputeFk(jointsComputed, eetrans, eerot);
            t_ikfast_all.open(TEST_DATA_PATH_T, std::ios::app);
            for(l=0;l<3;l++){
                for(c=0;c<3;c++){
                    t_ikfast_all << eerot[3*l+c] << "  ";
                }
                if(l<2){
                    t_ikfast_all << eetrans[l] << "  ";
                }
            }
            t_ikfast_all << eetrans[2] << "\n";
            t_ikfast_all.close();
        } 

        iter ++;
        std::getline(robot, line);
        endOfDocument = robot.eof();
        if(endOfDocument){
            break;
        }

        q_ikfast_all.open(TEST_DATA_PATH_Q, std::ios::app);
        q_ikfast_all << "\n";
        q_ikfast_all.close();
        t_ikfast_all.open(TEST_DATA_PATH_T, std::ios::app);
        t_ikfast_all << "\n ";
        t_ikfast_all.close();
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





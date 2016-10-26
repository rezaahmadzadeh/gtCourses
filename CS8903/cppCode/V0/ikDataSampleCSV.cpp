/**
 * IK computation for random values. Ok
 */


/*
 * IKFast Demo
 * 
 * Shows how to calculate FK from joint angles.
 * Calculates IK from rotation-translation matrix, or translation-quaternion pose.
 * Performance timing tests.
 *
 * Run the program to view command line parameters.
 * 
 * 
 * To compile, run:
 * g++ -lstdc++ -llapack -o compute ikfastdemo.cpp -lrt
 * (need to link with 'rt' for gettime(), it must come after the source file name)
 *
 * 
 * Tested with Ubuntu 11.10 (Oneiric)
 * IKFast54 from OpenRAVE 0.6.0
 * IKFast56/61 from OpenRave 0.8.2
 *
 * Author: David Butterworth, KAIST
 *         Based on code by Rosen Diankov
 * Date: November 2012
 */

/*
 * Copyright (c) 2012, David Butterworth, KAIST
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#define IKFAST_HAS_LIBRARY // Build IKFast with API functions
#define IKFAST_NO_MAIN // Don't include main() from IKFast

/*
Set which IKFast version you are using
The API calls are slightly different for versions > 54
*/

#define IK_VERSION 56
#include "output_ikfast7.cpp"

//#define IK_VERSION 56
//#include "ikfast56.Transform6D.0_1_2_3_4_5.cpp"

//#define IK_VERSION 54
//#include "output_ikfast54.cpp"


//----------------------------------------------------------------------------//

#include <stdio.h>
#include <stdlib.h>
#include <time.h> // for clock_gettime()
#include <fstream>
#include <string>

float SIGN(float x);
float NORM(float a, float b, float c, float d);

#if IK_VERSION > 54
#define IKREAL_TYPE IkReal // for IKFast 56,61
#else
#define IKREAL_TYPE IKReal // for IKFast 54
#endif

// Path joint trajectory with one variable joint
#define Q_INIT_0 0 
#define Q_INIT_1 M_PI_2
#define Q_INIT_2 -M_PI_2
#define Q_INIT_3 M_PI_2
#define Q_INIT_4 M_PI_2
#define Q_INIT_5 M_PI
#define Q_GOAL_1 M_PI

#define DIR_TEST "./randomTest/"
#define Q_PATH DIR_TEST "qpath.csv"
#define T_PATH DIR_TEST "tpath.csv"
#define Q_COMPUTED DIR_TEST "qcomputed.csv"
#define T_COMPUTED DIR_TEST "tcomputed.csv"


int main(int argc, char** argv)
{
    IKREAL_TYPE eerot[9],eetrans[3];
    unsigned int k,l,c;
    unsigned int num_of_joints = 6;
    unsigned int num_free_parameters = 0;
    unsigned jointVar = 1; //index of the modified joint 
    double stepSize=0.1;
    double jointInit = Q_INIT_1;
    double jointGoal = Q_GOAL_1;
    int pathLength = floor((jointGoal-jointInit)/stepSize);
    if(pathLength<0){
        pathLength = -pathLength;
        stepSize = -stepSize;
    }

    std::ofstream tpath, tcomputed, qpath, qcomputed;
    
    IKREAL_TYPE joints[num_of_joints]; //q_path
    IKREAL_TYPE jointsComputed[num_of_joints]; //q_computed
    joints[0] = Q_INIT_0; 
    joints[1] = Q_INIT_1; 
    joints[2] = Q_INIT_2; 
    joints[3] = Q_INIT_3; 
    joints[4] = Q_INIT_4; 
    joints[5] = Q_INIT_5; 
    
    for(k=0;k<(unsigned int)pathLength;k++){
        // q_path
        joints[jointVar] = Q_INIT_1 + k*stepSize;
        qpath.open(Q_PATH, std::ios::app);
        for(c=0;c<num_of_joints-1;c++){
            qpath << joints[c] << ',';
        }
        qpath << joints[num_of_joints-1] << "\n";
        qpath.close();

        // T_path
        ComputeFk(joints, eetrans, eerot);
        tpath.open(T_PATH, std::ios::app);
        for(l=0;l<3;l++){
            for(c=0;c<3;c++){
                tpath << eerot[3*l+c] << ',';
            }
            if(l<2) {
                tpath << eetrans[l] << ',';
            }
        }
        tpath << eetrans[2] << "\n";
        tpath.close();

        // q_computed
        IkSolutionList<IKREAL_TYPE> solutions;
        
        std::vector<IKREAL_TYPE> vfree(num_free_parameters);
        for(std::size_t i = 0; i < vfree.size(); ++i){ //never run because vfree.size()=0
            vfree[i] = atof(argv[13+i]);
        }
        
        bool bSuccess = ComputeIk(eetrans, eerot, vfree.size() > 0 ? &vfree[0] : NULL, solutions);
        if( !bSuccess ) {
            fprintf(stderr,"Failed to get ik solution\n");
            return -1;
        }

        unsigned int num_of_solutions = (int)solutions.GetNumSolutions();
        printf("Found %d ik solutions:\n", num_of_solutions ); 

        std::vector<IKREAL_TYPE> solvalues(num_of_joints);
        for(std::size_t i = 0; i < num_of_solutions; ++i) {
            const IkSolutionBase<IKREAL_TYPE>& sol = solutions.GetSolution(i);
            int this_sol_free_params = (int)sol.GetFree().size(); 
            printf("sol%d (free=%d): ", (int)i, this_sol_free_params );
            std::vector<IKREAL_TYPE> vsolfree(this_sol_free_params);

            sol.GetSolution(&solvalues[0],vsolfree.size()>0?&vsolfree[0]:NULL);
            for( std::size_t j = 0; j < solvalues.size(); ++j){
                jointsComputed[j] = solvalues[j]; 
                printf("%.15f, ", solvalues[j]);
            }
            printf("\n");
            
            // q_computed
            qcomputed.open(Q_COMPUTED, std::ios::app);
            for(c=0;c<num_of_joints-1;c++){
                qcomputed << jointsComputed[c] << ',';
            }  
            qcomputed << jointsComputed[num_of_joints-1] << "\n";
            qcomputed.close();

            // T_computed
            ComputeFk(jointsComputed, eetrans, eerot);
            tcomputed.open(T_COMPUTED, std::ios::app);
            for(l=0;l<3;l++){
                for(c=0;c<3;c++){
                    tcomputed << eerot[3*l+c] << ',';
                }
                if(l<2){
                    tcomputed << eetrans[l] << ',';
                }
            }
            tcomputed << eetrans[2] << "\n";
            tcomputed.close();
        }

        //TODO print separator between 2 IK computations
        //ft << 777 << "\n"; 

    }
    
    return 0;
}

float SIGN(float x) {
    return (x >= 0.0f) ? +1.0f : -1.0f;
}

float NORM(float a, float b, float c, float d) {
    return sqrt(a * a + b * b + c * c + d * d);
}

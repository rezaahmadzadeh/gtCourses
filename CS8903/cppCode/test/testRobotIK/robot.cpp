/**
 * \file robot.cpp
 * \brief Computes IKfast IK on ee pose from input file, apply joint constraints and use graph search to choose the joint solutions that leads to the minimum joint displacement of the arm. 
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
#include "GraphSearch.h"
#include "misc.h"

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
    std::size_t c;
    unsigned int num_of_joints = 6;
    unsigned int num_free_parameters = 0;
    IKREAL_TYPE jointsComputed[num_of_joints];
    double jointLimitMin[] = {J0_MIN, J1_MIN, J2_MIN, J3_MIN, J4_MIN, J5_MIN};
    double jointLimitMax[] = {J0_MAX, J1_MAX, J2_MAX, J3_MAX, J4_MAX, J5_MAX};
    bool jointViolation = false;
    bool oneSolution = false; // true when there exists at least one constrained solution
    q_t q(6,0);
    q_t qPrevious(6,0);
    int iter = 0;
    std::vector<double> tokens(12,0.0);
    std::ofstream q_ikfast, t_ikfast; 
    std::ifstream robot;
    
    std::vector<q_t> ikSolutions; //stores all joint solutions one after one, indexed by solIdx
    std::vector<int> solCard; //number of joint solution for each ee pose, indexed by eeIdx
    std::map<mapIndex, int> mapper; // maps a ee pose index and a joint solutions index to solIdx
    int solIdx, eeIdx, qIdx;
    solIdx = 0; //index joint solution in ikSolutions
    eeIdx = 0; //index the ee pose
    qIdx = 0; //index of the joint solution for a specific ee pose
    
    // Artificial initial node from which you reach the several possible solutions 
    // This joint configuration is specific to the trajecotry
    q_t q0;
    for(int i=0;i<6;i++){
        q0.push_back(0.0);
    }
    ikSolutions.push_back(q0);
    mapper[mapIndex(eeIdx, qIdx)]=solIdx;
    solCard.push_back(1);
    solIdx++;
    eeIdx++;
    qIdx=0;

    robot.open(SOURCE_DATA_PATH);
    while (robot.is_open() /*&& (iter<ITER_MAX)*/) {    
        qIdx=0;
        oneSolution=false;

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
                q[j] = solvalues[j];
            }
            //std::cout<<q[0]<<" "<<q[1] <<" "<<q[2]<<" "<<q[3]<<" "<<q[4]<<" "<<q[5]<<std::endl;

            // Apply joint constraints
            jointViolation=false;
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
                continue;
            }
            else{
                oneSolution=true;
            }

            // Intermediary storage used for graph construction 
            ikSolutions.push_back(q);
            mapper[mapIndex(eeIdx,qIdx)] = solIdx;
            qIdx++;
            solIdx++;    
        } 

        if(oneSolution){
            solCard.push_back(qIdx); // store number of solution for current ee pose
            eeIdx++; // index for new ee pose
            iter ++;
            //std::cout << iter << std::endl;
        }

    }
    robot.close();


    // Terminaison: Add the artificial last node 
    // This We don't know in advance which of the solution of the last ee pose to choose.
    // So we set the last ee pose to be the real last ee pose.
    //double qff[] = {J0_END, J1_END, J2_END, J3_END, J4_END, J5_END}; //raw data 
    q_t qf;
    for(int i=0;i<6;i++){
        qf.push_back(0.0);
    }
    ikSolutions.push_back(qf);
    qIdx=0;
    mapper[mapIndex(eeIdx, qIdx)]=solIdx;
    solCard.push_back(1);
    solIdx++; 
    eeIdx++;
    
    // eeIdx:
    // Correct number of slices with artificial init and final slice. 
    // So if you solve 10 ee, eeIdx=12
    // solIdx: Number of q solutions + artificial init and final node. 
    // So if you have 34 joint configuration, solIdx=36. The artificial final joint index is 35.

    // Initialization
    Graph graph(solIdx); 
    int eeIdxMax=eeIdx; //TODO: Check if is correct or need -1.
    double weight;

    // Build graph 
    int eeIdx1=0;
    int eeIdx2,qIdx1, qIdx2, qIdxMax1, qIdxMax2, nodeIdx1, nodeIdx2;
    q_t q1,q2;
    // For each ee pose indexed by eeIdx
    //  Take the following ee pose
    //    Build an edge between all possible joint configuration couples (q1,q2) 
    //    where q1 is a joint solution for the ee pose indexed by eeIdx, 
    //    and q2 a joint solution for the ee pose indexed by eeIdx+1
    // The vertices are not the joint configurations but their index in ikSolutions.
    // The weight is the manhattan norm between the joint vectors.
    for(eeIdx1=0;eeIdx1<(eeIdxMax-1);eeIdx1++){
        eeIdx2=eeIdx1+1;
        qIdxMax1=solCard[eeIdx1];
        qIdxMax2=solCard[eeIdx2];
        for(qIdx1=0;qIdx1<qIdxMax1;qIdx1++){ 
            for(qIdx2=0;qIdx2<qIdxMax2;qIdx2++){
                nodeIdx1=mapper[mapIndex(eeIdx1, qIdx1)];
                nodeIdx2=mapper[mapIndex(eeIdx2, qIdx2)];
                q1=ikSolutions[nodeIdx1];
                q2=ikSolutions[nodeIdx2];
                if((eeIdx1==0)||(eeIdx2==(eeIdxMax-1))){
                    weight = 0.0;
                    //std::cout << " FIrst node" << std::endl;
                }
                else{
                    weight = qNorm(q1,q2);
                }
                graph.addEdge(nodeIdx1, nodeIdx2, weight);
            }
        }
    }
    //std::cout << "Graph construction OK" << std::endl;
    
    Vertex parentTmp[graph.V]; // parentTmp[i]=j when j is the parent of i in the graph
    Vertex src=0;
    Vertex dst=solIdx-1;
    graph.shortestPath(src, dst, parentTmp);
    iter=0;

    //std::cout << "Graph search OK" << std::endl;

    // Save result to file
    Vertex trajectory = dst;
    std::stack<Vertex> parent; // path from first to last vertex

    // Do not push the first and last joint because they are far
    trajectory = parentTmp[trajectory];
    while( (trajectory!=src) && (iter<(graph.V+1))){
        //std::cout << trajectory << std::endl;
        parent.push(trajectory);
        trajectory = parentTmp[trajectory];
    }
    //std::cout << "Path record OK" << std::endl;
    
    // Save result to file
    q_ikfast.open(TEST_DATA_PATH_Q, std::ios::app);
    t_ikfast.open(TEST_DATA_PATH_T, std::ios::app);
    while(!parent.empty()){
        q = ikSolutions[parent.top()];
        parent.pop();
        q_ikfast <<q[0]<<"  "<<q[1]<<"  "<<q[2]<<"  "<<q[3]<<"  "<<q[4]<<"  "<<q[5]<<"  "<<"\n";
        for(c=0;c<num_of_joints;c++){
            jointsComputed[c]=q[c];
        }
        ComputeFk(jointsComputed, eetrans, eerot);
        t_ikfast << 
            eerot[0] << "  " << eerot[1] << "  " << eerot[2] << "  " << eetrans[0] << "  " <<
            eerot[3] << "  " << eerot[4] << "  " << eerot[5] << "  " << eetrans[1] << "  "  << 
            eerot[6] << "  " << eerot[7] << "  " << eerot[8] << "  " << eetrans[2] << "\n";
        iter++;
    }
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





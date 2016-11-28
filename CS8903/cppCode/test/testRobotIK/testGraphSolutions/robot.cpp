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
    robot.open(JOINT_CONSTRAINTS_Q_PATH);
    unsigned int l,c;
    std::size_t pos1, pos2;
    q_t q(6,0);
    std::ofstream q_ikfast, t_ikfast;
    int iter = 0;
    bool endOfDocument = false;
    unsigned int num_of_joints = 6;
    IKREAL_TYPE jointsComputed[num_of_joints];
    IKREAL_TYPE eerot[9], eetrans[3];

    std::vector<q_t> ikSolutions;
    int solIdx, eeIdx, qIdx;
    solIdx = 0;
    eeIdx = 0;
    qIdx = 0;
    std::vector<int> solCard;
    std::map<mapIndex, int> mapper;

    // TODO change Artificial starting node 
    double q0f[] = {J0_START, J1_START, J2_START, J3_START, J4_START, J5_START};
    q_t q0;
    for(int i=0;i<6;i++){
        q0.push_back(q0f[i]);
    }
    ikSolutions.push_back(q0);
    mapper[mapIndex(eeIdx, qIdx)]=solIdx;
    solCard.push_back(1);
    solIdx++;
    eeIdx++;
    qIdx=0;

    std::string line;
    std::getline(robot, line);
    endOfDocument = robot.eof();
    if(endOfDocument){
        exit(-1);
    }

    while (!endOfDocument /*&& (iter<ITER_MAX)*/) {
        qIdx=0;
        while(line!="\0"){ 
            // Parsing
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
            
            // Intermediary storage used for graph construction (cf schema)
            ikSolutions.push_back(q);
            mapper[mapIndex(eeIdx,qIdx)] = solIdx;
            qIdx++;
            solIdx++;
            std::getline(robot, line);
            endOfDocument = robot.eof();
            if(endOfDocument){
                break;
            }
        } // at this point, we have all the q solution for one ee in allQ 
        
        std::getline(robot, line);
        endOfDocument = robot.eof();
        if(endOfDocument){
            break;
        }

        solCard.push_back(qIdx);
        eeIdx++;
        iter ++;
    }
    robot.close();
    // At this point eeIdex = ITER_MAX

    // Build graph
    // Terminaison: Add the last node
    double qff[] = {J0_END, J1_END, J2_END, J3_END, J4_END, J5_END}; //raw data 
    q_t qf;
    for(int i=0;i<6;i++){
        qf.push_back(qff[i]);
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
    // So if you have 34 joint configuration, solIdx=36. The artificial final joint is 35

    // Initialization
    Graph graph(solIdx); // +1 for the source node already in solIdx
    int eeIdxMax=eeIdx; //TODO: Check if is correct or need -1.
    float weight;

    // Recursion
    int eeIdx1=0;
    int eeIdx2,qIdx1, qIdx2, qIdxMax1, qIdxMax2, nodeIdx1, nodeIdx2;
    q_t q1,q2;
    for(eeIdx1=0;eeIdx1<(eeIdxMax-1);eeIdx1++){
        eeIdx2=eeIdx1+1;
        qIdxMax1=solCard[eeIdx1];
        qIdxMax2=solCard[eeIdx2];
        for(qIdx1=0;qIdx1<qIdxMax1;qIdx1++){ // TODO qIdx1<(qIdxMax1-1)
            for(qIdx2=0;qIdx2<qIdxMax2;qIdx2++){
                nodeIdx1=mapper[mapIndex(eeIdx1, qIdx1)];
                nodeIdx2=mapper[mapIndex(eeIdx2, qIdx2)];
                q1=ikSolutions[nodeIdx1];
                q2=ikSolutions[nodeIdx2];
                weight = qNorm(q1,q2);
                graph.addEdge(nodeIdx1, nodeIdx2, weight);
            }
        }
    }

    Vertex parentTmp[graph.V]; // graph path
    Vertex src=0;
    Vertex dst=solIdx-1;
    graph.shortestPath(src, dst, parentTmp);
    iter=0;

    // Save result to file
    Vertex trajectory = dst;
    std::stack<Vertex> parent;
    
    // Do not push the first and last joint because they are far
    trajectory = parentTmp[trajectory];
    while( (trajectory!=src) && (iter<(graph.V+1))){
        parent.push(trajectory);
        trajectory = parentTmp[trajectory];
    }
    //parent.push(trajectory); //to push src 

    while(!parent.empty()){
        q = ikSolutions[parent.top()];
        parent.pop();
        q_ikfast.open(TEST_DATA_PATH_Q, std::ios::app);
        for(c=0;c<num_of_joints-1;c++){
            jointsComputed[c] = q[c];
            q_ikfast << q[c] << "  ";
        }
        // q_ikfast
        q_ikfast << q[num_of_joints-1] << "\n";
        jointsComputed[num_of_joints-1]=q[num_of_joints-1];
        q_ikfast.close();

        // T_computed
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
        iter++;
    }

    return 0;
}



float SIGN(float x) {
    return (x >= 0.0f) ? +1.0f : -1.0f;
}

float NORM(float a, float b, float c, float d) {
    return sqrt(a * a + b * b + c * c + d * d);
}



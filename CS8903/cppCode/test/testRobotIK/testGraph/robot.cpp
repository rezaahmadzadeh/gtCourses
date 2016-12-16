/**
 * \file testGraph/robot.cpp
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
	
    IKREAL_TYPE eerot[9], eetrans[3];
    unsigned int c;
    std::size_t pos1, pos2;
    q_t q(6,0);
    int iter = 0;
    bool endOfDocument = false;
    unsigned int num_of_joints = 6;
    IKREAL_TYPE jointsComputed[num_of_joints];
    std::ifstream robot;
    std::vector<double> tokens(12,0.0);
    std::ofstream q_ikfast, t_ikfast;
    robot.open(JOINT_CONSTRAINTS_Q_PATH);
    if(!robot.is_open()){
        std::cout << "NO such file: " << JOINT_CONSTRAINTS_Q_PATH << ". Exit." << std::endl;
        exit(-1);
    }

    std::vector<q_t> ikSolutions; //stores all joint solutions one after one, indexed by solIdx
    std::vector<int> solCard; //number of joint solution for each ee pose, indexed by eeIdx
    std::map<mapIndex, int> mapper; // maps a ee pose index and a joint solutions index to solIdx
    int solIdx, eeIdx, qIdx;
    solIdx = 0; //index joint solution in ikSolutions
    eeIdx = 0; //index the ee pose
    qIdx = 0; //index of the joint solution for a specific ee pose

    // Artificial initial node from which you reach the several possible solutions 
    // This joint configuration is specific to the trajecotry 
    // Artificial initial node from which you reach the several possible solutions 
    // This joint configuration is specific to the trajecotry
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
        // for each ee pose, store the joint solution in ikSolutions and update indices in mapper
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

            // Intermediary storage used for graph construction 
            ikSolutions.push_back(q);
            mapper[mapIndex(eeIdx,qIdx)] = solIdx;
            qIdx++;
            solIdx++;
            std::getline(robot, line); //get next joint solution for the same ee pose
            endOfDocument = robot.eof();
            if(endOfDocument){
                break;
            }
        } 

        std::getline(robot, line); // get first joint solution for new ee pose
        endOfDocument = robot.eof();
        if(endOfDocument){
            break;
        }

        solCard.push_back(qIdx); // store number of solution for current ee pose
        eeIdx++; // index for new ee pose
        iter ++;
    }
    robot.close();
    // At this point eeIdex = ITER_MAX

    // Terminaison: Add the artificial last node 
    // This We don't know in advance which of the solution of the last ee pose to choose.
    // So we set the last ee pose to be the real last ee pose.
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
    // So if you have 34 joint configuration, solIdx=36. The artificial final joint index is 35.

    // Initialization
    Graph graph(solIdx); 
    int eeIdxMax=eeIdx; //TODO: Check if is correct or need -1.
    float weight;

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
                    weight=0.0;
                }
                else{
                    weight = qNorm(q1,q2);
                }
                graph.addEdge(nodeIdx1, nodeIdx2, weight);
            }
        }
    }

    Vertex parentTmp[graph.V]; // parentTmp[i]=j when j is the parent of i in the graph
    Vertex src=0;
    Vertex dst=solIdx-1;
    graph.shortestPath(src, dst, parentTmp);
    iter=0;

    // Save result to file
    Vertex trajectory = dst;
    std::stack<Vertex> parent; // path from first to last vertex

    // Do not push the first and last joint because they are far
    trajectory = parentTmp[trajectory];
    while( (trajectory!=src) && (iter<(graph.V+1))){
        //std::cout << trajectory << std::endl;
        parent.push(trajectory);
        trajectory = parentTmp[trajectory];
        iter++;
    }

    // Store results to file
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



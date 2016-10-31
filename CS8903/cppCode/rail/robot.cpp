
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <fstream>
#include <cstring>
#include <string>
#include <vector>
#include <time.h>

#define IKFAST_HAS_LIBRARY // Build IKFast with API functions
#define IKFAST_NO_MAIN // Don't include main() from IKFast

#define DIR_TEST "./test_4_3/"
#define Q_PATH DIR_TEST "qpath.csv"
#define T_PATH DIR_TEST "tpath.csv"
#define Q_IKFAST DIR_TEST "q_ikfast.csv"
#define T_IKFAST DIR_TEST "t_ikfast.csv"

#define ITER_MAX 10

#define IK_VERSION 56
#include "output_ikfast7.cpp"

float SIGN(float x);
float NORM(float a, float b, float c, float d);

#define IKREAL_TYPE IkReal // for IKFast 56,61



const int MAX_CHARS_PER_LINE = 256;
const int MAX_TOKENS_PER_LINE = 15;
const char* const DELIMITER = ",";

int main(int argc, char** argv) {

    std::string robotFilename = "dataset4_3.csv";
    std::ifstream robot;
    robot.open(robotFilename.c_str());
    unsigned int l,c;
    std::size_t pos1, pos2;
    IKREAL_TYPE eerot[9], eetrans[3];
    unsigned int num_of_joints = 6;
    unsigned int num_free_parameters = 0;
    IKREAL_TYPE jointsComputed[num_of_joints]; 
    std::ofstream q_ikfast, t_ikfast;
    int iter = 0;
    
    while ((!robot.eof()) /*&& (iter<ITER_MAX)*/) {
        char line_c[256];
        robot.getline(line_c, 256); 
        std::string line(line_c);
        std::cout << line << std::endl;

        // parsing
        std::vector<std::string> tokens;
        pos1 = 0;
        pos1 = line.find(DELIMITER,pos1+1);
        while(pos1 != std::string::npos) {
            pos2 = line.find(DELIMITER, pos1+1);
            std::string t = line.substr(pos1+1, pos2-pos1-1);
            //std::cout << "t=" << t << std::endl;
            pos1 = line.find(DELIMITER,pos1+1);
            tokens.push_back(t);
        }

        // Iikfast computation
        IkSolutionList<IKREAL_TYPE> solutions;
        std::vector<IKREAL_TYPE> vfree(num_free_parameters);

        eetrans[0] = atof(tokens[6].c_str());
        eetrans[1] = atof(tokens[7].c_str());
        eetrans[2] = atof(tokens[8].c_str());
        double theta1=atof(tokens[9].c_str()); 
        double theta2=atof(tokens[10].c_str()); 
        double theta3=atof(tokens[11].c_str());
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

        for(std::size_t i = 0; i < vfree.size(); ++i){ 
            vfree[i] = atof(argv[13+i]);
        }
        
        bool bSuccess = ComputeIk(eetrans, eerot, vfree.size() > 0 ? &vfree[0] : NULL, solutions);
        if( !bSuccess ) {
            fprintf(stderr,"Failed to get ik solution\n");
            return -1;
        }

        unsigned int num_of_solutions = (int)solutions.GetNumSolutions();
        //printf("Found %d ik solutions:\n", num_of_solutions ); 

        std::vector<IKREAL_TYPE> solvalues(num_of_joints);
        for(std::size_t i = 0; i < num_of_solutions; ++i) {
            const IkSolutionBase<IKREAL_TYPE>& sol = solutions.GetSolution(i);
            int this_sol_free_params = (int)sol.GetFree().size(); 
            //printf("sol%d (free=%d): ", (int)i, this_sol_free_params );
            std::vector<IKREAL_TYPE> vsolfree(this_sol_free_params);

            sol.GetSolution(&solvalues[0],vsolfree.size()>0?&vsolfree[0]:NULL);
            for( std::size_t j = 0; j < solvalues.size(); ++j){
                jointsComputed[j] = solvalues[j]; 
                //printf("%.15f, ", solvalues[j]);
            }
            //printf("\n");
            
            // q_ikfast
            q_ikfast.open(Q_IKFAST, std::ios::app);
            for(c=0;c<num_of_joints-1;c++){
                q_ikfast << jointsComputed[c] << ',';
            }  
            q_ikfast << jointsComputed[num_of_joints-1] << "\n";
            q_ikfast.close();

            // T_computed
            ComputeFk(jointsComputed, eetrans, eerot);
            t_ikfast.open(T_IKFAST, std::ios::app);
            for(l=0;l<3;l++){
                for(c=0;c<3;c++){
                    t_ikfast << eerot[3*l+c] << ',';
                }
                if(l<2){
                    t_ikfast << eetrans[l] << ',';
                }
            }
            t_ikfast << eetrans[2] << "\n";
            t_ikfast.close();
        }
        iter ++;
        //std::cout << "Iteration " << iter << "OK" << std::endl;
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




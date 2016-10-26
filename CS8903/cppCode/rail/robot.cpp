
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <fstream>
#include <cstring>
#include <string>
#include <vector>
#include <time.h>

#define IKFAST_HAS_LIBRARY
#define IKFAST_NO_MAIN

#define IK_VERSION 56
#include "output_ikfast7.cpp"

float SIGN(float x);
float NORM(float a, float b, float c, float d);


const int MAX_CHARS_PER_LINE = 256;
const int MAX_TOKENS_PER_LINE = 15;
const char* const DELIMITER = ",";

int main(void) {

    std::string robotFilename = "dataset4_3.csv";
    std::ifstream robot;
    robot.open(robotFilename.c_str());
    int i=0;
    std::size_t pos1, pos2;
    
    while (!robot.eof()) {
        char line_c[256];
        robot.getline(line_c, 256);
        std::string line(line_c);
        std::cout << "get line OK" << std::endl;
        std::cout << line << std::endl;

        std::vector<std::string> tokens;

        pos1 = 0;
        pos1 = line.find(DELIMITER,pos1+1);
        while(pos1 != std::string::npos) {
            //pos1 = line.find(DELIMITER,pos1+1);
            pos2 = line.find(DELIMITER, pos1+1);
            std::string t = line.substr(pos1+1, pos2-pos1-1);
            std::cout << "t=" << t << std::endl;
            pos1 = line.find(DELIMITER,pos1+1);

            tokens.push_back(t);
        }

        // Cast string to double
         eePose[6] = {};
        for(i=0;i<6;i++){
            eePose[i] = atof(token[i]);
            std::cout << eePose[i] << std::endl;
        }

        robot.close();

        // call ik --> q_ikfast and store it
        
        // call fk on q_ikfast --> T_ikfast and store it


    }


    return 0;
}

/*
 *
 */
#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <string>
#include "engine.h"

#define PROCESSING_DIR "/home/abenbihi/gtCourses/CS8903/matlabCode/parseTrajectory"
#define COMMAND_DIR "cd " PROCESSING_DIR
#define COMMAND_FUNC "main"


int main() {

    Engine *ep;

    // Call engOpen with a NULL string. This starts a MATLAB process 
    // on the current host using the command "matlab".	
    if (!(ep = engOpen(""))) {
        fprintf(stderr, "\nCan't start MATLAB engine\n");
        return EXIT_FAILURE;
    }

    //Evaluate a function
    engEvalString(ep, COMMAND_DIR);	
    engEvalString(ep, COMMAND_FUNC);

    engClose(ep);

    return EXIT_SUCCESS;
}

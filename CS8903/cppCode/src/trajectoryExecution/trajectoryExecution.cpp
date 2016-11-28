/* -------------------------------------
 *  A Simple Trajectory Executer
 *
 *
 *
 * set singlePointTest for sending a single point to the module
 * set incrementalTest for sending points in an incremental loop
 * input the file including trajectory information recorded using the recorder module in the repo
 *
 *
 * <<< SINCE THERE IS NO OBSTACLE AVOIDANCE MONITORING THE ROBOT USING THIS MODULE CAN BE DANGEREOUS >>>
 *
 *
 *
 * Reza Ahmadzadeh (IRIM, April-2016)
 *
 * -------------------------------------*/

#include <iostream>
#include <dlfcn.h>
#include <KinovaTypes.h>
#include <Kinova.API.CommLayerUbuntu.h>
#include <stdio.h>
#include <fstream>
#include <termios.h>
#include <unistd.h>
#include <vector>
#include <fcntl.h>
#include <string>
#include <sstream>
#include "math.h"

#define R2D 180/M_PI

#define NUMSET "2_5"
#define TRAJECTORY_DIR "/home/abenbihi/gtCourses/CS8903/cppCode/test/testRobotIK/"
#define TRAJECTORY_TYPE "testGraphSolutions/"
#define TRAJECTORY_FILENAME TRAJECTORY_DIR TRAJECTORY_TYPE "data/q_ikfast" NUMSET ".txt"

#define ITER_MAX 10

using namespace std;
int main()
{
    int result;
    bool singlePointTest, incrementalTest, trajectoryTest, homeTorqueTest, homePoseTest;
    CartesianPosition data;
    AngularPosition angles;
    cout << "Executing trajectories using SendBasicTrajectory" << endl;
    //Handle for the library's command layer.
    void * commandLayer_handle;
    //Function pointers to the functions we need
    int (*MyInitAPI)();
    int (*MyCloseAPI)();
    int (*MySendBasicTrajectory)(TrajectoryPoint command);
    int (*MyStartControlAPI)();
    int (*MyMoveHome)();
    int (*MyGetCartesianCommand)(CartesianPosition &);
    int (*MyGetCartesianPosition)(CartesianPosition &);
    int (*MyGetAngularPosition)(AngularPosition &);

    //We load the library (Under Windows, use the function LoadLibrary)
    commandLayer_handle = dlopen("Kinova.API.USBCommandLayerUbuntu.so",RTLD_NOW|RTLD_GLOBAL);
    //We load the functions from the library (Under Windows, use GetProcAddress)
    MyInitAPI = (int (*)()) dlsym(commandLayer_handle,"InitAPI");
    MyCloseAPI = (int (*)()) dlsym(commandLayer_handle,"CloseAPI");
    MySendBasicTrajectory = (int (*)(TrajectoryPoint)) dlsym(commandLayer_handle,"SendBasicTrajectory");
    MyStartControlAPI = (int (*)()) dlsym(commandLayer_handle,"StartControlAPI");
    MyGetCartesianCommand = (int (*)(CartesianPosition &)) dlsym(commandLayer_handle,"GetCartesianCommand");
    MyGetCartesianPosition = (int (*)(CartesianPosition &)) dlsym(commandLayer_handle,"GetCartesianPosition");
    MyGetAngularPosition = (int(*)(AngularPosition &)) dlsym(commandLayer_handle,"GetAngularPosition");
    MyMoveHome = (int (*)()) dlsym(commandLayer_handle,"MoveHome");

    //If the was loaded correctly
    if((MyInitAPI == NULL) || (MyCloseAPI == NULL) || (MySendBasicTrajectory == NULL) || (MyStartControlAPI == NULL) || 
            (MyMoveHome == NULL) || (MyGetCartesianCommand == NULL) || (MyGetCartesianPosition == NULL))
    {
        cout << "Unable to initialize the command layer." << endl;
    }
    else
    {
        result = (*MyInitAPI)();

        result = (*MyGetCartesianPosition)(data);
        cout << " Home: [ " << data.Coordinates.X << "," << data.Coordinates.Y << "," << data.Coordinates.Z << " ]" << endl;
        usleep(3000);
        TrajectoryPoint trajectoryPoint;

        //MyMoveHome(); //TODO Uncomment.
        usleep(10000);
        ifstream ifs;
        std::cout << TRAJECTORY_FILENAME << std::endl;
        ifs.open(TRAJECTORY_FILENAME);
        if (!ifs.is_open())
            cout << "no such file! try again." << endl;
        int count=0;
        float j1,j2,j3,j4,j5,j6,x,y,z,tx,ty,tz;
        int iter=0;

        while (ifs.is_open() &&  ifs >> j1 >> j2 >> j3 >> j4 >> j5 >> j6 && iter<ITER_MAX)
        {
            printf("%i\t%f\t%f\t%f\t%f\t%f\t%f\n", count,j1,j2,j3,j4,j5,j6);
            trajectoryPoint.InitStruct();
            trajectoryPoint.Position.Type = ANGULAR_POSITION;
            // get the current pose of the end-effector
            trajectoryPoint.Position.Actuators.Actuator1 = j1*R2D;
            trajectoryPoint.Position.Actuators.Actuator2 = j2*R2D;
            trajectoryPoint.Position.Actuators.Actuator3 = j3*R2D;
            trajectoryPoint.Position.Actuators.Actuator4 = j4*R2D;
            trajectoryPoint.Position.Actuators.Actuator5 = j5*R2D;
            trajectoryPoint.Position.Actuators.Actuator6 = j6*R2D;
            //(*MySendBasicTrajectory)(trajectoryPoint); //TODO Uncomment
            usleep(3000);
            count++;
            iter++;
            //ifs >> j1;
        }

        ifs.close();
        result = (*MyCloseAPI)();

    }
    return 0;
}

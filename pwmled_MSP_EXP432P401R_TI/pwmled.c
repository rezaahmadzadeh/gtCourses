
/*
 *  ======== pwmled.c ========
 */
#include <xdc/std.h> //XDCtools Header files
#include <xdc/runtime/Log.h> //loginfo
#include <xdc/runtime/System.h>
#include <ti/sysbios/BIOS.h> //BIOS Header files
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Swi.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/hal/Hwi.h>
#include <ti/drivers/GPIO.h> //TI-RTOS Header files
#include <ti/drivers/PWM.h>
#include <xdc/runtime/Timestamp.h>
#include <xdc/runtime/Types.h>

#include "Board.h" //Example/Board Header files
#include "math.h"
#include "time.h"

#define TASKSTACKSIZE   512
#define TASK2STACKSIZE           768
#define true 1
#define false 0
#define WHEELCIRC_MM 185 // in mm
#define WHEELBASE_MM 104     // wheel base in mm
#define ODOSAMPLEPD 50 //ms
#define TICKS_PER_SECOND 48000000
#define SENSOR_LOOPS_PER_SECOND 20
#define M_PI 3.14159
#define deg2rad M_PI/180
#define DATALOG_SIZE 300
#define CAST10 10000

#define Timestamp_get   Timestamp_get32
#define getTimestampFreq Timestamp_getFreq
#define USERDEBUG 1 //Set this to 1 for extra diagnostic print statements

int countSensorSincePathCall = 0;
int countSensorSinceControlCall = 0;
int numSensorBetweenPathCall = 1;  //20
int numSensorBetweenControlCall = 1; //5

// Function Prototypes
Void taskSensorFxn (UArg arg0, UArg arg1);
Void taskControlFxn(UArg arg0, UArg arg1);
Void taskPathFxn   (UArg arg0, UArg arg1);
void gpioButtonFxn0(unsigned int index);
void gpioButtonFxn1(unsigned int index);

// BIOS Objects
Task_Struct taskSensorStruct, taskControlStruct, taskPathStruct, task2Struct;
Char taskSensorStack[TASKSTACKSIZE], taskControlStack[TASKSTACKSIZE], taskPathStack[TASKSTACKSIZE], task2Stack[TASK2STACKSIZE];
Task_Handle taskSensor, taskControl, taskPath, benchmarkHandle;
Semaphore_Struct semSensorStruct, semControlStruct, semPathStruct;
Semaphore_Handle semaSensor, semaControl, semaPath;

// Encoders count
uint32_t leftCount=0;
uint32_t rightCount=0;


// Message passing mock
float Xpos, Ypos, DistT; //vehicle position
float DegPos, RadPos; //vehicle heading
float vehicleVelocity;
float wheelVelocityL, wheelVelocityR;
float vel_des;
float turnRate_des;

/*
int XPosLog[DATALOG_SIZE];
int YPosLog[DATALOG_SIZE];
int DegPosLog[DATALOG_SIZE];

int vel_desLog[DATALOG_SIZE];
int eLLog[DATALOG_SIZE];
int eRLog[DATALOG_SIZE];
int TimeLog[DATALOG_SIZE];


int highLevelI=0;
int mediumLevelI=0;
*/

// Benchmark variables

volatile Int32 t1;    /* temp var for holding first Timestamp */
volatile Int32 t2;    /* temp var for holding second Timestamp */
Int32 delta;          /* var for output of t2-t1-offset in TIME() */
UInt32 minTimestamp;  // used to keep min Timestamp cycle count
UInt32 avgTimestamp;  // used to keep avg Timestamp cycle count
Float factor;         // used for clock ratio cpu/timestamp
UInt32 min;           // minimum cycle count for LOOP (output)
UInt32 max;           // maximum cycle count for LOOP (output)
UInt32 avg;           // average cycle count for LOOP (output)

// Macro used to LOOP functions and total averages. Assumes delta is externally updated by FXN such as TIME()
#define NUM_LOOPS 10
#define LOOP(FXN) {              \
    min = ~(0);                  \
    max = 0;                     \
    total = 0;                   \
    for (i = 0; i < NUM_LOOPS; i++) {\
        FXN;                     \
        total += delta;          \
        if(delta < min)          \
            min = delta;         \
        if (delta > max)         \
            max = delta;         \
    }                            \
    avg = total / NUM_LOOPS;     \
}

//to TIME function executions Input: FXN, offset; Output: delta (ex time)
#define TIME(FXN) {            \
    t1 = Timestamp_get();      \
    FXN;                       \
    t2 = Timestamp_get();      \
    delta = (t2);              \
    if (t1 > delta) {          \
        delta = 0;             \
    }                          \
    else {                     \
        delta = delta - (t1);  \
    }                          \
}




/*
 *  ======== printResults ========
 */
Void printResults(String name)
{
    /* Verify values will not underflow */
    if (min > minTimestamp) {
        min -= minTimestamp;
    }
    if (avg > avgTimestamp) {
        avg -= avgTimestamp;
    }
    if (max > avgTimestamp) {
        max -= avgTimestamp;
    }

    /* No computation can take 0 cycles */
    if (min == 0) {
        min = 1;
    }
    if (avg == 0) {
        avg = 1;
    }
    if (max == 0) {
        max = 1;
    }

    System_printf("%lu  %lu  %lu   %s\n",
        (UInt32)(min * factor),
        (UInt32)(max * factor),
        (UInt32)(avg * factor),
    name);
}


Void benchmarkTask(UArg a0, UArg a1){

	UInt32 total;         /* temporary variable used by LOOP() */
	UInt32 i;             /* temporary variable used for LOOP ctrl */
	Types_FreqHz freq1;   /* used to keep Timestamp frequency */
	Types_FreqHz freq2;   /* used to keep BIOS frequency */


	 //This will calculate the factor needed top correlate Timestamp delta

	getTimestampFreq(&freq1);
	BIOS_getCpuFreq(&freq2);
	factor = (Float)freq2.lo / freq1.lo;
	if(USERDEBUG) {
		System_printf("%lu\t%lu\t%lu\t Timestamp Freq, BIOS Freq, Factor\n",
			freq1.lo, freq2.lo, (UInt32) factor);
	}

	LOOP(TIME(;));
	minTimestamp = min;
	avgTimestamp = avg;
	if(USERDEBUG) {
		System_printf("%lu\t%lu\t Timestamps\n",
			(UInt32)(minTimestamp * factor),
			(UInt32)(avgTimestamp * factor));
	}

	LOOP(TIME(taskSensorFxn(a0,a1)));
    printResults("taskSensorFxn()");

    LOOP(TIME(taskControlFxn(a0,a1)));
    printResults("taskControlFxn()");


    LOOP(TIME(taskPathFxn(a0,a1)));
    printResults("taskPathFxn()");

    System_printf("Benchloop Complete\n");

    BIOS_exit(1);

}


int main(void)
{
	// Construct BIOS objects
	Task_Params taskParams;

	 // Call board init functions.
	Board_initGeneral();
	Board_initGPIO();
	Board_initPWM();

	Task_Params_init(&taskParams);
	taskParams.stackSize = TASKSTACKSIZE;

	taskParams.stack = &taskSensorStack;
	taskParams.arg0 = 50;
	//taskParams.arg0 = TICKS_PER_SECOND/SENSOR_LOOPS_PER_SECOND;
	taskParams.priority = 3;
	Task_construct(&taskSensorStruct, (Task_FuncPtr)taskSensorFxn, &taskParams, NULL);

	taskParams.stack = &taskControlStack;
	//taskParams.arg0 = TICKS_PER_SECOND/SENSOR_LOOPS_PER_SECOND;
	taskParams.arg0 = 50;
	taskParams.priority = 2;
	Task_construct(&taskControlStruct, (Task_FuncPtr)taskControlFxn, &taskParams, NULL);

	taskParams.stack = &taskPathStack;
	//taskParams.arg0 = TICKS_PER_SECOND/SENSOR_LOOPS_PER_SECOND;
	taskParams.priority = 1;
	taskParams.arg0 = 50;
	Task_construct(&taskPathStruct, (Task_FuncPtr)taskPathFxn, &taskParams, NULL);


	// Benchmark
/*
	taskParams.stackSize = TASK2STACKSIZE;
	taskParams.stack = &task2Stack;
	Task_construct(&task2Struct, (Task_FuncPtr)benchmarkTask, &taskParams, NULL);
*/

	/* Obtain instance handle */
	taskSensor  = Task_handle(&taskSensorStruct );
	taskControl = Task_handle(&taskControlStruct);
	taskPath    = Task_handle(&taskPathStruct   );
	benchmarkHandle = Task_handle(&task2Struct);

	/* Construct a Semaphore object, inital count 0 */

	Semaphore_Params semParamsSensor, semParamsControl, semParamsPath;
	Semaphore_Params_init(&semParamsSensor);
	Semaphore_Params_init(&semParamsControl);
	Semaphore_Params_init(&semParamsPath);


	Semaphore_construct(&semSensorStruct,  1, &semParamsSensor);
	Semaphore_construct(&semControlStruct, 1, &semParamsControl);
	Semaphore_construct(&semPathStruct,    1, &semParamsPath);

	// Obtain instance handle
	semaSensor  = Semaphore_handle(&semSensorStruct);
	semaControl = Semaphore_handle(&semControlStruct);
	semaPath    = Semaphore_handle(&semPathStruct);

    // Encoder interrupts

    GPIO_setCallback(Board_BUTTON0, gpioButtonFxn0);//install Button callback
    GPIO_enableInt(Board_BUTTON0); //Enable interrupts
    GPIO_setCallback(Board_BUTTON1, gpioButtonFxn1);
    GPIO_enableInt(Board_BUTTON1);

    /*
    int i=0;
    for(i=0;i<DATALOG_SIZE;i++){
    	XPosLog[i] = 0;
    	YPosLog[i] = 0;
    	DegPosLog[i] = 0;
	}
     */

    BIOS_start(); //Start BIOS
    return (0);
}


//Callback function for the GPIO interrupt on Board_BUTTON0.
void gpioButtonFxn0(unsigned int index) {leftCount++;}

// Callback function for the GPIO interrupt on Board_BUTTON1.
void gpioButtonFxn1(unsigned int index) {rightCount++;}


void taskSensorFxn(UArg arg0, UArg arg1) {
	// Data to pass.
	// Next line commented out by SCK. Initialization changed to 0.0 from 0 by SCK.
	Xpos = Ypos = RadPos = DegPos = 0.0;
	// Intermediary data.
	float DistL, DistR, DistC, DistT;
	float countL, countR;
	countL = countR = 0.0;
	DistL = DistR = DistC = DistT = 0.0;

	while(1){
		Semaphore_pend(semaSensor, BIOS_WAIT_FOREVER); // not TODO

		if ( countSensorSincePathCall == 0 ) {
			Semaphore_pend(semaPath,    BIOS_WAIT_FOREVER);
		}
		if ( countSensorSinceControlCall == 0 ) {
			Semaphore_pend(semaControl, BIOS_WAIT_FOREVER);
		}

		countL = (float) leftCount; //leftCount, rightCount are the count by the encoder
		countR = (float) rightCount;
		//Log_info1("leftCount [%u]", leftCount);
		//Log_info1("rightCount [%u]", rightCount);
		//Log_info1("countL [%u]", countL);
		//Log_info1("countR [%u]", countR);
		leftCount=0;
		rightCount=0;

		// Computations
		wheelVelocityL = ((float) countL)/((float) ODOSAMPLEPD); // degree/ ms
		wheelVelocityR = ((float) countR)/((float) ODOSAMPLEPD); // degree/ ms
		DistL = countL* (float) WHEELCIRC_MM / 3600.0;  // DistL cm
		DistR = countR* (float) WHEELCIRC_MM / 3600.0;  // DistR cm
		DistC = (DistL+DistR)/2;            // DistC cm
		DistT += DistC; // DistT cm
		RadPos += (DistL - DistR)/(WHEELBASE_MM/10); // // Rad xWHEELBASE
		/*
		if(RadPos>(2*3.14159+0.01)){
			RadPos -= 2*3.14159;
		}
		else if(RadPos<(-0.01)){
			RadPos += 2*3.14159;
		}
		*/

		if(RadPos>3.14159){
			RadPos = RadPos - 2*3.14159 ;
		}
		else if(RadPos< (-3.14159)){
			RadPos = 2*3.14159+RadPos;
		}
		DegPos = RadPos *180/3.14159;   // Deg, homogeneity checked.
		//if(DegPos<0) {DegPos+=360;}
		vehicleVelocity = DistC/ODOSAMPLEPD;						// Velocity cm/ms
		Xpos += DistC * (float) cos(DegPos*deg2rad);               // Xpos cm
		Ypos += DistC * (float) sin(DegPos*deg2rad);               // Ypos cm

		countSensorSincePathCall++;
		countSensorSinceControlCall++;
		/* Get access to resource */

		Semaphore_post(semaSensor);

		// If the sensor thread has been run enough times since the
		// path (or control) threads have been called, release the hold.

		if ( countSensorSincePathCall == numSensorBetweenPathCall ) {
			Semaphore_post(semaPath);
		}
		if ( countSensorSinceControlCall == numSensorBetweenControlCall ) {
			Semaphore_post(semaControl);
		}

		System_printf("Sensor suite OK\n");
		System_flush();

		Task_sleep((UInt) arg0);

	 }
}


//Task periodically increments the PWM duty for the on board LED.
Void taskControlFxn(UArg arg0, UArg arg1){

	PWM_Handle pwm1;
	PWM_Handle pwm2 = NULL;
	PWM_Params params;
	uint16_t   pwmPeriod = 1000;

	PWM_Params_init(&params);
	params.dutyUnits = PWM_DUTY_US;
	params.dutyValue = 1;
	params.periodUnits = PWM_PERIOD_US;
	params.periodValue = pwmPeriod;
	pwm1 = PWM_open(Board_PWM0, &params);
	if (pwm1 == NULL) {
		System_abort("Board_PWM0 did not open");
	}
	PWM_start(pwm1);
	pwm2 = PWM_open(Board_PWM1, &params);
	if (pwm2 == NULL) {
		System_abort("Board_PWM1 did not open");
	}
	PWM_start(pwm2);

	//Differential Controller Implementation Begins
	float leftMeasVel = 0.0;					//Measured Velocities Input from Sensor Suite
	float  rightMeasVel = 0.0;					//Input from Sensor Suite
	float desiredVel = 0.0;						//Input from Trajectory Planner
	float  biasTurnRate = 0.0;					//Input from Trajectory Planner

	volatile float Kp = 0.2; //0.5;	//.125				//Kp for both L&R controllers
	volatile float Ki = 0.02;	//.5				//One Ki for both L&R sides
	volatile float sumTotal = 0;
	volatile float eL = 0;					//e1 in controller diagram
	volatile float eR = 0;					//e2 in controller diagram
	volatile float e3 = 0;

    while (1) {
    	Semaphore_pend(semaControl, BIOS_WAIT_FOREVER);
    	countSensorSinceControlCall = 0;

    	leftMeasVel=wheelVelocityL;
    	rightMeasVel=wheelVelocityR;
    	desiredVel=vel_des;
    	biasTurnRate=turnRate_des;

    	Semaphore_post(semaControl);

    	e3 = leftMeasVel + biasTurnRate - rightMeasVel;
    	sumTotal = sumTotal + e3;
    	eL = desiredVel - leftMeasVel - Ki*sumTotal;
    	eR = desiredVel - rightMeasVel + Ki*sumTotal;
    	eL = Kp*eL*pwmPeriod/9.0;						//Didn't want to use more variables.
    	eR = Kp*eR*pwmPeriod/9.0;						//The *pwmPeriod/9.0 term is to scale for max(dutyCycle) and 9V max output

    	if(eL > pwmPeriod){eL = pwmPeriod;} else if(eL < 0){eL = pwmPeriod*0.05;}
    	if(eR > pwmPeriod){eR = pwmPeriod;}	else if(eR < 0){eR = pwmPeriod*0.05;}

    	PWM_setDuty(pwm1, eL);
    	PWM_setDuty(pwm2, eR);

    	//System_printf("LL controller OK\n");
    	//System_flush();
/*
    	eLLog[mediumLevelI]=eL;
    	eRLog[mediumLevelI]=eR;
    	mediumLevelI++;
    	int i=0;
		if(mediumLevelI==DATALOG_SIZE){
			for(i=0;i<highLevelI;i++){
				System_printf("%d %d %d %d %d\n", i, XPosLog[i], YPosLog[i], DegPosLog[i], vel_desLog[i]);
				System_flush();
			}
			for(i=0;i<DATALOG_SIZE;i++){
				System_printf("%d %d %d\n", i, eLLog[i], eRLog[i]);
				System_flush();
			}
			BIOS_exit(1);
		}

    	System_printf("LL controller OK\n");
    	System_flush();
*/
        Task_sleep((UInt) arg0);
    }

}


/*
 *  ======== taskPathFxn ========
 */
Void taskPathFxn(UArg arg0, UArg arg1)
{
	// Used in taskPathFxn
	float x_goal = 100.0;    //cm
	float y_goal = 100.0;    //cm
	float theta_goal = 90;  //degree
	float k_rho   = 0.5; //1; //0.5;
	float k_alpha = 0.09;  //2.0;
	float k_phi   = 0.09;   //1.0;

	volatile float x_err = 0.0;
	volatile float y_err = 0.0;
	volatile float theta_err = 0.0;
	volatile float dist_to_goal = x_goal;
	volatile float alpha = theta_goal;

	// Input to function
	float x = 0.0;
	float y = 0.0;
	float theta = 0.0;

    //UInt32 time;
	while (1) {
		Semaphore_pend(semaPath, BIOS_WAIT_FOREVER);
		countSensorSincePathCall = 0;
		x = Xpos;
		y = Ypos;
		theta = DegPos;

	    // Calculate the error between the goal and current values.
	    x_err     = x_goal     - x;
	    y_err     = y_goal     - y;
	    theta_err = theta_goal - theta;

	    // TODO: keep theta_err in range -pi/2:pi/2

	    // Calculate the distance to the goal.
	    dist_to_goal = sqrt( (x_err*x_err) + (y_err*y_err) );

	    // Calculate the angle between the current position and
	    // desired position minus the current heading.
	    alpha = atan2(x_err,y_err)/deg2rad - theta;

	    // Calculate the desired vehicle velocity and
	    //               desired vehicle turning rate
	    vel_des = k_rho*dist_to_goal;
	    turnRate_des = k_alpha*alpha + k_phi*theta_err;

	   Semaphore_post(semaPath);
	   /*
	    XPosLog[highLevelI]= (int) floor(x*CAST10);
	    YPosLog[highLevelI]=(int) floor(y*CAST10);
	    DegPosLog[highLevelI]=(int) floor(theta*CAST10);
	    vel_desLog[highLevelI] = (int) floor(vel_des*CAST10);
	    highLevelI++;

	    System_printf("Path planner OK\n");
	    System_flush();
	    */
	    Task_sleep((UInt) arg0);
}
}







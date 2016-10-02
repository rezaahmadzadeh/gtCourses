
/*
 * Code for ECE 6561 Motor control project 1
 * Daniel Dichek and Assia B.
 * 
 */

// Sampling
unsigned long tstart = 0;
float telapsed = 0;
volatile float count = 0; //converted to long because int overflows too quickly
float thetad = 0;
float lastThetad = 0;

volatile bool INTFLAG = false;
volatile bool startSample = false;

// Pins
const int OUTPWM = 38;
const int OUTDIR = 34;
const int ENCODEPIN1 = 32;
const int ENCODEPIN2 = 11;
const int ENCODECOUNTS1 = 180; //there are 180 counts per encoder output
                               //there are two encoder outputs 90* out of phase
                               //so if both encoders are counted, 
// Square wave reference signal
const int rmax = 200;
const int rmin = 150;
const int rperiod = 1000;
int r = 150;
unsigned long rstart = 0;
unsigned long relapsed = 0;


//PID control variables
//CONSIDER MAKING THESE ADJUSTABLE VIA SERIAL.
//that would mean two states: run motor and stop/set gains/reset.

// Errors
const float scalefactor = 1436.3; //found this in matlab
int preverror = 0;
int currerror = 0; 
float ei = 0;

// PID coefficients
const float kp = 1; //proportional gain. how granular should the gains be? scaling necessary?
const float ki = 0.0; //integral gain. 7
const float kd = 0.0; //derivative gain 0.006
int ep = 0;
float ed = 0;
 


//PID control function
//takes in the current errors and computes the control output
float PID(int ep, int ei, int ed){
  float out = 0;
  out = kp*ep + ki*ei + kd*ed;
  out = abs(out);
  if(out > 255.00){
    out = 255.00;
  }
  
  return out;
}

//this function marks a time at which a sampling period starts.
//this allows the chip to wait a specified time before doing a control calculation.
bool sampleStart(){
  tstart = millis(); 
  return true;
}



void setup() {
  Serial.begin(9600);
  pinMode(OUTPWM, OUTPUT);
  pinMode(OUTDIR, OUTPUT);
  pinMode(ENCODEPIN1, INPUT);
  pinMode(ENCODEPIN2, INPUT);
  attachInterrupt(ENCODEPIN1, encoderHandler, RISING);
}

void loop() {
  
  if(startSample == false){ 
	startSample = sampleStart(); // records the starting time for timer
  }
  unsigned long currentTime = millis();
    
  // Check if reference signal period has expired.
  relapsed = currentTime - rstart;
  if(relapsed > rperiod){
    rstart = currentTime;
    
    if (r == rmax) {
      r = rmin;
    }
    else {
      r = rmax;
    }
    
  }
  
  // Check if timer has expired. If yes, do computations.
  telapsed = currentTime - tstart; 
  if(telapsed > 99.9){ 
    telapsed /= 1000.00;
    startSample = false; //reset the sample start time trigger
    float localCount = count; // Reset the count
    count = 0;
    lastThetad = thetad;
    thetad = localCount / telapsed; // Compute current velocity  
    preverror = currerror;
    currerror = r - thetad;

    //ed = (preverror - currerror)/telapsed; // What we had before
    // PID computations
    ed = (currerror - preverror)/telapsed;
    ei += currerror*telapsed;
    if(ei > 255){
      ei = 255;
    }
    float out = kp*ep + ki*ei + kd*ed; 
    
    //ed = (thetad - lastThetad)/telapsed;
  	// float out = kp*ep + ki*ei - kd*ed; example one formula but weird
  	
  	if(out > 255.00){
    	out = 255.00;
  	}
  	else if(out < 0.0) {
  		out = 0.0;
  	}
    

    //float out = PID(currerror, ei, ed);
    
    //for part 4, overwrite the PID command with JUST a proportional command 

    //Serial.print('Current Time: ');
    Serial.print(millis());
    Serial.print(' ');
    //Serial.print(localCount);
    //Serial.print(' ');
    Serial.print(r);
    Serial.print(' ');
    Serial.print(thetad);
    Serial.print(' ');
    Serial.print(currerror);
    Serial.print(' ');
    Serial.println(out);
    
    /*
    Serial.print(thetad);
    Serial.print(' ');
    Serial.println(r);
    */

	//digitalWrite(OUTDIR, HIGH); // counter clock wise when the encoder output is at the left
	//analogWrite(OUTPWM, 0);
  } 
}

//this might be unstable - it interrupts the program with EVERY rising edge
//use the interrupt to perform the control computation - ISR happens every 10ms
void encoderHandler() {
  count ++;
}


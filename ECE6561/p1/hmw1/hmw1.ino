
/*
 * Code for ECE 6561 Motor control project 1
 * Daniel Dichek and Assia B.
 * 
 */

// Timer
const float timerPeriod = 100.0; //ms
unsigned long tstart = 0;
float telapsed = 0;

// Sampling
volatile float count = 0; //converted to long because int overflows too quickly
volatile float thetad = 0;
const float scalefactor = 1436.3; //found this in matlab
volatile float e = 0;
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
                               
//reference signal
const int rmax = 200;
const int rmin = 80;
const int rperiod = 1000;
int r = 0;
unsigned long rstart = 0;
unsigned long relapsed = 0;

//PID control variables
//CONSIDER MAKING THESE ADJUSTABLE VIA SERIAL.
//that would mean two states: run motor and stop/set gains/reset.
const float kp = 0.5; //proportional gain. how granular should the gains be? scaling necessary?
const float ki = 6; //integral gain.
const float kd = 0.001; //derivative gain
int ep = 0;
float ei = 0;
float eiprev = 0;
float eiprevprev = 0;
float ed = 0;
int preverror = 0;
int currerror = 0;  
float proportionalError = 0;


//PID control function
//takes in the current errors and computes the control output
float PID(int ep, int ei, int ed){
  float out = 0;
  proportionalError = kp*ep;
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
  attachInterrupt(ENCODEPIN1, encoderHandler, RISING); //encoder interrupt
}


void loop() {
  
  //sample the program timer ONCE to mark the "start" of a sampling period
  if(startSample == false){ 
    startSample = sampleStart(); // sampleStart records a time to tstart.
  }

  //compare current time with timer start time: if it has been ~10 ms, do a control calculation and then wait another 10 ms.
  unsigned long currentTime = millis();
  telapsed = currentTime - tstart;

  // Periodic square wave refence signal
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

  // Timer interrupt
  if(telapsed > timerPeriod){ 
    // Sampling
    telapsed /= 1000.00;
    startSample = false; //reset the sample start time trigger
    float localCount = count;
    count = 0;
    thetad = localCount / telapsed; //current-previous counts div time = speed 

    // Control computation
    preverror = currerror;
    currerror = r - thetad;
    ed = (preverror - currerror)/telapsed;
    ei += currerror*telapsed;
    if(ei > 255){
      ei = 255;
    }
    float out = PID(currerror, ei, ed);
    
    //for part 4, overwrite the PID command with JUST a proportional command 

    Serial.print(millis());
    Serial.print(",");
    //Serial.print(telapsed);
    //Serial.print(",");
    //Serial.println(proportionalError);
    Serial.print(thetad);
    Serial.print(",");
    Serial.println(r);
    /*Serial.print(");
    Serial.print(currerror);
    Serial.print(" => v=");
    Serial.println(out);
    */
    
    digitalWrite(OUTDIR, HIGH); // counter clock wise when the encoder output is at the left
    analogWrite(OUTPWM, proportionalError);
  }
}


//this might be unstable - it interrupts the program with EVERY rising edge
//use the interrupt to perform the control computation - ISR happens every 10ms
void encoderHandler() {
  count ++;
}


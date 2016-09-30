

volatile int count = 0;

const int OUTPWM = 38;
const int OUTDIR = 34;
const int ENCODEPIN1 = 32;
const int ENCODEPIN2 = 11;
const int TIMER_PERIOD = 10;
const int r = 0;

// Timer variable
int currentTime = 0;
int previousTime = 0;
int timer = 0;

// Experiment variable
int r[5];
int currentReference = 0;

void setup() {
    Serial.begin(9600);// put your setup code here, to run once:

    // Setup pins
    pinMode(OUTPWM, OUTPUT);
    pinMode(OUTDIR, OUTPUT);
    pinMode(ENCODEPIN1, INPUT);
    
    // Setup experiment environment;
    r[0] = 0;
    r[1] = 0;
    r[2] = 0;
    r[3] = 0;
    r[4] = 0;

    r = r[0];

    // Encoder Interrupt
    attachInterrupt(ENCODEPIN1, encoderHandler, RISING);
}

void loop() {
    
    int currentTime = millis();
    timer = currentTime- previousTime;

    // We could use an interrupt but this already mock it
    if(TIMER_PERIOD < timer) {
        // Reset timer
        previousTime = currentTime;
        timer = 0; 

        // Calculate/Sample velocity and error
        uint8_t localCount = count;
        count = 0;
        uint8_t thetad = localCount / TIMER_PERIOD;
        uint8_t e = r - thetad;

        // TODO Compute v with the correct numerical value
        int v = a*e/(b+s);

        if(v>0){
            digitalWrite(OUTDIR, LOW); // counter clock wise when the encoder output is at the left
            analogWrite(OUTPWM, 150);
            Serial.print(currentTime);
            Serial.print(r);
            Serial.print(thetad);
        }
        else if (v<0) {
            digitalWrite(OUTDIR, HIGH);
            analogWrite(OUTPWM, 150);
            Serial.print(currentTime);
            Serial.print(r);
            Serial.print(thetad);
        }
    }
  
}

void encoderHandler() {
  count ++;
}



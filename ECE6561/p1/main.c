
#include "driverlib.h"
#include <time.h>
#include <stdlib.h>



// Ports and pins constants, TODO Choose port and pin number value
#define INPUT_PORT_1  (GPIO_PORT_P4)
#define INPUT_PIN_1  (GPIO_PIN1)
#define INPUT_PORT_2 (GPIO_PORT_P4)
#define INPUT_PIN_2 (GPIO_PIN6)

#define OUTPUT_PORT_1  (GPIO_PORT_P4)
#define OUTPUT_PIN_1  (GPIO_PIN5)
#define OUTPUT_PORT_2 (GPIO_PORT_P4)
#define OUTPUT_PIN_2 (GPIO_PIN7)

// Speed measurement variable
uint8_t count = 0; //TODO Check that uint8_t can hold coun value, else change to larger type

// Timer variable
#define TIMER_PERIOD 45000


/* TimerA UpMode Configuration Parameter */
const Timer_A_UpModeConfig upConfig =
{
        TIMER_A_CLOCKSOURCE_SMCLK,              // SMCLK Clock Source
        TIMER_A_CLOCKSOURCE_DIVIDER_1,          // SMCLK/1 = 3MHz
        TIMER_PERIOD,                                  // 15ms debounce period
        TIMER_A_TAIE_INTERRUPT_DISABLE,         // Disable Timer interrupt
        TIMER_A_CCIE_CCR0_INTERRUPT_ENABLE ,    // Enable CCR0 interrupt
        TIMER_A_DO_CLEAR                        // Clear value
};


int main(void) {
	// Halting WDT and disabling master interrupts 
    MAP_WDT_A_holdTimer(); 
    MAP_Interrupt_disableMaster();
    
    // Set timer
    MAP_Timer_A_configureUpMode(TIMER_A1_BASE, &upConfig);
	
    // Set pins modes
    // Set P4.1 and P4.6 as inputs (receives encoder outputs)
    GPIO_setAsInputPin(INPUT_PORT_1, INPUT_PIN_1);
    GPIO_setAsInputPin(INPUT_PORT_1, INPUT_PIN_2);
	
    //Set encoder handler (-> measure speed constantly)
    // Warning: Note that only Port 1,2,A have this capability.
    MAP_GPIO_clearInterruptFlag(INPUT_PORT_1, INPUT_PIN_1); 
    GPIO_registerInterrupt (INPUT_PORT_1, encoderHandler);
    GPIO_enableInterrupt (INPUT_PORT_1, INPUT_PIN_1);
	
    // Set P4.5 and P4.7 as outputs (output control command)
    GPIO_setAsOutputPin(OUTPUT_PORT_1, OUTPUT_PIN_1);
    GPIO_setAsOutputPin(OUTPUT_PORT_1, OUTPUT_PIN_2); 
	
	
    MAP_Interrupt_enableMaster();
	
    // Start button debounce timer
    MAP_Timer_A_startCounter(TIMER_A1_BASE, TIMER_A_UP_MODE);
    
    //TODO : Enable timer interrupt
	
    while(1) {
				
    }

    return 0;
}



/*
 * Timer A1 interrupt handler. 
 */
void TA1_0_IRQHandler(void) {

    // Access encoder count
    // Make a local copy because a encoder interrupt can occur while we are in the timer handler
    // And we should not disable the encoder interrupt while we are in this function
    // Because the motor is still spinning 
    // TODO Check the coherency of these data accesses.  
    uint8_t localCount = count;
    count = 0;

    // Calculate velocity
    uint8_t thetad = localCount / TIMER_PERIOD;

    // Calculate error
    uint8_t e = r - thethad;

    // Compute motor command v, from PID computations


    // Output the control command to the motor
    // TODO Maybe find how to output analog value
    // TODO Check which of the pin is for the direction and the velocity.
    // TODO Rewrite the arbitrary written velocity direction
    // void 	GPIO_setOutputHighOnPin (uint_fast8_t selectedPort, uint_fast16_t selectedPins)
    // void 	GPIO_setOutputLowOnPin (uint_fast8_t selectedPort, uint_fast16_t selectedPins)
    if(v>0) {
        GPIO_setOutputHighOnPin (OUTPUT_PORT1, OUTPUT_PIN1);
        // Analog output

    }
    else if(v<0) {
        GPIO_setOutputLowOnPin(OUTPUT_PORT1, OUTPUT_PIN1);
        //Analog output
    }

    // Store time and motor position (reference and real one)


    MAP_Timer_A_clearCaptureCompareInterrupt(TIMER_A1_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_0);
    MAP_Timer_A_startCounter(TIMER_A1_BASE, TIMER_A_UP_MODE);
}


/**
 * Encoder handler
 */
void encoderHandler () {
    // Speed measurement choice:
    // Measure the number of times the encoder outputs a value
    // Divide the number of outputs by the time to get a pseudo velocity
    // This should be enough since we are concerned with control and not stay to a specific velocity
    count ++;
}



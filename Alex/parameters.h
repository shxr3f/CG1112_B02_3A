#ifndef __PARAMETERS_INC__
#define __PARAMETERS_INC__

//General parameters for Alex


    
//Alex power saving definitions
#define PRR_TWI_MASK            0b10000000
#define PRR_SPI_MASK            0b00000100
#define ADCSRA_ADC_MASK         0b10000000
#define PRR_ADC_MASK            0b00000001
#define PRR_TIMER2_MASK         0b01000000
#define PRR_TIMER0_MASK         0b00100000
#define PRR_TIMER1_MASK         0b00001000
#define SMCR_SLEEP_ENABLE_MASK  0b00000001
#define SMCR_IDLE_MODE_MASK     0b11110001
#define COMPA                   0b10000000
#define COMPB                   0b00100000
#define COMPAB                  0b10100000

//Alex serial definitions
#define UDRIEMASK               0b00010000

#define PI                      3.1415


//Alex movement parameters

//Timer definitions for motors

//Distance parameters
#define COUNTS_PER_REV          198
#define WHEEL_CIRC              20.41
//Turning parameters
#define ALEX_LENGTH   0
#define ALEX_BREADTH  6

//amt of time Alex should stop for to prevent unwanted movement
// in MILLISECONDS
#define STOPDELAY 500

//Speed used for calibration
#define CALSPEED  100

//Distance to do calibration for
#define CAL_DIST 10

//Distance to move if 0 was accidentally input as distance
#define DEFDIST 10
//Angle to turn if no angle was given
#define DEFANGLE 30

//Approximate speed in cm/s for timeout counter
#define APPROX_SPEED 1
//Multiplier for the movement timeout  
#define FAILSAFE_MULT 2

#define MINTORQUE 140

//Time to wait in ms between ADC readings for the colour sensor
#define ADCDELAY 10

//how many ticks of the delay timer for 1s
#define TIMER_SCALE 200






#endif

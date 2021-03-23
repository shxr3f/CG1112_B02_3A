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


//Alex movement parameters

//Timer definitions for motors

//Distance parameters
#define COUNTS_PER_REV          198
#define WHEEL_CIRC              20.41
//Turning parameters
#define ALEX_LENGTH   0
#define ALEX_BREADTH  6

//amt of time Alex should stop for to prevent unwanted movement
// in MICROSECONDS
#define STOPDELAY 500
//max speed in RPM (i.e. when 100% speed is sent)
#define CALSPEED  10
#define CAL_COUNT 200
//Number of rounds to do calibration with
#define CAL_ROUNDS  5
//Number of ticks to calibrate around
#define CAL_AVGTICKS  10

#define MINTORQUE 140

//Time to wait in between ADC readings for the colour sensor
#define ADCDELAY 100






#endif

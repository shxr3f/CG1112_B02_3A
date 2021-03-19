#ifndef __REGISTERS_INC__
#define __REGISTERS_INC__

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


//Alex movement parameters

//Timer definitions for motors
#define LF                      OCR0A
#define LR                      OCR0B
#define RF                      OCR1A
#define RR                      OCR1B

//Distance parameters
#define COUNTS_PER_REV          198
#define WHEEL_CIRC              20.41
//Turning parameters
#define ALEX_LENGTH   0
#define ALEX_BREADTH  6

//amt of time Alex should stop for to prevent unwanted movement
#define STOPDELAY 50
//max speed in RPM (i.e. when 100% speed is sent)
#define HALFSPEED 1
//Number of rounds to do calibration with
#define CALI_ROUNDS 10
//Number of ticks to calibrate around
#define CALI_AVGTICKS 5

//Time to wait in between ADC readings for the colour sensor
#define ADCDELAY 100






#endif

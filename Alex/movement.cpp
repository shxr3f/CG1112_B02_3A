

#include <stdarg.h>
#include "parameters.h"
#include <stdbool.h>
#include "movement.h"
#include "constants.h"
#include <avr/sleep.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <math.h>
#include "delaytimer.h"
#include <Arduino.h>


float AlexDiagonal = 0.0;
float AlexCirc = 0.0;

volatile TDirection dir = STOP;
volatile bool movementDone = false;
volatile bool mvtTimeout = false;
unsigned long timeout;

// Store the ticks from Alex's left and
// right encoders.
volatile unsigned long leftForwardTicks; 
volatile unsigned long rightForwardTicks;
volatile unsigned long rightReverseTicks;
volatile unsigned long leftReverseTicks;

volatile unsigned long leftForwardTicksTurns; 
volatile unsigned long rightForwardTicksTurns;
volatile unsigned long rightReverseTicksTurns;
volatile unsigned long leftReverseTicksTurns;
volatile bool calibrateRight =  false;
volatile bool calibrateLeft = false;

// Store the revolutions on Alex's left
// and right wheels
volatile unsigned long leftRevs;
volatile unsigned long rightRevs;

// Forward and backward distance traveled
volatile unsigned long forwardDist;
volatile unsigned long reverseDist;

unsigned long deltaTicks;
unsigned long targetLTicks;
unsigned long targetRTicks;

unsigned long startTime;
unsigned long caliLeft;
unsigned long caliRight;
double adjLeft  = 1;
double adjRight = 1;



/*
 * Alex's motor drivers.
 * 
 */


 // Stop motors and turn off timers




// Set up Alex's motors. Right now this is empty, but
// later you will replace it with code to set up the PWMs
// to drive the motors.
void setupMotors()
{
  DDRD |= ((1 << 6) | (1 << 5));
  DDRB |= ((1 << 2) | (1 << 1));
  
  //Reset timer2 counters
  TCNT0  = 0;
  TCNT1  = 0;
  
  //Set to 8bit Phase correct PWM mode
  TCCR0A |= 0b00000001;
  TCCR1A |= 0b00000001;
  TCCR0A &= ~COMPAB;
  TCCR1A &= ~COMPAB;
  
  TIMSK0 |= 0b110;
  TIMSK1 |= 0b110; 
  /* Our motor set up is:  
   *    A1IN - Pin 5, PD5, OC0B
   *    A2IN - Pin 6, PD6, OC0A
   *    B1IN - Pin 9 PB1, OC1A
   *    B2In - pIN 10, PB2, OC1B
   */
}


// Functions to be called by INT0 and INT1 ISRs.
void leftISR()
{

  switch(dir)
  {
    case FORWARD:
      leftForwardTicks++;
      if (leftForwardTicks >= targetLTicks)
      {        
        OCR0A = 255;
        OCR0B = 255;
        TCCR0A |= COMPAB;
      }
      break;

    case BACKWARD:
      leftReverseTicks++; 
      if (leftReverseTicks >= targetLTicks)
      {
        OCR0A = 255;
        OCR0B = 255;
        TCCR0A |= COMPAB;
      }
      break;

    case LEFT:
      leftReverseTicksTurns++; 
      if (leftReverseTicksTurns >= targetRTicks)
      {
        OCR0A = 255;
        OCR0B = 255;
        TCCR0A |= COMPAB;
      }
      break;

    case RIGHT:
      leftForwardTicksTurns++; 
      if (leftForwardTicksTurns >= targetLTicks)
      {
        OCR0A = 255;
        OCR0B = 255;
        TCCR0A |= COMPAB;
      }
      break;

      case CALIBRATE:
        if (calibrateLeft)
        {
          leftForwardTicksTurns++;
          if (leftForwardTicksTurns >= targetLTicks)
          {
            caliLeft = _timerTicks;
            OCR0A   = 255;
            OCR0B   = 255;
            TCCR0A |= COMPAB;
            calibrateLeft = false;
          }
        }
        break;

    //todo: calibration code
  }
}

void rightISR()
{

  switch(dir)
  {
    case FORWARD:
      rightForwardTicks++;
      if (rightForwardTicks >= targetRTicks)
      {        
        OCR1A = 255;
        OCR1B = 255;
        TCCR1A |= COMPAB;
      }
      break;

    case BACKWARD:
      rightReverseTicks++; 
      if (rightReverseTicks >= targetRTicks)
      {
        OCR1A = 255;
        OCR1B = 255;
        TCCR1A |= COMPAB;
      }
      break;

    case LEFT:
      rightForwardTicksTurns++; 
      if (rightForwardTicksTurns >= targetRTicks)
      {
        OCR1A = 255;
        OCR1B = 255;
        TCCR1A |= COMPAB;
      }
      break;

    case RIGHT:
      rightReverseTicksTurns++; 
      if (rightReverseTicksTurns >= targetRTicks)
      {
        OCR1A = 255;
        OCR1B = 255;
        TCCR1A |= COMPAB;
      }
      break;

      case CALIBRATE:
        if (calibrateRight)
        {
          rightForwardTicksTurns++;
          if (rightForwardTicksTurns >= targetRTicks)
          {
            caliRight = _timerTicks;
            OCR1A   = 255;
            OCR1B   = 255;
            TCCR1A |= COMPAB;
            calibrateRight = false;
          }
        }
        break;
  }
}

// Implement the external interrupt ISRs below.
// INT0 ISR should call leftISR while INT1 ISR
// should call rightISR.


// Start the PWM for Alex's motors.
// We will implement this later. For now it is
// blank.
void startMotors()
{
  PRR    &= ~(PRR_TIMER0_MASK);
  PRR    &= ~(PRR_TIMER1_MASK);
  TCCR0B |= 0b00000011;
  TCCR1B |= 0b00000011;
  TCCR0A  |= 0b00000001;
  TCCR1A  |= 0b00000001;
  switch(dir)
  {
    case FORWARD:
      TCCR0A  |= COMPA;
      TCCR1A  |= COMPA;
      break; 

    case BACKWARD:
      TCCR0A  |= COMPB;
      TCCR1A  |= COMPB; 
      break;

    case LEFT:
      TCCR0A  |= COMPA;
      TCCR1A  |= COMPB; 
      break;
  
    case RIGHT:
      TCCR0A  |= COMPB;
      TCCR1A  |= COMPA; 
      break;

    case STOP:
      TCCR0A  |= COMPAB;
      TCCR1A  |= COMPAB;
      break; 

    case CALIBRATE:
      TCCR0A  |= COMPA;
      TCCR1A  |= COMPA;
      break;
  }
}

// Convert percentages to PWM values
int pwmVal(float speed)
{
  if(speed < 0.0)
    speed = 0;

  if(speed > 100.0)
    speed = 100.0;

  return (int) ((speed / 100.0) * 255.0);
}

// Move Alex forward "dist" cm at speed "speed".
// "speed" is expressed as a percentage. E.g. 50 is
// move forward at half speed.
// Specifying a distance of 0 means Alex will
// continue moving forward indefinitely.
void forward(float dist, float speed)
{
  dir = FORWARD;
  movementDone = false;
  int val = pwmVal(speed);
  if (dist > 0 )
  {
    deltaTicks = ((double)dist * COUNTS_PER_REV / WHEEL_CIRC);
  }
  else
  {
    deltaTicks = (DEFDIST * COUNTS_PER_REV / WHEEL_CIRC);
  }
  targetLTicks = leftForwardTicks + deltaTicks;
  targetRTicks = rightForwardTicks + deltaTicks;
  OCR0A = val * adjLeft;
  OCR1A = val * adjRight;
  OCR0B = val * adjLeft;
  OCR1B = val * adjRight;
  startMotors();
  while (!movementDone) {
        if ((leftForwardTicks >= targetLTicks) && (rightForwardTicks >= targetLTicks) )
        {
          OCR0A = 255;
          OCR0B = 255;
          OCR1A = 255;
          OCR1B = 255;
          TCCR0A |= COMPAB;
          TCCR1A |= COMPAB;
          movementDone = true;
        }
  }
  delayms(50);
  stop();
}

// Reverse Alex "dist" cm at speed "speed".
// "speed" is expressed as a percentage. E.g. 50 is
// reverse at half speed.
// Specifying a distance of 0 means Alex will
// continue reversing indefinitely.
void reverse(float dist, float speed)
{

  dir = BACKWARD;
  movementDone = false;
  int val = pwmVal(speed);

  if (dist >0 )
  {
    deltaTicks = ((double)dist * COUNTS_PER_REV / WHEEL_CIRC);
  }
  else
  {
    deltaTicks = (DEFDIST * COUNTS_PER_REV / WHEEL_CIRC);
  }
  targetLTicks = leftReverseTicks + deltaTicks;
  targetRTicks = rightReverseTicks + deltaTicks;
  OCR0A = val * adjLeft;
  OCR1A = val * adjRight;
  OCR0B = val * adjLeft;
  OCR1B = val * adjRight;
  startMotors();
  while (!movementDone) {
    if ((leftReverseTicks >= targetLTicks) && (rightReverseTicks >= targetRTicks) )
    {
      OCR0A = 255;
      OCR0B = 255;
      OCR1A = 255;
      OCR1B = 255;
      TCCR0A |= COMPAB;
      TCCR1A |= COMPAB;
      movementDone = true;
    }
  }
  
  delay(50);
  stop();
}

unsigned long computeDeltaTicks(float ang)
{
  unsigned long ticks = (unsigned long) (ang * PI * ALEX_BREADTH * COUNTS_PER_REV) /(WHEEL_CIRC * 360.0);

  return ticks;
}

// Turn Alex left "ang" degrees at speed "speed".
// "speed" is expressed as a percentage. E.g. 50 is
// turn left at half speed.
// Specifying an angle of 0 degrees will cause Alex to
// turn left indefinitely.
void left(float ang, float speed)
{
  dir = LEFT;
  movementDone = false;
  int val = pwmVal(speed);
  
  if (ang > 0)
  {
    deltaTicks = computeDeltaTicks(ang);
  }
  else
  {
    deltaTicks = computeDeltaTicks(DEFANGLE);
  }
  targetLTicks = leftReverseTicksTurns + deltaTicks;
  targetRTicks = rightForwardTicksTurns + deltaTicks;

  //TODO: add ticks calculation
  OCR0A = val * adjLeft;
  OCR1A = val * adjRight;
  OCR0B = val * adjLeft;
  OCR1B = val * adjRight;
  startMotors();
  while (!movementDone) {        
    if ((leftReverseTicksTurns >= targetLTicks) && (rightForwardTicksTurns >= targetRTicks) )
        {
          OCR0A = 255;
          OCR0B = 255;
          OCR1A = 255;
          OCR1B = 255;
          TCCR0A |= COMPAB;
          TCCR1A |= COMPAB;
          movementDone = true;
        }
    }
  delayms(50);
  stop();
}

// Turn Alex right "ang" degrees at speed "speed".
// "speed" is expressed as a percentage. E.g. 50 is
// turn left at half speed.
// Specifying an angle of 0 degrees will cause Alex to
// turn right indefinitely.
void right(float ang, float speed)
{
  dir = RIGHT;
  movementDone = false;
  int val = pwmVal(speed);
  
  if (ang > 0)
  {
    deltaTicks = computeDeltaTicks(ang);
  }
  else
  {
    deltaTicks = computeDeltaTicks(DEFANGLE);
  }
  targetRTicks = rightReverseTicksTurns + deltaTicks;
  targetLTicks = leftForwardTicksTurns + deltaTicks;
  OCR0A = val * adjLeft;
  OCR1A = val * adjRight;
  OCR0B = val * adjLeft;
  OCR1B = val * adjRight;
  startMotors();
  while (!movementDone) {
        if ((leftForwardTicksTurns >= targetLTicks) && (rightReverseTicksTurns >= targetRTicks) )
        {
          OCR0A = 255;
          OCR0B = 255;
          OCR1A = 255;
          OCR1B = 255;
          TCCR0A |= COMPAB;
          TCCR1A |= COMPAB;
          movementDone = true;
        }
  }
  delayms(50);
  stop();
}


void stop()
{
  dir = STOP;
  TCNT0 = 0;
  TCNT1 = 0;
  //turn off all comparators
  TCCR0B &= ~(0b00000011);
  TCCR1B &= ~(0b00000011);
  TCCR0A &= ~COMPA;
  TCCR0A &= ~COMPB;
  TCCR1A &= ~COMPA;
  TCCR1A &= ~COMPB;
}

void calibrateMotors()
{
  //time taken for 1 tick
  startTimer();
  movementDone = false;
  calibrateLeft = true;
  calibrateRight = true;
  deltaTicks = (COUNTS_PER_REV * CAL_ROUNDS);
  targetLTicks = leftForwardTicksTurns + deltaTicks;
  targetRTicks = rightForwardTicksTurns + deltaTicks;
  dir = CALIBRATE;
  int val = pwmVal(CALSPEED);
  OCR0A = val;
  OCR0B = 0;
  OCR1A = val;
  OCR1B = 0;
  startTime = _timerTicks;
  startMotors();
  while (!movementDone) {        
    if ((leftForwardTicksTurns >= targetLTicks) && (rightReverseTicksTurns >= targetRTicks) )
        {
          OCR0A = 255;
          OCR0B = 255;
          OCR1A = 255;
          OCR1B = 255;
          TCCR0A |= COMPAB;
          TCCR1A |= COMPAB;
          movementDone = true;
        }
  }
  if (caliLeft > caliRight)
  {
    adjLeft = caliRight/caliLeft;
  }
  else
  {
    adjRight = caliLeft/caliRight;
  }
  delayms(50);
  stop();
  stopTimer();
}

unsigned long setTimeout(unsigned long targetTicks)
{
  timeout = _timerTicks + (targetTicks * APPROX_SPEED * WHEEL_CIRC * TIMER_SCALE / COUNTS_PER_REV);
  mvtTimeout = false;
}

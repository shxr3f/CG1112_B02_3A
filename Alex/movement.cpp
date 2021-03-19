

#include <stdarg.h>
#include "parameters.h"
#include <stdbool.h>
#include "movement.h"
#include "constants.h"
#include <avr/sleep.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <math.h>
#include <Arduino.h>


float AlexDiagonal = 0.0;
float AlexCirc = 0.0;

volatile TDirection dir = STOP;

volatile bool sideDone = false;
volatile bool movementDone = false;

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

// Store the revolutions on Alex's left
// and right wheels
volatile unsigned long leftRevs;
volatile unsigned long rightRevs;

// Forward and backward distance traveled
volatile unsigned long forwardDist;
volatile unsigned long reverseDist;

unsigned long deltaTicks;
unsigned long targetTicks;

unsigned long startTime;
unsigned long endTime;
unsigned int  caliTime;
double adjLeft;
double adjRight;



/*
 * Alex's motor drivers.
 * 
 */

ISR(TIMER0_COMPA_vect)
{ 
} 

ISR(TIMER0_COMPB_vect)
{ 
}

ISR(TIMER1_COMPA_vect)
{ 
} 

ISR(TIMER1_COMPB_vect)
{ 
}

 // Stop motors and turn off timers


void calibrateMotors()
{
  //time taken for 1 tick
  caliTime = (1000 * CALI_AVGTICKS) / (HALFSPEED * COUNTS_PER_REV);
  //Calibration for left motor
  deltaTicks = (COUNTS_PER_REV * CALI_ROUNDS);
  targetTicks = leftForwardTicksTurns + deltaTicks;
  dir = CALIBRATE;
  OCR0A = 127;
  OCR0B = 0;
  OCR1A = 255;
  OCR1B = 255;
  startMotors();
  while (!movementDone) {}

  //calibration for right motor
  sideDone = false;
  movementDone = false;
  targetTicks = rightForwardTicksTurns + deltaTicks;
  RF = 255;
  RR = 0;
  while (!movementDone) {}
}


// Set up Alex's motors. Right now this is empty, but
// later you will replace it with code to set up the PWMs
// to drive the motors.
void setupMotors()
{
  DDRD |= ((1 << 6) | (1 << 5));
  DDRB |= ((1 << 2) | (1 << 1));
  
  //Reset timer2 counters
  TCNT0  |= 0;
  TCNT1  |= 0;
  
  //Set to 8bit Phase correct PWM mode
  TCCR0A |= 0b00000001;
  TCCR1A |= 0b00000001;
  
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
      if (leftForwardTicks == targetTicks)
      {        
        TCCR0A |= (COMPA | COMPB);
        if (sideDone)
        {
          startTime = millis();
          while (millis() < (startTime + STOPDELAY)) {}
          forwardDist = leftForwardTicks * (WHEEL_CIRC);
         movementDone = true;
        }
        else
        {
          sideDone = true;
        }
      }
      break;

    case BACKWARD:
      leftReverseTicks++; 
      if (leftReverseTicks == targetTicks)
      {
        TCCR0A |= (COMPA | COMPB);
        if (sideDone)
        {
          startTime = millis();
          while (millis() < (startTime + STOPDELAY)) {}
          reverseDist = leftReverseTicks * (WHEEL_CIRC);
          movementDone = true;
        }
        else
        {
          sideDone = true;
        }
      }
      break;

    case LEFT:
      leftReverseTicksTurns++; 
      if (leftReverseTicksTurns == targetTicks)
      {
        TCCR0A |= (COMPA | COMPB);
        if (sideDone)
        {
          startTime = millis();
          while (millis() < (startTime + STOPDELAY)) {}
          movementDone = true;
        }
        else
        {
          sideDone = true;
        }
      }
      break;

    case RIGHT:
      leftForwardTicksTurns++; 
      if (leftForwardTicksTurns == targetTicks)
      {
        TCCR0A |= (COMPA | COMPB);
        if (sideDone)
        {
          startTime = millis();
          while (millis() < (startTime + STOPDELAY)) {}
          movementDone = true;
        }
        else
        {
          sideDone = true;
        }
      }
      break;

      case CALIBRATE:
        if (!sideDone)
        {
          leftForwardTicksTurns++;
          if (leftForwardTicksTurns == targetTicks)
          {
            sideDone = true;
            adjLeft = (double)LF / 127;
            LR = LF;
            LF = 0;
            targetTicks = leftReverseTicksTurns + deltaTicks;
          }
          else if (leftForwardTicksTurns % CALI_ROUNDS == 0)
          {
            if (millis() - startTime > caliTime)
            {
              LF--;
            }
            else if (millis() - startTime < caliTime)
            {
              LF++;
            }
          }
        }
        else
        {
          leftReverseTicksTurns++;
          if (leftReverseTicksTurns == targetTicks)
          {
            LF = LR = 255;
            movementDone = true;
          }
        }

    //todo: calibration code
  }
}

void rightISR()
{
  switch(dir)
  {
    case FORWARD:
      rightForwardTicks++;
      if (rightForwardTicks == targetTicks)
      {        
        TCCR1A |= (COMPA | COMPB);
        if (sideDone)
        {
          startTime = millis();
          while (millis() < (startTime + STOPDELAY)) {}
          forwardDist = rightForwardTicks * (WHEEL_CIRC);
         movementDone = true;
        }
        else
        {
          sideDone = true;
        }
      }
      break;

    case BACKWARD:
      rightReverseTicks++; 
      if (rightReverseTicks == targetTicks)
      {
        TCCR1A |= (COMPA | COMPB);
        if (sideDone)
        {
          startTime = millis();
          while (millis() < (startTime + STOPDELAY)) {}
          reverseDist = rightReverseTicks * (WHEEL_CIRC);
          movementDone = true;
        }
        else
        {
          sideDone = true;
        }
      }
      break;

    case LEFT:
      rightForwardTicksTurns++; 
      if (rightForwardTicksTurns == targetTicks)
      {
        TCCR1A |= (COMPA | COMPB);
        if (sideDone)
        {
          startTime = millis();
          while (millis() < (startTime + STOPDELAY)) {}
          movementDone = true;
        }
        else
        {
          sideDone = true;
        }
      }
      break;

    case RIGHT:
      rightReverseTicksTurns++; 
      if (rightReverseTicksTurns == targetTicks)
      {
        TCCR1A |= (COMPA | COMPB);
        if (sideDone)
        {
          startTime = millis();
          while (millis() < (startTime + STOPDELAY)) {}
          movementDone = true;
        }
        else
        {
          sideDone = true;
        }
      }
      break;

      case CALIBRATE:
        if (!sideDone)
        {
          rightForwardTicksTurns++;
          if (rightForwardTicksTurns == targetTicks)
          {
            sideDone = true;
            adjRight = (double)RF / 127;
            RR = RF;
            RF = 0;
            targetTicks = rightReverseTicksTurns + deltaTicks;
          }
          else if (rightForwardTicksTurns % CALI_ROUNDS == 0)
          {
            if (millis() - startTime > caliTime)
            {
              RF--;
            }
            else if (millis() - startTime < caliTime)
            {
              RF++;
            }
          }
        }
        else
        {
          rightReverseTicksTurns++;
          if (rightReverseTicksTurns == targetTicks)
          {
            RF = RR = 255;
            movementDone = true;
          }
        }

    //todo: calibration code
  }
}

// Implement the external interrupt ISRs below.
// INT0 ISR should call leftISR while INT1 ISR
// should call rightISR.
ISR(INT0_vect)
{
  leftISR();
}

ISR(INT1_vect)
{
  rightISR();
}

// Start the PWM for Alex's motors.
// We will implement this later. For now it is
// blank.
void startMotors()
{
  
  TCCR0B |= 0b00000011;
  TCCR1B |= 0b00000011;
  PRR    &= !(PRR_TIMER0_MASK);
  PRR    &= !(PRR_TIMER1_MASK);
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
      TCCR0A  = 0b00000001;
      TCCR1A  = 0b00000001;
      break; 

    case CALIBRATE:
      TCCR0A  |= COMPA;
      TCCR0A  |= COMPB;
      TCCR1A  |= COMPA;
      TCCR1A  |= COMPB;
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
  sideDone = false;
  int val = pwmVal(speed);
  
  if (dist >0 )
  {
    deltaTicks = ((double)dist * COUNTS_PER_REV / WHEEL_CIRC);
  }
  else
  {
    deltaTicks = 9999999;
  }
  targetTicks = leftForwardTicks + deltaTicks;

  //TODO: add ticks calculation
  startMotors();
  LF = val * adjLeft;
  RF = val * adjRight;
  LR = 0;
  RR = 0;
  while (!movementDone) {}
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
  sideDone = false;
  int val = pwmVal(speed);
  
  if (dist >0 )
  {
    deltaTicks = ((double)dist * COUNTS_PER_REV / WHEEL_CIRC);
  }
  else
  {
    deltaTicks = 9999999;
  }
  targetTicks = leftReverseTicks + deltaTicks;

  //TODO: add ticks calculation
  startMotors();
  LR = val * adjLeft;
  RR = val * adjRight;
  LF = 0;
  RF = 0;
  while (!movementDone) {}
  stop();
}

unsigned long computeDeltaTicks(float ang)
{
  unsigned long ticks = (unsigned long) ((ang * AlexCirc * COUNTS_PER_REV)/(360.0*WHEEL_CIRC));

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
  sideDone = false;
  int val = pwmVal(speed);
  
  if (ang >0 )
  {
    deltaTicks = computeDeltaTicks(ang);;
  }
  else
  {
    deltaTicks = 9999999;
  }
  targetTicks = leftReverseTicksTurns + deltaTicks;

  //TODO: add ticks calculation
  startMotors();
  LR = val * adjLeft;
  RF = val * adjRight;
  LF = 0;
  RR = 0;
  while (!movementDone) {}
  
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
  sideDone = false;
  int val = pwmVal(speed);
  
  if (ang > 0)
  {
    deltaTicks = computeDeltaTicks(ang);
  }
  else
  {
    deltaTicks = 9999999;
  }
  targetTicks = rightReverseTicksTurns + deltaTicks;

  startMotors();
  RR = val * adjLeft;
  LF = val * adjRight;
  RF = 0;
  LR = 0;
  while (!movementDone) {}
  
  stop();
}


void stop()
{
  dir = STOP;
  //Reset comparator compare to values
  OCR0A = 0;
  OCR0B = 0;
  OCR1A = 0;
  OCR1B = 0;
  //turn off all comparators
  TCCR0B &= !(0b00000011);
  TCCR1B &= !(0b00000011);
  TCCR0A &= !(0b10100000);
  TCCR1A &= !(0b10100000);
  //turn off timers
  PRR   |= (PRR_TIMER0_MASK | PRR_TIMER2_MASK);
}

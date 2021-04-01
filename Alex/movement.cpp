

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
unsigned long targetTicks;

unsigned long startTime;
unsigned long endTime;
unsigned long  caliTime;
double adjLeft  = 1;
double adjRight = 0.8;



/*
 * Alex's motor drivers.
 * 
 */


 // Stop motors and turn off timers


void calibrateMotors()
{
  //time taken for 1 tick
  startTimer();
  sideDone = false;
  movementDone = false;
  calibrateLeft = true;
  caliTime = (1000 * CAL_AVGTICKS * CALSPEED) / (COUNTS_PER_REV * 60);
  //Calibration for left motor
  deltaTicks = (COUNTS_PER_REV * CAL_ROUNDS);
  targetTicks = leftForwardTicksTurns + deltaTicks;
  dir = CALIBRATE;
  OCR0A = CAL_COUNT;
  OCR0B = 0;
  OCR1A = 255;
  OCR1B = 255;
  startTime = _timerTicks;
  startMotors();
  while (!movementDone) {}
  stop();
  //calibration for right motor
  calibrateLeft = false;
  calibrateRight = true;
  sideDone = false;
  movementDone = false;
  targetTicks = rightForwardTicksTurns + deltaTicks;
  OCR0A = 255;
  OCR0B = 255;
  OCR1A = CAL_COUNT;
  OCR1B = 0;
  startTime = _timerTicks;
  dir = CALIBRATE;
  startMotors();
  while (!movementDone) {}
  startTime = _timerTicks;
  while (_timerTicks - startTime < STOPDELAY) {}
  dir = STOP;
  stopTimer();
  stop();
}


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
      if (leftForwardTicks == targetTicks)
      {        
        OCR0A = 255;
        OCR0B = 255;
        TCCR0A |= COMPAB;
        if (sideDone)
        {
          forwardDist = leftForwardTicks * (WHEEL_CIRC);
          movementDone = true;
        }
        else
        {
          sideDone = true;
        }
      }
      else
      {
        if ((leftForwardTicks >= targetTicks) && (rightForwardTicks >= targetTicks) )
        {
          OCR0A = 255;
          OCR0B = 255;
          TCCR0A |= COMPAB;
          movementDone = true;
        }
      }
      break;

    case BACKWARD:
      leftReverseTicks++; 
      if (leftReverseTicks == targetTicks)
      {
        OCR0A = 255;
        OCR0B = 255;
        TCCR0A |= COMPAB;
        if (sideDone)
        {
          reverseDist = leftReverseTicks * (WHEEL_CIRC);
          movementDone = true;
        }
        else
        {
          sideDone = true;
        }
      }
      else
      {
        if ((leftReverseTicks >= targetTicks) && (rightReverseTicks >= targetTicks) )
        {
          OCR0A = 255;
          OCR0B = 255;
          TCCR0A |= COMPAB;
          movementDone = true;
        }
      }
      break;

    case LEFT:
      leftReverseTicksTurns++; 
      if (leftReverseTicksTurns == targetTicks)
      {
        OCR0A = 255;
        OCR0B = 255;
        TCCR0A |= COMPAB;
        if (sideDone)
        {
          movementDone = true;
        }
        else
        {
          sideDone = true;
        }
      }
      else
      {
        if ((leftReverseTicksTurns >= targetTicks) && (rightForwardTicksTurns >= targetTicks) )
        {
          OCR0A = 255;
          OCR0B = 255;
          TCCR0A |= COMPAB;
          movementDone = true;
        }
      }
      break;

    case RIGHT:
      leftForwardTicksTurns++; 
      if (leftForwardTicksTurns == targetTicks)
      {
        OCR0A = 255;
        OCR0B = 255;
        TCCR0A |= COMPAB;
        if (sideDone)
        {
          movementDone = true;
        }
        else
        {
          sideDone = true;
        }
      }
      else
      {
        if ((leftForwardTicksTurns >= targetTicks) && (rightReverseTicksTurns >= targetTicks) )
        {
          OCR0A = 255;
          OCR0B = 255;
          TCCR0A |= COMPAB;
          movementDone = true;
        }
      }
      break;

      case CALIBRATE:
        if (!sideDone && calibrateLeft)
        {
          leftForwardTicksTurns++;
//          Serial.println(leftForwardTicksTurns);
          if (leftForwardTicksTurns >= targetTicks)
          {
            sideDone = true;
            adjLeft = (double)OCR0A / 200;
            OCR0B = OCR0A;
            OCR0A = 0;
            targetTicks = leftReverseTicksTurns + deltaTicks;
          }
          else if (leftForwardTicksTurns % CAL_AVGTICKS == 0)
          {
            if (_timerTicks - startTime < caliTime)
            {
              if (OCR0A > MINTORQUE)
              OCR0A--;
            }
            else if (_timerTicks - startTime > caliTime)
            {
              if (OCR0A <= 255)
              {
                OCR0A++;
              }
            }
            startTime = _timerTicks;
          }
        }
        else
        {
          leftReverseTicksTurns++;
          if (leftReverseTicksTurns >= targetTicks)
          {
            OCR0A = 255;
            OCR0B = 255;
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
        OCR1A = 255;
        OCR1B = 255;
        TCCR1A |= COMPAB;
        if (sideDone)
        {
          forwardDist = rightForwardTicks * (WHEEL_CIRC);
         movementDone = true;
        }
        else
        {
          sideDone = true;
        }
      }
      else
      {
        if ((leftForwardTicks >= targetTicks) && (rightForwardTicks >= targetTicks) )
        {
          OCR1A = 255;
          OCR1B = 255;
          TCCR1A |= COMPAB;
          movementDone = true;
        }
      }
      break;

    case BACKWARD:
      rightReverseTicks++; 
      if (rightReverseTicks == targetTicks)
      {
        OCR1A = 255;
        OCR1B = 255;
        TCCR1A |= COMPAB;
        if (sideDone)
        {
          reverseDist = rightReverseTicks * (WHEEL_CIRC);
          movementDone = true;
        }
        else
        {
          sideDone = true;
        }
      }
      else
      {
        if ((leftReverseTicks >= targetTicks) && (rightReverseTicks >= targetTicks) )
        {
          OCR1A = 255;
          OCR1B = 255;
          TCCR1A |= COMPAB;
          movementDone = true;
        }
      }
      break;

    case LEFT:
      rightForwardTicksTurns++; 
      if (rightForwardTicksTurns == targetTicks)
      {
        OCR1A = 255;
        OCR1B = 255;
        TCCR1A |= COMPAB;
        if (sideDone)
        {
          movementDone = true;
        }
        else
        {
          sideDone = true;
        }
      }
      else
      {
        if ((leftReverseTicksTurns >= targetTicks) && (rightForwardTicksTurns >= targetTicks) )
        {
          OCR1A = 255;
          OCR1B = 255;
          TCCR1A |= COMPAB;
          movementDone = true;
        }
      }
      break;

    case RIGHT:
      rightReverseTicksTurns++; 
      if (rightReverseTicksTurns == targetTicks)
      {
        OCR1A = 255;
        OCR1B = 255;
        TCCR1A |= COMPAB;
        if (sideDone)
        {
          movementDone = true;
        }
        else
        {
          sideDone = true;
        }
      }
      else
      {
        if ((leftForwardTicksTurns >= targetTicks) && (rightReverseTicksTurns >= targetTicks) )
        {
          OCR1A = 255;
          OCR1B = 255;
          TCCR1A |= COMPAB;
          movementDone = true;
        }
      }
      break;

      case CALIBRATE:
        if (!sideDone && calibrateRight)
        {
          rightForwardTicksTurns++;
          if (rightForwardTicksTurns >= targetTicks)
          {
            sideDone = true;
            adjRight = (double)OCR1A / 200;
            OCR1B = OCR1A;
            OCR1A = 0;
            targetTicks = rightReverseTicksTurns + deltaTicks;
          }
          else if (rightForwardTicksTurns % CAL_AVGTICKS == 0)
          {
            if (_timerTicks - startTime < caliTime)
            {
              if (OCR1A > MINTORQUE)
              OCR1A--;
            }
            else if (_timerTicks - startTime > caliTime)
            {
              if (OCR1A <= 255)
              {
                OCR1A++;
              }
            }
            startTime = _timerTicks;
          }
        }
        else
        {
          rightReverseTicksTurns++;
          if (rightReverseTicksTurns >= targetTicks)
          {
            OCR1A = 255;
            OCR1B = 255;
            calibrateRight =  false;
            movementDone = true;
          }
        }

    //todo: calibration code
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
      TCCR0A  |= 0b10100001;
      TCCR1A  |= 0b10100001;
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
  OCR0A = val * adjLeft;
  OCR1A = val * adjRight;
  OCR0B = val * adjLeft;
  OCR1B = val * adjRight;
  startMotors();
  while (!movementDone) {}
  delay(50);
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
  OCR0A = val * adjLeft;
  OCR1A = val * adjRight;
  OCR0B = val * adjLeft;
  OCR1B = val * adjRight;
  startMotors();
  while (!movementDone) {}
  
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
  targetTicks = leftReverseTicksTurns + deltaTicks;

  //TODO: add ticks calculation
  OCR0A = val * adjLeft;
  OCR1A = val * adjRight;
  OCR0B = val * adjLeft;
  OCR1B = val * adjRight;
  startMotors();
  while (!movementDone) {}
  delay(50);
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
  
  OCR0A = val * adjLeft;
  OCR1A = val * adjRight;
  OCR0B = val * adjLeft;
  OCR1B = val * adjRight;
  startMotors();
  while (!movementDone) {}
  
  delay(50);

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

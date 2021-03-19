#ifndef __MOVEMENT_INC__
#define __MOVEMENT_INC__

typedef enum
{
  STOP = 0,
  FORWARD = 1,
  BACKWARD = 2,
  LEFT = 3,
  RIGHT = 4,
  CALIBRATE = 5
} TDirection;

extern float AlexDiagonal;
extern float AlexCirc;

extern volatile TDirection dir;

extern volatile bool sideDone;
extern volatile bool movementDone;

// Store the ticks from Alex's left and
// right encoders.
extern volatile unsigned long leftForwardTicks; 
extern volatile unsigned long rightForwardTicks;
extern volatile unsigned long rightReverseTicks;
extern volatile unsigned long leftReverseTicks;

extern volatile unsigned long leftForwardTicksTurns; 
extern volatile unsigned long rightForwardTicksTurns;
extern volatile unsigned long rightReverseTicksTurns;
extern volatile unsigned long leftReverseTicksTurns;

// Store the revolutions on Alex's left
// and right wheels
extern volatile unsigned long leftRevs;
extern volatile unsigned long rightRevs;

// Forward and backward distance traveled
extern volatile unsigned long forwardDist;
extern volatile unsigned long reverseDist;

extern unsigned long deltaTicks;
extern unsigned long targetTicks;

//amt of time Alex should stop for to prevent unwanted movement
#define STOPDELAY 50
extern unsigned long startTime;
extern unsigned long endTime;


void calibrateMotors();
void setupMotors();
void leftISR();
void rightISR();
void startMotors();
int pwmVal(float speed);
void forward(float dist, float speed);
void reverse(float dist, float speed);
unsigned long computeDeltaTicks(float ang);
void left(float ang, float speed);
void right(float ang, float speed);
void stop();

#endif

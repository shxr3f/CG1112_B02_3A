#ifndef __DELAYTIMER_INC__
#define __DELAYTIMER_INC__



extern volatile unsigned long _timerTicks;

void setupTimer();
void startTimer();
void stopTimer();
void delayms(unsigned long i);

#endif


#include <avr/sleep.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include "parameters.h"

unsigned long _timerTicks;

void setupTimer()
{
  TCNT2   = 0;
  OCR2A   = 199;
  TCCR2A |= 0b00000010;
  TIMSK2 |= 0b010; 
}

void startTimer()
{
  //disable power saving mode for timer 2
  PRR    &= !(PRR_TIMER2_MASK);
  TCCR2B |= 0b00000010;

}

void stopTimer()
{
  TCCR2B &= !(0b00000010);
  PRR    |= (PRR_TIMER2_MASK);
}


#include <avr/sleep.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include "parameters.h"
#include <Arduino.h>
#include "delaytimer.h"

volatile unsigned long _timerTicks = 0;

void setupTimer()
{
  TCNT2   = 0;
  TCCR2A |= 0b00000010;
  TIMSK2 |= 0b010; 
  OCR2A   = 125;
}

void startTimer()
{
  //disable power saving mode for timer 2
  PRR    &= !(PRR_TIMER2_MASK);
  TCCR2B |= 0b00000001;

}

void stopTimer()
{
  TCCR2B &= !(0b00000010);
  PRR    |= (PRR_TIMER2_MASK);
}

void delayms(unsigned long i)
{
  startTimer();
  long endTime = _timerTicks + (i*100);
  
//  Serial.println("Hello");
  while(_timerTicks <= endTime) {}
  
  stopTimer();
  
}

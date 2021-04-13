//#include <Wire.h>
#include "Adafruit_TCS34725.h"
#include "parameters.h"
#include <math.h>
/*

#include <stdarg.h>
#include <avr/sleep.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include "parameters.h"
*/
#include "delaytimer.h"
#include "serialarduino.h"


Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_700MS, TCS34725_GAIN_1X);

void setupSensor() {
  tcs.begin();
  /*

  Serial.begin(9600);

  if (tcs.begin()) {
//    Serial.println("Found sensor");
  } else {
    Serial.println("No TCS34725 found ... check your connections");
    while (1);
  }

  // Now we're ready to get readings!
  */
}

void offSensor()
{
  tcs.disable();
}

char colourValue()
{
  PORTD |= (1<<7);
  setupSensor();
  delayms(ADCDELAY);
  int16_t r, g, b, c, sum, lux;
  int16_t RGB_arr[2] = {0,0};
  tcs.getRawData(&r, &g, &b, &c);
  RGB_arr[0] =  r;
  RGB_arr[1] =  g;
  PORTD &= ~(1<<7);
  offSensor();

 if( (RGB_arr[0] - RGB_arr[1]) < 10 && (RGB_arr[0] - RGB_arr[1]) > -10 )
 {
  return 'N';
 }
 else if (RGB_arr[0] > RGB_arr[1])
 {
  return "R";
 }
 else
 {
  return "G";
 }
}

int lightBar()
{
  PORTD ^= (1 << 7);
  return ((PORTD >> 7) & 1);
}

/*
int startADC()
{
  ADCSRA |= (1 << ADSC);
  while(ADCSRA & (1 << ADSC));
  int adcvalue = ADCH;
  return adcvalue;
}

void setupADC()
{
  PRR &= ~(1 << PRADC); //Turn on Power for ADC
  ADCSRA |= ((1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0));
  ADMUX |= ((1 << REFS0) | (1 << ADLAR)); //Left adjusted to read 8 bits at A0
  
}

int colourValue()
{
  startTimer();
  int readings[5];
  int sum = 0;
  int finalvalue;
  long lastTime = 0;
  setupADC();
  for(int i = 0; i < 5; i++)
  {
    lastTime = _timerTicks;
    while( _timerTicks < lastTime + ADCDELAY)
    {}
    readings[i] = startADC();
    sum += readings[i];
  }
  finalvalue = sum / 5;
  PRR |= (1 << PRADC); //Turn off power for ADC
  stopTimer();
  return finalvalue;
}

char findColour()
{
  int colour[3] = {0,0,0};
  //Setup Red and Green LED at A1 and A2
  DDRC |= ((1 << 2) | (1 << 1));
  colour[0] = colourValue();
  
  //On Red
  PORTC |= (1 << 1);
  colour[1] = colourValue();

  //Off Red and ON Green
  PORTC &= ~(1 << 1);
  PORTC |= (1 << 2);
  colour[2] = colourValue();

  //Off Green
  PORTC &= ~(1 << 2);

  if(colour[2] > colour[1])
  {
    return 'R';
  }
  else
  {
    return 'G';
  }
}
*/

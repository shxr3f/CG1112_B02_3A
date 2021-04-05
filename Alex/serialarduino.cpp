#include <stdarg.h>
#include "parameters.h"
#include "movement.h"
#include "constants.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include "serialarduino.h"
//#include <buffer.h>
#include "packet.h"
#include <Arduino.h>


void setupSerial()
{
 Serial.begin(9600);
}

void startSerial()
{

}

int readSerial(char *buffer)
{

  int count = 0; 
	
  while (Serial.available())
  {
    buffer[count ++] = Serial.read();
  }
	return count;

}

void writeSerial(const unsigned char *buffer, int len)
{
  Serial.write(buffer, len);
}

/*
 * Uncomment to use baremetal serial
 * 
 * 

void setupSerial()
{
  UCSR0C = 0b00000110; //Set to 8N1
  UBRR0L = 103; //b = 103 for 9600 bps
  UBRR0H = 0;
  UCSR0A = 0;
}

void startSerial()
{
  UCSR0B = 0b10011000;
}

ISR(USART_RX_vect)
{
  unsigned char data = UDR0;

  writeBuffer(&_recvBuffer,data);
}

ISR(USART_UDRE_vect)
{
  unsigned char data;
  TBufferResult result = readBuffer(&_xmitBuffer, &data);
 
  if (result == BUFFER_OK) 
    UDR0A = data;
   else
   {
    if (result == BUFFER_EMPTY)
    {
      UCSR0B &= ~UDRIEMASK
    }
   }
}

int readSerial(char *buffer)
{
  int count = 0;

  TBufferResult result;
  do
  {
    result = readBuffer(&_recvBuffer, &buffer[count]);

    if(result == BUFFER_OK)
    {
      count++
    }
  }while(result == BUFFER_OK);
}

void writeSerial(const unsigned char *buffer, int len)
{
  TBufferResult result = BUFFER_OK;
	for(int i = 1; i < len && (result == BUFFER_OK); i++)
	{
		result = writeBuffer(&_xmitBuffer, buffer[i]);
	}
 UDR0 = buffer[1]
 UCSR0B  |= UDRIEMASK;
}
*/

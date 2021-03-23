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

//TBuffer _xmitBuffer[MAX_STR_LEN];


char dataSend;

void setupSerial()
{
  /*
	UCSR0C = 0b00000110; //Set to 8N1
	UBRR0L = 103; //b = 103 for 9600 bps
	UBRR0H = 0;
	UCSR0A = 0;
 */
 Serial.begin(9600);
}

void startSerial()
{
//	UCSR0B = 0b00011000;
}

int readSerial(char *buffer)
{

	int count = 0; 
	/*
	while( (UCSR0A & (1 << 7)) == 0)
	{
		buffer[count++] = UDR0;
	}
 */
 while (Serial.available())
 {
  buffer[count ++] = Serial.read();
 }
	return count;

}
/*
ISR(USART_UDRE_vect)
{
  unsigned char data;
 TResult result = readBuffer(&_xmitBuffer, &data);
 
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

void writeSerial(const char *buffer, int len)
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

void writeSerial(const unsigned char *buffer, int len)
{
  Serial.write(buffer, len);
}

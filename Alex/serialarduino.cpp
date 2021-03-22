#include <stdarg.h>
#include "parameters.h"
#include <stdbool.h>
#include "movement.h"
#include "constants.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <math.h>
#include <Arduino.h>

#define UDRIEMASK 0b00100000

char dataSend;

ISR(USART_UDRE_vect)
{
	UDR0 = dataSend;
	UCSR0B &= ~(UDRIEMASK);

}

void setupSerial()
{
	cli();
	UCSR0C = 0b00000110; //Set to 8N1
	UBRR0L = 103; //b = 103 for 9600 bps
	UBRR0H = 0;
	UCSR0A = 0;
	sei();
}

void startSerial()
{
	UCSR0B = 0b00011000;

}

int readserial(char *buffer)
{
	int count = 0;
	while(UCSR0A & (1 << 7))
	{
		buffer[count++] = UDR0;
	}
	return count

}

void writeSerial(const unsigned char *buffer, int len)
{
	for(int i = 0; i < len; i++)
	{
		dataSend = buffer[i];
		UCSR0B  |= UDRIEMASK;
	}

}

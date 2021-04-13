  #include <math.h>
#include <serialize.h>
#include <stdarg.h>
#include "packet.h"
#include "constants.h"
#include <avr/sleep.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include "movement.h"
#include "serialarduino.h"
#include "delaytimer.h"
#include "parameters.h"
#include "coloursensor.h"


/*
 * 
 * Alex Communication Routines.
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

ISR(TIMER2_COMPA_vect)
{ 
  _timerTicks++;
  //movement time out 
  if (dir != STOP)
  {
    if (_timerTicks > timeout)
    {
      mvtTimeout = true;
    }
  }
}

ISR(INT0_vect)
{
  leftISR();
}

ISR(INT1_vect)
{
  rightISR();
}


void WDT_off(void)
{
  //Clear WDRF in MCUSR
  MCUSR  &= ~(1<<WDRF);
  //Write 1 to WDCE and WDE
  WDTCSR |= (1<<WDCE) | (1<<WDE);
  // Turn off WDT
  WDTCSR  = 0x00;
}

void setupPowerSaving()
{
  //disable TWI, TIM1 and SPI interfaces
  PRR    |= (PRR_TWI_MASK | PRR_SPI_MASK | PRR_TIMER2_MASK);
  //disable ADC
  ADCSRA |= ADCSRA_ADC_MASK;
  PRR    |= (PRR_ADC_MASK);
  //set the Arduino to idle mode
  SMCR   |= SMCR_IDLE_MODE_MASK;
  //set up Port B Pin 5 and set it to logic LOW
  DDRB   |= (1 << 5);
  PORTB  &= ~(1 << 5);
  
}

void putArduinoToIdle()
{
  //shut down timer0 and timer1
  PRR |= (PRR_TIMER0_MASK | PRR_TIMER1_MASK);
  //set Arduino to idle mode
  SMCR |= SMCR_SLEEP_ENABLE_MASK;
  sleep_cpu();
  SMCR &= ~(SMCR_SLEEP_ENABLE_MASK);
}

 
TResult readPacket(TPacket *packet)
{
    // Reads in data from the serial port and
    // deserializes it.Returns deserialized
    // data in "packet".
    
    char buffer[PACKET_SIZE];
    int len;

    len = readSerial(buffer);

    if(len == 0)
      return PACKET_INCOMPLETE;
    else
      return deserialize(buffer, len, packet);
    
}


void sendStatus()
{
  TPacket statusPacket;
  statusPacket.packetType = PACKET_TYPE_RESPONSE;
  statusPacket.command = RESP_STATUS;
  statusPacket.params[0] = leftForwardTicks;
  statusPacket.params[1] = rightForwardTicks;
  statusPacket.params[2] = leftReverseTicks;
  statusPacket.params[3] = rightReverseTicks;
  statusPacket.params[4] = leftForwardTicksTurns;
  statusPacket.params[5] = rightForwardTicksTurns;
  statusPacket.params[6] = leftReverseTicksTurns;
  statusPacket.params[7] = rightReverseTicksTurns;
  statusPacket.params[8] = forwardDist;
  statusPacket.params[9] = reverseDist; 
  sendResponse(&statusPacket);
  
  // Implement code to send back a packet containing key
  // information like leftTicks, rightTicks, leftRevs, rightRevs
  // forwardDist and reverseDist
  // Use the params array to store this information, and set the
  // packetType and command files accordingly, then use sendResponse
  // to send out the packet. See sendMessage on how to use sendResponse.
  //
}


void sendMessage(const char *message)
{
  // Sends text messages back to the Pi. Useful
  // for debugging.

  TPacket messagePacket;
  messagePacket.packetType=PACKET_TYPE_MESSAGE;
  strncpy(messagePacket.data, message, MAX_STR_LEN);
  sendResponse(&messagePacket);
}

void dbprint(char *format, ...)
{
  va_list args;
  char buffer[128];

  va_start(args, format);
  vsprintf(buffer, format, args);
  sendMessage(buffer);
}

void sendBadPacket()
{
  // Tell the Pi that it sent us a packet with a bad
  // magic number.
  
  TPacket badPacket;
  badPacket.packetType = PACKET_TYPE_ERROR;
  badPacket.command = RESP_BAD_PACKET;
  sendResponse(&badPacket);
  
}

void sendBadChecksum()
{
  // Tell the Pi that it sent us a packet with a bad
  // checksum.
  
  TPacket badChecksum;
  badChecksum.packetType = PACKET_TYPE_ERROR;
  badChecksum.command = RESP_BAD_CHECKSUM;
  sendResponse(&badChecksum);  
}

void sendBadCommand()
{
  // Tell the Pi that we don't understand its
  // command sent to us.
  
  TPacket badCommand;
  badCommand.packetType=PACKET_TYPE_ERROR;
  badCommand.command=RESP_BAD_COMMAND;
  sendResponse(&badCommand);

}

void sendBadResponse()
{
  TPacket badResponse;
  badResponse.packetType = PACKET_TYPE_ERROR;
  badResponse.command = RESP_BAD_RESPONSE;
  sendResponse(&badResponse);
}

void sendOK()
{
  TPacket okPacket;
  okPacket.packetType = PACKET_TYPE_RESPONSE;
  okPacket.command = RESP_OK;
  sendResponse(&okPacket);  
}

void sendResponse(TPacket *packet)
{
  // Takes a packet, serializes it then sends it out
  // over the serial port.
  char buffer[PACKET_SIZE];
  int len;

  len = serialize(buffer, packet, sizeof(TPacket));
  writeSerial(buffer, len);
}


/*
 * Setup and start codes for external interrupts and 
 * pullup resistors.
 * 
 */
// Enable pull up resistors on pins 2 and 3
void enablePullups()
{
  // Use bare-metal to enable the pull-up resistors on pins
  // 2 and 3. These are pins PD2 and PD3 respectively.
  // We set bits 2 and 3 in DDRD to 0 to make them inputs. 
  DDRD &= ~((1 << 3) | (1 << 2));
  PORTD |= (1 << 3) | (1 << 2);
  
}


// Set up the external interrupt pins INT0 and INT1
// for falling edge triggered. Use bare-metal.
void setupEINT()
{
  // Use bare-metal to configure pins 2 and 3 to be
  // falling edge triggered. Remember to enable
  // the INT0 and INT1 interrupts.
  cli();
  EICRA = 0b00001010;
  EIMSK = 0B00000011;
  sei();
}


/*
 * Alex's setup and run codes
 * 
 */

// Clears all our counters
void clearCounters()
{
  leftForwardTicks=0;
  rightForwardTicks=0;
  leftReverseTicks=0;
  rightReverseTicks=0;
  leftForwardTicksTurns=0;
  rightForwardTicksTurns=0;
  leftReverseTicksTurns=0;
  rightReverseTicksTurns=0;
  leftRevs=0;
  rightRevs=0;
  forwardDist=0;
  reverseDist=0; 
}

// Clears one particular counter
void clearOneCounter(int which)
{
  switch(which)
  {
    case 0:
      leftForwardTicks = 0;
      break;

    case 1:
      rightForwardTicks = 0;
      break;

    case 2:
      leftReverseTicks = 0;
      break;

    case 3:
      rightReverseTicks = 0;
      break;

    case 4:
      leftForwardTicksTurns = 0;
      break;

    case 5:
      rightForwardTicksTurns = 0;
      break;

    case 6:
      leftReverseTicksTurns = 0;
      break; 

    case 7:
      rightReverseTicksTurns = 0;
      break;

    case 8:
      forwardDist = 0;
      break;

    case 9:
      reverseDist = 0;
      break;

    case 10:
      adjLeft = 0;
      adjRight = 0;
      break;
  } 
}

void initializeState()
{
  clearCounters();
}

void handleCommand(TPacket *command)
{
  char colour;
  int returnMsg;
  switch(command->command)
  {
    // For movement commands, param[0] = distance, param[1] = speed.
    case COMMAND_FORWARD:
      sendOK();
      forward((float) command->params[0], (float) command->params[1]);
      sendMessage("DNE");
      break;

    case COMMAND_TURN_LEFT:
      sendOK();
      left((float) command->params[0], (float) command->params[1]);
      sendMessage("DNE");
      break;

    case COMMAND_TURN_RIGHT:
      sendOK();
      right((float) command->params[0], (float) command->params[1]);
      sendMessage("DNE");
      break;

    case COMMAND_REVERSE:
      sendOK();
      reverse((float) command->params[0], (float) command->params[1]);
      sendMessage("DNE");
      break;

    case COMMAND_STOP:
      sendOK();
      stop();
      break;

    case COMMAND_GET_STATS:
      sendOK();
      sendStatus();
      break;

    case COMMAND_CLEAR_STATS:
      sendOK();
      clearOneCounter(command -> params[0]);
      break;
    
    case COMMAND_COLOUR_SENSOR:
      sendOK();
      colour = colourValue();
      if (colour == 'G')
      {
        sendMessage("C=G");
      }
      else if (colour == 'N')
      {
        
        sendMessage("NOC");
      }
      else
      {
      sendMessage("C=R");
      }
      break;

    case COMMAND_LIGHT_BAR:
      sendOK();
      returnMsg = lightBar();
      if (returnMsg == 1)
      {
        sendMessage("ONN");
      }
      else
      {
        sendMessage("OFF");
      }
      break;

    case COMMAND_CALIBRATE:
      sendOK();
      calibrateMotors();
      sendMessage("CAL");

        
    default:
      sendBadCommand();
  }
}

void waitForHello()
{
  int exit=0;

  while(!exit)
  {
    TPacket hello;
    TResult result;
    
    do
    {
      result = readPacket(&hello);
    } while (result == PACKET_INCOMPLETE);

    if(result == PACKET_OK)
    {
      if(hello.packetType == PACKET_TYPE_HELLO)
      {
        sendOK();
        exit=1;
      }
      else
        sendBadResponse();
    }
    else
      if(result == PACKET_BAD)
      {
        sendBadPacket();
      }
      else
        if(result == PACKET_CHECKSUM_BAD)
          sendBadChecksum();
  } // !exit
}

void setup() {
  DDRD |= (1 << 7);
  
  
  // put your setup code here, to run once:

  //Compute the diagonal
  cli();
  setupEINT();
  setupSerial();
  startSerial();
  setupTimer();
  setupMotors();
  startMotors();
  stop();
  enablePullups();
  initializeState();
  WDT_off();
//  setupPowerSaving();
  sei();
}

void handlePacket(TPacket *packet)
{
  switch(packet->packetType)
  {
    case PACKET_TYPE_COMMAND:
      handleCommand(packet);
      break;

    case PACKET_TYPE_RESPONSE:
      break;

    case PACKET_TYPE_ERROR:
      break;

    case PACKET_TYPE_MESSAGE:
      break;

    case PACKET_TYPE_HELLO:
      break;
  }
}

void loop() {
 // put your main code here, to run repeatedly:

  TPacket recvPacket; // This holds commands from the Pi


//  putArduinoToIdle();

  TResult result = readPacket(&recvPacket);
  
  if(result == PACKET_OK)
    handlePacket(&recvPacket);
  else
    if(result == PACKET_BAD)
    {
      sendBadPacket();
    }
    else
      if(result == PACKET_CHECKSUM_BAD)
      {
        sendBadChecksum();
      } 
}

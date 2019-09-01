/*

Target CPU: ESP32
UART Emulation with special features for LocoNet (half-duplex network with DCMA) by Hans Tanner. 
See Digitrax LocoNet PE documentation for more information

This library is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 2.1 of the License, or (arxpint your option) any later version.

This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License along with this library; if not, write to the Free Software
Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

*/

#include <IoTT_LocoNetUART.h>


uint8_t pinRx;
uint8_t pinTx;


#define defaultTransmitAttempts 25 //as per LocoNet standard
#define defaultNetworkAccessAttempts 1000 // = 15ms as per LocoNet standard
#define defaultCDBackoffPriority 80 //20 bit times

#define uartrxBufferSize 200
#define uarttxBufferSize 50 //longest possible LocoNet message is 48 bytes, buffer must be more than that

volatile uint8_t currentCDBackoffPriority = defaultCDBackoffPriority; //the current priority, maybe decreased after unsuccessful transmit attempts
volatile uint8_t cdBackoffCounter = defaultCDBackoffPriority;
volatile uint16_t transmitAttemptCounter;
volatile uint16_t networkAccessAttemptCounter;

enum uartModeType : uint8_t {uart_idle=0, uart_receive=1,uart_transmit=2, uart_collision=3};
enum uartRxStateType : uint8_t {waitStartBit=0, waitDataBit=1, waitStopBit=2};
enum uartTxStateType : uint8_t {sendStartBit=0, sendDataBit=1, sendStopBit=2};

volatile uartModeType uartMode = uart_idle;
volatile uartRxStateType uartRxState = waitStartBit;
volatile uartTxStateType uartTxState = sendStartBit;

volatile uint8_t bitCounter = 0;
volatile uint16_t bitMask;

volatile uint8_t rxrdPointer = 0;
volatile uint8_t rxwrPointer = 0;
volatile uint16_t rxBitBuffer = 0;
volatile uint16_t rxBuffer[uartrxBufferSize];
volatile uint16_t commError = 0;

volatile uint8_t txrdPointer = 0;
volatile uint8_t txtmprdPointer = 0;
volatile uint8_t txwrPointer = 0;
volatile uint8_t txBuffer[uarttxBufferSize];

volatile uint8_t lastByte;

volatile uint16_t lastBitTime;
volatile uint8_t  timerSelector;

volatile uint32_t lastCarrierOKTicker = carrierLossNotification;

volatile bool inverseLogic;

int8_t busyLED = -1;

hw_timer_t * timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE inputMux = portMUX_INITIALIZER_UNLOCKED;

/* this interrupt is called everytime LocoNet goes to SPACE=0 state
 * if the UART is idle, it starts receiving at this time by syncronizing the interrupt timer with the external trigger event and setting the status to receive
 * while the UART is not in idle mode, this interrupt still is called at every change from MARK=1 to SPACE=0, but it is only used to activate the LocoNet BUSY LED (if defined)
*/
void IRAM_ATTR _uart_isr()
{
  if (uartMode == uart_idle) //this is only used to trigger transmit and receive modes, which are handled by the timer routine
  {
    timerWrite(timer, 0); //synchronizing the interrupt timer with the external bit start event. First timer interrupt will occur in 15 microseconds from here
    cdBackoffCounter = currentCDBackoffPriority;
    timerSelector = 0x03; //0x03 gives 30us reading offset which is ideal as it places the read moment in the middle of the bit
    uartMode = uart_receive;
    uartRxState = waitStartBit;
    commError = 0;
  }
  if (busyLED >= 0)
	  digitalWrite(busyLED, HIGH); //swith on the BUSY LED, if defined
//  portENTER_CRITICAL_ISR(&inputMux);
//  portEXIT_CRITICAL_ISR(&inputMux);
}

/* the timer interrupt handles LocoNet access, transmit and receiving of data
 * in Idle state, it checks if data needs to be sent. If so, it does a prioritized network access based on CD. Prioritiy is increased one bit time (starting at 20) every 15ms as per LocoNet standard
 * in receiving mode, data is sampled 30 microseconds into the bit (in the middle of the bit). This ensures correct sampling over the entire byte even in case of slight difference in bittiming
 * in transmit mode, the line level is set and then a collision check is performed after 15, 30, 45 and 60 microseconds. This ensures any collision is properly detected (as it will be longer than 15 micros)
 * and gives a good protection against random spikes and instabilities after initially setting the bit level (LocoNet raise/fall times)
 * the service also handles the CD backoff timing and switching off the LocoNet BUSY LED (if defined)
 * in case of a collision, a 15 bit break is sent
*/
static void IRAM_ATTR _uart_isr_clock() //timer interrupt occurs every 15 micros.
{
  uint8_t tmpPointer;	
  uint8_t lineRx = digitalRead(pinRx);
  if (lineRx == (inverseLogic ? 1:0))
    cdBackoffCounter = currentCDBackoffPriority;
  else
    if (cdBackoffCounter > 0) 
      cdBackoffCounter--;
    else
		if (busyLED >= 0)
			digitalWrite(busyLED, LOW);
  switch (uartMode)
  {
    case uart_receive:
      switch (timerSelector & 0x03)
      {
        case 0: //receive incoming bit
          switch (uartRxState)    
          {
            case waitStartBit: 
              if (lineRx == (inverseLogic ? 0:1)) //oops, start bit is not set, interrupt triggered by a spike
              {
                commError |= errorFrame;
                tmpPointer = (rxwrPointer + 1) % uartrxBufferSize;
                rxBuffer[tmpPointer] = (commError<<8);
                rxwrPointer = tmpPointer; //send a zero byte with error information
                commError = 0;
                uartMode = uart_idle;
			  }
			  else //everything as expected, proceed with receiving
			  {
                uartRxState = waitDataBit;
                rxBitBuffer = 0;
                bitMask = 0x0001;
                bitCounter = 0;
			  }
              break; 
            case waitDataBit:
              if (lineRx == (inverseLogic ? 0:1))
                rxBitBuffer |= bitMask;
              bitMask = (bitMask << 1);
              if (bitMask == 0x0100)
                uartRxState = waitStopBit;
              break;
            case waitStopBit:
              if (lineRx == (inverseLogic ? 1:0))
                commError |= errorFrame;
              tmpPointer = (rxwrPointer + 1) % uartrxBufferSize;
              rxBuffer[tmpPointer] = rxBitBuffer + (commError<<8);
              rxwrPointer = tmpPointer;
              uartMode = uart_idle;
              break;   
          }
          break;
      }
      break;
    case uart_idle: //if transmitData is available and is a valid LocoNet message, access network and start sending
      if (txrdPointer != txwrPointer) //we have a need to send a message
      {
        if (txrdPointer == txtmprdPointer) //
        {
          txtmprdPointer = (txrdPointer + 1) % uarttxBufferSize; //set temporary pointer to data to be sent
          transmitAttemptCounter = defaultTransmitAttempts;
          networkAccessAttemptCounter = defaultNetworkAccessAttempts;
          commError = 0;
        }
        if (cdBackoffCounter == 0) //network is available
        {
          if (digitalRead(pinRx) == (inverseLogic ? 0:1))
          {
            digitalWrite(pinTx, (inverseLogic ? 1:0)); //start sending. This must come within 2us after last check
            uartMode = uart_transmit;
            uartTxState = sendStartBit;
            timerSelector = 0; 
          }
        }
        else
          if (networkAccessAttemptCounter > 0)
            networkAccessAttemptCounter--;
          else
            if (transmitAttemptCounter > 0)
            {
              transmitAttemptCounter--;
              networkAccessAttemptCounter = defaultNetworkAccessAttempts;
              if (currentCDBackoffPriority > 40) //10 bittimes, slower than throttle
                currentCDBackoffPriority -= 4; //increase CDBackoff Priority by 1 bittime
            }
            else //now we are out of options, no network access achieved
            {
              //no network access failure, permanently giving up, data is lost
              txwrPointer = txrdPointer;
              commError |= errorTimeout;
            }
      }      
      else 
	    if (cdBackoffCounter == currentCDBackoffPriority) //this means we have carrier loss
        {
		  lastCarrierOKTicker--;
		  if (lastCarrierOKTicker == 0)
		  {
			lastCarrierOKTicker = carrierLossNotification;
			commError |= errorCarrierLoss;
            tmpPointer = (rxwrPointer + 1) % uartrxBufferSize;
            rxBuffer[tmpPointer] = (commError<<8);
            commError = 0;
            rxwrPointer = tmpPointer;
		  }
	    }
	    else
		  lastCarrierOKTicker = carrierLossNotification;
      break;  
    case uart_transmit:
      //check for collisions, stop sending if collision is detected and send break. Not needed for MARK=1, but we do it anyway as it can not create a false collision detection
      if (digitalRead(pinRx) != digitalRead(pinTx)) //collision detected
      {
        digitalWrite(pinTx, (inverseLogic ? 1:0)); //send BREAK
        uartMode = uart_collision;
        bitCounter = 15; //15 bit times
        commError |= errorCollision;
        if (uartTxState == sendStopBit)
          commError |= errorFrame;
        tmpPointer = (rxwrPointer + 1) % uartrxBufferSize;
        rxBuffer[tmpPointer] = txBuffer[txrdPointer] + (commError<<8);
        commError = 0;
        rxwrPointer = tmpPointer;
      }
      else if ((timerSelector & 0x03) == 0)
      {
        //load next bit, send startbit, data bits, stopbit, load next bit, then load next byte until entire message is sent
        switch (uartTxState)
        {
          case sendStartBit: //if this is called, startBit is out, so we go to databits
            bitMask = 0x0001;
            digitalWrite(pinTx, ((bitMask & txBuffer[txtmprdPointer]) == (inverseLogic ? 0:1))); //send first databit, lsb first
            uartTxState = sendDataBit;
            break;
          case sendDataBit:
            bitMask = (bitMask << 1);
            if (bitMask == 0x0100)
            {
              digitalWrite(pinTx, (inverseLogic ? 0:1)); //send stop bit
              uartTxState = sendStopBit;
            }
            else
              digitalWrite(pinTx, ((bitMask & txBuffer[txtmprdPointer]) == (inverseLogic ? 0:1))); //send next databit
            break;
          case sendStopBit: //if this is called, stopBit is out, so we start next byte, if there is one
            txrdPointer = txtmprdPointer;
            tmpPointer = (rxwrPointer + 1) % uartrxBufferSize;
            commError |= msgEcho;
            rxBuffer[tmpPointer] = txBuffer[txrdPointer] + (commError<<8);
            commError = 0;
            rxwrPointer = tmpPointer;
            if (txrdPointer == txwrPointer) //no more data
            {
              currentCDBackoffPriority = defaultCDBackoffPriority; //reset network priority in case it was reduced in this transmit attempt
              uartMode = uart_idle;
            }
            else
            {
              txtmprdPointer = (txrdPointer + 1) % uarttxBufferSize;
              digitalWrite(pinTx, (inverseLogic ? 1:0)); //start sending start bit
              uartTxState = sendStartBit;
            }
            break;
        }
      }
      break;
    case uart_collision: //if collision has been detected, send BREAK (15 bit times SPACE=0, creates framing error on all listening devices)
      switch (timerSelector & 0x03)
      {
        case 0: //called every 60 microseconds (1 bit time) decrement bit counter, until zero. Then, break is done and we go back to idle status
        {
          bitCounter--;
          if (bitCounter == 0)
          {
            digitalWrite(pinTx, (inverseLogic ? 0:1));
            txwrPointer = txrdPointer;
            uartMode = uart_idle;
            commError = 0;
          }
          break;
        }
      }
  }
  timerSelector++; //increment counter. last 2 bits are used to determine sub-state
}

void uart_begin(uint8_t pinRxNum, uint8_t pinTxNum, bool invLogic)
{
	pinRx = pinRxNum;
	pinTx = pinTxNum;
	inverseLogic = invLogic;
	pinMode(pinRx, INPUT_PULLUP);
	pinMode(pinTx, OUTPUT);
	digitalWrite(pinTx, inverseLogic ? 0:1);
	lastCarrierOKTicker = carrierLossNotification;

    attachInterrupt(pinRx, _uart_isr, (inverseLogic ? RISING : FALLING));
    timer = timerBegin(1, 5, true); //prescale to 4MHz, counting up
    timerAttachInterrupt(timer, &_uart_isr_clock, true); 
    timerAlarmWrite(timer, 242, true); //every 60 microseconds for Tx
    timerAlarmEnable(timer);
}

void uart_end()
{
	Serial.println("Stop Interrupts");
	detachInterrupt(pinRx);
    timerAlarmDisable(timer);
}

void uart_setBusyLED(int8_t ledNr)
{
	if (busyLED != -1)
		pinMode(busyLED, INPUT);
	busyLED = ledNr;
	pinMode(ledNr, OUTPUT);
}

bool uart_availableForWrite()
{
	return (txwrPointer == txrdPointer);
}

uint16_t uart_available()
{
	return (rxwrPointer + uartrxBufferSize - rxrdPointer) % uartrxBufferSize;
}

bool uart_carrierOK()
{
  if (uartMode == uart_idle)
    return (cdBackoffCounter < currentCDBackoffPriority);
  else
    return true;
}

uint16_t uart_read() //always check if data is available before calling this function
{
	if (rxrdPointer != rxwrPointer)
	{
		uint8_t temprdPointer = (rxrdPointer + 1) % uartrxBufferSize;	
		uint16_t thisData = rxBuffer[temprdPointer];
		portENTER_CRITICAL_ISR(&inputMux);
		rxrdPointer = temprdPointer;
		portEXIT_CRITICAL_ISR(&inputMux);
		return thisData;
	}
	else
		return 0;
}

uint8_t uart_write(uint8_t * dataByte, uint8_t numBytes)
{
	uint8_t tempwrPointer = txwrPointer;
	uint8_t i;
	for (i=0; i < numBytes; i++)
	{
		tempwrPointer = (tempwrPointer + 1) % uarttxBufferSize;
		if (tempwrPointer == txwrPointer) //buffer overflow protection
			break;
		txBuffer[tempwrPointer] = dataByte[i];
	}
	if (i == numBytes)
	{
//		portENTER_CRITICAL_ISR(&inputMux);
		txwrPointer = tempwrPointer;
//		portEXIT_CRITICAL_ISR(&inputMux);
		return i;		
	}
	else
		return 0;
}

void uart_flush()
{
	rxrdPointer = rxwrPointer;
	txrdPointer = txwrPointer;
}

uint8_t  uart_LocoNetAvailable()
{
	if (digitalRead(pinRx) == (inverseLogic ? 1 : 0))
		return lnBusy;
	else
		if (cdBackoffCounter == 0)
		{
			return lnNetAvailable;
		}
		else
			return lnAwaitBackoff;
}


	

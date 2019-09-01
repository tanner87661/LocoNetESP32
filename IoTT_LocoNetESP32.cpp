/*

SoftwareSerial.cpp - Implementation of the Arduino software serial for ESP8266.
Copyright (c) 2015-2016 Peter Lerup. All rights reserved.

Adaptation to LocoNet (half-duplex network with DCMA) by Hans Tanner. 
See Digitrax LocoNet PE documentation for more information

This library is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 2.1 of the License, or (at your option) any later version.

This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License along with this library; if not, write to the Free Software
Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

*/

#include <IoTT_LocoNetESP32.h>

#define cdBackOffDelay 20  //20 bit-time Backoff Delay per LocoNet Standard

LocoNetESPSerial::LocoNetESPSerial(int receivePin, int transmitPin, bool inverse_logic)
{
   m_invert = inverse_logic;
   m_rxPin = receivePin;
   m_txPin = transmitPin;

   que_rdPos = que_wrPos = 0;
   receiveMode = true;
   transmitStatus = 0;
   begin();
 }

LocoNetESPSerial::~LocoNetESPSerial() {
	
}

void LocoNetESPSerial::begin() {

   uart_begin(m_rxPin, m_txPin, m_invert);
}

void LocoNetESPSerial::setBusyLED(int8_t ledNr)
{
	uart_setBusyLED(ledNr);
}

uint16_t LocoNetESPSerial::lnWriteMsg(lnTransmitMsg txData)
{
    uint8_t hlpQuePtr = (que_wrPos + 1) % queBufferSize;
//    Serial.printf("Read: %i Write: %i Hlp: %i \n", que_rdPos, que_wrPos, hlpQuePtr); 
    if (hlpQuePtr != que_rdPos) //override protection
    {
		transmitQueue[hlpQuePtr].lnMsgSize = txData.lnMsgSize;
		transmitQueue[hlpQuePtr].reqID = txData.reqID;
		transmitQueue[hlpQuePtr].reqRecTime = micros();
		memcpy(transmitQueue[hlpQuePtr].lnData, txData.lnData, txData.lnMsgSize);
		que_wrPos = hlpQuePtr;
		return txData.lnMsgSize;
	}
	else
		return -1;
}

uint16_t LocoNetESPSerial::lnWriteMsg(lnReceiveBuffer txData)
{
    uint8_t hlpQuePtr = (que_wrPos + 1) % queBufferSize;
//    Serial.printf("Read: %i Write: %i Hlp: %i \n", que_rdPos, que_wrPos, hlpQuePtr); 
    if (hlpQuePtr != que_rdPos) //override protection
    {
		transmitQueue[hlpQuePtr].lnMsgSize = txData.lnMsgSize;
		transmitQueue[hlpQuePtr].reqID = txData.reqID;
		transmitQueue[hlpQuePtr].reqRecTime = micros();
		memcpy(transmitQueue[hlpQuePtr].lnData, txData.lnData, txData.lnMsgSize);
		que_wrPos = hlpQuePtr;
		return txData.lnMsgSize;
	}
	else
		return -1;
}

void LocoNetESPSerial::processLNMsg(lnReceiveBuffer * recData)
{
	if (onLocoNetMessage) onLocoNetMessage(recData);
}

void LocoNetESPSerial::handleLNIn(uint8_t inData, uint8_t inFlags)
{
//	Serial.printf("Handle data %i Flags %i\n", inData, inFlags);
  if ((inFlags & (errorCollision | errorTimeout | errorFrame | errorCarrierLoss)) > 0)
  {
	lnInBuffer.errorFlags = inFlags | msgIncomplete | msgXORCheck; 
	if ((inFlags & msgEcho) > 0)
	  lnInBuffer.reqRespTime = micros() - lnInBuffer.reqRecTime;
    lnInBuffer.lnData[lnBufferPtr] = inData;
	lnInBuffer.reqRespTime = 0;
	lnInBuffer.lnMsgSize = lnBufferPtr+1;
	lnBufferPtr = 0;
	lnInBuffer.echoTime = 0;
	processLNMsg(&lnInBuffer);
    bitRecStatus = 0; //awaiting OpCode
	return;  
  }
  if (inData >= 0x80) //OpCode, start of new message
  {
	lnInBuffer.errorFlags = inFlags;
    if (bitRecStatus == 1) //awaiting data bytes but received OpCode
    {
      //incomplete message
      lnInBuffer.errorFlags |= (msgIncomplete | msgXORCheck);
	  if ((inFlags & msgEcho) > 0)
	    lnInBuffer.echoTime = micros() - lnInBuffer.reqRecTime;
      else
	    lnInBuffer.echoTime = 0;
	  lnInBuffer.reqRespTime = 0;
 	  lnInBuffer.lnMsgSize = lnBufferPtr;
 	  lnBufferPtr = 0;
      processLNMsg(&lnInBuffer); //get rid of previous message
    }
    bitRecStatus = 1; //await data bytes
    if ((inFlags & msgEcho) > 0)
    {
		lnInBuffer.reqID = respID;
		lnInBuffer.reqRecTime = respTime;
	}
	else
	{
		lnInBuffer.reqID = 0;
		lnInBuffer.reqRecTime = micros(); 
	}
    lnBufferPtr = 0;
    uint8_t swiByte = (inData & 0x60) >> 5;
    switch (swiByte)
    {
      case 0: lnExpLen  = 2; break;
      case 1: lnExpLen  = 4; break;
      case 2: lnExpLen  = 6; break;
      case 3: lnExpLen  = 0xFF; break;
      default: lnExpLen = 0;
    }
    lnXOR  = inData;
    lnInBuffer.lnData[lnBufferPtr] = inData;
    lnBufferPtr++; 
  }
  else //received regular data byte
  {
    lnInBuffer.errorFlags |= inFlags;
    if (bitRecStatus == 1) //collecting data
    {
      lnInBuffer.lnData[lnBufferPtr] = inData;
      if ((lnBufferPtr == 1) && (lnExpLen == 0xFF))
        lnExpLen  = (inData & 0x007F); //updating expected length for long message
      lnBufferPtr++; 
      if (lnBufferPtr == lnExpLen) //message length received
      {
		lnInBuffer.lnMsgSize = lnBufferPtr;  
  	    if ((lnInBuffer.errorFlags & msgEcho) > 0)
	      lnInBuffer.echoTime = micros() - lnInBuffer.reqRecTime;
	    else
	      lnInBuffer.echoTime = 0;
        if ((lnXOR ^ 0xFF) != inData)
		  lnInBuffer.errorFlags |= msgXORCheck;	
		  
		if (((lnInBuffer.errorFlags & msgEcho) > 0) || (((respOpCode & 0x08) > 0) && ((lnInBuffer.lnData[0]==0xB4) || (lnInBuffer.lnData[0]==0xE7) || (lnInBuffer.lnData[0]==0x81)) && ((lnInBuffer.errorFlags & msgEcho) == 0))) 
		{
			lnInBuffer.reqID = respID;
			lnInBuffer.reqRespTime = micros() - respTime;
		}
		else
		{
			lnInBuffer.reqID = 0;
			lnInBuffer.reqRespTime = 0;
		}
        processLNMsg(&lnInBuffer);
        lnBufferPtr = 0;
        bitRecStatus = 0; //awaiting OpCode
      }  
      else
        lnXOR ^= inData;
    }
    else
    {
      //unexpected data byte while waiting for OpCode
      lnInBuffer.errorFlags |= msgStrayData;
      lnInBuffer.lnMsgSize = 1;
      lnInBuffer.echoTime = 0;
      lnInBuffer.reqID = 0;
      lnInBuffer.reqRecTime = 0;
      lnInBuffer.reqRespTime = micros();
      lnBufferPtr = 0;
      processLNMsg(&lnInBuffer); 
    }
  }    
}

void LocoNetESPSerial::processLoop()
{
	if (receiveMode)
		processLNReceive();
	else
		processLNTransmit();
}

void LocoNetESPSerial::processLNReceive()
{
	lnInBuffer.reqRecTime = 0;	
	while (uart_available() > 0) //empty that input buffer
	{
		uint16_t newData = uart_read();
        handleLNIn((newData & 0x00FF), (newData & 0xFF00) >> 8); //and process incoming bytes
	}
	if ((que_wrPos != que_rdPos) && (uart_LocoNetAvailable() == lnNetAvailable)) //if rxBuffer is empty, load next command, if available
		receiveMode = false;
}

void LocoNetESPSerial::processLNTransmit()
{
	switch (transmitStatus)
	{
		case 0: //not yet started, transfer data and start transmission
		{
			if (!uart_availableForWrite()) //uart only takes one message at a time
			{
				receiveMode = true;
				return;
			}
			//do not set que_rdPos as this would break mode switch from Receive to Transmit. que_rdPos is only updated after successful transmission
			transmitStatus = 1; //in case we have to stop transmission, process can be repeated
			//break; //don't break, just go on and start transmit
		}
		case 1: //
		{
			numRead = 0;
			int hlpQuePtr = (que_rdPos + 1) % queBufferSize;
			numWrite = transmitQueue[hlpQuePtr].lnMsgSize;
			uart_write(&transmitQueue[hlpQuePtr].lnData[0], transmitQueue[hlpQuePtr].lnMsgSize); //send bytes
		    respOpCode = transmitQueue[hlpQuePtr].lnData[0];
		    respTime = micros();
		    respID = transmitQueue[hlpQuePtr].reqID;
			lnInBuffer.reqRecTime = transmitQueue[hlpQuePtr].reqRecTime;
			transmitTime = micros() + (numWrite * 600) + 500000; //set timeout condition, LocoNet not echoing bytes sent. must be > 500ms to allow network access trys and low processing rate
			transmitStatus = 2; //set status for verification of echo bytes
			break;
		}
		case 2: //message sent to buffer. Check for incoming bytes and verify collision status. Return to receive mode when complete or timeout
		{
			//wait for echo bytes, verify against buffer
			while ((uart_available() > 0) && (numRead < numWrite))
			{
				uint16_t newByte = uart_read();
				handleLNIn(newByte & 0x00FF, newByte >> 8);
				numRead++;
			}
			//success, update read pointer, set status and mode
			if ((numRead == numWrite) || (micros() > transmitTime)) //success
			{
				que_rdPos = (que_rdPos + 1) % queBufferSize;
				transmitStatus = 0;
				receiveMode = true;
			}
			break;
		}
	}
}

int LocoNetESPSerial::cdBackoff() 
{
	return uart_LocoNetAvailable();
}	
	
bool LocoNetESPSerial::carrierOK()
{
	return uart_carrierOK();
}

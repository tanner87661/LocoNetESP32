/*
IoTT_LocoNetUART.h

Target CPU: ESP32
UART Emulation with special features for LocoNet (half-duplex network with DCMA) by Hans Tanner. 
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

#ifndef IoTT_LocoNetUART_h
#define IoTT_LocoNetUART_h

#include <inttypes.h>
#include <arduino.h>
#include <IoTTCommDef.h>

#define lnBusy 0
#define lnAwaitBackoff 1
#define lnNetAvailable 2

#define lnMaxMsgSize 48

#define carrierLossNotification 333333 //notify every 5000 ms if carrier is missing

#define lnRespTimeout  500000 //microseconds

void uart_begin(uint8_t pinRxNum, uint8_t pinTxNum, bool invLogic);
void uart_end();
void uart_setBusyLED(int8_t ledNr);
bool uart_availableForWrite();
uint16_t uart_available();
bool uart_carrierOK();
uint16_t uart_read();
uint8_t uart_write(uint8_t * dataByte, uint8_t numBytes=1);
void uart_flush();
uint8_t  uart_LocoNetAvailable();

#endif

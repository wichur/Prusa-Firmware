//uart2.h
#ifndef _UART2_H
#define _UART2_H

#define USART_BAUDRATE  115200
#define MMU_F_CPU       16000000UL
#define BAUD_PRESCALE (((MMU_F_CPU / (USART_BAUDRATE * 8UL))) - 1)
//#define UART_BAUD_SELECT(baudRate,xtalCpu) (((float)(xtalCpu))/(((float)(baudRate))*8.0)-1.0+0.5)
#define ACK           "ACK"
#define NAK           "NAK"
#define BLK           (char)0x2D     // Blank data filler

#include <inttypes.h>
#include <stdio.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include "Arduino.h"

extern volatile unsigned char rxData1, rxData2, rxData3, rxCSUM;
extern volatile bool startRxFlag, confirmedPayload, txNACK;
extern bool pendingACK;

extern char lastTxPayload[3];

extern void uart2_init(void);

extern void uart2_txPayload(unsigned char payload[3]);

#endif //_UART2_H

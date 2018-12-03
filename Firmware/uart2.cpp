//uart2.cpp
#include "uart2.h"

volatile unsigned char readRxBuffer, rxData1, rxData2, rxData3, rxCSUM;
volatile bool startRxFlag = false, confirmedPayload, txNACK;
bool pendingACK = false;

char lastTxPayload[3] = {0, 0, 0};

void uart2_init(void)
{
    //DDRH &=	~0x01;
    //PORTH |= 0x01;
    //rbuf_ini(uart2_ibuf, sizeof(uart2_ibuf) - 4);
    cli();
    UCSR2A = (0 << U2X2); // baudrate multiplier
    UCSR2B = (1 << RXEN2) | (1 << TXEN2) | (0 << UCSZ22); // enable receiver and transmitter
    UCSR2C = (0 << UMSEL21) | (0 << UMSEL20) | (0 << UPM21) |
    (0 << UPM20) | (1 << USBS2) |(1 << UCSZ21) | (1 << UCSZ20); // Use 8-bit character sizes

    UBRR2H = (BAUD_PRESCALE >> 8); // Load upper 8-bits of the baud rate value into the high byte of the UBRR register
    UBRR2L = BAUD_PRESCALE; // Load lower 8-bits of the baud rate value into the low byte of the UBRR register
    
    UCSR2B |= (1 << RXCIE2); // enable rx interrupt
    sei();
}

ISR(USART2_RX_vect)
{
  cli();
  readRxBuffer = UDR2;
  if ((readRxBuffer == 0x7F) && (!startRxFlag)) {// check for start of framing bytes
    startRxFlag = true;
  } else if (startRxFlag == true) {
      if (rxData1 > 0) {
        if (rxData2 > 0) {
          if (rxData3 > 0) {
            if (rxCSUM > 0) {
              if (readRxBuffer == 0x7F) {
                confirmedPayload = true; // set confirm payload bit true for processing my main loop
                printf_P(PSTR("\nUART2 RX PL 0x%2X %2X %2X %2X\n"), rxData1, rxData2, rxData3, rxCSUM);
              } else {
                printf_P(PSTR("\nUART2 RX 0x%2X %2X %2X %2X\n"), rxData1, rxData2, rxData3, rxCSUM);
                startRxFlag = false;
                rxData1 = 0;
                rxData2 = 0;
                rxData3 = 0;
                rxCSUM = 0;
                txNACK = true; // **send universal nack here **
              }
            } else {
              rxCSUM = readRxBuffer;
            }
          } else {
            rxData3 = readRxBuffer;
          }
        } else {
          rxData2 = readRxBuffer;
        }
      } else {
        rxData1 = readRxBuffer;
      }
    } //else {
      //startRxFlag = false;
      //rxData1, rxData2, rxData3, rxCSUM = 0; // Clear rx vars
      //txNACK = true; // **send universal nack here **
    //}
  sei();
}
/*
datasheet example

void USART_Transmit( unsigned char data ) {
 //Wait for empty transmit buffer
 while ( !( UCSRnA & (1<<UDREn)) );
 //Put data into buffer, sends the data
 UDRn = data;
}

*/
void uart2_txPayload(unsigned char payload[3])
{
  for (int i = 0; i < 3; i++) lastTxPayload[i] = payload[i];
  uint8_t csum = 0;
  loop_until_bit_is_set(UCSR2A, UDRE2); // Check is UDRE is that and not something with a 2 in it - Do nothing until UDR is ready for more data to be written to it
  UDR2 = 0x7F;                                    // Start byte
  delay(10);
  for (int i = 0; i < 3; i++) {                   // Send data
    loop_until_bit_is_set(UCSR2A, UDRE2); // Check is UDRE is that and not something with a 2 in it - Do nothing until UDR is ready for more data to be written to it
    UDR2 = (0xFF & payload[i]);
    delay(10);
    csum += (0xFF & payload[i]);
  }
  csum = 0x2D; //(csum/3);
  loop_until_bit_is_set(UCSR2A, UDRE2); // Check is UDRE is that and not something with a 2 in it - Do nothing until UDR is ready for more data to be written to it
  UDR2 = (0xFF & csum);                                    // Send Checksum
  delay(10);
  loop_until_bit_is_set(UCSR2A, UDRE2); // Check is UDRE is that and not something with a 2 in it - Do nothing until UDR is ready for more data to be written to it
  UDR2 = 0x7F;                                    // End byte
  delay(10);
  printf_P(PSTR("\nUART2 TX 0x7F %c,%c,%c %2X7F\n"), payload[0], payload[1], payload[2], csum);
  if ((payload != ACK) && (payload != NAK)) pendingACK = true;                                          // Set flag to wait for ACK
}

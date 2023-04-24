#ifndef __UART_H
#define __UART_H

#include <Arduino.h>

#define UART1_RX 19
#define UART1_TX 21

extern HardwareSerial uart1;

void uartInit( int port );


#endif

#include "uart.h"

HardwareSerial uart1(1);

void uartInit( int port )
{
    uart1.begin( port, SERIAL_8N1, UART1_RX, UART1_TX );
}

#include <xc.h>
#include "uart1.h"

#define FCLK 100000000

    // Initialize at the given baud rate
    // using RPF4 for U1Rx
    // and RPF5 for U1Tx
    void uart1_init(uint32_t baudrate) {
//        ANSELA = 0x5;  // turn off analog
//        TRISF = 0x5;   // configure as inputs (not really necessary)
        U1BRG = (FCLK / (baudrate * 16)) - 1;
        U1STAbits.UTXEN = 1;
        U1STAbits.URXEN = 1;
        RPF5R = 0x1;  // connect U1TX to RPF5 (pin 42 "PIC_RXD" on PIC32 --> pin 1 "CAN_H" on J6)
        U1RXR = 0x2;  // connect U1RX to RPF4 (pin 41 "PIC_TXD" on PIC32 --> pin 2 "CAN_L" on J6)
        U1MODEbits.ON = 1; // turn your radio on!
    }
   
    uint8_t uart1_txrdy() {
        return !(U1STAbits.UTXBF);
    }
   
    void uart1_txwrite(uint8_t c) {
        U1TXREG = c;
    }
   
    void uart1_txwrite_str(char *cp) {
        while (*cp != '\0') {
            while (!uart1_txrdy()) ;  // wait for txready!
            uart1_txwrite(*cp);
            cp++;
        }
 
    }
   
    uint8_t uart1_rxrdy() {
        return (U1STAbits.URXDA != 0);
    }
   
    uint8_t uart1_rxread() {
        return (U1RXREG);
    }

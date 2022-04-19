#include <xc.h>
#include "uart1.h"

#define FCLK 100000000

// Initialize at the given baud rate
// using RPF4 for U1Rx
// and RPF5 for U1Tx
void uart1_init(uint32_t baudrate) {
//        ANSELA = 0x5;  // turn off analog
//        TRISF = 0x5;   // configure as inputs (not really necessary)
    TRISD = 0xff;
    U1BRG = (FCLK / (baudrate * 16)) - 1;
    U1STAbits.UTXEN = 1;
    U1STAbits.URXEN = 1;
//    RPF0R = 0x1;  // connect U1TX to RPF0 (pin 56 on PIC / 92 on BOB)
//    U1RXR = 0x3;  // connect U1RX to RPF1 (pin 57 on PIC / 93 on BOB)
    RPF5R = 0x1; // connect U1TX (White) to RPF5 (pin 42 on PIC / 69 on BOB)
    U1RXR = 0x2; // connect U1RX (Green) to RPF4 (pin 41 on PIC / 68 on BOB)
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
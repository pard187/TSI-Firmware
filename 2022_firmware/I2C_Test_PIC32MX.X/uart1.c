#include <xc.h>
#include "uart1.h"

#define FCLK 40000000

    // Initialize at the given baud rate
    // using RPA2 for U1Tx (pin 2)
    // and RPA4 for U1Rx (pin 9))
    void uart1_init(uint32_t baudrate) {
        ANSELA = 0x5;  // turn off analog
        TRISA = 0x5;   // configure as inputs (not really necessary)
        U1BRG = (FCLK / (baudrate * 16)) - 1;
        U1STAbits.UTXEN = 1;
        U1STAbits.URXEN = 1;
        RPA0R = 0x1;  // connect URTX to RPA0 (pin 2)
        U1RXR = 0x0;  // connect U1RX to RPA4 (pin 9)
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

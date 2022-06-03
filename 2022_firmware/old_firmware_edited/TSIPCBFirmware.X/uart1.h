/*
 * File:   uart1.h
 * Author: nestorj
 *
 * Created on September 24, 2019, 1:21 PM
 */

#ifndef UART1_H
#define UART1_H

#ifdef __cplusplus
extern "C" {
#endif
   
#include <inttypes.h>

    // Initialize at the given baud rate
    // using RPA2 for U1Tx (pin 2)
    // and RPA4 for U1Rx (pin 9))
    extern void uart1_init(uint32_t baudrate);
   
    // return true if transmitter can accept another character
    extern uint8_t uart1_txrdy();
   
    // write a character to the transmitter
    void uart1_txwrite(uint8_t c);
   
    // blocking write of a string to the transmitter
    void uart1_txwrite_str(char *c);
   
    // return true if receiver has a character ready for reading
    extern uint8_t uart1_rxrdy();
   
    // read a character from the receiver
    extern uint8_t uart1_rxread();


#ifdef __cplusplus
}
#endif

#endif /* UART1_H */
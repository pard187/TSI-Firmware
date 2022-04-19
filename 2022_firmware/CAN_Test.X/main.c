//////////////////////////////////////////////////// MICROCHIP CODE ////////////////////////////////////////////////////

// Section: Pragmas
/*** DEVCFG0 ***/

#pragma config DEBUG =      OFF
#pragma config JTAGEN =     OFF
#pragma config ICESEL =     ICS_PGx1
#pragma config TRCEN =      OFF
#pragma config BOOTISA =    MIPS32
#pragma config FECCCON =    OFF_UNLOCKED
#pragma config FSLEEP =     OFF
#pragma config DBGPER =     PG_ALL
#pragma config SMCLR =      MCLR_NORM
#pragma config SOSCGAIN =   GAIN_LEVEL_3
#pragma config SOSCBOOST =  ON
#pragma config POSCGAIN =   GAIN_LEVEL_3
#pragma config POSCBOOST =  ON
#pragma config EJTAGBEN =   NORMAL
#pragma config CP =         OFF

/*** DEVCFG1 ***/

#pragma config FNOSC =      SPLL
#pragma config DMTINTV =    WIN_127_128
#pragma config FSOSCEN =    OFF
#pragma config IESO =       OFF
#pragma config POSCMOD =    OFF
#pragma config OSCIOFNC =   OFF
#pragma config FCKSM =      CSECME
#pragma config WDTPS =      PS1048576
#pragma config WDTSPGM =    STOP
#pragma config FWDTEN =     OFF
#pragma config WINDIS =     NORMAL
#pragma config FWDTWINSZ =  WINSZ_25
#pragma config DMTCNT =     DMT31
#pragma config FDMTEN =     OFF
/*** DEVCFG2 ***/

#pragma config FPLLIDIV =   DIV_2
#pragma config FPLLRNG =    RANGE_5_10_MHZ
#pragma config FPLLICLK =   PLL_FRC
#pragma config FPLLMULT =   MUL_50
#pragma config FPLLODIV =   DIV_2
#pragma config UPLLFSEL =   FREQ_24MHZ
/*** DEVCFG3 ***/

#pragma config USERID =     0xffff
#pragma config FMIIEN =     ON
#pragma config FETHIO =     ON
#pragma config PGL1WAY =    ON
#pragma config PMDL1WAY =   ON
#pragma config IOL1WAY =    ON
#pragma config FUSBIDIO =   ON

/*** BF1SEQ0 ***/

#pragma config TSEQ =       0x0000
#pragma config CSEQ =       0xffff


// Section: Included Files
#include <xc.h>
#include "uart1.h"
#include "plib_can1.h"
#include <stddef.h>                     // Defines NULL
#include <stdbool.h>                    // Defines true
#include <stdlib.h>                     // Defines EXIT_FAILURE
#include <string.h>
#include <stdio.h>


//////////////////////////////////////////////////// Loopback CODE ////////////////////////////////////////////////////
//#include <sys/kmem.h>
//
//#define MY_SID 0x146
//
//#define FIFO_0_SIZE 4
//#define FIFO_1_SIZE 2
//#define MB_SIZE 4
//
//// buffer for CAN FIFOs
//static volatile unsigned int fifos[(FIFO_0_SIZE + FIFO_1_SIZE) * MB_SIZE];
//
//int main(){
//    char buffer[100];
//    int to_send = 9;
//    unsigned int* addr;
//    
//    //NU32_Startup();
//    uart1_init(9600);
//    
//    C1CONbits.ON = 1;
//    C1CONbits.REQOP = 4;
//    while(C1CONbits.OPMOD != 4) {;}
//    
//    C1FIFOCON0bits.FSIZE = FIFO_0_SIZE - 1;
//    C1FIFOCON0bits.TXEN = 0;
//    
//    C1FIFOCON1bits.FSIZE = FIFO_1_SIZE - 1;
//    C1FIFOCON1bits.TXEN = 1;
//    C1FIFOBA = KVA_TO_PA(fifos);
//    
//    C1RXM0bits.SID = 0x7FF;
//    
//    C1FLTCON0bits.FSEL0 = 0;
//    C1FLTCON0bits.MSEL0 = 0;
//    C1RXF0bits.SID = MY_SID;
//    C1FLTCON0bits.FLTEN0 = 1;
//    
//    C1CONbits.REQOP = 2;
//    while(C1CONbits.OPMOD != 2){;}
//    
//    while(1){
//        char buf1[100];
//        sprintf(buf1, "Sending: %d", to_send);
//        uart1_txwrite_str(buf1);
//        
//        addr = PA_TO_KVA1(C1FIFOUA1);
//        addr[0] = MY_SID;
//        addr[1] = sizeof(to_send);
//        addr[2] = to_send;
//        
//        C1FIFOCON1SET = 0x2000;
//        C1FIFOCON1bits.TXREQ = 1;
//        
//        while(!C1FIFOINT0bits.RXNEMPTYIF) {;}
//        addr = PA_TO_KVA1(C1FIFOUA0);
//        sprintf(buf1, "Received %d with SID = 0x%x\r\n", addr[2], addr[0]&0x7ff);
//        uart1_txwrite_str(buf1);
//        C1FIFOCON0SET = 0x2000;
//    }
//    
//    return 0;
//}

//////////////////////////////////////////////////// Loopback CODE ////////////////////////////////////////////////////




//////////////////////////////////////////////////// MICROCHIP CODE ////////////////////////////////////////////////////

// Section: Main Entry Point
int main ( void )
{
    uint32_t messageID = 0;
    uint32_t rx_messageID = 0;
    uint8_t message[8];
    uint8_t rx_message[8];
    uint32_t status = 0;
    uint8_t messageLength = 0;
    uint8_t rx_messageLength = 0;
    uint8_t count = 0;
    uint8_t user_input = 0;
    CAN_MSG_RX_ATTRIBUTE msgAttr = CAN_MSG_RX_DATA_FRAME;
    
    uart1_init(9600);
    uart1_txwrite_str("BEST MAIN\r\n");
    
    CAN1_Initialize();
    C1RXR = 0x4; // connect C1RX (Purple) to RPF0 (pin 56 on PIC / 92 on BOB)
    RPF1R = 0xf; // connect C1TX (Blue) to RPF1 (pin 57 on PIC / 93 on BOB)
    
    // Prepare the message to send //
    messageID = 0x469;
    messageLength = 1;
    for (count = 1; count >=1; count--){
        message[count - 1] = count;
    }

    while ( true )
    {
        //////////// Code for Transmitting Message ////////////
        uart1_txwrite_str("Transmitting Message: ");
        if(CAN1_TxFIFOIsFull(0) == true){
            uart1_txwrite_str("FIFO is Full");
        }
        else{
            uart1_txwrite_str("FIFO is NOT Full\n");
        }
        if (CAN1_MessageTransmit(messageID, messageLength, message, 0, CAN_MSG_TX_DATA_FRAME) == true)
        {
            uart1_txwrite_str("Success \r\n");
        }
        else
        {
            uart1_txwrite_str("Failed \r\n");
        }
                    
        //////////// Code for Receiving Message ////////////
//            uart1_txwrite_str(" Waiting for message: \r\n");
////            uint32_t can_filtermask = CAN1_MessageAcceptanceFilterMaskGet(0);
//            
//            while (true)
//            {
//                  char buf[100];
//                  sprintf(buf, "%d\t", can_filtermask);
//                  uart1_txwrite_str(buf);
////                if (CAN1_InterruptGet(1, CAN_FIFO_INTERRUPT_RXNEMPTYIF_MASK))
////                {
////                    uart1_txwrite_str("After INT \r\n");
//                    // Check CAN Status //
//                    status = CAN1_ErrorGet();
//                    uart1_txwrite_str("After Error Status \r\n");
//                    if (status == CAN_ERROR_NONE)
//                    {
//                        memset(rx_message, 0x00, sizeof(rx_message));
//
//                        // Receive New Message //
//                        if (CAN1_MessageReceive(&rx_messageID, &rx_messageLength, rx_message, 0, 1, &msgAttr) == true)
//                        {
//                            uart1_txwrite_str(" New Message Received    \r\n");
//                            status = CAN1_ErrorGet();
//                            if (status == CAN_ERROR_NONE)
//                            {
//                                // Print message to Console //
//                                uint8_t length = rx_messageLength;
//                                char buf1[200];
//                                sprintf(buf1, " Message - ID : 0x%x Length : 0x%x ", (unsigned int) rx_messageID,(unsigned int) rx_messageLength);
//                                uart1_txwrite_str(buf1);
//                                uart1_txwrite_str("Message : ");
//                                while(length)
//                                {
//                                    char buf2[200];
//                                    sprintf(buf2, "0x%x ", rx_message[rx_messageLength - length--]);
//                                    uart1_txwrite_str(buf2);
//                                }
//                                uart1_txwrite_str("\r\n");
//                                break;
//                            }
//                            else
//                            {
//                                uart1_txwrite_str("Error in received message");
//                            }
//                        }
//                        else
//                        {
//                            uart1_txwrite_str("Message Reception Failed \r\n");
//                        }
//                    }
//                    else
//                    {
//                        uart1_txwrite_str("Error in last received message");
//                    }
//            }
//            }
    }

    // Execution should not come here during normal operation //
    return ( EXIT_FAILURE );
}
//////////////////////////////////////////////////// MICROCHIP CODE ////////////////////////////////////////////////////



//////////////////////////////////////////////////// OUR CODE ////////////////////////////////////////////////////
/*
int main(){
    uart1_init(9600);
    uart1_txwrite_str("BEST MAIN\n");
//    char buf[100];
    C1RXR = 0x2; // connect C1RX to RPF5 (pin 42 on PIC / 69 on BOB)
    RPF4R = 0xf; // connect C1TX to RPF4 (pin 41 on PIC / 68 on BOB)
//    uart1_txwrite_str("Here0 ");
    CAN1_Initialize();
    uint32_t id = 0b00001111000;
    uint8_t length = 1;
    uint8_t data = 0xda;
    while(1){
//        uart1_txwrite_str("Here1 ");
        if(CAN1_MessageTransmit(id, 1, &data, 0, CAN_MSG_TX_DATA_FRAME) == true){
            uart1_txwrite_str("Success ");        
        }
        else{
            uart1_txwrite_str("Fail ");
        }
    }
}
*/
//////////////////////////////////////////////////// OUR CODE ////////////////////////////////////////////////////

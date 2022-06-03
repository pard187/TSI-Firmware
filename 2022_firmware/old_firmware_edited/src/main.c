/*******************************************************************************
  MPLAB Harmony Project Main Source File

  Company:
    Microchip Technology Inc.
  
  File Name:
    main.c

  Summary:
    This file contains the "main" function for an MPLAB Harmony project.

  Description:
    This file contains the "main" function for an MPLAB Harmony project.  The
    "main" function calls the "SYS_Initialize" function to initialize the state 
    machines of all MPLAB Harmony modules in the system and it calls the 
    "SYS_Tasks" function from within a system-wide "super" loop to maintain 
    their correct operation. These two functions are implemented in 
    configuration-specific files (usually "system_init.c" and "system_tasks.c")
    in a configuration-specific folder under the "src/system_config" folder 
    within this project's top-level folder.  An MPLAB Harmony project may have
    more than one configuration, each contained within it's own folder under
    the "system_config" folder.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013-2014 released Microchip Technology Inc.  All rights reserved.

//Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
 *******************************************************************************/
// DOM-IGNORE-END


// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include <stddef.h>                     // Defines NULL
#include <stdbool.h>                    // Defines true
#include <stdlib.h>                     // Defines EXIT_FAILURE
#include "system/common/sys_module.h"   // SYS function prototypes
#include <stdio.h>
#include <inttypes.h>
#include <xc.h>

// *****************************************************************************
// *****************************************************************************
// Section: Main Entry Point
// *****************************************************************************
// *****************************************************************************

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

int main ( void )
{
//    /* Initialize all MPLAB Harmony modules, including application(s). */
//    SYS_Initialize ( NULL );
//
//
//    while ( true )
//    {
//        /* Maintain state machines of all polled MPLAB Harmony modules. */
////        PORTEbits.RE2 = 1;
//        SYS_Tasks ( );
//        
//    }
//
//    /* Execution should not come here during normal operation */
//
//    return ( EXIT_FAILURE );
    char *buf_test = "\nTEST Main\n";
    char buf_r0[10];
    uart1_init(9600);
    uart1_txwrite_str(buf_test);
    int MCP23016B_reading;
    uint32_t a1, a2;
    while(1){
//        MCP23016B_reading = get_MCP23016B();
//        sprintf(buf_r0, "0x%x\t",  MCP23016B_reading);
//        uart1_txwrite_str(buf_r0);
        APP_DATAIO_Tasks();
        a1 = get_NCD9830_reading(3);
        a2 = get_NCD9830_reading(4);
        sprintf(buf_r0, "0x%x\t",  a1);
        uart1_txwrite_str(buf_r0);
    }
}


/*******************************************************************************
 End of File
*/


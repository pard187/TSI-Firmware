/*
No idea why this module is doing
*/
#include "app_uart.h"

APP_UART_DATA app_uartData;

void APP_UART_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    app_uartData.state = APP_UART_STATE_INIT;
}

void APP_UART_Tasks ( void )
{

    /* Check the application's current state. */
    switch ( app_uartData.state )
    {
        /* Application's initial state. */
        case APP_UART_STATE_INIT:
        {
            bool appInitialized = true;
            if (appInitialized)
            {            
                app_uartData.state = APP_UART_STATE_SERVICE_TASKS;
            }
            break;
        }

        case APP_UART_STATE_SERVICE_TASKS:
        {
            break;
        }  

        /* The default state should never be executed. */
        default:
        {
            /* TODO: Handle error in application's state machine. */
            break;
        }
    }
}

 

/*******************************************************************************
 End of File
 */

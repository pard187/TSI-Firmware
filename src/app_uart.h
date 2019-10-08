#ifndef _APP_UART_H
#define _APP_UART_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include "system_config.h"
#include "system_definitions.h"

// DOM-IGNORE-BEGIN
#ifdef __cplusplus  // Provide C++ Compatibility

extern "C" {

#endif

typedef enum
{
	/* Application's state machine's initial state. */
	APP_UART_STATE_INIT=0,
	APP_UART_STATE_SERVICE_TASKS,
} APP_UART_STATES;

typedef struct
{
    /* The application's current state */
    APP_UART_STATES state;
} APP_UART_DATA;

void APP_UART_Initialize ( void );

void APP_UART_Tasks( void );


#endif /* _APP_UART_H */

#ifdef __cplusplus
}
#endif


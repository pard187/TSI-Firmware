#ifndef _APP_CAN_H
#define _APP_CAN_H 

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include "system_config.h"
#include "system_definitions.h"

#ifdef __cplusplus  // Provide C++ Compatibility

extern "C" {

#endif

typedef enum
{
	/* Application's state machine's initial state. */
	APP_CAN_STATE_INIT=0,
	APP_CAN_STATE_WAIT,
  APP_CAN_STATE_SEND,
} APP_CAN_STATES;

typedef struct
{
  /* The application's current state */
  APP_CAN_STATES state;
} APP_CAN_DATA;

void APP_CAN_Initialize ( void );

void APP_CAN_Tasks( void );
uint8_t canrecievemessage[8];


#endif /* _APP_CAN_H */

#ifdef __cplusplus
}
#endif


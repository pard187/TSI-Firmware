#ifndef _APP_DATAIO_H
#define _APP_DATAIO_H

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
	APP_DATAIO_STATE_INIT=0,
	APP_DATAIO_STATE_SERVICE_TASKS,
} APP_DATAIO_STATES;

typedef struct
{
    /* The application's current state */
    APP_DATAIO_STATES state;
} APP_DATAIO_DATA;

void APP_DATAIO_Initialize ( void );

void APP_DATAIO_Tasks( void );

#endif /* _APP_DATAIO_H */

#ifdef __cplusplus
}
#endif

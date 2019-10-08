#ifndef _APP_DRV_BTN_H
#define _APP_DRV_BTN_H

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
	APP_DRV_BTN_STATE_INIT=0,
	RELEASED,
  PUSHING,
  RELEASING,
} APP_DRV_BTN_STATES;

typedef struct
{
    /* The application's current state */
    APP_DRV_BTN_STATES state;
} APP_DRV_BTN_DATA;

void APP_DRV_BTN_Initialize ( void );

void APP_DRV_BTN_Tasks( void );

volatile uint32_t driver_btn_timer_flag;
uint32_t drive_btn_pushing;

#endif /* _APP_DRV_BTN_H */

#ifdef __cplusplus
}
#endif


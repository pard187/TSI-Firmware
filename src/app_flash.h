#ifndef _APP_FLASH_H
#define _APP_FLASH_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include "system_config.h"
#include "system_definitions.h"
#include "system_config.h"
#include "driver/driver_common.h"
#include "driver/flash/drv_flash.h"
#include "peripheral/ports/plib_ports.h"
#include "app_can.h"

#ifdef __cplusplus  // Provide C++ Compatibility

extern "C" {

#endif

typedef enum
{
	/* Application's state machine's initial state. */
	APP_FLASH_STATE_INIT=0,
	APP_FLASH_STATE_SERVICE_TASKS,
  APP_FLASH_WRITE_INIT_VALUE,
  APP_FLASH_LISTENING,
  APP_FLASH_WRITE_TO_FLASH,
  APP_FLASH_WAIT,
} APP_FLASH_STATES;

typedef struct
{
    /* The application's current state */
    APP_FLASH_STATES state;
} APP_FLASH_DATA;

void APP_FLASH_Initialize ( void );

void APP_FLASH_Tasks( void );
uint32_t overCurr_toWrite;
uint32_t overCurr_intr_flag;
uint8_t overCurr_val;
volatile uint32_t flash_wait_tmr_flag;

#endif /* _APP_FLASH_H */

#ifdef __cplusplus
}
#endif
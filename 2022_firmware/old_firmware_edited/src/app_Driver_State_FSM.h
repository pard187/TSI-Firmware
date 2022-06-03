//DOM-IGNORE-END

#ifndef _APP_DRIVER_STATE_FSM_H
#define _APP_DRIVER_STATE_FSM_H

#define mov_avg_size 10

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

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
	APP_DRIVER_STATE_FSM_STATE_INIT=0,
	IDLE,
    PRECHARGE,
    DRIVE_SETUP,
    OVERCURRENT,
    DRIVE,
    DRIVE_RTDS,
    DRIVE_RETURN,

} APP_DRIVER_STATE_FSM_STATES;

typedef struct
{
    /* The application's current state */
    APP_DRIVER_STATE_FSM_STATES state;

} APP_DRIVER_STATE_FSM_DATA;

/*FSM conditions*/
//typedef struct {
//    uint8_t buttonPushed;
//    uint8_t brakePressed;
//    uint8_t safetyLoopClosed;
//    uint8_t throttleImplausibility;
//    uint8_t throttleLessThan; // throttle < 0.5v
//    uint8_t prechargeComplete;
//    uint8_t overCurrent;
//    uint8_t airsOpen;
//    uint8_t throttleControl;
//} driver_state_fsm_conditions;

typedef enum {
    // different cases that car exits drive state
    drive_ok = 0x0,
    safety_loop = 0x1,
    over_current_cond = 0x2,
    drivebtn_brake = 0x3,
    throttle_Implausible = 0x4,
    throttle_brake = 0x5,
    mc_not_active = 0x6 
} return_condition;

void APP_DRIVER_STATE_FSM_Initialize ( void );

void APP_DRIVER_STATE_FSM_Tasks( void );

void send_conditions();

int saftyLoopClosed,PC_Ready, DriverButton_Pushed, throttleImplausibility, brakePressed, overCurr, airsOpen, MC_Activate, saftyloop, throttlegreaterthan, throttlepressed;
int BUZZER_CTRL, D_LED_CTRL, Throttle_SEL, Cooling_Relay_CTRL, Spare_LED_CTRL;
uint32_t throttle;
int boardtemp,CoolTemp_1,CoolTemp_2,FlowRate,a1,a2;
void update_value();
int states;
int buzz_flag;
volatile uint32_t send_condition_flag; // triggered by interrupt when timer count reach. Send current state and condition to VSCADA
volatile uint32_t buzz_timer_flag;
volatile uint32_t flow_rate;
int DriveSetUpDriveBtnRelease;

// function declaration
uint32_t moving_avg(uint32_t *last_vals, uint32_t update) {
	int full = 1;
	uint32_t sum = 0;
	uint32_t mov_avg = 0;
	int i = 0;
    // has to hard code this because variable array size not supported by MPLAB complier (XC32, which is not even C99)
    uint32_t data_size = mov_avg_size;
	
//	while (i < data_size) {
//		if (*(last_vals + i) == 0) {
//			*(last_vals + i) = update;
//			sum += *(last_vals + i);
//			full = 0;
//			mov_avg = sum / (i+1);
//			break;
//		} else {
//			sum += *(last_vals + i);
//		}
//		i++;
//	}
//	
//	if (full) {
		sum = 0;
		i = 0;
		while (i < data_size - 1) {
			*(last_vals + i) = *(last_vals + i + 1);
			sum += *(last_vals + i);
			i++;
		}
		*(last_vals + data_size - 1) = update; 
		sum += *(last_vals + data_size - 1);
		mov_avg = sum / data_size;
//	}
	
	return mov_avg;
}


#endif /* _APP_DRIVER_STATE_FSM_H */

//DOM-IGNORE-BEGIN
#ifdef __cplusplus
}
#endif
//DOM-IGNORE-END

/*******************************************************************************
 End of File
 */
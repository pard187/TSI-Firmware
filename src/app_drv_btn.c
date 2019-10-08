/*
This module is the drive button debouncer.
It contains 2 timers for pushing and releasing.
Outputs drive_btn_pushing.
*/
#include "app_drv_btn.h"

APP_DRV_BTN_DATA app_drv_btnData;

uint32_t drive_btn_timer;
uint32_t drive_btn_releasing_timer;

/*
  Function:
    void APP_DRV_BTN_Initialize ( void )

  Remarks:
    See prototype in app_drv_btn.h.
 */

void APP_DRV_BTN_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    app_drv_btnData.state = APP_DRV_BTN_STATE_INIT;
    drive_btn_pushing = 0;
    drive_btn_releasing_timer = 0;
    drive_btn_timer = 0;
}

void APP_DRV_BTN_Tasks ( void )
{
//    PORTBbits.RB4 = drive_btn_pushing;
    /* Check the application's current state. */
    switch ( app_drv_btnData.state )
    {
        /* Application's initial state. */
        case APP_DRV_BTN_STATE_INIT:
        {
            bool appInitialized = true;
            if (appInitialized)
            {    
                app_drv_btnData.state = RELEASED;
            }
            break;
        }

        case RELEASED:
        {
            drive_btn_pushing = 0;
            // flag cleared and PIC32 PIN61 is 0 (activate low)
            if (driver_btn_timer_flag && PORTEbits.RE1 == 0) {
                drive_btn_timer++;
                driver_btn_timer_flag = 0;
                if (drive_btn_timer > 2) {
                    app_drv_btnData.state = PUSHING;
//                    PORTBbits.RB4 = ~PORTBbits.RB4;
                    drive_btn_timer = 0;
                }
            }
            else if (PORTEbits.RE1 == 1) {
                drive_btn_pushing = 0;
                drive_btn_timer = 0;
            }           
            break;
        }
        
        case PUSHING:
        {
            drive_btn_pushing = 1;           
            if (PORTEbits.RE1 == 1) {
                app_drv_btnData.state = RELEASING;
            }    
            break;
        }
        
        case RELEASING:
        {
            drive_btn_pushing = 0;
            
            if (driver_btn_timer_flag) {
                driver_btn_timer_flag = 0;
                drive_btn_releasing_timer++;
                if (drive_btn_releasing_timer > 10) {
                    app_drv_btnData.state = RELEASED;
                    drive_btn_releasing_timer = 0;
                } 
            }           
            break;
        }
        
        default:
        {
            /* TODO: Handle error in application's state machine. */
            break;
        }
    }
}
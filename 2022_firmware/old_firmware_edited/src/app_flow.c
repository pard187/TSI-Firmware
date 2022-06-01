#include "app_flow.h"

#define READ_CORE_TIMER()                 _CP0_GET_COUNT()          // Read the MIPS Core Timer

APP_FLOW_DATA app_flowData;
//uint8_t flow_rate_tmr;
uint16_t flow_rate_tmr_timeout = 2000;
uint16_t flow_avg[mov_avg_size];


void APP_FLOW_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    app_flowData.state = APP_FLOW_STATE_INIT;
    flow_rate_trigger_flag = 0;
    flow_rate_timer_flag = 0;
    flow_rate_tmr = 0;
    flow_rate_raw = 0;
    
}

void APP_FLOW_Tasks ( void )
{

    /* Check the application's current state. */
    switch ( app_flowData.state )
    {
        /* Application's initial state. */
        case APP_FLOW_STATE_INIT:
        {
            bool appInitialized = true;
       
        
            if (appInitialized)
            {
            
                app_flowData.state = APP_FLOW_STATE_SERVICE_TASKS;
            }
            break;
        }

        case APP_FLOW_STATE_SERVICE_TASKS:
        {
            if (flow_rate_timer_flag == 1) {
                flow_rate_timer_flag = 0;
                if (flow_rate_tmr > flow_rate_tmr_timeout) {
                    flow_rate_tmr = 0;
                    flow_rate_raw = 0;
                } else {
                    flow_rate_tmr++;
                }
            }
            
            if (flow_rate_trigger_flag == 1) {
                flow_rate_trigger_flag = 0;
                flow_rate_raw = flow_rate_tmr;
                flow_rate_tmr = 0;
            }
            
                        
            
//            flow_rate_raw = moving_avg(flow_avg, flow_rate_raw);
            break;
        }
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

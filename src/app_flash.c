#include "app_flash.h"

APP_FLASH_DATA app_flashData;
uint8_t result8[4];

//uint32_t ToWrite = 0xBBAA;    
//uint32_t init_flagbytes= 0xAA; 

uint32_t storeflag; // read from flash address to see if the program already store values for overcurrent
uint8_t storeflagbyte;
uint8_t overwriteflagbyte;
uint32_t flash_wait_tmr;

void APP_FLASH_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    app_flashData.state = APP_FLASH_STATE_INIT;
    DRV_HANDLE flashHandle = DRV_FLASH_Open(DRV_FLASH_INDEX_0, intent);
    overCurr_toWrite = 0x5757;
    overCurr_intr_flag = 0;
    flash_wait_tmr = 0;
}

void APP_FLASH_Tasks ( void )
{

    /* Check the application's current state. */
    switch ( app_flashData.state )
    {
        /* Application's initial state. */
        case APP_FLASH_STATE_INIT:
        {
            bool appInitialized = true;
            update_flag_byte_from_flash();
            
            if (appInitialized)
            {
//                if (storeflagbyte != 0xAA) {
//                    app_flashData.state = APP_FLASH_WRITE_INIT_VALUE;
//                }
//                else {
//                    app_flashData.state = APP_FLASH_LISTENING;
//                }
                app_flashData.state = APP_FLASH_LISTENING;
            }
            break;
        }
        
        case APP_FLASH_WRITE_INIT_VALUE: {
            update_flag_byte_from_flash();
            if(!DRV_FLASH_IsBusy(flashHandle)){
                DRV_FLASH_WriteWord(flashHandle, APP_PROGRAM_FLASH_BASE_ADDRESS,overCurr_toWrite);
                 app_flashData.state = APP_FLASH_LISTENING;
            }
            break;
        }
        
        case APP_FLASH_LISTENING: {
            if (overCurr_intr_flag == 1) {
                overCurr_intr_flag = 0;
                DRV_FLASH0_ErasePage(APP_PROGRAM_FLASH_BASE_ADDRESS);
            }
            else if (overCurr_intr_flag == 2) {
                overCurr_intr_flag = 0;
                overCurr_toWrite = canrecievemessage[1];
                DRV_FLASH0_ErasePage(APP_PROGRAM_FLASH_BASE_ADDRESS);
//                if(!DRV_FLASH_IsBusy(flashHandle)){
//                    DRV_FLASH_WriteWord(flashHandle, APP_PROGRAM_FLASH_BASE_ADDRESS,overCurr_toWrite);
//                }
                app_flashData.state = APP_FLASH_WAIT;
            }
            break;
        }
        
        case APP_FLASH_WAIT: 
        {
            // if tmr < 15, stays in WAIT state
            if (flash_wait_tmr_flag) {
                flash_wait_tmr_flag = 0;
                flash_wait_tmr++;
                if (flash_wait_tmr > 15) {
                    app_flashData.state = APP_FLASH_WRITE_TO_FLASH;
                    flash_wait_tmr = 0;
                }
            }
            break;
        }
        
        case APP_FLASH_WRITE_TO_FLASH:
        {
            if(!DRV_FLASH_IsBusy(flashHandle)){
                DRV_FLASH_WriteWord(flashHandle, APP_PROGRAM_FLASH_BASE_ADDRESS,overCurr_toWrite);
                app_flashData.state = APP_FLASH_LISTENING;
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

int update_flag_byte_from_flash() {
    storeflag = *(uint32_t*)(APP_PROGRAM_FLASH_BASE_ADDRESS); // read from flash address to see if the program already store values for overcurrent
    storeflagbyte = (storeflag & 0x0000ff00) >> 8;
    overCurr_val = (storeflag & 0x000000ff);
} 


/*
This module contains drive mode FSM. The FSM contains 8 stages.
It also calls methods to send signals through CANBus.
INIT: Initialization. The FSM will never go back to this stage.
IDLE: GLV off. Goes here when safety loop opens(AMS/IMD/BOT faults, GLV/TS MS open, etc.).
PRE_CHARGE: MC pre-charging.
DRIVE_SETUP: TS energized, not RTDS, MC not activated.
DRIVE_RTDS: MC activated, throttle plausibility checked, brake pressed and drive btn pressed.
DRIVE: GO! brake pressed but drive btn not pressed.
OVER_CURRENT: when there is an overcurrent signal from CANbus, pause and wait
DRIVE_RTN: error
/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    app_driver_state_fsm.c
Forgive me if the comment is not clear, because the guy who originally wrote this code DID NOT HAVE ANY COMMENT.
If you are seeing this, hope you can comment your code in any of your projects. 
*/
  Summary:
    This file contains the source code for the MPLAB Harmony application.
  Description:
    This file contains the source code for the MPLAB Harmony application.  It 
    implements the logic of the application's state machine and it may call 
    API routines of other MPLAB Harmony modules in the system, such as drivers,
    system services, and middleware.  However, it does not call any of the
    system interfaces (such as the "Initialize" and "Tasks" functions) of any of
    the modules in the system or make any assumptions about when those functions
    are called.  That is the responsibility of the configuration-specific system
    files.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013-2014 released Microchip Technology Inc.  All rights reserved.
Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).
You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.
SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
 *******************************************************************************/
// DOM-IGNORE-END


// *****************************************************************************
// *****************************************************************************
// Section: Included Files 
// *****************************************************************************
// *****************************************************************************

#include "app_driver_state_fsm.h"

@@ -24,6 +63,7 @@ uint32_t buzz_timer;
uint32_t buzz_sound_count;
int reading_1;
int reading_2;

uint32_t cur_sen_cntr;
uint32_t cur_sen_val;
uint32_t cur_nor_val;
@@ -40,6 +80,9 @@ void APP_DRIVER_STATE_FSM_Initialize ( void )
    cur_sen_cntr = 0;
    cur_sen_val = 0;
    cur_nor_val = 0;
    /* TODO: Initialize your application's state machine and other
     * parameters.
     */
}


@@ -207,6 +250,10 @@ void APP_DRIVER_STATE_FSM_Tasks ( void )
            }
            break;
        }


        /* TODO: implement your application state machine.*/


        /* The default state should never be executed. */
        default:
@@ -219,60 +266,54 @@ void APP_DRIVER_STATE_FSM_Tasks ( void )


void update_value() {
    // read values from I/O Expander
    // see ThrottlePlausibility page in schematic
    int MCP23016B_reading = get_MCP23016B();

    // pin48
//    PC_Ready = ((MCP23016B_reading & 0x80) == 0x80); 
//    PC_Ready = (PORTDbits.RD9 != 0);
//    PC_Ready = !PORTDbits.RD9;
    PC_Ready = !PORTCbits.RC14;
//    PC_Ready = 1;

    // GPB6
//    throttleImplausibility = !((MCP23016B_reading & 0x3F) == 0x3F);
    throttleImplausibility = ((MCP23016B_reading & 0b01000000) == 0b01000000);

    // updated from CANBus
    overCurr = over_current();

    // name should be safetyLoop?
    // pin4
    saftyloop = !PORTGbits.RG6;
//    saftyloop = get_ADCCh(45)/0x10;

//    DriverButton_Pushed = PORTDbits.RD1;

    // pin 64
    brakePressed = !PORTEbits.RE4;

    // when MC+ > TSV - 10
    MC_Activate = mc_active();
//    MC_Activate = 1;


    // read values from an ADC on board
    // see isolators page in schematic
    reading_mc_11 = get_NCD9830_reading(7);
    reading_mc_22 = get_NCD9830_reading(6);
    a1 = get_NCD9830_reading(3);
    a2 = get_NCD9830_reading(4);

    // TODO: input of get_ADCCh should be analog input port# (e.g. AN1)
    PORTDbits.RD0 = D_LED_CTRL;
    PORTBbits.RB8 = Throttle_SEL;

    PORTDbits.RD11 = Spare_LED_CTRL;

    throttle = get_ADCCh(0)/0x10;

    FlowRate = flow_rate_freq;
//    throttle = get_ADCCh(14)/0x10;

    throttlegreaterthan = throttle > 0x26; // 0.8v
    throttlepressed = throttle > 0x50; // 1.2v

    // updates by interrupt at RPD1 (FlowRate input to PIC32)
    // to change the interrupt input port
    // go to system_config/default/framework/system/ports/src/sys_ports.static.c 
    FlowRate = flow_rate_freq;
    a1 = get_NCD9830_reading(3);
    a2 = get_NCD9830_reading(4);

    cur_sen_cntr++;
    cur_sen_val = cur_sen_val + abs(a2-a1);
    cur_nor_val = cur_sen_val/cur_sen_cntr;

    // should not work. see throttle
    CoolTemp_1 = get_ADCCh(1)/0x10;
    CoolTemp_2 = get_ADCCh(2)/0x10;

    PORTDbits.RD0 = D_LED_CTRL;
    PORTBbits.RB8 = Throttle_SEL;
    PORTDbits.RD11 = Spare_LED_CTRL;
    // for testing
//    PORTBbits.RB3 = DriverButton_Pushed;
}

int over_current() {
@@ -300,7 +341,11 @@ void send_conditions() {
    uint8_t conditions = throttlepressed + (throttleImplausibility*2) + (overCurr*4) + (saftyloop*8) + (drive_btn_pushing*16) + 
                         (brakePressed*32) + (MC_Activate*64) + (PC_Ready*128);

    can_send_bytes(0x555,conditions,reading_mc_11,reading_mc_22,CoolTemp_1,CoolTemp_2,flow_rate_freq,states,cur_nor_val);

    can_send_bytes(0x555,conditions,reading_mc_11,reading_mc_22,CoolTemp_1,CoolTemp_2,flow_rate_freq,states,cur_nor_val);
    cur_sen_cntr = 0;
    cur_sen_val = 0;
    
@@ -311,3 +356,6 @@ void send_conditions() {
//    rand() % 0xFF
//    can_send_bytes(0x400,a1,a2,reading_mc_22,CoolTemp_1,CoolTemp_2,FlowRate,states,rand() % 0xFF);
}
/*******************************************************************************
 End of File
 */
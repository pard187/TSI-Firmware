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

APP_DRIVER_STATE_FSM_DATA app_driver_state_fsmData;
uint32_t reading_mc_11; // MC+
uint32_t reading_mc_22; // TSV
uint32_t send_cond_timer; // timer for sending conditions
uint32_t buzz_timer;
uint32_t buzz_sound_count;
uint32_t heartbeat_timer;
uint32_t red_timer;
uint32_t blue_timer;
uint8_t state_led;
int reading_1;
int reading_2;

// high current variables
uint32_t cur_sen_cntr;
uint32_t cur_sen_val;
uint32_t cur_nor_val;
uint32_t cur_hv_val;

// moving-averaged value
uint32_t mc_voltage_val;
uint32_t ts_voltage_val;

// pwm counter
uint32_t pwm_cntr;

// array to store moving average history
uint32_t hv_avg[mov_avg_size];
uint32_t mc_avg[mov_avg_size];
uint32_t ts_avg[mov_avg_size];

// D_LED_CTRL flash counter
uint32_t flash_cntr;

// initialize return condition enum
return_condition rc = (return_condition)0;

void APP_DRIVER_STATE_FSM_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    app_driver_state_fsmData.state = APP_DRIVER_STATE_FSM_STATE_INIT;
    send_cond_timer = 0;
    buzz_timer = 0;
    buzz_sound_count = 0;
    DriveSetUpDriveBtnRelease = 0;
    
    cur_sen_cntr = 0;
    cur_sen_val = 0;
    cur_nor_val = 0;
    cur_hv_val = 0;
    flash_cntr = 0;
    
    mc_voltage_val = 0;
    ts_voltage_val = 0;
    state_led = 0b000;
    /* TODO: Initialize your application's state machine and other
     * parameters.
     */
}


void APP_DRIVER_STATE_FSM_Tasks ( void )
{
    if (send_condition_flag) {
        send_condition_flag = 0;
        send_cond_timer++;
        update_value();
//        buzz_timer++;
        if (send_cond_timer == 10) {
            send_conditions();
            send_cond_timer = 0;
//            PORTBbits.RB2 = ~PORTBbits.RB2;
        }
//        if (buzz_timer > 10) {
////            uint8_t a = PORTBbits.RB15;
////            PORTBbits.RB15 = ~PORTBbits.RB15;
//            buzz_timer = 0;
//            if (app_driver_state_fsmData.state == DRIVE_RTDS && buzz_sound_count < 6) {
//                buzz_sound_count++;
//                PORTBbits.RB15 = ~PORTBbits.RB15;
//            }
//        }
        
    }
    
    // LED lights up according to state_led bits
//    PORTBbits.RB2 = ((0b00000100 & state_led) == 0b00000100);
//    PORTBbits.RB3 = ((0b00000010 & state_led) == 0b00000010);
//    PORTBbits.RB4 = ((0b00000001 & state_led) == 0b00000001);
//      PORTBbits.RB2 = 1;
//      PORTBbits.RB3 = 1;
//      PORTBbits.RB4 = 1;
    
//    PORTBbits.RB2 = saftyloop;
    
      PORTBbits.RB2 = drive_btn_pushing;
      PORTBbits.RB3 = throttlepressed;
      PORTBbits.RB4 = (throttle < 0xF00);
    
    
    /* Check the application's current state. */
    switch ( app_driver_state_fsmData.state )
    {
        /* Application's initial state. */
        case APP_DRIVER_STATE_FSM_STATE_INIT:
        {
            states = 0x0;
            bool appInitialized = true;
//            BUZZER_CTRL=0;
            D_LED_CTRL=0;
            Throttle_SEL=0;
            Cooling_Relay_CTRL=0;
            Spare_LED_CTRL = 0;
            // initialize return condition to 0
            rc = drive_ok;
            // state led
            state_led = 0b000;
            
            if (appInitialized)
            {            
                app_driver_state_fsmData.state = IDLE;
            }
            break;
        }

        case IDLE:
        {
            states = 0x1;
//            BUZZER_CTRL=0;
            D_LED_CTRL=0;
            Throttle_SEL=1;
            Cooling_Relay_CTRL=0;
            Spare_LED_CTRL = 0;
            
            state_led = 0b001;
            
            if (rc == safety_loop) {
                if (flash_cntr < 800000000) {
                    flash_cntr++;
                } else {
                    flash_cntr = 0;
                }
                D_LED_CTRL = (flash_cntr < 400000000);
            }
            
            if(saftyloop){
                app_driver_state_fsmData.state = PRECHARGE;
            }
            break;
        }
        
        case PRECHARGE:
        {
            states = 0x2;
//            BUZZER_CTRL=0;
            D_LED_CTRL=0;
            rc = drive_ok;
            Throttle_SEL=0;
            Cooling_Relay_CTRL=0;
            Spare_LED_CTRL = 0;
            
            state_led = 0b010;
            
            if(saftyloop == 0){
                rc = safety_loop;
                app_driver_state_fsmData.state = IDLE;
            } else if(PC_Ready){
                app_driver_state_fsmData.state = DRIVE_SETUP;
            }
            break;
        }
        
        case DRIVE_SETUP: 
        {
            update_value();
            states = 0x3;
            Throttle_SEL=0;
            Cooling_Relay_CTRL=1;
//            D_LED_CTRL = 0;
            Spare_LED_CTRL = 0;
//            buzz_sound_count = 0; // reset buzzer sound time counter
            
            state_led = 0b011;
            
            // D_LED blink differently for different return conditions
            if (rc == throttle_Implausible) {
                if (flash_cntr < 400000000) {
                    flash_cntr++;
                } else {
                    flash_cntr = 0;
                }
                D_LED_CTRL = (flash_cntr < 100000000);
            } else if (rc == throttle_brake) {
                if (flash_cntr < 600000000) {
                    flash_cntr++;
                } else {
                    flash_cntr = 0;
                }
                D_LED_CTRL = (flash_cntr < 100000000 | 
                (flash_cntr < 300000000 & flash_cntr > 200000000));
            } else if (rc == mc_not_active) {
                if (flash_cntr < 40000000) {
                    flash_cntr++;
                } else {
                    flash_cntr = 0;
                }
                D_LED_CTRL = (flash_cntr < 20000000);
            }else if (rc == drive_ok) {
                D_LED_CTRL = 0;
            }
            
            if(saftyloop == 0){
                rc = safety_loop;
                app_driver_state_fsmData.state = IDLE;
            } else if(drive_btn_pushing & !throttleImplausibility & brakePressed & MC_Activate & !throttlepressed){
                app_driver_state_fsmData.state = DRIVE_RTDS;
                // and throttle not pressed
            }
            
            break;
        }
        
        
        // use the opamp to do calculation
        // add throttle voltage to be sent
        // send two packet 
        // mc voltage
        // add spare bits in can message
        case DRIVE_RTDS:
        {
            states = 0x4;
            PORTEbits.RE0 = 1;
            // clear return condition
            rc == drive_ok;
            flash_cntr = 0;
            if (buzz_timer_flag) {
                buzz_timer_flag = 0;
                buzz_timer++;
                if (buzz_timer > 10) {
                    buzz_timer = 0;
                    if (buzz_sound_count < 6) {
                        buzz_sound_count++;
                        PORTEbits.RE0 = 1;
                    }
                    //add another state count to 10
                }
            }
            
            state_led = 0b100;
            
            if(saftyloop == 0){
                PORTEbits.RE0 = 0;
                rc = safety_loop;
                app_driver_state_fsmData.state = IDLE;
            }
            else if (buzz_sound_count >= 6) {
                PORTEbits.RE0 = 0;
                if (!drive_btn_pushing && brakePressed && !throttlepressed) {
                    app_driver_state_fsmData.state = DRIVE;
                    // if throttle pressed and brake can't go
                }
            }
            break;
        }
        
        case DRIVE:
        {
            //buzz_sound_count = 0;
            // set ctrl to 0
            states = 0x5;
            D_LED_CTRL=1;
            Throttle_SEL=1;
//            Cooling_Relay_CTRL=1;
//            BUZZER_CTRL=~buzz_flag;
            Spare_LED_CTRL = 1;
            // no need to clear count
            
            state_led = 0b101;
            
            if(saftyloop == 0){
                rc = safety_loop;
                app_driver_state_fsmData.state = IDLE;
            } else if(throttleImplausibility | (throttlepressed & brakePressed) | !MC_Activate | (drive_btn_pushing & brakePressed)){
                // different blinking for different return condition
                if (throttleImplausibility) {
                    rc = throttle_Implausible;
                } else if (throttlepressed & brakePressed) {
                    rc = throttle_brake;
                } else if (!MC_Activate) {
                    rc = mc_not_active;
                } else if (drive_btn_pushing & brakePressed) {
                    rc = drivebtn_brake;
                }              
                
                app_driver_state_fsmData.state = DRIVE_RETURN;
            } else if(overCurr){
                rc = over_current_cond;
                app_driver_state_fsmData.state = OVERCURRENT;
            }
            
            break;
        }
        
        case OVERCURRENT:
        {
            states = 0x6;
            //D_LED_CTRL flash
//            BUZZER_CTRL=0;
//            D_LED_CTRL=1;
            Throttle_SEL=0;
            Cooling_Relay_CTRL=1;
            Spare_LED_CTRL = 0;
            // blink D_LED to indicate state
            if (flash_cntr < 200000000) {
                flash_cntr++;
            } else {
                flash_cntr = 0;
            }
            D_LED_CTRL = (flash_cntr < 100000000);
            
            state_led = 0b110;
            
            if(saftyloop == 0){
                rc = safety_loop;
                app_driver_state_fsmData.state = IDLE;
            } else if(!overCurr & throttlegreaterthan){
                app_driver_state_fsmData.state = DRIVE;
            } else {
                // use 50% pwm to reduce throttle
                if (pwm_cntr < 10000) {
                    pwm_cntr++;
                } else {
                    pwm_cntr = 0;
                }
                Throttle_SEL = (pwm_cntr < 5000);
            }
            break;
        }
        
        case DRIVE_RETURN: 
        {
            // turn off drive LED
            // different frequency to blink for different problem
            states = 0x8;
            
            state_led = 0b111;
            
            // flash to tell the driver to stop pushing drive button
            if (flash_cntr < 1000000000) {
                flash_cntr++;
            } else {
                flash_cntr = 0;
            }
            D_LED_CTRL = (flash_cntr < 100000000 | 
            flash_cntr > 200000000 & flash_cntr < 300000000 |
            flash_cntr > 400000000 & flash_cntr < 500000000);
            
            if(saftyloop == 0){
                rc = safety_loop;
                app_driver_state_fsmData.state = IDLE;
            }
            if (!drive_btn_pushing) {
                app_driver_state_fsmData.state = DRIVE_SETUP;
            }
            
            break;
        }
        
        
        /* TODO: implement your application state machine.*/
        

        /* The default state should never be executed. */
        default:
        {
            /* TODO: Handle error in application's state machine. */
            break;
        }
    }
}


void update_value() {
    // read values from I/O Expander
    // see ThrottlePlausibility page in schematic
    int MCP23016B_reading = get_MCP23016B();
    
    
//    PC_Ready = ((MCP23016B_reading & 0x80) == 0x80); 
//    PC_Ready = (PORTDbits.RD9 != 0);
//    PC_Ready = !PORTDbits.RD9;
    // pin48
    PC_Ready = !PORTCbits.RC14;
//    PC_Ready = 1;
    
//    throttleImplausibility = !((MCP23016B_reading & 0x3F) == 0x3F);
    // GPB6
    throttleImplausibility = ((MCP23016B_reading & 0b01000000) == 0b01000000);
    
    // updated from CANBus
    overCurr = over_current();
    
    // name should be safetyloop
    // pin4
    saftyloop = !PORTGbits.RG6;
//    saftyloop = get_ADCCh(45)/0x10;
    
//    DriverButton_Pushed = PORTDbits.RD1;
    // pin64
    brakePressed = !PORTEbits.RE4;
    // when MC+ > TSV - 10
    MC_Activate = mc_active();
//    MC_Activate = 1;
    
    // read values from an ADC on board
    // see isolators page in schematic
    reading_mc_11 = get_NCD9830_reading(7);
    reading_mc_22 = get_NCD9830_reading(6);
    
    // TODO: input of get_ADCCh should be analog input port# (e.g. AN1)
    PORTDbits.RD0 = D_LED_CTRL;
    PORTBbits.RB8 = Throttle_SEL;

    PORTDbits.RD11 = Spare_LED_CTRL;
    
    throttle = get_ADCCh(0);

    //FlowRate = flow_rate_freq;
//    throttle = get_ADCCh(14)/0x10;
    
    throttlegreaterthan = throttle > 0x26; // 0.8v
    throttlepressed = throttle > 0b010000000000; // 1.2v
    
    // updates by interrupt at RPD1 (FlowRate input to PIC32)
    // to change the interrupt input port
    // go to system_config/default/framework/system/ports/src/sys_ports.static.c
    a1 = get_NCD9830_reading(3);
    a2 = get_NCD9830_reading(4);
    
    cur_sen_cntr++;
    cur_sen_val = cur_sen_val + abs(a2-a1);
//    cur_hv_val = 1000 * cur_sen_val/cur_sen_cntr;
//    cur_hv_val = moving_avg(hv_avg, a1-a2);
    cur_hv_val = abs(a1 - a2);

    cur_nor_val = cur_sen_val/cur_sen_cntr;
    
    // should not work. see throttle
    CoolTemp_1 = get_ADCCh(1)/16;
    CoolTemp_2 = get_ADCCh(2)/16;
//    CoolTemp_1 = get_ADCCh(5);
//    CoolTemp_2 = get_ADCCh(7);
    // for testing
//    PORTBbits.RB3 = DriverButton_Pushed;
}

int over_current() {
    return cur_nor_val >= overCurr_val;
}

int mc_active() {
    
    reading_mc_11 = get_NCD9830_reading(7); // MC+
    reading_mc_22 = get_NCD9830_reading(6); // TSV
	mc_voltage_val = moving_avg(mc_avg, reading_mc_11);
	ts_voltage_val = moving_avg(ts_avg, reading_mc_22);
	
    
/*     if (mc_voltage_sum + reading_mc_11 > mc_voltage_sum) {
//    if (mc_voltage_cntr <= 100) {
        mc_voltage_cntr++;
        mc_voltage_sum += reading_mc_11;
        mc_voltage_val = mc_voltage_sum / mc_voltage_cntr;
    } 
//    else {
//        mc_voltage_cntr = 1;
//        mc_voltage_sum = reading_mc_11;
//    }
    
    
    if (ts_voltage_sum + reading_mc_22 > ts_voltage_sum) {
//    if (ts_voltage_cntr <= 100) {
        ts_voltage_cntr++;
        ts_voltage_sum += reading_mc_22;
        ts_voltage_val = ts_voltage_sum / ts_voltage_cntr;
    } 
//    else {
//        ts_voltage_cntr = 1;
//        ts_voltage_sum = reading_mc_22;
//    } */
    
    
//    if (reading_mc_11 > reading_mc_22) return 1;
//    if (reading_mc_11 > 0) return 1;
    // mc_22 - 10
//    else return 0;
    return 1;
}

int get_pad_voltage() {
    return get_ADCCh(8);
    
}

int return_Throttp() {
    return (int)throttleImplausibility;
}

void send_conditions() {
    uint8_t conditions = throttlepressed + (throttleImplausibility*2) + (overCurr*4) + (saftyloop*8) + (drive_btn_pushing*16) + 
                         (brakePressed*32) + (MC_Activate*64) + (PC_Ready*128);
    
    can_send_bytes(0x555,conditions,mc_voltage_val,ts_voltage_val,CoolTemp_1,CoolTemp_2,flow_rate_raw,states,cur_hv_val);
    can_send_bytes(0x596,0x0A,throttle,0x00,0x00,0x00,0x00,0x00,0x00);
    // send one more frame to a different address, check space availability
    cur_sen_cntr = 0;
    cur_sen_val = 0;
    
    //mc_voltage_val = 0;
    //ts_voltage_val = 0;
    
//    boardtemp = get_ADCCh(9);
//    can_send_bytes(0x300,get_ADCCh(0)/0x10,get_ADCCh(1)/0x10,get_ADCCh(2)/0x10,get_ADCCh(4)/0x10,3,get_ADCCh(9),PC_Ready,rand() % 0xFF);
//    can_send_bytes(0x300,a2,a1,abs(a2),abs(a1),3,3,4,4);
//    can_send_bytes(0x300,states,reading_1,reading_2,reading_mc_11,reading_mc_22,a1,a2,abs(a2-a1));
//    rand() % 0xFF
//    can_send_bytes(0x400,a1,a2,reading_mc_22,CoolTemp_1,CoolTemp_2,FlowRate,states,rand() % 0xFF);
}

/*******************************************************************************
 End of File
 */

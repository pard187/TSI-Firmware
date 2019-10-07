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

Forgive me if the comment is not clear, because the guy who originally wrote this code DID NOT HAVE ANY COMMENT.
If you are seeing this, hope you can comment your code in any of your projects. 
*/

#include "app_driver_state_fsm.h"

APP_DRIVER_STATE_FSM_DATA app_driver_state_fsmData;
uint32_t reading_mc_11; // MC+
uint32_t reading_mc_22; // TSV
uint32_t send_cond_timer; // timer for sending conditions
uint32_t buzz_timer;
uint32_t buzz_sound_count;
int reading_1;
int reading_2;
uint32_t cur_sen_cntr;
uint32_t cur_sen_val;
uint32_t cur_nor_val;


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
            PORTBbits.RB2 = ~PORTBbits.RB2;
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
            Throttle_SEL=0;
            Cooling_Relay_CTRL=0;
            Spare_LED_CTRL = 0;
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
            Throttle_SEL=0;
            Cooling_Relay_CTRL=0;
            Spare_LED_CTRL = 0;
            if(saftyloop == 0){
                app_driver_state_fsmData.state = IDLE;
            } else if(PC_Ready){
                app_driver_state_fsmData.state = DRIVE_SETUP;
            }
            break;
        }
        
        case DRIVE_SETUP: 
        {
            states = 0x3;
            Throttle_SEL=0;
            Cooling_Relay_CTRL=1;
            D_LED_CTRL = 0;
            Spare_LED_CTRL = 0;
//            buzz_sound_count = 0; // reset buzzer sound time counter
            if(saftyloop == 0){
                app_driver_state_fsmData.state = IDLE;
            } else if(drive_btn_pushing & !throttleImplausibility & brakePressed & MC_Activate){
                app_driver_state_fsmData.state = DRIVE_RTDS;
            }
            
            break;
        }
        
        case DRIVE_RTDS:
        {
            states = 0x4;
            PORTEbits.RE0 = 1;
            if (buzz_timer_flag) {
                buzz_timer_flag = 0;
                buzz_timer++;
                if (buzz_timer > 10) {
                    buzz_timer = 0;
                    if (buzz_sound_count < 6) {
                        buzz_sound_count++;
                        PORTEbits.RE0 = 1;
                    }
                }
            }
            if(saftyloop == 0){
                PORTEbits.RE0 = 0;
                app_driver_state_fsmData.state = IDLE;
            }
            else if (buzz_sound_count >= 6) {
                PORTEbits.RE0 = 0;
                if (!drive_btn_pushing && brakePressed) {
                    app_driver_state_fsmData.state = DRIVE;
                }
            }
            break;
        }
        
        case DRIVE:
        {
            buzz_sound_count = 0;
            states = 0x5;
            D_LED_CTRL=1;
            Throttle_SEL=1;
//            Cooling_Relay_CTRL=1;
//            BUZZER_CTRL=~buzz_flag;
            Spare_LED_CTRL = 1;
            if(saftyloop == 0){
                app_driver_state_fsmData.state = IDLE;
            } else if(throttleImplausibility | (throttlepressed & brakePressed) | !MC_Activate | (drive_btn_pushing & brakePressed)){
                app_driver_state_fsmData.state = DRIVE_RETURN;
            } else if(overCurr){
                app_driver_state_fsmData.state = OVERCURRENT;
            }
            
            break;
        }
        
        case OVERCURRENT:
        {
            states = 0x6;
            //D_LED_CTRL flash
//            BUZZER_CTRL=0;
            D_LED_CTRL=0;
            Throttle_SEL=0;
            Cooling_Relay_CTRL=1;
            Spare_LED_CTRL = 0;
            if(saftyloop == 0){
                app_driver_state_fsmData.state = IDLE;
            } else if(!overCurr & throttlegreaterthan){
                app_driver_state_fsmData.state = DRIVE;
            }
            break;
        }
        
        case DRIVE_RETURN: 
        {
            states = 0x8;
            if (!drive_btn_pushing) {
                app_driver_state_fsmData.state = DRIVE_SETUP;
            }
            break;
        }

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
    
    // pin48
    PC_Ready = !PORTCbits.RC14;
    
    // GPB6
    throttleImplausibility = ((MCP23016B_reading & 0b01000000) == 0b01000000);
    
    // updated from CANBus
    overCurr = over_current();
    
    // name should be safetyLoop?
    // pin4
    saftyloop = !PORTGbits.RG6;
    
//    DriverButton_Pushed = PORTDbits.RD1;

    // pin 64
    brakePressed = !PORTEbits.RE4;

    // when MC+ > TSV - 10
    MC_Activate = mc_active();
    

    // read values from an ADC on board
    // see isolators page in schematic
    reading_mc_11 = get_NCD9830_reading(7);
    reading_mc_22 = get_NCD9830_reading(6);
    a1 = get_NCD9830_reading(3);
    a2 = get_NCD9830_reading(4);
    
    // TODO: input of get_ADCCh should be analog input port# (e.g. AN1)
    throttle = get_ADCCh(0)/0x10;
    throttlegreaterthan = throttle > 0x26; // 0.8v
    throttlepressed = throttle > 0x50; // 1.2v
    
    // updates by interrupt at RPD1 (FlowRate input to PIC32)
    // to change the interrupt input port
    // go to system_config/default/framework/system/ports/src/sys_ports.static.c 
    FlowRate = flow_rate_freq;
    
    cur_sen_cntr++;
    cur_sen_val = cur_sen_val + abs(a2-a1);
    cur_nor_val = cur_sen_val/cur_sen_cntr;
    
    // should not work. see throttle
    CoolTemp_1 = get_ADCCh(1)/0x10;
    CoolTemp_2 = get_ADCCh(2)/0x10;

    PORTDbits.RD0 = D_LED_CTRL;
    PORTBbits.RB8 = Throttle_SEL;
    PORTDbits.RD11 = Spare_LED_CTRL;
}

int over_current() {
    return cur_nor_val >= overCurr_val;
}

int mc_active() {
    reading_mc_11 = get_NCD9830_reading(7); // MC+
    reading_mc_22 = get_NCD9830_reading(6); // TSV
    
    if ((reading_mc_11 > (reading_mc_22 - 10) )) return 1;
    else return 0;
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
    
    can_send_bytes(0x555,conditions,reading_mc_11,reading_mc_22,CoolTemp_1,CoolTemp_2,flow_rate_freq,states,cur_nor_val);
    cur_sen_cntr = 0;
    cur_sen_val = 0;
    
//    boardtemp = get_ADCCh(9);
//    can_send_bytes(0x300,get_ADCCh(0)/0x10,get_ADCCh(1)/0x10,get_ADCCh(2)/0x10,get_ADCCh(4)/0x10,3,get_ADCCh(9),PC_Ready,rand() % 0xFF);
//    can_send_bytes(0x300,a2,a1,abs(a2),abs(a1),3,3,4,4);
//    can_send_bytes(0x300,states,reading_1,reading_2,reading_mc_11,reading_mc_22,a1,a2,abs(a2-a1));
//    rand() % 0xFF
//    can_send_bytes(0x400,a1,a2,reading_mc_22,CoolTemp_1,CoolTemp_2,FlowRate,states,rand() % 0xFF);
}

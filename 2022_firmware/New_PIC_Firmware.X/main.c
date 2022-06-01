// PRAGMAS //
/*** DEVCFG0 ***/
#pragma config DEBUG =      OFF
#pragma config JTAGEN =     OFF
#pragma config ICESEL =     ICS_PGx1
#pragma config TRCEN =      OFF
#pragma config BOOTISA =    MIPS32
#pragma config FECCCON =    OFF_UNLOCKED
#pragma config FSLEEP =     OFF
#pragma config DBGPER =     PG_ALL
#pragma config SMCLR =      MCLR_NORM
#pragma config SOSCGAIN =   GAIN_LEVEL_3
#pragma config SOSCBOOST =  ON
#pragma config POSCGAIN =   GAIN_LEVEL_3
#pragma config POSCBOOST =  ON
#pragma config EJTAGBEN =   NORMAL
#pragma config CP =         OFF

/*** DEVCFG1 ***/
#pragma config FNOSC =      SPLL
#pragma config DMTINTV =    WIN_127_128
#pragma config FSOSCEN =    OFF
#pragma config IESO =       OFF
#pragma config POSCMOD =    OFF
#pragma config OSCIOFNC =   OFF
#pragma config FCKSM =      CSECME
#pragma config WDTPS =      PS1048576
#pragma config WDTSPGM =    STOP
#pragma config FWDTEN =     OFF
#pragma config WINDIS =     NORMAL
#pragma config FWDTWINSZ =  WINSZ_25
#pragma config DMTCNT =     DMT31
#pragma config FDMTEN =     OFF

/*** DEVCFG2 ***/
#pragma config FPLLIDIV =   DIV_1
#pragma config FPLLRNG =    RANGE_5_10_MHZ
#pragma config FPLLICLK =   PLL_FRC
#pragma config FPLLMULT =   MUL_50
#pragma config FPLLODIV =   DIV_2
#pragma config UPLLFSEL =   FREQ_24MHZ

/*** DEVCFG3 ***/
#pragma config USERID =     0xffff
#pragma config FMIIEN =     ON
#pragma config FETHIO =     ON
#pragma config PGL1WAY =    ON
#pragma config PMDL1WAY =   ON
#pragma config IOL1WAY =    ON
#pragma config FUSBIDIO =   ON

/*** BF1SEQ0 ***/
#pragma config TSEQ =       0x0000
#pragma config CSEQ =       0xffff



// LIBRARAY AND FILE INCLUDES //
#include <xc.h>
#include <inttypes.h>
#include <stdio.h>
#include "uart1.h"
#include "timer1.h"
#include "i2c4.h"



// NEW TYPE DEFS //
typedef enum {throttle_impl, zero_flowrate} err_type; // define a new error type - not used yet
static enum DRIVE_STATES {IDLE, DRIVE_SETUP, DRIVE_READY, DRIVE, ERROR, COOLDOWN} DRIVE_STATE = IDLE; // define all drive states


// INPUT VARIABLES //
// uint32_t Flowrate_LOGIC; // OLD WAY: RB7 --> 18 (PIC) / 27 (BOB) - DIP1
float Flowrate_ANALOG; // NEW WAY: CH7 on the ADC (U43)
uint32_t Brake_Pressed_uC; // RC12 --> 31 (PIC) / 40 (BOB) - DIP2
uint32_t Safety_Loop; // RC13 --> 47 (PIC) / 74 (BOB) - DIP3
uint32_t Drive_BTN_LOGIC; // RC14 --> 48 (PIC) / 75 (BOB) - DIP4
uint32_t PC_Ready; // RC15 --> 32 (PIC) / 41 (BOB) - DIP5
uint32_t Throttle_PL_LOGIC; // RE6 --> 2 (PIC) / 2 (BOB) - DIP6


// OUTPUT VARIABLES //
uint32_t Cooling_CTRL; // RD0 --> 46 (PIC) / 73 (BOB) - BLUE
uint32_t D_LED_CTRL; // RD1 --> 49 (PIC) / 85 (BOB) - RED
uint32_t Throttle_EN; // RD3 --> 51 (PIC) / 87 (BOB) - GREEN
uint32_t RTDS_CTRL; // RE0 --> 58 (PIC) / 94 (BOB) - YELLOW


// INTERNAL VARIABLES //
uint16_t ta, tb, cnt_cur = 0; // timer variables
uint16_t rst_flash1, rst_flash2, rst_flash3; // reset variables for the different drive button LED flashing seq
uint16_t rst_rtds, rst_dbtn_unpress, rst_dbtn_press, rst_cooldn; // reset variables for other timers
uint32_t cnt_rtds = 20, cnt_dbtn_unpress = 10, cnt_dbtn_press = 3, cnt_cooldn = 30; // count duration for other timers (counts of ~100 ms)
uint32_t done_rtds, done_dbtn_unpress, done_dbtn_press, done_cooldn; // treated as booleans to store the state of each timer
char value; // stores the analog value of flowrate coming straight from the ADC
uint32_t Flowrate_LOGIC, Flowrate_LOGIC_old, Flowrate_CALC = 1; // more variables for processing and calculating flowrate
uint32_t flowrate_low_thresh = 0, flowrate_upp_thresh = 0; // lower and upper threshold for transitions based on flowrate - currently both at zero, can change later
uint16_t ta1 = 0, tb1 = 0, rst_timeout, tb_timeout, cnt_cur1, cnt_timeout = 600; // timer variables specific to the flowrate timeout - timeout at 1 min
uint16_t first_read; // treated as boolean to implement the flowrate logic
uint32_t Drive_BTN_LOGIC_old; // stores the old DRIVE_BTN_LOGIC value for debouncing
char buf[100]; // string buffer for UART printing



// HELPER FUNCTIONS //
// flashes the drive button LED at a pattern specified by cnt_on and cnt_off which are counts of ~100ms
void flash_LED(uint16_t rst_flash, uint16_t cnt_on, uint16_t cnt_off){
    if(D_LED_CTRL == 1 && timer1_cnt_auto(rst_flash, &ta, &tb, &cnt_cur, cnt_on)){
        D_LED_CTRL = 0;
        return;
    }
    else if(D_LED_CTRL == 0 && timer1_cnt_auto(rst_flash, &ta, &tb, &cnt_cur, cnt_off)){
        D_LED_CTRL = 1;
        return;
    }
} // flash_LED()


// calculates "actual" flowrate, returns FLOWRATE_CALC which is zero for no flowrate, non-zero otherwise
uint32_t read_flowrate(){
    if(Flowrate_LOGIC == !Flowrate_LOGIC_old){
        rst_timeout = 1;
        if(!first_read){
            ta1 = timer1_read();
            first_read = 1;
        }
        else{
            tb1 = timer1_read();
            first_read = 0;
        }
    }
    else{
        if(Flowrate_CALC == 0){
            return Flowrate_CALC;
        }
        else{
            tb_timeout = timer1_read();
            ////  uncomment for testing  ////
            //    sprintf(buf, "CNT_CUR1: %d", cnt_cur1);
            //    uart1_txwrite_str(buf);
            if(first_read && timer1_cnt_auto(rst_timeout, &ta1, &tb_timeout, &cnt_cur1, cnt_timeout) || !first_read && timer1_cnt_auto(rst_timeout, &tb1, &tb_timeout, &cnt_cur1, cnt_timeout)){
                Flowrate_CALC = 0;
                return Flowrate_CALC;
            }
            rst_timeout = 0;
        }
    }
    Flowrate_CALC = 7.77*1000.0/timer1_elapsed_ms(ta1, tb1); // approximated based on flowrate sensor datasheet
    return Flowrate_CALC;
} // read_flowrate()


// Sets all input/output and digital/analog registers
void GPIO_setup(){
    // Inputs
    TRISBbits.TRISB7 = 1;
    TRISCbits.TRISC12 = 1;
    TRISCbits.TRISC13 = 1;
    TRISCbits.TRISC14 = 1;
    TRISCbits.TRISC15 = 1;
    ANSELEbits.ANSE6 = 0;
    TRISEbits.TRISE6 = 1;

    // Outputs
    TRISDbits.TRISD0 = 0;
    TRISDbits.TRISD1 = 0;
    TRISDbits.TRISD3 = 0;
    TRISEbits.TRISE0 =  0;
} // GPIO_setup()


// Updates all input and output values
void val_update(){
    // Inputs
    //    Flowrate_LOGIC = PORTBbits.RB7; // OLD WAY: read as digital - inaccurate
    NCD9830_read(U43_NCD9830_ADDRESS, CH7, &value); // NEW WAY: read as analog (via I2C) then convert to digital - works
    Flowrate_ANALOG = (float) (value/255.0) * 5.0;
    Flowrate_LOGIC_old = Flowrate_LOGIC;
    if(Flowrate_ANALOG <= 0.7) Flowrate_LOGIC = 0;
    else Flowrate_LOGIC = 1;
    Flowrate_CALC = read_flowrate(); // Flowrate_CALC is zero for no flow, non-zero otherwise
    Brake_Pressed_uC = PORTCbits.RC12;
    Safety_Loop = PORTCbits.RC13;
    Drive_BTN_LOGIC_old = Drive_BTN_LOGIC;
    Drive_BTN_LOGIC = !PORTCbits.RC14;
    PC_Ready = PORTCbits.RC15;
    Throttle_PL_LOGIC = !PORTEbits.RE6;

    // Outputs
    LATDbits.LATD0 = Cooling_CTRL;
    LATDbits.LATD1 = D_LED_CTRL;
    LATDbits.LATD3 = Throttle_EN;
    LATEbits.LATE0 = RTDS_CTRL;
} // val_upadte()


// Implements Drive States FSM - diagram here: https://drive.google.com/file/d/12cyzbJN2y9wIt-cWi9elMI7VQYl9TgO6/view?usp=sharing
void DRIVE_STATE_FSM(){
    // Drive States Outputs
    switch(DRIVE_STATE){
        case IDLE:
            uart1_txwrite_str("IDLE \t - \t");
            Cooling_CTRL = 0;
            RTDS_CTRL = 0;
            Throttle_EN = 0;
            D_LED_CTRL = 0;
            rst_flash1 = 1;
            break;
        case DRIVE_SETUP:
            uart1_txwrite_str("DRIVE_SETUP \t - \t");
            Cooling_CTRL = 1;
            RTDS_CTRL = 0;
            Throttle_EN = 0;
            rst_rtds = 1;
            rst_flash2 = 1;
            rst_flash3 = 1;
            if(Brake_Pressed_uC){
                flash_LED(rst_flash1, 10, 10);
                rst_flash1 = 0;
            }
            else{
                D_LED_CTRL = 0;
                rst_flash1 = 1;
            }
            break;
        case DRIVE_READY:
            uart1_txwrite_str("DRIVE_READY \t - \t");
            done_rtds = timer1_cnt_auto(rst_rtds, &ta, &tb, &cnt_cur, cnt_rtds);
            rst_rtds = 0;
            Cooling_CTRL = 1;
            RTDS_CTRL = 1;
            Throttle_EN = 1;
            done_dbtn_unpress = 0;
            rst_dbtn_unpress = 1;
            rst_dbtn_press = 1;
            D_LED_CTRL = 1;
            break;
        case DRIVE:
            uart1_txwrite_str("DRIVE \t - \t");
            Cooling_CTRL = 1;
            RTDS_CTRL = 0;
            Throttle_EN = 1;
            D_LED_CTRL = 1;
            rst_flash2 = 1;
            rst_flash3 = 1;
            rst_cooldn = 1;
            if(!done_dbtn_unpress && !Drive_BTN_LOGIC){
                done_dbtn_unpress = timer1_cnt_auto(rst_dbtn_unpress, &ta, &tb, &cnt_cur, cnt_dbtn_unpress);
                rst_dbtn_unpress = 0;
            }
            else if(!done_dbtn_unpress && Drive_BTN_LOGIC){
                done_dbtn_unpress = 0;
                rst_dbtn_unpress = 1;
            }
            if(done_dbtn_unpress && Brake_Pressed_uC && Drive_BTN_LOGIC){
                done_dbtn_press = timer1_cnt_auto(rst_dbtn_press, &ta, &tb, &cnt_cur, cnt_dbtn_press);
                rst_dbtn_press = 0;
            }
            else{
                done_dbtn_press = 0;
                rst_dbtn_press = 1;
            }
            break;
        case ERROR:
            uart1_txwrite_str("\nERROR \t - \t");
            Cooling_CTRL = 1;
            RTDS_CTRL = 0;
            Throttle_EN = 0;
            D_LED_CTRL = 0;
            rst_rtds = 1;
            rst_flash1 = 1;
            if(!Throttle_PL_LOGIC){
                flash_LED(rst_flash2, 2, 2);
                rst_flash2 = 0;
                rst_flash3 = 1;
            }
            else if(Flowrate_CALC <= flowrate_low_thresh){
                flash_LED(rst_flash3, 5, 15);
                rst_flash2 = 1;
                rst_flash3 = 0;
            }
            break;
        case COOLDOWN:
            uart1_txwrite_str("COOLDOWN \t - \t");
            done_cooldn = timer1_cnt_auto(rst_cooldn, &ta, &tb, &cnt_cur, cnt_cooldn);
            rst_cooldn = 0;
            Cooling_CTRL = 1;
            RTDS_CTRL = 0;
            Throttle_EN = 0;
            D_LED_CTRL = 0;
            rst_flash1 = 1;
            break;
        default:
            uart1_txwrite_str("DEFAULT \t - \t");
            Cooling_CTRL = 0;
            RTDS_CTRL = 0;
            Throttle_EN = 0;
            D_LED_CTRL = 0;
            rst_rtds = 1;
            rst_dbtn_unpress = 1;
            rst_dbtn_press = 1;
            rst_cooldn = 1;
            break;
    }

    // Drive States Transitions
    switch(DRIVE_STATE){
        case IDLE:
            if(PC_Ready && Safety_Loop) DRIVE_STATE = DRIVE_SETUP;
            else DRIVE_STATE = IDLE;
            break;
        case DRIVE_SETUP:
            if(!Safety_Loop) DRIVE_STATE = IDLE;
            else if(Brake_Pressed_uC && Drive_BTN_LOGIC) DRIVE_STATE = DRIVE_READY;
            else if(!Throttle_PL_LOGIC || Flowrate_CALC <= flowrate_low_thresh) DRIVE_STATE = ERROR;
            else DRIVE_STATE = DRIVE_SETUP;
            break;
        case DRIVE_READY:
            if(!Safety_Loop) DRIVE_STATE = IDLE;
            else if(done_rtds) DRIVE_STATE = DRIVE;
            else DRIVE_STATE = DRIVE_READY;
            break;
        case DRIVE:
            if(!Safety_Loop) DRIVE_STATE = IDLE;
            else if(!Throttle_PL_LOGIC || Flowrate_CALC <= flowrate_low_thresh) DRIVE_STATE = ERROR;
            else if(done_dbtn_press) DRIVE_STATE = COOLDOWN;
            else DRIVE_STATE = DRIVE;
            break;
        case ERROR:
            if(!Safety_Loop) DRIVE_STATE = IDLE;
            else if(Throttle_PL_LOGIC && Flowrate_CALC > flowrate_upp_thresh) DRIVE_STATE = DRIVE_SETUP;
            else DRIVE_STATE = ERROR;
            break;
        case COOLDOWN:
            if(!Safety_Loop) DRIVE_STATE = IDLE;
            else if(done_cooldn && !Brake_Pressed_uC) DRIVE_STATE = DRIVE_SETUP;
            else DRIVE_STATE = COOLDOWN;
            break;
        default:
            DRIVE_STATE = IDLE;
            break;
    }
} // DRIVE_STATE_FSM()



// MAIN //
void main(){
    uart1_init(9600); // initialize UART1 at baud rate = 9600 bit/sec
    uart1_txwrite_str("\nTEST Main\n");

    timer1_init(); // initialize timer1 - same timer is used to time different tasks
    Setup_I2C(); // initialize I2C for 100 kHz communication by default
    GPIO_setup(); // set up all input and output pins
    ta = timer1_read(); // initialize the first timer reading (before loop)

    while(1){
        val_update(); // update all input and output values
        DRIVE_STATE_FSM(); // write outputs and transition through the FSM

        ////  uncomment to print values via UART  ////
        //    sprintf(buf, "Flowrate_LOGIC_old: %d\t", Flowrate_LOGIC_old);
        //    uart1_txwrite_str(buf);
        //    sprintf(buf, "Flowrate_ANALOG: %f\t", Flowrate_ANALOG);
        //    uart1_txwrite_str(buf);
        //    sprintf(buf, "Flowrate_LOGIC: %d\t", Flowrate_LOGIC);
        //    uart1_txwrite_str(buf);
        //    sprintf(buf, "Flowrate_CALC: %d\n", Flowrate_CALC);
        //    uart1_txwrite_str(buf);
        //    sprintf(buf, "Brake: %d\t", Brake_Pressed_uC);
        //    uart1_txwrite_str(buf);
        //    sprintf(buf, "Safety: %d\t", Safety_Loop);
        //    uart1_txwrite_str(buf);
        //    sprintf(buf, "Drv_Btn: %d\t", Drive_BTN_LOGIC);
        //    uart1_txwrite_str(buf);
        //    sprintf(buf, "PC: %d\t", PC_Ready);
        //    uart1_txwrite_str(buf);
        //    sprintf(buf, "Throttle: %d\r\n", Throttle_PL_LOGIC);
        //    uart1_txwrite_str(buf);
    }
} // main()

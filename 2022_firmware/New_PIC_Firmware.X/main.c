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


#include <xc.h>
#include <inttypes.h>
#include <stdio.h>
#include "uart1.h"
#include "timer1.h"


typedef enum {throttle_impl, zero_flowrate} err_type;

static enum DRIVE_STATES {IDLE, DRIVE_SETUP, DRIVE_READY, DRIVE, ERROR, COOLDOWN} DRIVE_STATE = IDLE;


// Inputs
uint32_t Flowrate_LOGIC; // RB7 --> 18 (PIC) / 27 (BOB) - DIP1
uint32_t Brake_Pressed_uC; // RC12 --> 31 (PIC) / 40 (BOB) - DIP2
//uint32_t Safety_Loop; // RC13 --> 47 (PIC) / 74 (BOB) - DIP3
//uint32_t Drive_BTN_LOGIC; // RC14 --> 48 (PIC) / 75 (BOB) - DIP4
uint32_t Safety_Loop; // RE1 --> 61 (PIC) / 97 (BOB) - DIP3
uint32_t Drive_BTN_LOGIC; // RE2 --> 62 (PIC) / 98 (BOB) - DIP4
uint32_t PC_Ready; // RC15 --> 32 (PIC) / 41 (BOB) - DIP5
//uint32_t Throttle_PL_LOGIC; // RE6 --> 2 (PIC) / 2 (BOB) - DIP6
uint32_t Throttle_PL_LOGIC; // RB8 --> 21 (PIC) / 30 (BOB) - DIP6
  

// Outputs
//uint32_t Cooling_CTRL; // RD0 --> 46 (PIC) / 73 (BOB) - BLUE
//uint32_t D_LED_CTRL; // RD1 --> 49 (PIC) / 85 (BOB) - RED
uint32_t Cooling_CTRL; // RE3 --> 63 (PIC) / 99 (BOB) - BLUE
uint32_t D_LED_CTRL; // RE4 --> 64 (PIC) / 100 (BOB) - RED
uint32_t Throttle_EN; // RD3 --> 51 (PIC) / 87 (BOB) - GREEN
uint32_t RTDS_CTRL; // RE0 --> 58 (PIC) / 94 (BOB) - YELLOW


// Internal Variables
err_type err;
uint16_t ta, tb, cnt_cur = 0;
uint16_t rst_flash1, rst_flash2, rst_flash3;
uint16_t rst_rtds, rst_brake, rst_cooldn;
uint32_t cnt_rtds = 20, cnt_brake = 30, cnt_cooldn = 30;
uint32_t done_rtds, done_brake, done_cooldn;
uint32_t flowrate_low_thresh = 0, flowrate_upp_thresh = 0;
char buf[100];


void flash_LED(uint16_t rst_flash, uint16_t cnt_on, uint16_t cnt_off){ // cnt_on and cnt_off are # of 100ms
    if(D_LED_CTRL == 1 && timer1_cnt_auto(rst_flash, &ta, &tb, &cnt_cur, cnt_on)){
        D_LED_CTRL = 0;
        return;
    }
    else if(D_LED_CTRL == 0 && timer1_cnt_auto(rst_flash, &ta, &tb, &cnt_cur, cnt_off)){
        D_LED_CTRL = 1;
        return;
    }
}

uint16_t ta1 = 0, tb1 = 0, tb_timeout, cnt_cur1, cnt_timeout = 60;
uint32_t Flowrate_LOGIC_old;
uint32_t Flowrate_calc;
uint16_t first_read;

uint32_t read_flowrate(){
    if(Flowrate_LOGIC == !Flowrate_LOGIC_old){
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
        if(Flowrate_calc == 0){
            return Flowrate_calc;
        }
        else{
            tb_timeout = timer1_read();
            if(timer1_cnt_auto(0, &ta1, &tb_timeout, &cnt_cur1, cnt_timeout)){
                Flowrate_calc = 0;
                return Flowrate_calc;
            } 
        }
    }
    Flowrate_calc = 7.77*1000.0/timer1_elapsed_ms(ta1, tb1);
    Flowrate_LOGIC_old = Flowrate_LOGIC;
    return Flowrate_calc;
}

void GPIO_setup(){
    // Inputs
    TRISBbits.TRISB7 = 1;
    TRISCbits.TRISC12 = 1;
//    TRISCbits.TRISC13 = 1;
//    TRISCbits.TRISC14 = 1;
    TRISEbits.TRISE1 = 1;
    TRISEbits.TRISE2 = 1;
    TRISCbits.TRISC15 = 1;
//    TRISEbits.TRISE6 = 1;
    TRISBbits.TRISB8 = 1;
    
    // Outputs
//    TRISDbits.TRISD0 = 0;
//    TRISDbits.TRISD1 = 0;
    TRISEbits.TRISE3 =  0;
    TRISEbits.TRISE4 =  0;    
    TRISDbits.TRISD3 = 0;
    TRISEbits.TRISE0 =  0;
}

void GPIO_update(){
    // Inputs
    Flowrate_LOGIC = PORTBbits.RB7;
    Flowrate_calc = read_flowrate();
    Brake_Pressed_uC = PORTCbits.RC12;
//    Safety_Loop = PORTCbits.RC13;
//    Drive_BTN_LOGIC = PORTCbits.RC14;
    Safety_Loop = PORTEbits.RE1;
    Drive_BTN_LOGIC = PORTEbits.RE2;
    PC_Ready = PORTCbits.RC15;
//    Throttle_PL_LOGIC = PORTEbits.RE6;
    Throttle_PL_LOGIC = PORTBbits.RB8;
    
    // Outputs
//    LATDbits.LATD0 = Cooling_CTRL;
//    LATDbits.LATD1 = D_LED_CTRL;
    LATEbits.LATE3 = Cooling_CTRL;
    LATEbits.LATE4 = D_LED_CTRL;
    LATDbits.LATD3 = Throttle_EN;
    LATEbits.LATE0 = RTDS_CTRL;
}

void DRIVE_STATE_FSM(){
    // drive state outputs
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
            rst_brake = 1;
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
            if(Brake_Pressed_uC && Drive_BTN_LOGIC){
                done_brake = timer1_cnt_auto(rst_brake, &ta, &tb, &cnt_cur, cnt_brake);
                rst_brake = 0;
            }
            else{
                done_brake = 0;
                rst_brake = 1;
            }
            break;
        case ERROR:
            uart1_txwrite_str("ERROR \t - \t");
            Cooling_CTRL = 1;
            RTDS_CTRL = 0;
            Throttle_EN = 0;
            D_LED_CTRL = 0;
            rst_rtds = 1;
            rst_flash1 = 1;
            if(!Throttle_PL_LOGIC){
                flash_LED(rst_flash2, 1, 1);
                rst_flash2 = 0;
                rst_flash3 = 1;
            }
            else if(Flowrate_calc <= flowrate_low_thresh){
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
            rst_brake = 1;
            rst_cooldn = 1;
            break;
    }
    
    // drive state transitions
    switch(DRIVE_STATE){
        case IDLE:
            if(PC_Ready && Safety_Loop) DRIVE_STATE = DRIVE_SETUP;
            else DRIVE_STATE = IDLE;
            break;
        case DRIVE_SETUP:
            if(!Safety_Loop) DRIVE_STATE = IDLE;
            else if(Brake_Pressed_uC && Drive_BTN_LOGIC) DRIVE_STATE = DRIVE_READY;
            else if(!Throttle_PL_LOGIC || Flowrate_calc <= flowrate_low_thresh) DRIVE_STATE = ERROR;
            else DRIVE_STATE = DRIVE_SETUP;
            break;
        case DRIVE_READY:
            if(!Safety_Loop) DRIVE_STATE = IDLE;
            else if(done_rtds) DRIVE_STATE = DRIVE;
            else DRIVE_STATE = DRIVE_READY;
            break;
        case DRIVE:
            if(!Safety_Loop) DRIVE_STATE = IDLE;
            else if(!Throttle_PL_LOGIC || Flowrate_calc <= flowrate_low_thresh) DRIVE_STATE = ERROR;
            else if(done_brake) DRIVE_STATE = COOLDOWN;
            else DRIVE_STATE = DRIVE;
            break;
        case ERROR:
            if(!Safety_Loop) DRIVE_STATE = IDLE;
            else if(Throttle_PL_LOGIC && Flowrate_calc > flowrate_upp_thresh) DRIVE_STATE = DRIVE_SETUP;
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
}


void main(){    
    uart1_init(9600);
    uart1_txwrite_str("\nTEST Main\n");
    
    timer1_init();
    GPIO_setup();
    ta = timer1_read();
    while(1){
        GPIO_update();
        DRIVE_STATE_FSM();
        sprintf(buf, "Flowrate: %d\t", Flowrate_LOGIC);
        uart1_txwrite_str(buf);
        sprintf(buf, "Flowrate_calc: %04d\r\n", Flowrate_calc);
        uart1_txwrite_str(buf);
//        sprintf(buf, "Brake: %d\t", Brake_Pressed_uC);
//        uart1_txwrite_str(buf);
//        sprintf(buf, "Safety: %d\t", Safety_Loop);
//        uart1_txwrite_str(buf);
//        sprintf(buf, "Drv_Btn: %d\t", Drive_BTN_LOGIC);
//        uart1_txwrite_str(buf);
//        sprintf(buf, "PC: %d\t", PC_Ready);
//        uart1_txwrite_str(buf);
//        sprintf(buf, "Throttle: %d\r\n", Throttle_PL_LOGIC);
//        uart1_txwrite_str(buf);

    }
}
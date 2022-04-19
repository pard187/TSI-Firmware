//#pragma config POSCMOD = OFF
//#pragma config FNOSC = SPLL
//#pragma config FPLLICLK = PLL_FRC
//#pragma config FPLLIDIV = DIV_1
//#pragma config FPLLRNG = RANGE_5_10_MHZ
//#pragma config FPLLMULT = MUL_50
//#pragma config FPLLODIV = DIV_2
//#define SYSFREQ (200000000L)


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

#pragma config FPLLIDIV =   DIV_2
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
#include <stdint.h>
#include <stdio.h>

#define FCLK 100000000
#define PRESCALE 256
#define CLOCK_FREQ 100000000
#define SYS_FREQ 100000000


//////////////////////////////////////////////////////////////////// uart1 ////////////////////////////////////////////////////////////////////
// Initialize at the given baud rate
void uart1_init(uint32_t baudrate) {
//        ANSELA = 0x5;  // turn off analog
//        TRISF = 0x5;   // configure as inputs (not really necessary)
    U1BRG = (FCLK / (baudrate * 16)) - 1;
    U1STAbits.UTXEN = 1;
    U1STAbits.URXEN = 1;
    RPF5R = 0x1;  // connect U1TX to RPF5
    U1RXR = 0x2;  // connect U1RX to RPF4
//    RPD11R = 0x1;  // connect U1TX to RPD11
//    U1RXR = 0x3;  // connect U1RX to RPD10
    U1MODEbits.ON = 1; // turn your radio on!
}

uint8_t uart1_txrdy() {
    return !(U1STAbits.UTXBF);
}

void uart1_txwrite(uint8_t c) {
    U1TXREG = c;
}

void uart1_txwrite_str(char *cp) {
    while (*cp != '\0') { 
        while (!uart1_txrdy()) ;  // wait for txready!
        uart1_txwrite(*cp);
        cp++;
    }

}

uint8_t uart1_rxrdy() {
    return (U1STAbits.URXDA != 0);
}

uint8_t uart1_rxread() {
    return (U1RXREG);
}

//////////////////////////////////////////////////////////////////// timer1 ////////////////////////////////////////////////////////////////////
void timer1_init(){
    T1CON = 0x8030; // prescale = 256
    TMR1 = 0;
}

uint16_t timer1_read(){
    return TMR1;
}

uint16_t inline ticks_to_ms(uint16_t dt){
    return ((uint16_t)dt * PRESCALE) / (CLOCK_FREQ/1000);
}

uint16_t timer1_elapsed_ms(uint16_t t1, uint16_t t2){
    if(t2 >= t1) return ticks_to_ms(t2-t1);
    else return ticks_to_ms((uint32_t) (65536 + t2 - t1));
}

uint16_t timer1_cnt(uint16_t *ta, uint16_t *tb, uint16_t *cnt_cur){
    *tb = timer1_read();
    if(timer1_elapsed_ms(*ta, *tb) >= 100){
        *ta = *tb;
        *cnt_cur+=1;
        return *cnt_cur;
    }
}

uint16_t timer1_cnt_auto(uint16_t rst, uint16_t *ta, uint16_t *tb, uint16_t *cnt_cur, uint16_t cnt_des){
    if(rst){
        *ta = timer1_read();
        *cnt_cur = 0;
    }
    *tb = timer1_read();
    if(timer1_elapsed_ms(*ta, *tb) >= 100){
        *ta = *tb;
        *cnt_cur+=1;
    }
    if(*cnt_cur == cnt_des){
        *cnt_cur = 0;
        return 1;
    }
    else{
        return 0;
    }
}

uint32_t cnt = 0;
void init_timer2(int frequency)
{
    T2CON	= 0x0;		// Disable timer 2 when setting it up
    TMR2	= 0;		// Set timer 2 counter to 0
    IEC0bits.T2IE = 1;	// Disable Timer 2 Interrupt
    
    // Set up the period. Period = PBCLK3 frequency, which is SYS_FREQ / 2, divided by the frequency we want and then divided by 8 for our chosen pre-scaler.
    PR2 = SYS_FREQ / 2 / frequency / 8;
	
	// Set up the pre-scaler
    T2CONbits.TCKPS = 0b011; // Pre-scale of 8
	
	IFS0bits.T2IF = 0;	// Clear interrupt flag for timer 2
    IPC2bits.T2IP = 3; 	// Interrupt priority 3
    IPC2bits.T2IS = 1; 	// Sub-priority 1
    IEC0bits.T2IE = 1;	// Enable Timer 2 Interrupt

	// Turn on timer 2
	T2CONbits.TON	= 1;
}


void __attribute__((vector(_TIMER_2_VECTOR), interrupt(ipl3srs), nomips16)) timer2_handler()
{
	IFS0bits.T2IF = 0;	// Clear interrupt flag for timer 2
	cnt++;
}


uint16_t ta1 = 0, tb1 = 0;
uint32_t Flowrate_LOGIC, Flowrate_LOGIC_old;
float Flowrate_calc, t_elap = 0;
char buf[500];
uint16_t first_read;

uint16_t read_flowrate(){
    if(Flowrate_LOGIC == !Flowrate_LOGIC_old){
        if(!first_read){
            ta1 = timer1_read();
            first_read = 1;
        }
        else{
            tb1 = timer1_read();
    //        Flowrate_calc = 7.77/(tb1 - ta1);
            t_elap = timer1_elapsed_ms(ta1, tb1);
            sprintf(buf, "t_elap: %d\t", t_elap);
            uart1_txwrite_str(buf);
            first_read = 0;
        }
    }
    sprintf(buf, "ta1: %d\t", ta1);
    uart1_txwrite_str(buf);
    sprintf(buf, "tb1: %d\t", tb1);
    uart1_txwrite_str(buf);
    Flowrate_LOGIC_old = Flowrate_LOGIC;
    return t_elap; // 7.7*1000./t_elap
}

uint32_t D_LED_CTRL = 1; // RD1
uint16_t ta, tb, cnt_cur = 0, cnt_des = 5;
uint16_t ta1, tb1;

void flash_LED(uint16_t cnt_on, uint16_t cnt_off){ // cnt_on and cnt_off are #cnt of 100ms
    if(D_LED_CTRL == 1 && timer1_cnt_auto(0, &ta, &tb, &cnt_cur, cnt_on)){
        D_LED_CTRL = 0;
        return;
    }
    else if(D_LED_CTRL == 0 && timer1_cnt_auto(0, &ta, &tb, &cnt_cur, cnt_off)){
        D_LED_CTRL = 1;
        return;
    }
}

void main(){

    INTCONbits.MVEC = 1;
    uart1_init(9600);
    timer1_init();
//    PRISS = 0x76543210;

    
    uart1_txwrite_str("GPIO MAIN\n");
//    char buf[100];
//    init_timer2(10);
    TRISBbits.TRISB7 = 1;

//    TRISB = 0x0;
//    LATB = 0xff;
//    TRISC = 0x0;
//    LATC = 0xffff;
//    TRISD = 0x0;
//    LATD = 0xffff;
//    TRISE = 0x0;
//    LATE = 0xff;
//    TRISDbits.TRISD2 = 0;
//    LATDbits.LATD2 = D_LED_CTRL;
    ta = timer1_read();
    ta1 = timer1_read();
    
    while(1){
        Flowrate_LOGIC = PORTBbits.RB7;
        Flowrate_calc = read_flowrate();
        sprintf(buf, "Flowrate: %d\t", Flowrate_LOGIC);
        uart1_txwrite_str(buf);
        sprintf(buf, "Flowrate_calc: %04d\r\n", Flowrate_calc);
        uart1_txwrite_str(buf);
//        if(timer1_cnt_auto(&ta, &tb, &cnt_cur, cnt_des)){
//            LATB = ~LATB;
//        }
//        if(timer1_cnt(&ta, &tb, &cnt_cur) % 5 == 0){
//            uart1_txwrite_str("HERE");
//            LATB = ~LATB;
//        }
//        if(timer1_cnt_auto(&ta, &tb, &cnt_cur, cnt_des)){
//            D_LED_CTRL = !D_LED_CTRL;
//            LATDbits.LATD2 = D_LED_CTRL;
//        }
//        flash_LED(1,1);
//        LATEbits.LATE4 = D_LED_CTRL;
//        flash_LED(10,10);
//        LATEbits.LATE4 = D_LED_CTRL;
//        sprintf(buf, "CNT: %d\r\n", cnt);
//        uart1_txwrite_str(buf);
//        if(cnt % 10 == 0) D_LED_CTRL = !D_LED_CTRL;
//         LATEbits.LATE4 = D_LED_CTRL;
    }

    
//        tb = timer1_read();
//        if(timer1_elapsed_ms(ta, tb) >= 100){
//            ta = tb;
//            cnt++;
//        }
//        if(cnt == 5){
//            LATB = ~LATB;
//            cnt = 0;
//        }
    
//    TRISD = 0x0;
//    LATD = 0x0f;
    
//    TRISDbits.TRISD0 = 0; // STATE_OUT_1
//    TRISDbits.TRISD1 = 0; // STATE_OUT_2
//    TRISDbits.TRISD2 = 0; // STATE_OUT_3
//    TRISDbits.TRISD3 = 0; // STATE_OUT_1

//    PORTBbits.RB3 = 1;    
//    PORTBbits.RB4 = 1;        
//    PORTBbits.RB5 = 1;    

//    PORTDbits.RD0 = 1;    
//    PORTDbits.RD1 = 1;
//    PORTDbits.RD2 = 1;
//    PORTDbits.RD3 = 0;
}
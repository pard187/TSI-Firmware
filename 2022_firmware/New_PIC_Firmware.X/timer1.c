#include "timer1.h"
#include "uart1.h"

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
    if(timer1_elapsed_ms(*ta, *tb) >= 30){
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
#ifndef TIMER1_H
#define TIMER1_H

#ifdef __cplusplus
extern "C" {
#endif
   
#include <xc.h>
#include <stdint.h>
#include <stdio.h>

#define PRESCALE 256
#define CLOCK_FREQ 100000000

extern void timer1_init();

extern uint16_t timer1_read();

extern uint16_t inline ticks_to_ms(uint16_t dt);

extern uint16_t timer1_elapsed_ms(uint16_t t1, uint16_t t2);

extern uint16_t timer1_cnt(uint16_t *ta, uint16_t *tb, uint16_t *cnt_cur);

extern uint16_t timer1_cnt_auto(uint16_t rst, uint16_t *ta, uint16_t *tb, uint16_t *cnt_cur, uint16_t cnt_des);

#ifdef __cplusplus
}
#endif

#endif /* UART1_H */
#ifndef GBEMU_TIM_H
#define GBEMU_TIM_H

#define DIV_ADDR    0xFF04

int init_timer();
void timer_update(int cycles);

#endif
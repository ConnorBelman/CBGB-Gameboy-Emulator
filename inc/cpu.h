#ifndef GBEMU_CPU_H
#define GBEMU_CPU_H

#include <stdint.h>

#define CLOCK_SPEED 4194304

int init_cpu(void);
int execute(void);
uint16_t get_program_counter(void);
void dump_cpu(void);
void setup_interrupt(uint16_t int_addr);
void set_halted(int val);

#endif
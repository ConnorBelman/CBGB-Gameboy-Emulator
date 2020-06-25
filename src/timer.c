#include <stdint.h>
#include "../inc/cpu.h"
#include "../inc/int.h"
#include "../inc/mem.h"
#include "../inc/timer.h"

#define TIMA_ADDR   0xFF05
#define TMA_ADDR    0xFF06
#define TAC_ADDR    0xFF07

// TAC register bit offsets
#define TAC_SPEED 0
#define TAC_START 2

#define CLK_DIV_0 CLOCK_SPEED / 4096
#define CLK_DIV_1 CLOCK_SPEED / 262144
#define CLK_DIV_2 CLOCK_SPEED / 65536
#define CLK_DIV_3 CLOCK_SPEED / 16384

typedef struct {
    int div_clock_cycles;
    int tima_clock_cycles;

    uint8_t *div;
    uint8_t *tima;
    uint8_t *tma;
    uint8_t *tac;
} Timer;

Timer timer;
int clock_dividers[] = {CLK_DIV_0, CLK_DIV_1, CLK_DIV_2, CLK_DIV_3};

int init_timer() {
    timer.div_clock_cycles = 0;
    timer.tima_clock_cycles = 0;
    timer.div = get_addr_ptr(DIV_ADDR);
    timer.tima = get_addr_ptr(TIMA_ADDR);
    timer.tma = get_addr_ptr(TMA_ADDR);
    timer.tac = get_addr_ptr(TAC_ADDR);
    return 0;
}

// Updates the divider timer, which is fixed at 16384 Hz, or 1/256 the CPU clock speed
void update_div(int cycles) {
    timer.div_clock_cycles += cycles;
    if(timer.div_clock_cycles >= CLK_DIV_3) {
        timer.div_clock_cycles = 0;
        *timer.div += 1;
    }
}

// Updates the TIM A timer is it enabled.
// The speed of TIM A is based on the value in the TAC register.
// On overflow, sends an interrupt and sets TIMA register to start value from TMA register.
void update_tima(int cycles) {
    if(*timer.tac & (1 << TAC_START)) {
        timer.tima_clock_cycles += cycles;
        if(timer.tima_clock_cycles >= clock_dividers[*timer.tac & 3]) {
            timer.tima_clock_cycles = 0;
            *timer.tima += 1;
            if( *timer.tima == 0) {
                *timer.tima = *timer.tma;
                send_irq(TIMER_INT);
            }
        }
    }
}

void timer_update(int cycles) {
    update_div(cycles);
    update_tima(cycles);
}
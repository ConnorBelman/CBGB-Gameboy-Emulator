#include "../inc/cpu.h"
#include "../inc/mem.h"

#define VBLANK_ADDR 0x0040
#define LCDC_ADDR   0x0048
#define TIMER_ADDR  0x0050
#define SERIAL_ADDR 0x0058
#define HI_LO_ADDR  0x0060

#define IF_ADDR     0xFF0F
#define IE_ADDR     0xFFFF

#define NUM_INTS    5

typedef struct {
    uint8_t *ie_reg;
    uint8_t *if_reg;
    int ime;
} Interrupt_Info;

Interrupt_Info int_info;

int init_interrupt_info() {
    int_info.ie_reg = get_addr_ptr(IE_ADDR);
    int_info.if_reg = get_addr_ptr(IF_ADDR);
    int_info.ime = 0;
    return 0;
}

int get_ime() {
    return int_info.ime;
}

void set_ime(int val) {
    int_info.ime = val;
}

// Checks for pending interrupts.
// If there is a pending interrupt and interrupts are enabled, sets up the interrupt.
// Otherwise, un-halts the CPU if it is halted.
int handle_interrupts() {
    if(int_info.ime > 1) {
        int_info.ime = int_info.ime >> 1;
        return 0;
    }

    int i;
    if(int_info.ime == 1) {
        for(i = 0; i < NUM_INTS; i++) {
            if((*int_info.ie_reg & (1 << 0)) && (*int_info.if_reg & (1<< 0))) {
                int_info.ime = 0;
                *int_info.if_reg &= ~(1 << i);
                setup_interrupt(0x40 + (8 * i));
                set_halted(0);
                return 12;
            }
        }
    }
    else {
        for(i = 0; i < NUM_INTS; i++) {
            if((*int_info.ie_reg & (1 << 0)) && (*int_info.if_reg & (1<< 0))) {
                *int_info.if_reg &= ~(1 << i);
                set_halted(0);
            }
        }
    }
    return 0;
}

// Enables an interrupt request for a given interrupt
void send_irq(uint8_t irq) {
    *int_info.if_reg |= irq;
}
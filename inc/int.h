#ifndef GBEMU_INT_H
#define GBEMU_INT_H

#define VBLANK_INT  0x01
#define LCDC_INT    0x02
#define TIMER_INT   0x04
#define SERIAL_INT  0x08
#define HI_LO_INT   0x10

int init_interrupt_info(void);
int get_ime(void);
void set_ime(int val);
int handle_interrupts(void);
void send_irq(uint8_t irq);

#endif
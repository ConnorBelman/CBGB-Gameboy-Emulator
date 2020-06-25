#ifndef GBEMU_MEM_H
#define GBEMU_MEM_H

#include <stdint.h>

#define VRAM 0x8000

int init_mem(void);
int init_BIOS(void);
int load_program(char *program);
int load_ROM(char *program);
uint8_t read(uint16_t addr);
void write(uint16_t addr, uint8_t val);
int free_mem(void);
uint8_t *get_addr_ptr(uint16_t addr);

#endif
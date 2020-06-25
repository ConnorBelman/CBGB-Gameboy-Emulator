#ifndef GBEMU_DMA_H
#define GBEMU_DMA_H

#include <stdint.h>

#define DMA_ADDR 0xFF46

int init_dma(void);
void start_dma_transfer(uint8_t addrH);
void dma_update(int cycles);

#endif
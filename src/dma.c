#include <stdint.h>
#include "../inc/dma.h"
#include "../inc/gpu.h"
#include "../inc/mem.h"

#define DMA_CYCLES 640

typedef struct {
    uint16_t srcAddr;
    int clock;
    int transfering;
} Dma;

Dma dma;

int init_dma() {
    dma.srcAddr = 0;
    dma.clock = 0;
    dma.transfering = 0;
    return 0;
}

// Sets the DMA to start counting cycles and sets address to copy from
void start_dma_transfer(uint8_t addrH) {
    dma.srcAddr = addrH << 8;
    dma.transfering = 1;
}

// Copies OAM_SIZE bytes from the source address to OAM once DMA_CYCLES cycles
// have passed since transfer started
void dma_update(int cycles) {
    if(dma.transfering) {
        dma.clock += cycles;
        if(dma.clock >= DMA_CYCLES) {
            dma.transfering = 0;
            dma.clock = 0;
            int i;

            for(i = 0; i < OAM_SIZE; i++) {
                write(OAM_ADDR + i, read(dma.srcAddr + i));
            }
        }
    }
}


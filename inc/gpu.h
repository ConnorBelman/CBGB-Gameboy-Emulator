#ifndef GBEMU_GPU_H
#define GBEMU_GPU_H

#define OAM_ADDR 0xFE00
#define OAM_SIZE 0xA0

int init_gpu();
void gpu_update(int);

#endif
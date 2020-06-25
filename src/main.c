#include <SDL2/SDL.h>
#include <stdio.h>
#include <stdint.h>
#include "../inc/cpu.h"
#include "../inc/dma.h"
#include "../inc/gpu.h"
#include "../inc/mem.h"
#include "../inc/int.h"
#include "../inc/screen.h"
#include "../inc/timer.h"
#include "../inc/joypad.h"

#define POLL_INTERVAL 60

int main(int argc, char *argv[]) {
    int poll = 0;
    //int quit = 0;
    int debugMode = 0;
    int cycles = 0;
    SDL_Event event;

    SDL_Init(SDL_INIT_VIDEO);

    if(argv[2] != NULL) {
        if(strcmp(argv[2], "-D") == 0) {
            debugMode = 1;
        }
    }

    init_cpu();
    init_mem();
    init_gpu();
    init_dma();
    init_timer();
    init_interrupt_info();
    init_joypad();
    if(load_program(argv[1]) < 0) {
        goto quit;
    }
    init_BIOS();
    init_screen();

    // Run BIOS
    while(get_program_counter() < 0x100) {
        if(poll >= POLL_INTERVAL) {
                poll = 0;
                if(SDL_PollEvent(&event)) {
                    switch (event.type)
                    {
                        case SDL_QUIT:
                            goto quit;
                            break;
                    }
                }
        }
        cycles = execute();
        gpu_update(cycles);
        poll++;
    }
    load_ROM(argv[1]);

    if(debugMode) {
        uint16_t breakpoint = 0;
        char input[10];
        char currCommand[20];
        strcpy(currCommand, "n");
        while(/*!quit*/ 1) { 
            if(poll >= POLL_INTERVAL) {
                poll = 0;
                uint8_t keysPressed = 0;
                if(SDL_PollEvent(&event)) {
                    switch (event.type)
                    {
                        case SDL_QUIT:
                            goto quit;
                            break;
                    }
                }
            }

            if(get_program_counter() == breakpoint) {
                dump_cpu();
                printf("Enter command: ");
                fgets(input, sizeof(input), stdin);
                if(strlen(input) > 1) {
                    strcpy(currCommand, input);
                }

                if(strncmp(currCommand, "n", 1) == 0) {
                    cycles = execute();
                    gpu_update(cycles);
                    dma_update(cycles);
                    cycles = handle_interrupts();
                    if(cycles > 0) {
                        gpu_update(cycles);
                    }
                    update_joypad();
                    breakpoint = get_program_counter();
                }
                else if(strncmp(currCommand, "b", 1) == 0) {
                    breakpoint = (uint16_t)strtol(input + 2, NULL, 16);
                    printf("Breakpoint set at 0x%X\n", breakpoint);
                    cycles = execute();
                    gpu_update(cycles);
                    dma_update(cycles);
                    cycles = handle_interrupts();
                    if(cycles > 0) {
                        gpu_update(cycles);
                    }
                    update_joypad();
                }
                else if(strncmp(currCommand, "c", 1) == 0) {
                    cycles = execute();
                    gpu_update(cycles);
                    dma_update(cycles);
                    cycles = handle_interrupts();
                    if(cycles > 0) {
                        gpu_update(cycles);
                    }
                    update_joypad();
                }
                else if(strncmp(currCommand, "r", 1) == 0) {
                    uint16_t addr = (uint16_t)strtol(input + 2, NULL, 16);
                    printf("Value at address 0x%X: %X\n", addr, read(addr));
                }
                else if(strncmp(currCommand, "q", 1) == 0) {
                    goto quit;
                }
                else {
                    printf("Invalid command\n");
                }
            }
            else{
                cycles = execute();
                gpu_update(cycles);
                dma_update(cycles);
                cycles = handle_interrupts();
                if(cycles > 0) {
                    gpu_update(cycles);
                }
                update_joypad();
            }
            poll++;
        }
    }

    else {
        while(/*!quit*/ 1) {
            if(poll >= POLL_INTERVAL) {
                poll = 0;
                if(SDL_PollEvent(&event)) {
                    switch (event.type)
                    {
                        case SDL_QUIT:
                            goto quit;
                            break;
                    }
                }
            }
            
            cycles = execute();
            gpu_update(cycles);
            dma_update(cycles);
            timer_update(cycles);
            cycles = handle_interrupts();
            if(cycles > 0) {
                gpu_update(cycles);
                dma_update(cycles);
                timer_update(cycles);
            }
            update_joypad();
            poll++;
        }
    }

    quit:

    destroy_screen();
    free_mem();
    SDL_Quit();
    return 0;
}
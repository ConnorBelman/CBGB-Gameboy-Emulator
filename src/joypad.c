#include <SDL2/SDL.h>
#include "../inc/int.h"
#include "../inc/mem.h"

#define P1_ADDR 0xFF00

#define ROW1 0x01
#define ROW2 0x02
#define ROW3 0x04
#define ROW4 0x08
#define COL1 0x10
#define COL2 0x20

// Probably unneccesary
typedef struct {
    uint8_t *p1;
} Joypad;

Joypad joypad;

int init_joypad() {
    joypad.p1 = get_addr_ptr(P1_ADDR);
    *joypad.p1 = 0x3F;
    return 0;
}

// Checks inputs and updates joypad register P1 based on buttons currently pressed.
// Logic low means button is pressed.
void update_joypad() {
    const uint8_t *keystate = SDL_GetKeyboardState(NULL);

    uint8_t prev_p1 = *joypad.p1;
    *joypad.p1 = ((*joypad.p1 & COL1) && keystate[SDL_SCANCODE_A]) || ((*joypad.p1 & COL2) && keystate[SDL_SCANCODE_RIGHT]) ? *joypad.p1 & ~(ROW1) : *joypad.p1 | ROW1;
    *joypad.p1 = ((*joypad.p1 & COL1) && keystate[SDL_SCANCODE_S]) || ((*joypad.p1 & COL2) && keystate[SDL_SCANCODE_LEFT]) ? *joypad.p1 & ~(ROW2) : *joypad.p1 | ROW2;
    *joypad.p1 = ((*joypad.p1 & COL1) && keystate[SDL_SCANCODE_Q]) || ((*joypad.p1 & COL2) && keystate[SDL_SCANCODE_UP]) ? *joypad.p1 & ~(ROW3) : *joypad.p1 | ROW3;
    *joypad.p1 = ((*joypad.p1 & COL1) && keystate[SDL_SCANCODE_RETURN]) || ((*joypad.p1 & COL2) && keystate[SDL_SCANCODE_DOWN]) ? *joypad.p1 & ~(ROW4) : *joypad.p1 | ROW4;

    // Send interrupt if any buttons transitioned from high to low
    // NOTE: Currently send on low to high also 
    if(prev_p1 ^ *joypad.p1) {
        send_irq(HI_LO_INT);
    }
    
    return;
}
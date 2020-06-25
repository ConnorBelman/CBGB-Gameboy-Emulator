#include <SDL2/SDL.h>
#include "../inc/screen.h"
#include "../inc/gpu.h"

SDL_Event event;
SDL_Window *window;
SDL_Renderer *renderer;
SDL_Texture *texture;
uint32_t *screen_buffer;

int init_screen() {
    SDL_Init(SDL_INIT_VIDEO);
    window = SDL_CreateWindow("CBGB", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, SCREEN_WIDTH, SCREEN_HEIGHT, 0);
    renderer = SDL_CreateRenderer(window, -1, 0);
    texture = SDL_CreateTexture(renderer, SDL_PIXELFORMAT_ARGB8888, SDL_TEXTUREACCESS_STATIC, SCREEN_WIDTH, SCREEN_HEIGHT);

    screen_buffer = malloc(sizeof(uint32_t) * SCREEN_WIDTH * SCREEN_HEIGHT);
    if(screen_buffer == 0) {
        return -1;
    }
    memset(screen_buffer, 255, SCREEN_WIDTH * SCREEN_HEIGHT * sizeof(uint32_t));

    SDL_UpdateTexture(texture, NULL, screen_buffer, SCREEN_WIDTH * sizeof(uint32_t));
    SDL_RenderClear(renderer);
	SDL_RenderCopy(renderer, texture, NULL, NULL);
	SDL_RenderPresent(renderer);
    return 0;
}

int destroy_screen() {
    SDL_DestroyTexture(texture);
	SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    return 0;
}

int update_screen() {
    SDL_UpdateTexture(texture, NULL, screen_buffer, SCREEN_WIDTH * sizeof(uint32_t));
    SDL_RenderClear(renderer);
    SDL_RenderCopy(renderer, texture, NULL, NULL);
    SDL_RenderPresent(renderer);
    return 0;
}

// Returns a pointer to the screen buffer so the GPU can update the pixels
uint32_t *get_screen_buffer() {
    return screen_buffer;
}
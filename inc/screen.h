#ifndef GBEMU_SCREEN_H
#define GBEMU_SCREEN_H

#define SCREEN_WIDTH 160
#define SCREEN_HEIGHT 144

#define WHITE       0xFFFFFFFF
#define LIGHT_GRAY  0xFFC0C0C0
#define DARK_GRAY   0xFF606060
#define BLACK       0xFF000000

int init_screen(void);
int destroy_screen(void);
int update_screen(void);
uint32_t *get_screen_buffer(void);

#endif
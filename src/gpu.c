// GPU mode and pixel processing algorithms based on Imran Nazar's "Gameboy Emulation in Javascript" blog
// http://imrannazar.com/GameBoy-Emulation-in-JavaScript:-Graphics
#include <stdint.h>
#include "../inc/gpu.h"
#include "../inc/int.h"
#include "../inc/mem.h"
#include "../inc/screen.h"


#define OAM_CYCLES 80
#define VRAM_CYCLES 172
#define HBLANK_CYCLES 204
#define VBLANK_CYCLES 456

#define TILE_WIDTH 8
#define NUM_SPRITES 40
#define MAX_SPRITES 10

// GPU register and data addresses
#define TILE_DATA_TABLE_UNSIGNED    0x8000
#define TILE_DATA_TABLE_SIGNED      0x9000
#define TILE_MAP_0                  0x9800
#define TILE_MAP_1                  0x9C00
#define LCDC_ADDR                   0xFF40
#define STAT_ADDR                   0xFF41
#define SCY_ADDR                    0xFF42
#define SCX_ADDR                    0xFF43
#define LY_ADDR                     0xFF44
#define BGP_ADDR                    0xFF47
#define OBP0_ADDR                   0xFF48
#define OBP1_ADDR                   0xFF49


// LCDC register bit offsets
#define BG_ON               0
#define OBJ_ON              1
#define OBJ_SIZE            2
#define BG_TILE_MAP_SELECT  3
#define BG_TILE_DATA_SELECT 4
#define LCD_ON              7

// STAT register bit offsets
#define MODE_FLAG 0

#define OAM_FLAG_PRIORITY   7
#define OAM_FLAG_FLIP_Y     6
#define OAM_FLAG_FLIP_X     5
#define OAM_FLAG_PALETTE    4

uint32_t colors[] = {WHITE, LIGHT_GRAY, DARK_GRAY, BLACK};

typedef struct {
    int clock;
    int vblank_irq_sent;

    // GPU registers in ZRAM
    uint8_t *lcdc;
    uint8_t *stat;
    uint8_t *scy;
    uint8_t *scx;
    uint8_t *ly;
    uint8_t *bgp;
    uint8_t *obp0;
    uint8_t *obp1;
} Gpu;

typedef struct {
    uint8_t posY;
    uint8_t posX;
    uint8_t pattern;
    uint8_t flags;
} Sprite_Block;

Gpu gpu;

int init_gpu() {
    gpu.clock = 0;
    gpu.vblank_irq_sent = 0;
    gpu.lcdc = get_addr_ptr(LCDC_ADDR);
    gpu.stat = get_addr_ptr(STAT_ADDR);
    gpu.scy = get_addr_ptr(SCY_ADDR);
    gpu.scx = get_addr_ptr(SCX_ADDR);
    gpu.ly = get_addr_ptr(LY_ADDR);
    gpu.bgp = get_addr_ptr(BGP_ADDR);
    gpu.obp0 = get_addr_ptr(OBP0_ADDR);
    gpu.obp1 = get_addr_ptr(OBP1_ADDR);
    return 0;
}

// Renders a single line of the screen. Occurs during mode 3 of the gpu cycle
void renderscan() {
    uint32_t *screen_buffer = get_screen_buffer();

    int screenOffset = *gpu.ly * SCREEN_WIDTH;

    uint16_t mapAddr = *gpu.lcdc & (1 << BG_TILE_MAP_SELECT) ? TILE_MAP_1 : TILE_MAP_0;
    uint16_t rowAddr = mapAddr + ((((*gpu.ly + *gpu.scy) & 0xFF) / TILE_WIDTH) * 32);
    uint16_t colOffset =  *gpu.scx / TILE_WIDTH;
    uint8_t pixelOffsetY = (*gpu.ly + *gpu.scy) & 7;
    uint8_t pixelOffsetX = *gpu.scx & 7;
    uint8_t tile = read(rowAddr + colOffset);
    uint16_t tileData = *gpu.lcdc & (1 << BG_TILE_DATA_SELECT) ? TILE_DATA_TABLE_UNSIGNED + (tile * 16) : TILE_DATA_TABLE_SIGNED + ((int8_t)tile * 16);
    uint8_t tileL = read(tileData + (pixelOffsetY * 2));
    uint8_t tileH = read(tileData + (pixelOffsetY * 2) + 1);

    int i;
    if(*gpu.lcdc & (1 << BG_ON)) {
        for(i = 0; i < SCREEN_WIDTH; i++) {
            uint8_t color = ((tileL >> (7 - pixelOffsetX)) & 1) + (((tileH >> (7 - pixelOffsetX)) & 1) * 2);
            screen_buffer[screenOffset++] = colors[(*gpu.bgp >> (color * 2)) & 3];
            pixelOffsetX++;

            if(pixelOffsetX == 8) {
                pixelOffsetX = 0;
                colOffset = (colOffset + 1) & 31;

                tile = read(rowAddr + colOffset);
                tileData = *gpu.lcdc & (1 << BG_TILE_DATA_SELECT) ? TILE_DATA_TABLE_UNSIGNED + (tile * 16) : TILE_DATA_TABLE_SIGNED + ((int8_t)tile * 16);
                tileL = read(tileData + (pixelOffsetY * 2));
                tileH = read(tileData + (pixelOffsetY * 2) + 1);
            }
        }
    }
    else {
        for(i = 0; i < SCREEN_WIDTH; i++) {
            screen_buffer[screenOffset++] = WHITE;
        }
    }

    // If sprites are enabled, update the render line to include sprites that are on render line
    if(*gpu.lcdc & (1 << OBJ_ON)) {

        // 8 x 8 sprites
        if(!(*gpu.lcdc & (1 << OBJ_SIZE))) {
            for(i = 0; i < NUM_SPRITES; i++) {
                Sprite_Block *sprite = (Sprite_Block *)get_addr_ptr(OAM_ADDR + (i * 4));
                uint8_t posX = sprite->posX - 8;
                uint8_t posY = sprite->posY - 16;
                // If sprite exists on the render line
                if((posY <= *gpu.ly) && ((posY + TILE_WIDTH) > *gpu.ly)) {
                    tileData = TILE_DATA_TABLE_UNSIGNED + (sprite->pattern * 16);
                    tileData += sprite->flags & (1 << OAM_FLAG_FLIP_Y) ? 14 - ((*gpu.ly - posY) * 2) : (*gpu.ly - posY) * 2;
                    tileL = read(tileData);
                    tileH = read(tileData + 1);
                    screenOffset = (*gpu.ly * 160) + posX;

                    // Render each pixel in the row that appears in the screen
                    int x;
                    for(x = 0; x < TILE_WIDTH; x++)  {
                        if(((posX + x) >= 0) && ((posX + x) < 160)) {
                            pixelOffsetX = sprite->flags & (1 << OAM_FLAG_FLIP_X) ? x : 7 - x;
                            uint8_t color = ((tileL >> pixelOffsetX) & 1) + (((tileH >> pixelOffsetX) & 1) * 2);
                            uint32_t screenColor = sprite->flags & (1 << OAM_FLAG_PALETTE) ? colors[(*gpu.obp1 >> (color * 2)) & 3] : colors[(*gpu.obp0 >> (color * 2)) & 3];
                            
                            // Render the pixel if the sprite is in the foreground or if the background pixel is transparent
                            if(!(sprite->flags & (1 << OAM_FLAG_PRIORITY)) || (screen_buffer[screenOffset] == WHITE)) {
                                screen_buffer[screenOffset++] = screenColor;
                            }
                        }
                    }

                }
            }
        }
    }
}

void update_gpu_mode(int mode) {
    *gpu.stat &= ~(3 << MODE_FLAG);
    *gpu.stat |= mode << MODE_FLAG;
}

// Update the mode of the GPU based on how many cycles have passed
void gpu_update(int cycles) {
    gpu.clock += cycles;

    switch(*gpu.stat & (3 << MODE_FLAG)) {

        // During H-Blank
        case 0:
            if(gpu.clock >= HBLANK_CYCLES) {
                gpu.clock = 0;
                *gpu.ly += 1;

                if(*gpu.ly == SCREEN_HEIGHT - 1) {
                    update_gpu_mode(1);
                    update_screen();
                }
                else {
                    update_gpu_mode(2);
                }
            }
            break;

        // During V-Blank
        case 1:
            // Send V-Blank interrupt request at beginning of V-Blank period
            if(!gpu.vblank_irq_sent) {
                send_irq(VBLANK_INT);
                gpu.vblank_irq_sent = 1;
            }
            if(gpu.clock >= VBLANK_CYCLES) {
                gpu.clock = 0;
                *gpu.ly += 1;

                if(*gpu.ly > 153) {
                    gpu.vblank_irq_sent = 0;
                    update_gpu_mode(2);
                    *gpu.ly = 0;
                }
            }
            break;

        // During searching OAM RAM
        case 2:
            if(gpu.clock >= OAM_CYCLES) {
                update_gpu_mode(3);
                gpu.clock = 0;
            }
            break;

        // During data transfer to screen
        case 3:
            if(gpu.clock >= VRAM_CYCLES) {
                update_gpu_mode(0);
                gpu.clock = 0;
                renderscan();
            }
            break;
    }
}
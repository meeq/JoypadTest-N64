/**
 * @file main.c
 * @author Christopher Bonhage (me@christopherbonhage.com)
 * @brief Work-in-progress accessory utilities test ROM
 */

#include <string.h>
#include <libdragon.h>

#include "joybus_n64_accessory.h"
#include "joypad.h"

#define BLACK 0x000000FF
#define WHITE 0xFFFFFFFF

#define GLYPH_WIDTH  8
#define GLYPH_HEIGHT 8

#define X_LABEL (3 * GLYPH_WIDTH)
#define X_VALUE (320 - (3 * GLYPH_WIDTH))
#define X_RIGHT(str) (X_VALUE - (strlen(str) * GLYPH_WIDTH))

#define Y_TITLE (2 * GLYPH_HEIGHT)
#define Y_PORT (4 * GLYPH_HEIGHT)
#define Y_ADDR (5 * GLYPH_HEIGHT)
#define Y_WRITEBUF_LABEL (7 * GLYPH_HEIGHT)
#define Y_WRITEBUF_DATA_1 (9 * GLYPH_HEIGHT)
#define Y_WRITEBUF_DATA_2 (10 * GLYPH_HEIGHT)
#define Y_READBUF_LABEL (12 * GLYPH_HEIGHT)
#define Y_READBUF_DATA_1 (14 * GLYPH_HEIGHT)
#define Y_READBUF_DATA_2 (15 * GLYPH_HEIGHT)

typedef enum
{
    FOCUS_PORT,
    FOCUS_ADDR,
    FOCUS_WRITEBUF,
} focus_t;

static display_context_t disp = 0;
static focus_t focus = FOCUS_PORT;
static joypad_port_t port = JOYPAD_PORT_1;
// static size_t focus_addr_nibble = 0;
static uint16_t addr = 0x0000;
// static size_t focus_write_nibble = 0;
// static uint8_t write_data[JOYBUS_N64_ACCESSORY_DATA_SIZE] = {0};
// static bool read_data_valid = false;
// static uint8_t read_data[JOYBUS_N64_ACCESSORY_DATA_SIZE] = {0};
static joypad_buttons_t p1_pressed;

void set_focus_color(focus_t field)
{
    if (focus == field) graphics_set_color(BLACK, WHITE);
    else graphics_set_color(WHITE, BLACK);
}

void focus_adjust(int incr)
{
    int result = focus + incr;
    if (result > FOCUS_WRITEBUF) result = FOCUS_PORT;
    if (result < FOCUS_PORT) result = FOCUS_WRITEBUF;
    focus = result;
}

void port_adjust(int incr)
{
    int result = port + incr;
    if (result > JOYPAD_PORT_4) result = JOYPAD_PORT_1;
    if (result < JOYPAD_PORT_1) result = JOYPAD_PORT_4;
    port = result;
}

void accessory_read(void)
{

}

void accessory_write(void)
{

}

int main(void)
{
    timer_init();
    display_init(RESOLUTION_320x240, DEPTH_32_BPP, 2, GAMMA_NONE, ANTIALIAS_RESAMPLE);
    debug_init_isviewer();

    joypad_init();

    char port_str[sizeof("1")];
    char addr_str[sizeof("0x0000")];
    int p1_stick_y_pressed;

    while (1)
    {
        while( !(disp = display_lock()) ) { /* Spinloop */ }
        joypad_scan();
        p1_pressed = joypad_get_buttons_pressed(JOYPAD_PORT_1);
        p1_stick_y_pressed = joypad_get_axis_pressed(JOYPAD_PORT_1, JOYPAD_AXIS_STICK_Y);

        graphics_fill_screen(disp, BLACK);

        graphics_set_color(WHITE, BLACK);
        graphics_draw_text(disp, X_LABEL, Y_PORT, "Controller Port:");
        graphics_draw_text(disp, X_LABEL, Y_ADDR, "Accessory Address:");
        graphics_draw_text(disp, X_LABEL, Y_WRITEBUF_LABEL, "Write Buffer:");
        graphics_draw_text(disp, X_LABEL, Y_READBUF_LABEL, "Read Buffer:");

        set_focus_color(FOCUS_PORT);
        sprintf(port_str, "%1d", port + 1);
        graphics_draw_text(disp, X_RIGHT(port_str), Y_PORT, port_str);

        set_focus_color(FOCUS_ADDR);
        sprintf(addr_str, "0x%04X", addr);
        graphics_draw_text(disp, X_RIGHT(addr_str), Y_ADDR, addr_str);

        // Port selection
        if (p1_pressed.d_up || p1_stick_y_pressed > 0) focus_adjust(-1);
        if (p1_pressed.d_down || p1_stick_y_pressed < 0) focus_adjust(+1);
        if (focus == FOCUS_PORT)
        {
            if (p1_pressed.c_down) port_adjust(-1);
            if (p1_pressed.c_up) port_adjust(+1);
        }
        else if (focus == FOCUS_ADDR)
        {
            if (p1_pressed.c_down) addr -= 1;
            if (p1_pressed.c_up) addr += 1;
        }
        else if (focus == FOCUS_WRITEBUF)
        {
            
        }
        if (p1_pressed.a) accessory_read();
        if (p1_pressed.b) accessory_write();

        display_show( disp );
    }
}

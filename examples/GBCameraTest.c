/**
 * @file main.c
 * @author Christopher Bonhage (me@christopherbonhage.com)
 * @brief N64 test ROM for Joypad subsystem
 */

#ifdef ROM_VERSION
static const char ROM_TITLE[] = "GB Camera Test " ROM_VERSION " by Meeq";
#else
static const char ROM_TITLE[] = "GB Camera Test by Meeq";
#endif

#include <math.h>
#include <string.h>
#include <libdragon.h>

#include "joybus_commands.h"
#include "joypad.h"
#include "../src/joypad_accessory.h"

#define SCREEN_WIDTH 320
#define SCREEN_HEIGHT 240

extern void *__safe_buffer[];
#define __get_buffer( x ) __safe_buffer[(x)-1]

#define GB_CART_ADDR_HEADER      0x0100
#define GB_CART_ADDR_RAM_ENABLE  0x0000
#define GB_CART_ADDR_RAM_BANK    0x4000
#define GB_CART_ADDR_CAMERA_REGS 0xA000
#define GB_CART_ADDR_CAMERA_DATA 0xA100

#define GB_CART_TYPE_CAMERA     0xFC
#define GB_RAM_ENABLE           0x0A
#define GB_RAM_DISABLE          0x00
#define GB_RAM_BANK_CAMERA_REGS 0x10
#define GB_RAM_BANK_CAMERA_SRAM 0x00

#define GB_CAMERA_FILTER_MODE_NEGATIVE 0b00
#define GB_CAMERA_FILTER_MODE_POSITIVE 0b01
#define GB_CAMERA_FILTER_MODE_EDGE_1   0b10
#define GB_CAMERA_FILTER_MODE_EDGE_2   0b11

#define GB_CAMERA_EXPOSURE_MIN          0x0020
#define GB_CAMERA_EXPOSURE_MAX          0xF000
#define GB_CAMERA_EXPOSURE_SUN_BRIGHT   0x0030
#define GB_CAMERA_EXPOSURE_SUN_AMBIENT  0x0300
#define GB_CAMERA_EXPOSURE_ROOM_BRIGHT  0x0800
#define GB_CAMERA_EXPOSURE_ROOM_AMBIENT 0x2C00
#define GB_CAMERA_EXPOSURE_ROOM_DIM     0x5000
#define GB_CAMERA_EXPOSURE_ROOM_DARK    0xF000

#define GB_CAMERA_CALIBRATE_DISABLED 0b00
#define GB_CAMERA_CALIBRATE_NEGATIVE 0b01
#define GB_CAMERA_CALIBRATE_POSITIVE 0b10

#define GB_CAMERA_VREF_BIAS_0_0 0b000
#define GB_CAMERA_VREF_BIAS_0_5 0b001
#define GB_CAMERA_VREF_BIAS_1_0 0b010
#define GB_CAMERA_VREF_BIAS_1_5 0b011
#define GB_CAMERA_VREF_BIAS_2_0 0b100
#define GB_CAMERA_VREF_BIAS_2_5 0b101
#define GB_CAMERA_VREF_BIAS_3_0 0b110
#define GB_CAMERA_VREF_BIAS_3_5 0b111

#define GB_CAMERA_EDGE_RATIO_50  0b0000
#define GB_CAMERA_EDGE_RATIO_75  0b0001
#define GB_CAMERA_EDGE_RATIO_100 0b0010
#define GB_CAMERA_EDGE_RATIO_125 0b0011
#define GB_CAMERA_EDGE_RATIO_200 0b0100
#define GB_CAMERA_EDGE_RATIO_300 0b0101
#define GB_CAMERA_EDGE_RATIO_400 0b0110
#define GB_CAMERA_EDGE_RATIO_500 0b0111

#define GB_CAMERA_EDGE_MODE_NONE 0b00
#define GB_CAMERA_EDGE_MODE_H    0b01
#define GB_CAMERA_EDGE_MODE_V    0b10
#define GB_CAMERA_EDGE_MODE_VH   0b11

#define GB_CAMERA_VREF_OFFSET_SIGN_MASK 0x20
#define GB_CAMERA_VREF_OFFSET_STEP_MASK 0x1F
#define GB_CAMERA_VREF_OFFSET_STEP_INCR 0.032

#define GB_CAMERA_INIT_FILTER_MODE   GB_CAMERA_FILTER_MODE_POSITIVE
#define GB_CAMERA_INIT_GAIN          5
#define GB_CAMERA_INIT_EDGE_VH_MODE  GB_CAMERA_EDGE_MODE_NONE
#define GB_CAMERA_INIT_EDGE_V_ONLY   0
#define GB_CAMERA_INIT_EXPOSURE      0x1000
#define GB_CAMERA_INIT_VREF_BIAS     GB_CAMERA_VREF_BIAS_2_5
#define GB_CAMERA_INIT_INVERT        0
#define GB_CAMERA_INIT_EDGE_RATIO    GB_CAMERA_EDGE_RATIO_100
#define GB_CAMERA_INIT_VREF_OFFSET   0.0
#define GB_CAMERA_INIT_CALIBRATE     GB_CAMERA_CALIBRATE_POSITIVE
#define GB_CAMERA_INIT_MATRIX_SLOPE  10
#define GB_CAMERA_INIT_MATRIX_OFFSET 100

#define GB_CAMERA_CAPTURE_WIDTH  128
#define GB_CAMERA_CAPTURE_HEIGHT 112
#define GB_CAMERA_CAPTURE_PIXELS (128 * 112)
#define GB_CAMERA_CAPTURE_SIZE   (128 * 112 * 2 / 8)

typedef enum
{
    STATE_TPAK_INIT = 0,
    STATE_TPAK_ENABLE,
    STATE_TPAK_DISABLE,
    STATE_GB_ROM_HEADER_READ,
    STATE_GB_RAM_ENABLE_WRITE,
    STATE_CAM_REGS_BANK_WRITE,
    STATE_CAM_REGS_DATA_WRITE,
    STATE_CAM_CAPTURE_WRITE,
    STATE_CAM_CAPTURE_WAIT,
    STATE_CAM_RAM_BANK_WRITE,
    STATE_CAM_RAM_DATA_READ,
    STATE_CAM_LOOP,
} state_t;

static const uint32_t GB_PIXEL_TO_RGBA[] = {
    0x9BBC0FFF,
    0x8BAC0FFF,
    0x306230FF,
    0x0F380FFF,
};

typedef struct __attribute__((__packed__))
{
    uint capture_mode : 1;
    uint filter_mode  : 2;
    uint              : 5;
    uint gain         : 5;
    uint edge_vh_mode : 2;
    uint edge_v_only  : 1;
    uint exposure_hi  : 8;
    uint exposure_lo  : 8;
    uint vref_bias    : 3;
    uint invert       : 1;
    uint edge_ratio   : 4;
    uint vref_offset  : 6;
    uint calibrate    : 2;
    uint8_t matrix[48];
    uint8_t padding[10];
} gb_camera_regs_t;

typedef struct
{
    uint16_t exposure;
    uint8_t matrix_slope;
    uint8_t matrix_offset;
    uint8_t matrix_qlevels[4];
    float vref_offset;
    gb_camera_regs_t regs;
    uint8_t capture_raw[GB_CAMERA_CAPTURE_SIZE];
} gb_camera_t;

static const uint8_t matrix_layout[16] = {
    0x00, 0x1E, 0x18, 0x06,
    0x0F, 0x2D, 0x27, 0x15,
    0x0C, 0x2A, 0x24, 0x12,
    0x03, 0x21, 0x1B, 0x09,
};

static void gb_camera_set_gain(gb_camera_t *cam, uint8_t gain)
{
    cam->regs.gain = gain;
}

static void gb_camera_set_exposure(gb_camera_t *cam, uint16_t exposure)
{
    if (exposure > GB_CAMERA_EXPOSURE_MAX) exposure = GB_CAMERA_EXPOSURE_MAX;
    else if (exposure < GB_CAMERA_EXPOSURE_MIN) exposure = GB_CAMERA_EXPOSURE_MIN;
    cam->exposure = exposure;
    cam->regs.exposure_hi = (exposure >> 8) & 0xFF;
    cam->regs.exposure_lo = exposure & 0xFF;
}

static void gb_camera_set_matrix(gb_camera_t *cam, uint8_t slope, uint8_t offset)
{
    uint32_t c;
	uint16_t acc, inc, v;
	uint8_t pixel;

    cam->matrix_slope = slope;
    cam->matrix_offset = offset;

	// This is a hardcoded 16 entry LUT @ ROMA:7C20 in the GB Cam ROM
	// Slope ~= -contrast, offset ~= -brightness
    for (c = 0; c < 4; c++)
    {
    	v = offset + (c * slope);
    	cam->matrix_qlevels[c] = (v < 256) ? v : 255;
    }

	// Each entry in the 4x4 matrix is 3 bytes indicating the threshold levels for each shade of grey
	// Pixel voltage < byte A -> black
	// byte A < Pv < byte B -> dark grey
	// byte B < Pv < byte C -> light grey
	// byte C < Pv -> white
	// Those are used to generate the voltages for the 3 comparators in the GB Cam ASIC
	// The GB Cam ROM generates the whole matrix from 4 bytes: qlevels array (and threshold levels also),
	// which are "spread out" by interpolation on the 16 pixels.
	// Bytes A for each pixel are qlevels[0] -> qlevels[1]
	// Bytes B for each pixel are qlevels[1] -> qlevels[2]
	// Bytes C for each pixel are qlevels[2] -> qlevels[3]
    for (c = 0; c < 3; c++)
    {
    	acc = cam->matrix_qlevels[c];
    	inc = cam->matrix_qlevels[c + 1] - acc;	// Delta between 2 qlevels
    	acc <<= 4;					// 4.4 fixed point
		for (pixel = 0; pixel < 16; pixel++)
        {
			cam->regs.matrix[c + matrix_layout[pixel]] = acc >> 4;
			acc += inc;
		}
    }
}

static void gb_camera_set_vref_offset(gb_camera_t *cam, float offset)
{
    bool negative = offset < 0.0;
    float offset_abs = fabsf(offset);
    // Register expects a signed 6-bit number: build it bitwise
    uint8_t offset_steps = offset_abs / GB_CAMERA_VREF_OFFSET_STEP_INCR;
    offset_steps = offset_steps & GB_CAMERA_VREF_OFFSET_STEP_MASK;
    if (negative) offset_steps |= GB_CAMERA_VREF_OFFSET_SIGN_MASK;
    cam->regs.vref_offset = offset_steps;
    cam->vref_offset = offset;
}

static void gb_camera_init(gb_camera_t *cam)
{
    cam->regs.capture_mode = 0;
    cam->regs.filter_mode = GB_CAMERA_INIT_FILTER_MODE;
    gb_camera_set_gain(cam, GB_CAMERA_INIT_GAIN);
    cam->regs.edge_vh_mode = GB_CAMERA_INIT_EDGE_VH_MODE;
    cam->regs.edge_v_only = GB_CAMERA_INIT_EDGE_V_ONLY;
    gb_camera_set_exposure(cam, GB_CAMERA_INIT_EXPOSURE);
    cam->regs.vref_bias = GB_CAMERA_INIT_VREF_BIAS;
    cam->regs.invert = GB_CAMERA_INIT_INVERT;
    cam->regs.edge_ratio = GB_CAMERA_INIT_EDGE_RATIO;
    gb_camera_set_vref_offset(cam, GB_CAMERA_INIT_VREF_OFFSET);
    cam->regs.calibrate = GB_CAMERA_INIT_CALIBRATE;
    gb_camera_set_matrix(cam, GB_CAMERA_INIT_MATRIX_SLOPE, GB_CAMERA_INIT_MATRIX_OFFSET);
    memset(cam->regs.padding, 0x00, sizeof(cam->regs.padding));
}

static void gb_camera_process(const uint8_t *raw, uint32_t *rgba)
{
    uint32_t i = 0;
    for (uint32_t yt = 0; yt < GB_CAMERA_CAPTURE_HEIGHT; yt++) {

        uint32_t yto = (yt & 7) + ((yt & 0x78) << 4);

        for (uint32_t xt = 0; xt < 16; xt++) {
            // (xt << 3): tile # x * pixel rows per tile
            // + (yt & 7): pixel row # in tile
            // + ((yt & 0x78) << 4): tile # y * tiles per row
            // << 1: 2 bytes per 8x 2bpp pixel row
            uint32_t addr = ((xt << 3) + yto) << 1;

            // Get 8 pixels worth of data
            uint8_t data_l = raw[addr];
            uint8_t data_h = raw[addr + 1];

            for (uint32_t x = 0; x < 8; x++) {
                // Planar bitfields to RGBA pixels
                uint8_t pixel = (data_h & 0x80) ? 2 : 0;
                if (data_l & 0x80) pixel |= 1;

                rgba[i++] = GB_PIXEL_TO_RGBA[pixel];
                data_l <<= 1;
                data_h <<= 1;
            }
        }
    }
}

static void gb_camera_draw(display_context_t disp, uint32_t *pixels)
{
    uint32_t *buf = __get_buffer(disp);
    size_t i = 0, j;
    for (size_t y = 0; y < GB_CAMERA_CAPTURE_HEIGHT; ++y)
    {
        j = y * SCREEN_WIDTH;
        for (size_t x = 0; x < GB_CAMERA_CAPTURE_WIDTH; ++x)
        {
            buf[x + j] = pixels[i++];
        }
    }

}

static void wait_for_start_button(joypad_port_t port)
{
    joypad_buttons_t pressed;
    printf("Press Start button to continue...\n");
    console_render();
    while (1)
    {
        joypad_scan();
        pressed = joypad_get_buttons_pressed(port);
        if (pressed.start) break;
    }
}

int main(void)
{
    timer_init();

    joypad_init();

    display_context_t disp = 0;
    state_t state = STATE_TPAK_INIT;
    joypad_port_t port = JOYPAD_PORT_1;
    joypad_style_t style;
    joypad_accessory_type_t accessory_type;
    // joypad_buttons_t pressed;
    int accessory_state;
    int accessory_error;
    joybus_n64_transfer_pak_status_t transfer_pak_status;
    volatile struct gameboy_cartridge_header gb_cart_info = {0};
    gb_camera_t cam;
    volatile uint8_t tpak_data[JOYBUS_N64_ACCESSORY_DATA_SIZE];
    volatile uint8_t capture_raw[GB_CAMERA_CAPTURE_SIZE];
    uint32_t capture_rgba[GB_CAMERA_CAPTURE_PIXELS];

    console_init();
    console_set_render_mode(RENDER_MANUAL);
    console_set_debug(false);

    while (1)
    {
        console_clear();
        printf("%s\n\n", ROM_TITLE);

        joypad_scan();
        style = joypad_get_style(port);
        accessory_type = joypad_get_accessory_type(port);
        // pressed = joypad_get_buttons_pressed(port);
        accessory_state = joypad_get_accessory_state(port);
        accessory_error = joypad_get_accessory_error(port);
        transfer_pak_status.raw = joypad_get_accessory_transfer_pak_status(port);

        if (style != JOYPAD_STYLE_N64)
        {
            state = STATE_TPAK_INIT;
            printf("Connect an N64 Controller\n");
            console_render();
            continue;
        }
        if (accessory_type != JOYPAD_ACCESSORY_TYPE_TRANSFER_PAK)
        {
            state = STATE_TPAK_INIT;
            printf("Insert a Transfer Pak\n");
            console_render();
            continue;
        }
        if (accessory_state != JOYPAD_ACCESSORY_STATE_IDLE)
        {
            // Waiting for accessory command to finish
            continue;
        }
        if (accessory_error > JOYPAD_ACCESSORY_ERROR_NONE)
        {
            printf("Accessory command error: %d\n", accessory_error);
            wait_for_start_button(port);
            state = STATE_TPAK_INIT;
            continue;
        }
        if (state == STATE_TPAK_INIT)
        {
            state = STATE_TPAK_ENABLE;
            joypad_n64_transfer_pak_enable_async(port, true);
            printf("Enabling the Transfer Pak...\n");
            console_render();
            wait_ms(1000);
            continue;
        }
        if (!transfer_pak_status.power || !transfer_pak_status.access)
        {
            joypad_n64_transfer_pak_enable_async(port, false);
            printf("Transfer Pak has become disabled\n");
            wait_for_start_button(port);
            state = STATE_TPAK_INIT;
            continue;
        }
        if (state == STATE_TPAK_ENABLE)
        {
            state = STATE_GB_ROM_HEADER_READ;
            uint16_t addr = GB_CART_ADDR_HEADER;
            void *read_data = (void *)&gb_cart_info;
            size_t read_len = sizeof(gb_cart_info);
            joypad_n64_transfer_pak_load_async(port, addr, read_data, read_len);
            printf("Reading the GB cartridge header...\n");
            console_render();
            wait_ms(1000);
            continue;
        }
        if (state == STATE_TPAK_DISABLE)
        {
            printf("Transfer Pak disabled\n");
            wait_for_start_button(port);
            continue;
        }
        if (state == STATE_GB_ROM_HEADER_READ)
        {
            if (gb_cart_info.cartridge_type != GB_CART_TYPE_CAMERA)
            {
                joypad_n64_transfer_pak_enable_async(port, false);
                printf("Insert GB Camera cartridge\n");
                wait_for_start_button(port);
                state = STATE_TPAK_INIT;
                continue;
            }
            printf("Hello, GB Camera!\n");
            wait_for_start_button(port);
            console_close();
            display_init(
                RESOLUTION_320x240,
                DEPTH_32_BPP,
                2,
                GAMMA_NONE,
                ANTIALIAS_RESAMPLE
            );
            gb_camera_init(&cam);
            state = STATE_GB_RAM_ENABLE_WRITE;
            memset((void *)tpak_data, GB_RAM_ENABLE, sizeof(tpak_data));
            joypad_n64_transfer_pak_store_async(
                port, GB_CART_ADDR_RAM_ENABLE,
                (void *)tpak_data, sizeof(tpak_data)
            );
            continue;
        }
        if (state == STATE_GB_RAM_ENABLE_WRITE || state == STATE_CAM_LOOP)
        {
            state = STATE_CAM_REGS_BANK_WRITE;
            memset((void *)tpak_data, GB_RAM_BANK_CAMERA_REGS, sizeof(tpak_data));
            joypad_n64_transfer_pak_store_async(
                port, GB_CART_ADDR_RAM_BANK,
                (void *)tpak_data, sizeof(tpak_data)
            );
            continue;
        }
        if (state == STATE_CAM_REGS_BANK_WRITE)
        {
            state = STATE_CAM_REGS_DATA_WRITE;
            cam.regs.capture_mode = 0;
            joypad_n64_transfer_pak_store_async(
                port, GB_CART_ADDR_CAMERA_REGS,
                (void *)&cam.regs, sizeof(cam.regs)
            );
            continue;
        }
        if (state == STATE_CAM_REGS_DATA_WRITE)
        {
            state = STATE_CAM_CAPTURE_WRITE;
            cam.regs.capture_mode = 1;
            joypad_n64_transfer_pak_store_async(
                port, GB_CART_ADDR_CAMERA_REGS,
                (void *)&cam.regs, sizeof(cam.regs)
            );
            continue;
        }
        if (state == STATE_CAM_CAPTURE_WRITE)
        {
            state = STATE_CAM_CAPTURE_WAIT;
            joypad_n64_transfer_pak_load_async(
                port, GB_CART_ADDR_CAMERA_REGS,
                (void *)tpak_data, sizeof(tpak_data)
            );
            continue;
        }
        if (state == STATE_CAM_CAPTURE_WAIT)
        {
            if (tpak_data[0] & 1)
            {
                // Still capturing; continue waiting
                joypad_n64_transfer_pak_load_async(
                    port, GB_CART_ADDR_CAMERA_REGS,
                    (void *)tpak_data, sizeof(tpak_data)
                );
                continue;
            }
            state = STATE_CAM_RAM_BANK_WRITE;
            memset((void *)tpak_data, 0x00, sizeof(tpak_data));
            joypad_n64_transfer_pak_store_async(
                port, GB_CART_ADDR_RAM_BANK,
                (void *)tpak_data, sizeof(tpak_data)
            );
            continue;
        }
        if (state == STATE_CAM_RAM_BANK_WRITE)
        {
            state = STATE_CAM_RAM_DATA_READ;
            joypad_n64_transfer_pak_load_async(
                port, GB_CART_ADDR_CAMERA_DATA,
                (void *)capture_raw, sizeof(capture_raw)
            );
            continue;
        }
        if (state == STATE_CAM_RAM_DATA_READ)
        {
            state = STATE_CAM_LOOP;
            gb_camera_process((void *)capture_raw, capture_rgba);
            while (!(disp = display_lock())) { /* Spinlock! */ }
            graphics_fill_screen( disp, 0x000000FF );
            gb_camera_draw(disp, capture_rgba);
            display_show(disp);
        }
    }
}

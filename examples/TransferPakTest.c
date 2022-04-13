/**
 * @file main.c
 * @author Christopher Bonhage (me@christopherbonhage.com)
 * @brief N64 test ROM for Joypad subsystem
 */

#include <string.h>
#include <libdragon.h>

#include "joybus_commands.h"
#include "joypad.h"
#include "../src/joypad_accessory.h"

int main(void)
{
    timer_init();
    display_init(RESOLUTION_320x240, DEPTH_32_BPP, 2, GAMMA_NONE, ANTIALIAS_RESAMPLE);
    debug_init_isviewer();

    joypad_init();

    joypad_style_t style;
    joypad_accessory_type_t accessory_type;
    joypad_buttons_t pressed;
    int accessory_state;
    int accessory_error;
    joybus_n64_transfer_pak_status_t transfer_pak_status;
    volatile struct gameboy_cartridge_header gb_headers[JOYPAD_PORT_COUNT];
    memset((void *)gb_headers, 0, sizeof(gb_headers));
    uint8_t rumble_data[JOYBUS_N64_ACCESSORY_DATA_SIZE];

    console_init();
    console_set_render_mode(RENDER_MANUAL);
    console_set_debug(false);

    while (1)
    {
        console_clear();

        printf("Joypad Transfer Pak Test v1 by Meeq\n");
        printf("A to enable Pak; B to disable Pak; C-Left to read ROM header\n");
        printf("C-Up to start MBC5 rumble; C-Down to stop MBC5 rumble\n\n");

        joypad_scan();

        JOYPAD_PORT_FOREACH (port)
        {
            style = joypad_get_style(port);
            accessory_type = joypad_get_accessory_type(port);
            pressed = joypad_get_buttons_pressed(port);

            printf("Port %d ", port + 1);
            if (style == JOYPAD_STYLE_N64)
            {
                if (accessory_type == JOYPAD_ACCESSORY_TYPE_TRANSFER_PAK)
                {
                    accessory_state = joypad_get_accessory_state(port);
                    accessory_error = joypad_get_accessory_error(port);
                    transfer_pak_status.raw = joypad_get_accessory_transfer_pak_status(port);
                    printf("Transfer Pak detected! ");
                    printf("Accessory State: %02d ", accessory_state);
                    printf("Error: %d\n", accessory_error);
                    printf("Transfer Pak Power: %d ", transfer_pak_status.power);
                    printf("Access Mode:  %d ", transfer_pak_status.access);
                    printf("Cart Removed: %d\n", transfer_pak_status.cart_pulled);
                    printf("                      ");
                    printf("Reset Status: %d ", transfer_pak_status.booting);
                    printf("Reset Detect: %d\n", transfer_pak_status.reset);
                    printf("Cartridge Type: %02X ", gb_headers[port].cartridge_type);
                    printf("ROM Size Code: %02X ", gb_headers[port].rom_size_code);
                    printf("RAM Size Code: %02X\n", gb_headers[port].ram_size_code);

                    if (!accessory_state && pressed.a)
                    {
                        joypad_n64_transfer_pak_enable_async(port, true);
                    }
                    if (transfer_pak_status.power && pressed.b)
                    {
                        joypad_n64_transfer_pak_enable_async(port, false);
                    }
                    if (transfer_pak_status.access && pressed.c_left)
                    {
                        void *read_data = (void *)&gb_headers[port];
                        size_t read_len = sizeof(gb_headers[port]);
                        joypad_n64_transfer_pak_load_async(port, 0x0100, read_data, read_len);
                    }
                    if (transfer_pak_status.access && pressed.c_up)
                    {
                        memset(rumble_data, 0x08, sizeof(rumble_data));
                        joypad_n64_transfer_pak_store_async(port, 0x4000, rumble_data, sizeof(rumble_data));
                    }
                    if (transfer_pak_status.access && pressed.c_down)
                    {
                        memset(rumble_data, 0x00, sizeof(rumble_data));
                        joypad_n64_transfer_pak_store_async(port, 0x4000, rumble_data, sizeof(rumble_data));
                    }
                }
                else
                {
                    printf("Insert Transfer Pak to test\n");
                }
            }
            else
            {
                printf("Insert N64 Controller to test\n");
            }
            printf("\n");
        }

        console_render();
    }
}

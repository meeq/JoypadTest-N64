/**
 * @file main.c
 * @author Christopher Bonhage (me@christopherbonhage.com)
 * @brief N64 test ROM for Joypad subsystem
 */

#ifdef ROM_VERSION
static const char ROM_TITLE[] = "Snap Station Test " ROM_VERSION " by Meeq";
#else
static const char ROM_TITLE[] = "Snap Station Test by Meeq";
#endif

#include <string.h>
#include <libdragon.h>

#include "joybus_n64_accessory.h"
#include "joypad.h"

static joypad_buttons_t pressed;

const char *accessory_probe_format(uint8_t probe_value)
{
    switch (probe_value)
    {
        case 0x00:
            return "Inactive";
        case JOYBUS_N64_ACCESSORY_PROBE_RUMBLE_PAK:
            return "Rumble Pak";
        case JOYBUS_N64_ACCESSORY_PROBE_TRANSFER_PAK_ON:
            return "Active Transfer Pak";
        case JOYBUS_N64_ACCESSORY_PROBE_BIO_SENSOR:
            return "Bio Sensor";
        case JOYBUS_N64_ACCESSORY_PROBE_SNAP_STATION:
            return "Active Snap Station";
        default:
            return "Unknown";
    }
}

static const char *accessory_data_crc_status_format(int crc_status)
{
    switch (crc_status)
    {
        case JOYBUS_N64_ACCESSORY_DATA_CRC_STATUS_OK:
            return "CRC OK";
        case JOYBUS_N64_ACCESSORY_DATA_CRC_STATUS_NO_PAK:
            return "No Pak";
        case JOYBUS_N64_ACCESSORY_DATA_CRC_STATUS_MISMATCH:
            return "CRC Mismatch";
        default:
            return "Unknown";
    }
}

static const char *snap_station_state_format(uint8_t state)
{
    switch (state)
    {
        case JOYBUS_N64_SNAP_STATION_STATE_PRE_SAVE:
            return "Pre-Save";
        case JOYBUS_N64_SNAP_STATION_STATE_POST_SAVE:
            return "Post-Save";
        case JOYBUS_N64_SNAP_STATION_STATE_RESET_CONSOLE:
            return "Reset Console";
        case JOYBUS_N64_SNAP_STATION_STATE_PRE_ROLL:
            return "Pre-Roll";
        case JOYBUS_N64_SNAP_STATION_STATE_CAPTURE_PHOTO:
            return "Capture Photo";
        case JOYBUS_N64_SNAP_STATION_STATE_POST_ROLL:
            return "Post-Roll";
        case JOYBUS_N64_SNAP_STATION_STATE_BUSY:
            return "Busy";
        case JOYBUS_N64_SNAP_STATION_STATE_IDLE:
            return "Idle";
        default:
            return "Unknown";
    }
}

static void wait_for_start_button()
{
    joypad_init();
    printf("Press Start button to continue...\n");
    console_render();
    while (1)
    {
        joypad_scan();
        pressed = joypad_get_buttons_pressed(JOYPAD_PORT_1);
        if (pressed.start) break;
    }
}

static void snap_station_probe_read(void)
{
    const uint16_t addr = JOYBUS_N64_ACCESSORY_ADDR_PROBE;
    uint8_t data[JOYBUS_N64_ACCESSORY_DATA_SIZE];
    printf("Probing accessory on port 4...\n");
    console_render();

    int crc_status = joybus_n64_accessory_read_sync(JOYPAD_PORT_4, addr, data);
    if (crc_status != JOYBUS_N64_ACCESSORY_DATA_CRC_STATUS_OK)
    {
        const char *crc_error = accessory_data_crc_status_format(crc_status);
        printf("Accessory Read error: %s\n", crc_error);
        wait_for_start_button();
    }
    else
    {
        uint8_t probe_value = data[sizeof(data)-1];
        const char * probe_str = accessory_probe_format(probe_value);
        printf("Accessory Read complete: 0x%02X (%s)", probe_value, probe_str);
        printf("\n");
        wait_for_start_button();
    }
}

static void snap_station_state_read(void)
{
    const uint16_t addr = JOYBUS_N64_ACCESSORY_ADDR_SNAP_STATE;
    uint8_t data[JOYBUS_N64_ACCESSORY_DATA_SIZE];
    int crc_status;
    uint8_t state = JOYBUS_N64_SNAP_STATION_STATE_IDLE;
    bool cancel_message_shown = false;
    printf("Reading Snap Station state...\n");
    console_render();
    do
    {
        crc_status = joybus_n64_accessory_read_sync(JOYPAD_PORT_4, addr, data);
        if (crc_status != JOYBUS_N64_ACCESSORY_DATA_CRC_STATUS_OK)
        {
            const char *crc_error = accessory_data_crc_status_format(crc_status);
            printf("Accessory Read error: %s\n", crc_error);
            wait_for_start_button();
            break;
        }
        else
        {
            state = data[sizeof(data)-1];
            const char * state_str = snap_station_state_format(state);
            printf("Accessory Read complete: 0x%02X (%s)\n", state, state_str);
            console_render();
            if (state != JOYBUS_N64_SNAP_STATION_STATE_BUSY)
            {
                wait_for_start_button();
            }
            else if (joypad_get_buttons(JOYPAD_PORT_1).start)
            {
                break;
            }
            else if (!cancel_message_shown)
            {
                printf("Waiting for Snap Station; press Start to cancel\n");
                console_render();
                cancel_message_shown = true;
            }
        }
    }
    while (state == JOYBUS_N64_SNAP_STATION_STATE_BUSY);

}

static void snap_station_command(uint8_t command)
{
    const uint16_t addr = JOYBUS_N64_ACCESSORY_ADDR_SNAP_STATE;
    uint8_t data[JOYBUS_N64_ACCESSORY_DATA_SIZE] = {0};
    data[sizeof(data)-1] = command;
    const char * command_str = snap_station_state_format(command);
    printf("Writing 0x%02X (%s) command...\n", command, command_str);
    int crc_status = joybus_n64_accessory_write_sync(JOYPAD_PORT_4, addr, data);
    if (crc_status != JOYBUS_N64_ACCESSORY_DATA_CRC_STATUS_OK)
    {
        const char *crc_error = accessory_data_crc_status_format(crc_status);
        printf("Accessory Write error: %s\n", crc_error);
        wait_for_start_button();
    }
    else
    {
        printf("Accessory Write complete! ");
        snap_station_state_read();
    }
}

int main(void)
{
    timer_init();
    display_init(RESOLUTION_320x240, DEPTH_32_BPP, 2, GAMMA_NONE, ANTIALIAS_RESAMPLE);
    debug_init_isviewer();

    console_init();
    console_set_render_mode(RENDER_MANUAL);
    console_set_debug(false);

    console_clear();
    printf("%s\n\n", ROM_TITLE);
    printf("Pre-check to determine if Snap Station is already active:\n");
    snap_station_probe_read();

    joypad_init();

    while (1)
    {
        console_clear();
        printf("%s\n\n", ROM_TITLE);

        joypad_scan();
        pressed = joypad_get_buttons_pressed(JOYPAD_PORT_1);

        if (joypad_get_style(JOYPAD_PORT_4) != JOYPAD_STYLE_N64)
        {
            printf("Please Connect a Snap Station on port 4 to test\n");
        }
        else if (joypad_get_accessory_type(JOYPAD_PORT_4) != JOYPAD_ACCESSORY_TYPE_SNAP_STATION)
        {
            printf("Accessory on port 4 is not a Snap Station\n");
        }
        else
        {
            printf("Snap Station detected on port 4!\n");
        }

        printf("Command List:\n");
        printf("R       = Read State\n");
        printf("A       = Pre-Save      (0x%02X)\n", JOYBUS_N64_SNAP_STATION_STATE_PRE_SAVE);
        printf("B       = Post-Save     (0x%02X)\n", JOYBUS_N64_SNAP_STATION_STATE_POST_SAVE);
        printf("C-Down  = Reset Console (0x%02X)\n", JOYBUS_N64_SNAP_STATION_STATE_RESET_CONSOLE);
        printf("C-Left  = Pre-Roll      (0x%02X)\n", JOYBUS_N64_SNAP_STATION_STATE_PRE_ROLL);
        printf("C-Up    = Capture Photo (0x%02X)\n", JOYBUS_N64_SNAP_STATION_STATE_CAPTURE_PHOTO);
        printf("C-Right = Post-Roll     (0x%02X)\n", JOYBUS_N64_SNAP_STATION_STATE_POST_ROLL);
        printf("\n");
        console_render();

        if (pressed.r)
            snap_station_state_read();
        else if (pressed.a)
            snap_station_command(JOYBUS_N64_SNAP_STATION_STATE_PRE_SAVE);
        else if (pressed.b)
            snap_station_command(JOYBUS_N64_SNAP_STATION_STATE_POST_SAVE);
        else if (pressed.c_down)
            snap_station_command(JOYBUS_N64_SNAP_STATION_STATE_RESET_CONSOLE);
        else if (pressed.c_left)
            snap_station_command(JOYBUS_N64_SNAP_STATION_STATE_PRE_ROLL);
        else if (pressed.c_up)
            snap_station_command(JOYBUS_N64_SNAP_STATION_STATE_CAPTURE_PHOTO);
        else if (pressed.c_right)
            snap_station_command(JOYBUS_N64_SNAP_STATION_STATE_POST_ROLL);
    }
}

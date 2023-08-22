/**
 * @file main.c
 * @author Christopher Bonhage (me@christopherbonhage.com)
 * @brief N64 test ROM for Joypad subsystem
 */

#ifdef ROM_VERSION
static const char ROM_TITLE[] = "N64 Accessory Short Write Test " ROM_VERSION " by Meeq";
#else
static const char ROM_TITLE[] = "N64 Accessory Short Write Test by Meeq";
#endif

#include <string.h>
#include <libdragon.h>

#include "joybus_commands.h"
#include "joybus_n64_accessory.h"
#include "joypad.h"

const char *format_joypad_style(joypad_style_t style)
{
    switch (style)
    {
    case JOYPAD_STYLE_NONE:
        return "None   ";
    case JOYPAD_STYLE_N64:
        return "N64    ";
    case JOYPAD_STYLE_GCN:
        return "GCN    ";
    case JOYPAD_STYLE_MOUSE:
        return "Mouse  ";
    default:
        return "Unknown";
    }
}

const char *format_joypad_accessory_type(joypad_accessory_type_t accessory_type)
{
    switch (accessory_type)
    {
    case JOYPAD_ACCESSORY_TYPE_NONE:
        return "None        ";
    case JOYPAD_ACCESSORY_TYPE_CONTROLLER_PAK:
        return "Memory      ";
    case JOYPAD_ACCESSORY_TYPE_RUMBLE_PAK:
        return "Rumble Pak  ";
    case JOYPAD_ACCESSORY_TYPE_TRANSFER_PAK:
        return "Transfer Pak";
    case JOYPAD_ACCESSORY_TYPE_BIO_SENSOR:
        return "Bio Sensor  ";
    case JOYPAD_ACCESSORY_TYPE_SNAP_STATION:
        return "Snap Station";
    default:
        return "Unknown     ";
    }
}

const char *format_joypad_rumble(bool supported, bool enabled)
{
    if (!supported) return "Unavailable";
    if (enabled) return "Active";
    return "Idle";
}

void print_joypad_inputs(joypad_inputs_t inputs)
{
    printf("Stick: %+04d,%+04d C-Stick: %+04d,%+04d L-Trig:%03d R-Trig:%03d\n", inputs.stick_x, inputs.stick_y, inputs.cstick_x, inputs.cstick_y, inputs.analog_l, inputs.analog_r);
    printf("A:%d B:%d X:%d Y:%d L:%d R:%d Z:%d Start:%d\n", inputs.a, inputs.b, inputs.x, inputs.y, inputs.l, inputs.r, inputs.z, inputs.start);
    printf("D-U:%d D-D:%d D-L:%d D-R:%d C-U:%d C-D:%d C-L:%d C-R:%d\n", inputs.d_up, inputs.d_down, inputs.d_left, inputs.d_right, inputs.c_up, inputs.c_down, inputs.c_left, inputs.c_right);
}

void accessory_short_write_sync(int port, uint16_t addr, const uint8_t *data, size_t data_len)
{
    ASSERT_JOYBUS_CONTROLLER_PORT_VALID(port);
    uint8_t input[JOYBUS_BLOCK_SIZE] = {0};
    uint8_t output[JOYBUS_BLOCK_SIZE] = {0};
    size_t i = port * JOYBUS_COMMAND_SKIP_SIZE;

    const size_t send_len = data_len + 1; // Plus one for command ID
    const size_t recv_len = 1;
    const uint16_t addr_checksum = joybus_n64_accessory_addr_checksum(addr);

    input[i++] = send_len;
    input[i++] = recv_len;
    input[i++] = JOYBUS_COMMAND_ID_N64_ACCESSORY_WRITE;
    input[i++] = (addr_checksum >> 8) & 0xFF;
    input[i++] = addr_checksum & 0xFF;
    memcpy(&input[i], data, data_len);
    i += data_len;

    // Close out the Joybus operation block
    input[i] = 0xFE;
    input[sizeof(input) - 1] = 0x01;

    joybus_exec(input, output);
}

int main(void)
{
    timer_init();
    display_init(RESOLUTION_320x240, DEPTH_32_BPP, 2, GAMMA_NONE, ANTIALIAS_RESAMPLE);
    debug_init_isviewer();

    joypad_init();

    joypad_style_t style;
    joypad_accessory_type_t accessory_type;
    bool rumble_supported;
    bool rumble_active;
    joypad_inputs_t inputs;
    uint8_t accessory_data[JOYBUS_N64_ACCESSORY_DATA_SIZE];
    size_t data_len[JOYBUS_CONTROLLER_PORT_COUNT] = {
        JOYBUS_N64_ACCESSORY_DATA_SIZE,
        JOYBUS_N64_ACCESSORY_DATA_SIZE,
        JOYBUS_N64_ACCESSORY_DATA_SIZE,
        JOYBUS_N64_ACCESSORY_DATA_SIZE,
    };

    console_init();
    console_set_render_mode(RENDER_MANUAL);
    console_set_debug(false);

    while (1)
    {
        console_clear();

        printf("%s\n", ROM_TITLE);
        printf("\n");
        printf("Connect a Rumble Pak, then press A to start and B to stop.\n\n");

        joypad_scan();

        JOYPAD_PORT_FOREACH (port)
        {
            style = joypad_get_style(port);
            accessory_type = joypad_get_accessory_type(port);
            rumble_supported = joypad_get_rumble_supported(port);
            rumble_active = joypad_get_rumble_active(port);
            inputs = joypad_get_inputs(port);

            if (accessory_type == JOYPAD_ACCESSORY_TYPE_RUMBLE_PAK)
            {
                if (inputs.c_up)
                {
                    data_len[port]++;
                    if (data_len[port] > 32) data_len[port] = 1;
                }
                else if (inputs.c_down)
                {
                    data_len[port]--;
                    if (data_len[port] < 1) data_len[port] = 32;
                }
                if (inputs.a)
                {
                    memset(accessory_data, 0x01, sizeof(accessory_data));
                    accessory_short_write_sync(
                        port,
                        JOYBUS_N64_ACCESSORY_ADDR_RUMBLE_MOTOR,
                        accessory_data,
                        data_len[port]
                    );
                }
                else if (inputs.b)
                {
                    memset(accessory_data, 0x00, sizeof(accessory_data));
                    accessory_short_write_sync(
                        port,
                        JOYBUS_N64_ACCESSORY_ADDR_RUMBLE_MOTOR,
                        accessory_data,
                        data_len[port]
                    );
                }
            }

            printf("Port %d ", port + 1);
            printf("Style: %s ", format_joypad_style(style));
            printf("Pak: %s ", format_joypad_accessory_type(accessory_type));
            printf("Rumble: %s\n", format_joypad_rumble(rumble_supported, rumble_active));
            print_joypad_inputs(inputs);
            printf("Accessory write bytes: %02d", data_len[port]);
            printf("\n");
        }

        console_render();
    }
}

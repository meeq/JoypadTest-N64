/**
 * @file main.c
 * @author Christopher Bonhage (me@christopherbonhage.com)
 * @brief N64 test ROM for Joypad subsystem
 */

#include <string.h>
#include <libdragon.h>

#include "joypad.h"

const char *format_joypad_style(joypad_style_t style)
{
    switch (style)
    {
    case JOYPAD_STYLE_NONE:
        return "None";
    case JOYPAD_STYLE_N64:
        return "N64";
    case JOYPAD_STYLE_GCN:
        return "GCN";
    default:
        return "Unknown";
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
    printf("Stick: %d,%d C-Stick: %d,%d L-Trig:%d R-Trig:%d\n", inputs.stick_x, inputs.stick_y, inputs.cstick_x, inputs.cstick_y, inputs.analog_l, inputs.analog_r);
    printf("A:%d B:%d X:%d Y:%d L:%d R:%d Z:%d Start:%d\n", inputs.a, inputs.b, inputs.x, inputs.y, inputs.l, inputs.r, inputs.z, inputs.start);
    printf("D-U:%d D-D:%d D-L:%d D-R:%d C-U:%d C-D:%d C-L:%d C-R:%d\n", inputs.d_up, inputs.d_down, inputs.d_left, inputs.d_right, inputs.c_up, inputs.c_down, inputs.c_left, inputs.c_right);
}

int main(void)
{
    timer_init();
    display_init(
        RESOLUTION_320x240,
        DEPTH_32_BPP,
        2,
        GAMMA_NONE,
        ANTIALIAS_RESAMPLE
    );
    debug_init_isviewer();

    joypad_init();

    joypad_style_t style;
    bool rumble_supported;
    bool rumble_state;
    joypad_inputs_t inputs;

    console_init();
    console_set_render_mode(RENDER_MANUAL);

    while (1)
    {
        console_clear();

        printf("N64/GCN Joypad Subsystem Test\n\n");

        joypad_poll();

        for (joypad_port_t port = JOYPAD_PORT_1; port < JOYPAD_PORT_COUNT; ++port)
        {
            style = joypad_style(port);
            rumble_supported = joypad_is_rumble_supported(port);
            rumble_state = joypad_get_rumble_state(port);
            inputs = joypad_inputs(port);

            if (rumble_supported)
            {
                if (inputs.a && !rumble_state)
                {
                    joypad_set_rumble_state(port, true);
                }
                else if (!inputs.a && rumble_state)
                {
                    joypad_set_rumble_state(port, false);
                }
            }

            printf("Port %d ", port + 1);
            printf("Style: %s ", format_joypad_style(style));
            printf("Rumble: %s\n", format_joypad_rumble(rumble_supported, rumble_state));
            print_joypad_inputs(inputs);
            printf("\n");
        }

        console_render();
    }
}

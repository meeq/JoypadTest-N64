/**
 * @file main.c
 * @author Christopher Bonhage (me@christopherbonhage.com)
 * @brief N64 test ROM for Joypad subsystem
 */

#include <string.h>
#include <libdragon.h>

#include "joypad.h"
#include "bio_sensor.h"

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
    joypad_scan();

    bio_sensor_init();

    console_init();
    console_set_render_mode(RENDER_MANUAL);
    console_set_debug(false);

    joypad_inputs_t pressed;
    int bpm;

    while (1)
    {
        console_clear();
        joypad_scan();

        printf("Bio Sensor Test v0.7\n");
        printf("A silly experiment by Meeq\n");
        printf("Connect up to 4 controllers with Bio Sensors\n");
        printf("\n");
        printf("Press A to start reading the Bio Sensor\n");
        printf("Press B to stop reading the Bio Sensor\n");
        printf("Press Z to simulate a pulse without a Bio Sensor\n");
        printf("\n");

        JOYPAD_PORT_FOR_EACH (port)
        {
            pressed = joypad_pressed(port);
            bpm = bio_sensor_get_bpm(port);

            printf("Port %d ", port + 1);
            printf("BPM: %03d", bpm);
            if (bio_sensor_get_active(port))
            {
                if (pressed.b)
                {
                    bio_sensor_read_stop(port);
                    printf(" (Stopping)");
                }
                else
                {
                    printf(" (Active)");
                }
            }
            else
            {
                if (pressed.a)
                {
                    bio_sensor_read_start(port, false);
                    printf(" (Starting)");
                }
                else if (pressed.z)
                {
                    bio_sensor_read_start(port, true);
                    printf(" (Simulating)");
                }
                else
                {
                    printf(" (Stopped)");
                }
            }
            printf("\n");
        }

        console_render();
    }
}

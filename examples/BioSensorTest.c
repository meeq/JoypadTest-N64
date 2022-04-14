/**
 * @file main.c
 * @author Christopher Bonhage (me@christopherbonhage.com)
 * @brief N64 test ROM for Bio Sensor subsystem
 */

#ifdef ROM_VERSION
static const char ROM_TITLE[] = "Bio Sensor Subsystem Test " ROM_VERSION " by Meeq";
#else
static const char ROM_TITLE[] = "Bio Sensor Subsystem Test by Meeq";
#endif

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

    joypad_buttons_t pressed;
    joypad_accessory_type_t accessory_type;
    int bpm;

    while (1)
    {
        console_clear();
        joypad_scan();

        printf("%s\n\n", ROM_TITLE);
        printf("Connect up to 4 controllers with Bio Sensor accessories\n");
        printf("\n");
        printf("Press A to start reading the Bio Sensor\n");
        printf("Press B to stop reading the Bio Sensor\n");
        printf("\n");

        JOYPAD_PORT_FOREACH (port)
        {
            pressed = joypad_get_buttons_pressed(port);
            accessory_type = joypad_get_accessory_type(port);
            bpm = bio_sensor_get_bpm(port);

            printf("Port %d ", port + 1);
            printf("BPM: %03d ", bpm);
            if (bio_sensor_get_active(port))
            {
                if (pressed.b)
                {
                    bio_sensor_read_stop(port);
                    printf("(Stopping)");
                }
                else if (bio_sensor_get_pulsing(port))
                {
                    printf("(Pulsing)");
                }
                else
                {
                    printf("(Resting)");
                }
            }
            else if (accessory_type == JOYPAD_ACCESSORY_TYPE_BIO_SENSOR)
            {
                if (pressed.a)
                {
                    bio_sensor_read_start(port);
                    printf("(Starting)");
                }
                else
                {
                    printf("(Stopped)");
                }
            }
            else
            {
                printf("(Unavailable)");
            }
            printf("\n");
        }

        console_render();
    }
}

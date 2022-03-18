/**
 * @file main.c
 * @author Christopher Bonhage (me@christopherbonhage.com)
 * @brief N64 test ROM for Joypad subsystem
 */

#include <string.h>
#include <libdragon.h>

#include "joybus_commands.h"
#include "joybus_n64_accessory.h"
#include "joypad.h"

#define BEAT_PERIODS_COUNT 60
#define BEAT_PERIOD_INTERVAL_TICKS TICKS_PER_SECOND
#define TICKS_PER_MINUTE (TICKS_PER_SECOND * 60)
#define BEAT_PERIODS_PER_MINUTE ((float)TICKS_PER_MINUTE / (float)BEAT_PERIOD_INTERVAL_TICKS)

typedef enum
{
    BIO_SENSOR_STATE_IDLE = 0,
    BIO_SENSOR_STATE_COUNTING,
    BIO_SENSOR_STATE_PULSING
} bio_sensor_state_t;

typedef struct
{
    bio_sensor_state_t state;
    int64_t period_start_ticks;
    unsigned period_beats;
    unsigned period_cursor;
    unsigned period_counter;
    unsigned beats_per_period[BEAT_PERIODS_COUNT];
} bio_sensor_reader_t;

static volatile bio_sensor_reader_t bio_sensor_readers[JOYPAD_PORT_COUNT] = {0};

void bio_sensor_read_callback(uint64_t *out_dwords, void *ctx)
{
    const uint8_t *out_bytes = (void *)out_dwords;
    joypad_port_t port = (joypad_port_t)ctx;
    volatile bio_sensor_reader_t *reader = &bio_sensor_readers[port];
    bio_sensor_state_t current_state = reader->state;
    if (current_state == BIO_SENSOR_STATE_IDLE) return;

    const joybus_cmd_n64_accessory_read_port_t *recv_cmd = (void *)&out_bytes[port];
    int crc_status = joybus_n64_accessory_data_crc_compare(recv_cmd->data, recv_cmd->data_crc);
    if (crc_status != JOYBUS_N64_ACCESSORY_DATA_CRC_STATUS_OK)
    {
        reader->state = BIO_SENSOR_STATE_IDLE;
        reader->period_counter = 0;
        return;
    }

    int64_t now_ticks = timer_ticks();
    unsigned period_beats = reader->period_beats;
    if (reader->period_start_ticks + BEAT_PERIOD_INTERVAL_TICKS < now_ticks)
    {
        if (period_beats > 0)
        {
            unsigned cursor = reader->period_cursor;
            reader->beats_per_period[cursor++] = reader->period_beats;
            if (cursor >= BEAT_PERIODS_COUNT) cursor = 0;
            reader->period_cursor = cursor;
            reader->period_beats = 0;
            reader->period_counter++;
        }
        reader->period_start_ticks = now_ticks;
    }

    uint8_t byte = recv_cmd->data[0];
    bio_sensor_state_t next_state = BIO_SENSOR_STATE_IDLE;
    if (byte == 0x00) next_state = BIO_SENSOR_STATE_PULSING;
    if (byte == 0x03) next_state = BIO_SENSOR_STATE_COUNTING;

    if (
        current_state == BIO_SENSOR_STATE_PULSING &&
        next_state == BIO_SENSOR_STATE_COUNTING
    ) {
        reader->period_beats = period_beats + 1;
    }
    reader->state = next_state;

    joybus_n64_accessory_read_async(
        port,
        0xC000,
        bio_sensor_read_callback,
        (void *)port
    );
}

void bio_sensor_read_stop(joypad_port_t port)
{
    bio_sensor_readers[port].state = BIO_SENSOR_STATE_IDLE;
    bio_sensor_readers[port].period_counter = 0;
}

void bio_sensor_read_start(joypad_port_t port)
{
    volatile bio_sensor_reader_t *reader = &bio_sensor_readers[port];
    memset((void *)reader, 0, sizeof(*reader));
    reader->state = BIO_SENSOR_STATE_COUNTING;
    reader->period_start_ticks = timer_ticks();
    joybus_n64_accessory_read_async(
        port,
        0xC000,
        bio_sensor_read_callback,
        (void *)port
    );
}

float bio_sensor_read_pulse(joypad_port_t port)
{
    volatile bio_sensor_reader_t *reader = &bio_sensor_readers[port];
    size_t num_periods = reader->period_counter;
    if (num_periods > BEAT_PERIODS_COUNT) num_periods = BEAT_PERIODS_COUNT;
    if (num_periods == 0) return 0.0;
    float sum = 0.0;
    for (size_t i = 0; i < num_periods; i++)
    {
        sum += reader->beats_per_period[i];
    }
    return (sum / (float)num_periods) * BEAT_PERIODS_PER_MINUTE;
}

char * format_reader_state(bio_sensor_state_t state)
{
    switch (state)
    {
        case BIO_SENSOR_STATE_IDLE:     return "Idle    ";
        case BIO_SENSOR_STATE_COUNTING: return "Counting";
        case BIO_SENSOR_STATE_PULSING:  return "Pulsing ";
        default:                        return "Unknown ";
    }
}

int main(void)
{
    timer_init();
    display_init(RESOLUTION_320x240, DEPTH_32_BPP, 2, GAMMA_NONE, ANTIALIAS_RESAMPLE);
    debug_init_isviewer();

    joypad_init();
    joypad_scan();

    joypad_style_t prev_styles[JOYPAD_PORT_COUNT] = {0};
    float pulses[JOYPAD_PORT_COUNT] = {0};
    joypad_style_t style;
    joypad_inputs_t pressed;
    bio_sensor_state_t state;

    console_init();
    console_set_render_mode(RENDER_MANUAL);
    console_set_debug(false);

    while (1)
    {
        console_clear();

        printf("Bio Sensor Test\n");
        printf("Connect up to 4 controllers with Bio Sensors\n");
        printf("Press A to start reading the pulse rate\n");
        printf("\n");

        joypad_scan();
        disable_interrupts();
        JOYPAD_PORT_FOR_EACH (port)
        {
            pulses[port] = bio_sensor_read_pulse(port);
        }
        enable_interrupts();

        JOYPAD_PORT_FOR_EACH (port)
        {
            style = joypad_get_style(port);
            pressed = joypad_pressed(port);
            if (style != JOYPAD_STYLE_N64 && prev_styles[port] == JOYPAD_STYLE_N64)
            {
                bio_sensor_read_stop(port);
            }

            printf("Port %d ", port + 1);
            if (style == JOYPAD_STYLE_N64)
            {
                state = bio_sensor_readers[port].state;
                if (state)
                {
                    printf("%s", format_reader_state(state));
                    printf("BPM: %f", pulses[port]);
                }
                else
                {
                    printf("Pulse Reading Stopped!");
                    if (pressed.a)
                    {
                        bio_sensor_read_start(port);
                    }
                }
            } 
            else
            {
                printf("Bio Sensor Not Supported");
            }
            printf("\n");
        }

        console_render();
    }
}

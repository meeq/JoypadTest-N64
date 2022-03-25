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

#define BEAT_PERIODS_COUNT 16
#define BEAT_PERIODS_MINIMUM 8
#define BEAT_PERIOD_INTERVAL_TICKS (TICKS_PER_SECOND / 2)
#define BEAT_PERIODS_PER_MINUTE (60 * 2)

typedef enum
{
    BIO_SENSOR_STATE_IDLE = 0,
    BIO_SENSOR_STATE_COUNTING,
    BIO_SENSOR_STATE_PULSING
} bio_sensor_state_t;

typedef struct
{
    bool simulated;
    bool read_pending;
    bio_sensor_state_t state;
    int64_t period_start_ticks;
    unsigned period_beats;
    unsigned period_cursor;
    unsigned period_counter;
    unsigned beats_per_period[BEAT_PERIODS_COUNT];
} bio_sensor_reader_t;

static volatile bio_sensor_reader_t bio_sensor_readers[JOYPAD_PORT_COUNT] = {0};

void bio_sensor_read_stop(joypad_port_t port)
{
    bio_sensor_readers[port].state = BIO_SENSOR_STATE_IDLE;
    bio_sensor_readers[port].period_counter = 0;
}

void bio_sensor_read_start(joypad_port_t port, bool simulated)
{
    if (bio_sensor_readers[port].state != BIO_SENSOR_STATE_IDLE) return;
    volatile bio_sensor_reader_t *reader = &bio_sensor_readers[port];
    memset((void *)reader, 0, sizeof(*reader));
    reader->state = BIO_SENSOR_STATE_COUNTING;
    reader->period_start_ticks = timer_ticks();
    reader->simulated = simulated;
}

static void bio_sensor_period_tick(joypad_port_t port)
{
    volatile bio_sensor_reader_t *reader = &bio_sensor_readers[port];
    unsigned period_beats = reader->period_beats;
    int64_t now_ticks = timer_ticks();
    if (reader->period_start_ticks + BEAT_PERIOD_INTERVAL_TICKS < now_ticks)
    {
        unsigned cursor = reader->period_cursor;
        reader->beats_per_period[cursor++] = period_beats;
        if (cursor >= BEAT_PERIODS_COUNT) cursor = 0;
        reader->period_cursor = cursor;
        reader->period_beats = 0;
        reader->period_counter++;
        reader->period_start_ticks = now_ticks;
    }
}

void bio_sensor_read_callback(uint64_t *out_dwords, void *ctx)
{
    const uint8_t *out_bytes = (void *)out_dwords;
    joypad_port_t port = (joypad_port_t)ctx;
    volatile bio_sensor_reader_t *reader = &bio_sensor_readers[port];
    bio_sensor_state_t current_state = reader->state;
    // Ignore this read if this sensor has been stopped
    if (current_state == BIO_SENSOR_STATE_IDLE) return;

    uint8_t sensor_data;
    if (reader->simulated)
    {
        joypad_inputs_t held = joypad_held(port);
        sensor_data = held.z ? 0x00 : 0x03;
    }
    else
    {
        const joybus_cmd_n64_accessory_read_port_t *recv_cmd = (void *)&out_bytes[port];
        int crc_status = joybus_n64_accessory_data_crc_compare(recv_cmd->data, recv_cmd->data_crc);
        if (crc_status == JOYBUS_N64_ACCESSORY_DATA_CRC_STATUS_OK)
        {
            sensor_data = recv_cmd->data[0];
        }
        else
        {
            // Stop reading if the Bio Sensor has been disconnected
            if (crc_status == JOYBUS_N64_ACCESSORY_DATA_CRC_STATUS_DISCONNECTED)
            {
                bio_sensor_read_stop(port);
            }
            // Skip this read if it fails the CRC check
            reader->read_pending = false;
            return;
        }
    }

    bio_sensor_period_tick(port);

    bio_sensor_state_t next_state = BIO_SENSOR_STATE_IDLE;
    if (sensor_data == 0x00) next_state = BIO_SENSOR_STATE_PULSING;
    if (sensor_data == 0x03) next_state = BIO_SENSOR_STATE_COUNTING;

    if (
        current_state == BIO_SENSOR_STATE_PULSING &&
        next_state == BIO_SENSOR_STATE_COUNTING
    ) {
        reader->period_beats += 1;
    }
    reader->state = next_state;
    reader->read_pending = false;
}

void bio_sensor_vi_interrupt_callback(void)
{
    JOYPAD_PORT_FOR_EACH (port)
    {
        if (
            bio_sensor_readers[port].read_pending == false &&
            bio_sensor_readers[port].state != BIO_SENSOR_STATE_IDLE
        )
        {
            bio_sensor_readers[port].read_pending = true;
            joybus_n64_accessory_read_async(
                port,
                0xC000,
                bio_sensor_read_callback,
                (void *)port
            );
        }
    }
}

int bio_sensor_calculate_pulse(joypad_port_t port)
{
    volatile bio_sensor_reader_t *reader = &bio_sensor_readers[port];
    float num_periods = reader->period_counter;
    if (num_periods < BEAT_PERIODS_MINIMUM)
    {
        // Prevent divide-by-zero
        return 0;
    }
    if (num_periods > BEAT_PERIODS_COUNT) 
    {
        // Prevent reads past the end of beats_per_period
        num_periods = BEAT_PERIODS_COUNT;
    }
    float sum = 0.0;
    for (size_t i = 0; i < num_periods; i++)
    {
        sum += reader->beats_per_period[i];
    }
    return (sum / num_periods) * BEAT_PERIODS_PER_MINUTE;
}

const char * format_reader_state(bio_sensor_state_t state)
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

    register_VI_handler(bio_sensor_vi_interrupt_callback);

    unsigned pulses[JOYPAD_PORT_COUNT] = {0};
    joypad_inputs_t pressed;
    bio_sensor_state_t state;

    console_init();
    console_set_render_mode(RENDER_MANUAL);
    console_set_debug(false);

    while (1)
    {
        console_clear();
        joypad_scan();

        disable_interrupts();
        JOYPAD_PORT_FOR_EACH (port)
        {
            pulses[port] = bio_sensor_calculate_pulse(port);
        }
        enable_interrupts();

        printf("Bio Sensor Test\n");
        printf("Connect up to 4 controllers with Bio Sensors\n");
        printf("\n");

        JOYPAD_PORT_FOR_EACH (port)
        {
            pressed = joypad_pressed(port);
            state = bio_sensor_readers[port].state;

            printf("Port %d ", port + 1);
            printf("%s ", format_reader_state(state));
            if (state == BIO_SENSOR_STATE_IDLE)
            {
                printf("Press A to start reading pulse");
                if (pressed.a)
                {
                    bio_sensor_read_start(port, false);
                }
                printf(" or Z to simulate");
                if (pressed.z)
                {
                    bio_sensor_read_start(port, true);
                }
            }
            else
            {
                printf("BPM: %d", pulses[port]);
                if (pressed.b)
                {
                    bio_sensor_read_stop(port);
                }
            }
            printf("\n");
        }

        console_render();
    }
}

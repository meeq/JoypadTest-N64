/**
 * @file bio_sensor.c
 * @author Christopher Bonhage (me@christopherbonhage.com)
 * @brief Bio Sensor Subsystem
 * @ingroup bio_sensor
 */

#include <string.h>
#include <libdragon.h>

#include "joybus_commands.h"
#include "joybus_n64_accessory.h"
#include "bio_sensor.h"

#define BIO_SENSOR_PERIODS_MINIMUM 8
#define BIO_SENSOR_PERIODS_MAXIMUM 16
#define BIO_SENSOR_PERIOD_INTERVAL_TICKS (TICKS_PER_SECOND / 2)
#define BIO_SENSOR_PERIODS_PER_MINUTE (60 * 2)

typedef enum
{
    BIO_SENSOR_STATE_STOPPED = 0,
    BIO_SENSOR_STATE_RESTING,
    BIO_SENSOR_STATE_PULSING,
} bio_sensor_state_t;

typedef struct
{
    bool read_pending;
    bio_sensor_state_t state;
    int64_t period_start_ticks;
    unsigned period_beats;
    unsigned period_cursor;
    unsigned period_counter;
    unsigned beats_per_period[BIO_SENSOR_PERIODS_MAXIMUM];
} bio_sensor_reader_t;

static volatile bio_sensor_reader_t bio_sensor_readers[JOYBUS_CONTROLLER_PORT_COUNT] = {0};

static void bio_sensor_read_callback(uint64_t *out_dwords, void *ctx)
{
    const uint8_t *out_bytes = (void *)out_dwords;
    int port = (int)ctx;
    volatile bio_sensor_reader_t *reader = &bio_sensor_readers[port];
    bio_sensor_state_t current_state = reader->state;
    // Ignore this read if this sensor has been stopped
    if (current_state == BIO_SENSOR_STATE_STOPPED) return;

    const joybus_cmd_n64_accessory_read_port_t *recv_cmd = (void *)&out_bytes[port];
    int crc_status = joybus_n64_accessory_data_crc_compare(recv_cmd->data, recv_cmd->data_crc);
    if (crc_status != JOYBUS_N64_ACCESSORY_DATA_CRC_STATUS_OK)
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

    int64_t now_ticks = timer_ticks();
    if (reader->period_start_ticks + BIO_SENSOR_PERIOD_INTERVAL_TICKS < now_ticks)
    {
        unsigned cursor = reader->period_cursor;
        reader->beats_per_period[cursor++] = reader->period_beats;
        if (cursor >= BIO_SENSOR_PERIODS_MAXIMUM) cursor = 0;
        reader->period_cursor = cursor;
        reader->period_beats = 0;
        reader->period_counter++;
        reader->period_start_ticks = now_ticks;
    }

    uint8_t sensor_data = recv_cmd->data[0];
    bio_sensor_state_t next_state = BIO_SENSOR_STATE_STOPPED;
    if (sensor_data == 0x00) next_state = BIO_SENSOR_STATE_PULSING;
    if (sensor_data == 0x03) next_state = BIO_SENSOR_STATE_RESTING;

    if (
        current_state == BIO_SENSOR_STATE_PULSING &&
        next_state == BIO_SENSOR_STATE_RESTING
    ) {
        reader->period_beats += 1;
    }
    reader->state = next_state;
    reader->read_pending = false;
}

static void bio_sensor_vi_interrupt_callback(void)
{
    for (int port = 0; port < JOYBUS_CONTROLLER_PORT_COUNT; ++port)
    {
        if (
            bio_sensor_readers[port].read_pending == false &&
            bio_sensor_readers[port].state != BIO_SENSOR_STATE_STOPPED
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

void bio_sensor_init(void)
{
    register_VI_handler(bio_sensor_vi_interrupt_callback);
}

void bio_sensor_close(void)
{
    unregister_VI_handler(bio_sensor_vi_interrupt_callback);
}

void bio_sensor_read_start(int port)
{
    ASSERT_JOYBUS_CONTROLLER_PORT_VALID(port);
    if (bio_sensor_readers[port].state != BIO_SENSOR_STATE_STOPPED) return;
    volatile bio_sensor_reader_t *reader = &bio_sensor_readers[port];
    memset((void *)reader, 0, sizeof(*reader));
    reader->state = BIO_SENSOR_STATE_RESTING;
    reader->period_start_ticks = timer_ticks();
}

void bio_sensor_read_stop(int port)
{
    ASSERT_JOYBUS_CONTROLLER_PORT_VALID(port);
    volatile bio_sensor_reader_t *reader = &bio_sensor_readers[port];
    memset((void *)reader, 0, sizeof(*reader));
}

bool bio_sensor_get_active(int port)
{
    ASSERT_JOYBUS_CONTROLLER_PORT_VALID(port);
    return bio_sensor_readers[port].state != BIO_SENSOR_STATE_STOPPED;
}

bool bio_sensor_get_pulsing(int port)
{
    ASSERT_JOYBUS_CONTROLLER_PORT_VALID(port);
    return bio_sensor_readers[port].state == BIO_SENSOR_STATE_PULSING;
}

int bio_sensor_get_bpm(int port)
{
    ASSERT_JOYBUS_CONTROLLER_PORT_VALID(port);
    volatile bio_sensor_reader_t *reader = &bio_sensor_readers[port];
    float num_periods = reader->period_counter;
    if (num_periods < BIO_SENSOR_PERIODS_MINIMUM)
    {
        // Prevent divide-by-zero
        return 0;
    }
    if (num_periods > BIO_SENSOR_PERIODS_MAXIMUM) 
    {
        // Prevent reads past the end of beats_per_period
        num_periods = BIO_SENSOR_PERIODS_MAXIMUM;
    }
    float sum = 0.0;
    for (size_t i = 0; i < num_periods; i++)
    {
        sum += reader->beats_per_period[i];
    }
    return (sum / num_periods) * BIO_SENSOR_PERIODS_PER_MINUTE;
}

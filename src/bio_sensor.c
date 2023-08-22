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
#include "joypad.h"
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

typedef struct __attribute__((packed))
{
    /* metadata */
    uint8_t send_len;
    uint8_t recv_len;
    /* send data */
    union
    {
        uint8_t send_bytes[0x03];
        struct __attribute__((__packed__))
        {
            uint8_t command;
            uint16_t addr_checksum;
        };
    };
    /* recv_data */
    union
    {
        uint8_t recv_bytes[0x01];
        uint8_t data;
    };
} bio_sensor_short_read_port_t;

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
    int64_t now_ticks = timer_ticks();
    const uint8_t *out_bytes = (void *)out_dwords;
    uint8_t send_len, recv_len, command_id;
    size_t i = 0;

    JOYPAD_PORT_FOREACH (port)
    {
        // Check send_len to figure out if this port has a command on it
        send_len = out_bytes[i + JOYBUS_COMMAND_OFFSET_SEND_LEN];
        if (send_len == 0)
        {
            i += JOYBUS_COMMAND_SKIP_SIZE;
            continue;
        }
        else
        {
            recv_len = out_bytes[i + JOYBUS_COMMAND_OFFSET_RECV_LEN];
            command_id = out_bytes[i + JOYBUS_COMMAND_OFFSET_COMMAND_ID];
        }

        volatile bio_sensor_reader_t *reader = &bio_sensor_readers[port];
        bio_sensor_state_t current_state = reader->state;
        if (
            current_state == BIO_SENSOR_STATE_STOPPED ||
            command_id != JOYBUS_COMMAND_ID_N64_ACCESSORY_READ
        )
        {
            i += JOYBUS_COMMAND_METADATA_SIZE + send_len + recv_len;
            continue;
        }

        const bio_sensor_short_read_port_t *recv_cmd;
        recv_cmd = (void *)&out_bytes[i];
        i += sizeof(*recv_cmd);
        
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

        uint8_t sensor_data = recv_cmd->data;
        bio_sensor_state_t next_state = BIO_SENSOR_STATE_STOPPED;
        if (sensor_data == 0x00) next_state = BIO_SENSOR_STATE_PULSING;
        if (sensor_data == 0x03) next_state = BIO_SENSOR_STATE_RESTING;

        if (
            current_state == BIO_SENSOR_STATE_PULSING &&
            next_state == BIO_SENSOR_STATE_RESTING
        )
        {
            reader->period_beats += 1;
        }
        reader->state = next_state;
        reader->read_pending = false;
    }
}

static void bio_sensor_vi_interrupt_callback(void)
{
    const uint16_t addr = JOYBUS_N64_ACCESSORY_ADDR_BIO_PULSE;
    const bio_sensor_short_read_port_t send_cmd = {
        .send_len = sizeof(send_cmd.send_bytes),
        .recv_len = sizeof(send_cmd.recv_bytes),
        .command = JOYBUS_COMMAND_ID_N64_ACCESSORY_READ,
        .addr_checksum = joybus_n64_accessory_addr_checksum(addr),
    };
    uint8_t input[JOYBUS_BLOCK_SIZE] = {0};
    size_t i = 0;

    JOYPAD_PORT_FOREACH (port)
    {
        volatile bio_sensor_reader_t *reader = &bio_sensor_readers[port];
        if (
            reader->read_pending == false &&
            reader->state != BIO_SENSOR_STATE_STOPPED
        )
        {
            reader->read_pending = true;
            // Micro-optimization: Minimize copy length
            const size_t recv_offset = offsetof(typeof(send_cmd), recv_bytes);
            memcpy(&input[i], &send_cmd, recv_offset);
            i += sizeof(send_cmd);
        }
        else
        {
            // Skip this port
            i += JOYBUS_COMMAND_SKIP_SIZE;
        }
    }

    if (i)
    {
        // Close out the Joybus operation block
        input[i] = 0xFE;
        input[JOYBUS_BLOCK_SIZE - 1] = 0x01;
        joybus_exec_async(input, bio_sensor_read_callback, NULL);
    }
}

void bio_sensor_init(void)
{
    register_VI_handler(bio_sensor_vi_interrupt_callback);
}

void bio_sensor_close(void)
{
    unregister_VI_handler(bio_sensor_vi_interrupt_callback);
    memset((void *)bio_sensor_readers, 0, sizeof(bio_sensor_readers));
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

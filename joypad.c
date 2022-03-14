/**
 * @file joypad.c
 * @author Christopher Bonhage (me@christopherbonhage.com)
 * @brief Joypad Subsystem
 * @ingroup joypad
 */

#include <stdint.h>
#include <string.h>
#include <libdragon.h>

// Expose libdragon joybus internal API
typedef void (*joybus_callback_t)(uint64_t *out_dwords, void *ctx);
void joybus_exec_async(const void * input, joybus_callback_t callback, void *ctx);

#include "joybus_commands.h"
#include "joypad_utils.h"
#include "joypad.h"

/**
 * @defgroup joypad Joypad Subsystem
 * @brief Joypad abstraction interface.
 *
 * The Joypad subsystem is the successor to the LibDragon Controller subsystem.
 * The Joypad subsystem is an abstraction layer for the system's controller ports that
 * makes it easy for developers to support a variety of controller types such as:
 *
 * * Standard N64 controllers
 * * GameCube controllers (with a passive adapter)
 */

#define JOYPAD_IDENTIFY_INTERVAL_TICKS TICKS_PER_SECOND

typedef enum
{
    JOYPAD_N64_ACCESSORY_STATE_IDLE = 0,
    JOYPAD_N64_ACCESSORY_STATE_DETECT_WRITE_PENDING,
    JOYPAD_N64_ACCESSORY_STATE_DETECT_READ_PENDING,
    JOYPAD_N64_ACCESSORY_STATE_RUMBLE_WRITE1_PENDING,
    JOYPAD_N64_ACCESSORY_STATE_RUMBLE_WRITE2_PENDING,
    JOYPAD_N64_ACCESSORY_STATE_RUMBLE_WRITE3_PENDING,
} joypad_n64_accessory_state_t;

typedef enum
{
    JOYPAD_N64_ACCESSORY_CRC_STATUS_OK = 0,
    JOYPAD_N64_ACCESSORY_CRC_STATUS_NOPAK,
    JOYPAD_N64_ACCESSORY_CRC_STATUS_BADCRC,
} joypad_n64_accessory_crc_status_t;

typedef struct joypad_gcn_origin_s
{
    bool checked;
    uint8_t stick_x;
    uint8_t stick_y;
    uint8_t cstick_x;
    uint8_t cstick_y;
    uint8_t analog_l;
    uint8_t analog_r;
} joypad_gcn_origin_t;

#define JOYPAD_GCN_ORIGIN_INIT ((joypad_gcn_origin_t){ 0, 127, 127, 127, 127, 0, 0 })

typedef union
{
    uint64_t raw;
    joypad_inputs_t inputs;
    struct
    {
        uint64_t digital : 16;
        uint64_t analog  : 48;
    };
} joypad_data_t;

#define JOYPAD_DATA_INIT ((joypad_data_t){ .raw = 0ULL })

typedef struct joypad_device_s
{
    // Controller-detection fields
    const joypad_port_t port;
    uint16_t identifier;
    joypad_style_t style;
    // Rumble-related fields
    bool rumble_supported;
    bool rumble_active;
    // "Nintendo 64"-specific fields
    joypad_n64_accessory_state_t accessory_state;
    // GameCube-specific fields
    joypad_gcn_origin_t origin;
    // Input data snapshots
    joypad_data_t current;
    joypad_data_t previous;
} joypad_device_t;

#define JOYPAD_DEVICES_INIT {{JOYPAD_PORT_1}, {JOYPAD_PORT_2}, {JOYPAD_PORT_3}, {JOYPAD_PORT_4}}

// "Hot" (interrupt-driven) global state
static volatile int64_t joypad_identify_last_ticks = 0;
static volatile bool joypad_identify_pending = false;
static volatile bool joypad_read_pending = false;
static volatile bool joypad_gcn_origin_pending = false;
static volatile joypad_device_t joypad_hot_devices[JOYPAD_PORT_COUNT] = JOYPAD_DEVICES_INIT;

// "Cold" (stable) global state
static joypad_device_t joypad_cold_devices[JOYPAD_PORT_COUNT] = JOYPAD_DEVICES_INIT;

static void joypad_device_reset(joypad_port_t port, uint16_t identifier)
{
    volatile joypad_device_t *device = &joypad_hot_devices[port];
    device->identifier = identifier;
    device->style = JOYPAD_STYLE_NONE;
    device->rumble_supported = false;
    device->rumble_active = false;
    device->accessory_state = JOYPAD_N64_ACCESSORY_STATE_IDLE;
    device->origin = JOYPAD_GCN_ORIGIN_INIT;
    device->current = JOYPAD_DATA_INIT;
    device->previous = JOYPAD_DATA_INIT;
}

static uint16_t __calc_addr_crc(uint16_t address)
{
    static const uint16_t xor_table[16] = { 
        0x00, 0x00, 0x00, 0x00,
        0x00, 0x15, 0x1F, 0x0B,
        0x16, 0x19, 0x07, 0x0E,
        0x1C, 0x0D, 0x1A, 0x01
    };
    uint16_t crc = 0;
    address &= ~0x1F;
    for (int i = 15; i >= 5; i--)
        if ((address >> i) & 0x1)
            crc ^= xor_table[i];
    crc &= 0x1F;
    return address | crc;
}

static uint8_t __calc_data_crc(const uint8_t *data)
{
    uint8_t ret = 0;
    for (int i = 0; i <= 32; i++)
    {
        for (int j = 7; j >= 0; j--)
        {
            int tmp = 0;
            if (ret & 0x80) tmp = 0x85;
            ret <<= 1;
            if (i < 32 && data[i] & (0x01 << j)) ret |= 0x1;
            ret ^= tmp;
        }
    }
    return ret;
}

static joypad_n64_accessory_crc_status_t __check_data_crc(uint8_t actual, uint8_t expected)
{
    if (expected == actual) return JOYPAD_N64_ACCESSORY_CRC_STATUS_OK;
    else if (expected == (actual ^ 0xFF)) return JOYPAD_N64_ACCESSORY_CRC_STATUS_NOPAK;
    else return JOYPAD_N64_ACCESSORY_CRC_STATUS_BADCRC;
}

static void joypad_n64_accessory_read_async(joypad_port_t port, uint16_t addr, joybus_callback_t callback, void *ctx)
{
    uint16_t addr_crc = __calc_addr_crc(addr);
    uint8_t input[JOYBUS_BLOCK_SIZE] = {0};
    size_t i = port;

    const joybus_cmd_n64_accessory_read_port_t send_cmd = {
        .send_len = sizeof(send_cmd.send_bytes),
        .recv_len = sizeof(send_cmd.recv_bytes),
        .command = JOYBUS_COMMAND_N64_ACCESSORY_READ,
        .addr_crc = addr_crc,
    };
    // Micro-optimization: Minimize copy length
    const size_t recv_offset = offsetof(typeof(send_cmd), recv_bytes);
    memcpy(&input[i], &send_cmd, recv_offset);
    i += sizeof(send_cmd);

    // Close out the Joybus operation block
    input[i] = 0xFE;
    input[sizeof(input) - 1] = 0x01;

    joybus_exec_async(input, callback, ctx);
}

static void joypad_n64_accessory_write_async(joypad_port_t port, uint16_t addr, uint8_t *data, joybus_callback_t callback, void *ctx)
{
    uint16_t addr_crc = __calc_addr_crc(addr);
    uint8_t input[JOYBUS_BLOCK_SIZE] = {0};
    size_t i = port;

    const joybus_cmd_n64_accessory_write_port_t send_cmd = {
        .send_len = sizeof(send_cmd.send_bytes),
        .recv_len = sizeof(send_cmd.recv_bytes),
        .command = JOYBUS_COMMAND_N64_ACCESSORY_WRITE,
        .addr_crc = addr_crc,
    };
    // Micro-optimization: Minimize copy length
    const size_t data_offset = offsetof(typeof(send_cmd), data);
    memcpy(&input[i], &send_cmd, data_offset);
    memcpy(&input[i + data_offset], data, sizeof(send_cmd.data));
    i += sizeof(send_cmd);

    // Close out the Joybus operation block
    input[i] = 0xFE;
    input[sizeof(input) - 1] = 0x01;

    joybus_exec_async(input, callback, ctx);
}

static void joypad_n64_rumble_detect_read_callback(uint64_t *out_dwords, void *ctx)
{
    const uint8_t *out_bytes = (void *)out_dwords;
    volatile joypad_device_t *device = ctx;
    joypad_n64_accessory_state_t state = device->accessory_state;
    joypad_port_t port = device->port;

    const joybus_cmd_n64_accessory_read_port_t *recv_cmd = (void *)&out_bytes[port];
    uint8_t data_crc = __calc_data_crc(recv_cmd->data);
    joypad_n64_accessory_crc_status_t crc_status = __check_data_crc(recv_cmd->data_crc, data_crc);

    if (state == JOYPAD_N64_ACCESSORY_STATE_DETECT_READ_PENDING)
    {
        if (
            crc_status == JOYPAD_N64_ACCESSORY_CRC_STATUS_OK && 
            recv_cmd->data[0] == JOYBUS_N64_ACCESSORY_PROBE_RUMBLE_PAK
        )
        {
            joypad_hot_devices[port].rumble_supported = true;
        }
        else
        {
            joypad_hot_devices[port].rumble_supported = false;
            joypad_hot_devices[port].rumble_active = false;
        }
        device->accessory_state = JOYPAD_N64_ACCESSORY_STATE_IDLE;
    }
}

static void joypad_n64_rumble_detect_write_callback(uint64_t *out_dwords, void *ctx)
{
    const uint8_t *out_bytes = (void *)out_dwords;
    volatile joypad_device_t *device = ctx;
    joypad_n64_accessory_state_t state = device->accessory_state;
    joypad_port_t port = device->port;

    const joybus_cmd_n64_accessory_write_port_t *recv_cmd = (void *)&out_bytes[port];
    uint8_t data_crc = __calc_data_crc(recv_cmd->data);
    joypad_n64_accessory_crc_status_t crc_status = __check_data_crc(recv_cmd->data_crc, data_crc);

    if (crc_status != JOYPAD_N64_ACCESSORY_CRC_STATUS_OK)
    {
        // Accessory write failed: no accessory or bad connection
        joypad_hot_devices[port].rumble_supported = false;
        joypad_hot_devices[port].rumble_active = false;
        device->accessory_state = JOYPAD_N64_ACCESSORY_STATE_IDLE;
    }
    else if (state == JOYPAD_N64_ACCESSORY_STATE_DETECT_WRITE_PENDING)
    {
        // Step 2: Read back the "accessory probe sector"
        device->accessory_state = JOYPAD_N64_ACCESSORY_STATE_DETECT_READ_PENDING;
        joypad_n64_accessory_read_async(
            port,
            JOYBUS_N64_ACCESSORY_SECTOR_PROBE,
            joypad_n64_rumble_detect_read_callback,
            ctx
        );
    }
}

static void joypad_n64_rumble_detect_async(joypad_port_t port)
{
    volatile joypad_device_t *device = &joypad_hot_devices[port];
    if (device->accessory_state == JOYPAD_N64_ACCESSORY_STATE_IDLE)
    {
        // Step 1: Write the Rumble Pak identifier to the "accessory probe sector"
        device->accessory_state = JOYPAD_N64_ACCESSORY_STATE_DETECT_WRITE_PENDING;
        uint8_t probe_data[JOYBUS_N64_ACCESSORY_DATA_SIZE];
        memset(probe_data, JOYBUS_N64_ACCESSORY_PROBE_RUMBLE_PAK, sizeof(probe_data));
        joypad_n64_accessory_write_async(
            port,
            JOYBUS_N64_ACCESSORY_SECTOR_PROBE,
            probe_data,
            joypad_n64_rumble_detect_write_callback,
            (void *)device
        );
    }
}

static void joypad_n64_rumble_toggle_write_callback(uint64_t *out_dwords, void *ctx)
{
    const uint8_t *out_bytes = (void *)out_dwords;
    volatile joypad_device_t *device = ctx;
    joypad_n64_accessory_state_t state = device->accessory_state;
    joypad_port_t port = device->port;

    const joybus_cmd_n64_accessory_write_port_t *recv_cmd = (void *)&out_bytes[port];
    uint8_t data_crc = __calc_data_crc(recv_cmd->data);
    joypad_n64_accessory_crc_status_t crc_status = __check_data_crc(recv_cmd->data_crc, data_crc);

    if (crc_status != JOYPAD_N64_ACCESSORY_CRC_STATUS_OK)
    {
        device->rumble_supported = false;
        device->rumble_active = false;
        device->accessory_state = JOYPAD_N64_ACCESSORY_STATE_IDLE;
    }
    else if (state == JOYPAD_N64_ACCESSORY_STATE_RUMBLE_WRITE3_PENDING)
    {
        device->accessory_state = JOYPAD_N64_ACCESSORY_STATE_IDLE;
    }
    else
    {
        // Proceed to the next state
        if (state == JOYPAD_N64_ACCESSORY_STATE_RUMBLE_WRITE1_PENDING)
        {
            device->accessory_state = JOYPAD_N64_ACCESSORY_STATE_RUMBLE_WRITE2_PENDING;
        }
        else if (state == JOYPAD_N64_ACCESSORY_STATE_RUMBLE_WRITE2_PENDING)
        {
            device->accessory_state = JOYPAD_N64_ACCESSORY_STATE_RUMBLE_WRITE3_PENDING;
        }
        else return;
        // For best results, rumble start/stop should be sent three times
        uint8_t motor_data[JOYBUS_N64_ACCESSORY_DATA_SIZE];
        memset(motor_data, device->rumble_active, sizeof(motor_data));
        joypad_n64_accessory_write_async(
            port,
            JOYBUS_N64_ACCESSORY_SECTOR_RUMBLE,
            motor_data,
            joypad_n64_rumble_toggle_write_callback,
            ctx
        );
    }
}

static void joypad_n64_rumble_toggle_async(joypad_port_t port, bool active)
{
    volatile joypad_device_t *device = &joypad_hot_devices[port];
    device->accessory_state = JOYPAD_N64_ACCESSORY_STATE_RUMBLE_WRITE1_PENDING;
    device->rumble_active = active;
    uint8_t motor_data[JOYBUS_N64_ACCESSORY_DATA_SIZE];
    memset(motor_data, active, sizeof(motor_data));
    joypad_n64_accessory_write_async(
        port,
        JOYBUS_N64_ACCESSORY_SECTOR_RUMBLE,
        motor_data,
        joypad_n64_rumble_toggle_write_callback,
        (void *)device
    );
}

static void joypad_gcn_origin_callback(uint64_t *out_dwords, void *ctx)
{
    const uint8_t *out_bytes = (void *)out_dwords;
    const joybus_cmd_gcn_controller_origin_port_t *recv_cmd;
    volatile joypad_device_t *device;
    size_t i = 0;

    for (joypad_port_t port = JOYPAD_PORT_1; port < JOYPAD_PORT_COUNT; ++port)
    {
        // Check send_len to figure out if this port has a command on it
        if (out_bytes[i] == 0)
        {
            // Skip this port
            i++;
            continue;
        }

        recv_cmd = (void *)&out_bytes[i];
        i += sizeof(*recv_cmd);

        device = &joypad_hot_devices[port];
        if (device->style == JOYPAD_STYLE_GCN)
        {
            device->origin = (joypad_gcn_origin_t){
                .checked = true,
                .stick_x = recv_cmd->stick_x,
                .stick_y = recv_cmd->stick_y,
                .cstick_x = recv_cmd->cstick_x,
                .cstick_y = recv_cmd->cstick_y,
                .analog_l = recv_cmd->analog_l,
                .analog_r = recv_cmd->analog_r,
            };
        }
    }

    joypad_gcn_origin_pending = false;
}

static void joypad_gcn_origin_check_async(void)
{
    // Bail if this operation is already in-progress
    if (joypad_gcn_origin_pending) return;

    const joybus_cmd_gcn_controller_origin_port_t send_cmd = {
        .send_len = sizeof(send_cmd.send_bytes),
        .recv_len = sizeof(send_cmd.recv_bytes),
        .command = JOYBUS_COMMAND_GCN_CONTROLLER_ORIGIN,
    };
    const size_t recv_offset = offsetof(typeof(send_cmd), recv_bytes);

    uint8_t input[JOYBUS_BLOCK_SIZE] = {0};
    volatile joypad_device_t *device;
    size_t i = 0;

    // Populate the Joybus commands on each port
    for (joypad_port_t port = JOYPAD_PORT_1; port < JOYPAD_PORT_COUNT; ++port)
    {
        device = &joypad_hot_devices[port];
        if (device->style == JOYPAD_STYLE_GCN)
        {
            // Micro-optimization: Minimize copy length
            memcpy(&input[i], &send_cmd, recv_offset);
            i += sizeof(send_cmd);
        }
        else
        {
            // Skip this port
            i++;
        }
    }

    // Close out the Joybus operation block
    input[i] = 0xFE;
    input[sizeof(input) - 1] = 0x01;

    if (!joypad_gcn_origin_pending)
    {
        joypad_gcn_origin_pending = true;
        joybus_exec_async(input, joypad_gcn_origin_callback, NULL);
    }
}

static void joypad_identify_callback(uint64_t *out_dwords, void *ctx)
{
    const uint8_t *out_bytes = (void *)out_dwords;
    const joybus_cmd_identify_port_t *recv_cmd;
    volatile joypad_device_t *device;
    size_t i = 0;

    for (joypad_port_t port = JOYPAD_PORT_1; port < JOYPAD_PORT_COUNT; ++port)
    {
        recv_cmd = (void *)&out_bytes[i];
        i += sizeof(*recv_cmd);

        device = &joypad_hot_devices[port];
        const uint16_t identifier = recv_cmd->identifier;

        if (device->identifier != identifier)
        {
            // The identifier has changed; reset device state
            joypad_device_reset(port, identifier);
        }

        if (identifier == JOYBUS_IDENTIFIER_N64_CONTROLLER)
        {
            device->style = JOYPAD_STYLE_N64;
            if (recv_cmd->status == JOYBUS_IDENTIFY_STATUS_N64_ACCESSORY)
            {
                joypad_n64_rumble_detect_async(port);
            }
            else
            {
                device->rumble_supported = false;
                device->rumble_active = false;
            }
        }
        else if (identifier & JOYBUS_IDENTIFIER_MASK_GCN_CONTROLLER)
        {
            device->style = JOYPAD_STYLE_GCN;
            bool has_rumble = !(identifier & JOYBUS_IDENTIFIER_MASK_GCN_NORUMBLE);
            device->rumble_supported = has_rumble;
            device->rumble_active = has_rumble && device->rumble_active;
        }
    }

    joypad_identify_pending = false;
    joypad_identify_last_ticks = timer_ticks();
}

static void joypad_identify_async(bool reset)
{
    // Bail if this operation is already in-progress
    if (joypad_identify_pending) return;

    const joybus_cmd_identify_port_t send_cmd = {
        .send_len = sizeof(send_cmd.send_bytes),
        .recv_len = sizeof(send_cmd.recv_bytes),
        .command = reset ? JOYBUS_COMMAND_RESET : JOYBUS_COMMAND_IDENTIFY,
    };
    const size_t recv_offset = offsetof(typeof(send_cmd), recv_bytes);

    uint8_t input[JOYBUS_BLOCK_SIZE] = {0};
    size_t i = 0;

    // Populate the Joybus commands on each port
    for (joypad_port_t port = JOYPAD_PORT_1; port < JOYPAD_PORT_COUNT; ++port)
    {
        // Rumble motors should be deactivated by a reset
        if (reset) joypad_set_rumble_active(port, false);

        // Micro-optimization: Minimize copy length
        memcpy(&input[i], &send_cmd, recv_offset);
        i += sizeof(send_cmd);
    }

    // Close out the Joybus operation block
    input[i] = 0xFE;
    input[sizeof(input) - 1] = 0x01;

    if (!joypad_identify_pending)
    {
        joypad_identify_pending = true;
        joybus_exec_async(input, joypad_identify_callback, NULL);
    }
}

static void joypad_read_callback(uint64_t *out_dwords, void *ctx)
{
    const uint8_t *out_bytes = (void *)out_dwords;
    volatile joypad_device_t *device;
    joypad_style_t style;
    uint8_t command_len;
    uint8_t command_id;
    bool check_origins;
    size_t i = 0;

    for (joypad_port_t port = JOYPAD_PORT_1; port < JOYPAD_PORT_COUNT; ++port)
    {
        // Check send_len to figure out if this port has a command on it
        if (out_bytes[i] == 0)
        {
            // Skip this port
            i++;
            continue;
        }

        // Commands always start with send_len, recv_len, command_id
        command_len = 2 + out_bytes[i] + out_bytes[i + 1];
        command_id = out_bytes[i + 2];

        device = &joypad_hot_devices[port];
        style = device->style;
        if (style == JOYPAD_STYLE_N64)
        {
            // Ensure this command matches the identified controller
            if (command_id != JOYBUS_COMMAND_N64_CONTROLLER_READ)
            {
                // Skip this port
                i += command_len;
                continue;
            }

            const joybus_cmd_n64_controller_read_port_t *recv_cmd;
            recv_cmd = (void *)&out_bytes[i];
            i += sizeof(*recv_cmd);

            int cstick_x = 0;
            if (recv_cmd->c_left && !recv_cmd->c_right) cstick_x = -JOYBUS_RANGE_GCN_CSTICK_MAX;
            if (!recv_cmd->c_left && recv_cmd->c_right) cstick_x = +JOYBUS_RANGE_GCN_CSTICK_MAX;

            int cstick_y = 0;
            if (recv_cmd->c_up && !recv_cmd->c_down) cstick_y = -JOYBUS_RANGE_GCN_CSTICK_MAX;
            if (!recv_cmd->c_up && recv_cmd->c_down) cstick_y = +JOYBUS_RANGE_GCN_CSTICK_MAX;

            device->previous = device->current;
            device->current.inputs = (joypad_inputs_t){
                .a = recv_cmd->a,
                .b = recv_cmd->b,
                .z = recv_cmd->z,
                .start = recv_cmd->start,
                .d_up = recv_cmd->d_up,
                .d_down = recv_cmd->d_down,
                .d_left = recv_cmd->d_left,
                .d_right = recv_cmd->d_right,
                .y = 0,
                .x = 0,
                .l = recv_cmd->l,
                .r = recv_cmd->r,
                .c_up = recv_cmd->c_up,
                .c_down = recv_cmd->c_down,
                .c_left = recv_cmd->c_left,
                .c_right = recv_cmd->c_right,
                .stick_x = recv_cmd->stick_x,
                .stick_y = recv_cmd->stick_y,
                .cstick_x = cstick_x,
                .cstick_y = cstick_y,
                .analog_l = recv_cmd->l ? JOYBUS_RANGE_GCN_TRIGGER_MAX : 0,
                .analog_r = recv_cmd->r ? JOYBUS_RANGE_GCN_TRIGGER_MAX : 0,
            };
        }
        else if (style == JOYPAD_STYLE_GCN)
        {
            // Ensure this command matches the identified controller
            if (command_id != JOYBUS_COMMAND_GCN_CONTROLLER_READ)
            {
                // Skip this port
                i += command_len;
                continue;
            }

            const joybus_cmd_gcn_controller_read_port_t *recv_cmd;
            recv_cmd = (void *)&out_bytes[i];
            i += sizeof(*recv_cmd);

            if (recv_cmd->check_origin)
            {
                device->origin.checked = false;
                check_origins = true;
            }

            // Bias the analog values with the corresponding origin
            const int stick_x = CLAMP_ANALOG_STICK(recv_cmd->stick_x - device->origin.stick_x);
            const int stick_y = CLAMP_ANALOG_STICK(recv_cmd->stick_y - device->origin.stick_y);
            const int cstick_x = CLAMP_ANALOG_STICK(recv_cmd->cstick_x - device->origin.cstick_x);
            const int cstick_y = CLAMP_ANALOG_STICK(recv_cmd->cstick_y - device->origin.cstick_y);
            const int analog_l = CLAMP_ANALOG_TRIGGER(recv_cmd->analog_l - device->origin.analog_l);
            const int analog_r = CLAMP_ANALOG_TRIGGER(recv_cmd->analog_r - device->origin.analog_r);

            static const int cstick_threshold = JOYBUS_RANGE_GCN_CSTICK_MAX / 2;

            device->previous = device->current;
            device->current.inputs = (joypad_inputs_t){
                .a = recv_cmd->a,
                .b = recv_cmd->b,
                .z = recv_cmd->z,
                .start = recv_cmd->start,
                .d_up    = recv_cmd->d_up,
                .d_down  = recv_cmd->d_down,
                .d_left  = recv_cmd->d_left,
                .d_right = recv_cmd->d_right,
                .y = recv_cmd->y,
                .x = recv_cmd->x,
                .l = recv_cmd->l,
                .r = recv_cmd->r,
                .c_up    = cstick_y > +cstick_threshold,
                .c_down  = cstick_y < -cstick_threshold,
                .c_left  = cstick_x < -cstick_threshold,
                .c_right = cstick_x > +cstick_threshold,
                .stick_x = stick_x,
                .stick_y = stick_y,
                .cstick_x = cstick_x,
                .cstick_y = cstick_y,
                .analog_l = analog_l,
                .analog_r = analog_r,
            };
        }
        else
        {
            // Skip this port
            i += command_len;
        }
    }

    joypad_read_pending = false;
    if (check_origins) joypad_gcn_origin_check_async();
}

static void joypad_read_async(void)
{
    // Bail if this operation is already in-progress
    if (joypad_read_pending) return;

    uint8_t input[JOYBUS_BLOCK_SIZE] = {0};
    volatile joypad_device_t *device;
    joypad_style_t style;
    size_t i = 0;
    
    // Populate the Joybus commands on each port
    for (joypad_port_t port = JOYPAD_PORT_1; port < JOYPAD_PORT_COUNT; ++port)
    {
        device = &joypad_hot_devices[port];
        style = device->style;

        if (style == JOYPAD_STYLE_N64)
        {
            const joybus_cmd_n64_controller_read_port_t send_cmd = {
                .send_len = sizeof(send_cmd.send_bytes),
                .recv_len = sizeof(send_cmd.recv_bytes),
                .command = JOYBUS_COMMAND_N64_CONTROLLER_READ,
            };
            // Micro-optimization: Minimize copy length
            const size_t recv_offset = offsetof(typeof(send_cmd), recv_bytes);
            memcpy(&input[i], &send_cmd, recv_offset);
            i += sizeof(send_cmd);
        }
        else if (style == JOYPAD_STYLE_GCN)
        {
            const joybus_cmd_gcn_controller_read_port_t send_cmd = {
                .send_len = sizeof(send_cmd.send_bytes),
                .recv_len = sizeof(send_cmd.recv_bytes),
                .command = JOYBUS_COMMAND_GCN_CONTROLLER_READ,
                .mode = 3, // Most-compatible analog mode
                .rumble = device->rumble_active,
            };
            // Micro-optimization: Minimize copy length
            const size_t recv_offset = offsetof(typeof(send_cmd), recv_bytes);
            memcpy(&input[i], &send_cmd, recv_offset);
            i += sizeof(send_cmd);
        }
        else
        {
            // Skip this port
            i++;
        }
    }

    // Close out the Joybus operation block
    input[i] = 0xFE;
    input[sizeof(input) - 1] = 0x01;

    if (!joypad_read_pending)
    {
        joypad_read_pending = true;
        joybus_exec_async(input, joypad_read_callback, NULL);
    }
}

static void joypad_vi_interrupt_callback(void)
{
    joypad_read_async();
    if (joypad_identify_last_ticks + JOYPAD_IDENTIFY_INTERVAL_TICKS < timer_ticks())
    {
        joypad_identify_async(false);
    }
}

void joypad_init(void)
{
    for (joypad_port_t port = JOYPAD_PORT_1; port < JOYPAD_PORT_COUNT; ++port)
    {
        joypad_device_reset(port, JOYBUS_IDENTIFIER_UNKNOWN);
    }
    joypad_identify(true);
    register_VI_handler(joypad_vi_interrupt_callback);
}

void joypad_close(void)
{
    unregister_VI_handler(joypad_vi_interrupt_callback);
}

void joypad_identify(bool reset)
{
    disable_interrupts();
    joypad_identify_async(reset);
    enable_interrupts();
    while (joypad_identify_pending) { /* Spinlock */ }
}

void joypad_scan(void)
{
    disable_interrupts();
    assert(sizeof(joypad_cold_devices) == sizeof(joypad_hot_devices));
    memcpy(&joypad_cold_devices, (void*)&joypad_hot_devices, sizeof(joypad_hot_devices));
    enable_interrupts();
}

void joypad_set_rumble_active(joypad_port_t port, bool active)
{
    disable_interrupts();
    volatile joypad_device_t *hot_device = &joypad_hot_devices[port];
    if (hot_device->rumble_supported)
    {
        if (hot_device->style == JOYPAD_STYLE_N64)
        {
            joypad_n64_rumble_toggle_async(port, active);
        }
        else
        {
            hot_device->rumble_active = active;
        }
        joypad_cold_devices[port].rumble_active = active;
    }
    enable_interrupts();
}

joypad_style_t joypad_get_style(joypad_port_t port)
{
    return joypad_cold_devices[port].style;
}

bool joypad_get_rumble_supported(joypad_port_t port)
{
    return joypad_cold_devices[port].rumble_supported;
}

bool joypad_get_rumble_active(joypad_port_t port)
{
    return joypad_cold_devices[port].rumble_active;
}

joypad_inputs_t joypad_inputs(joypad_port_t port)
{
    return joypad_cold_devices[port].current.inputs;
}

joypad_inputs_t joypad_pressed(joypad_port_t port)
{
    const joypad_data_t current = joypad_cold_devices[port].current;
    const joypad_data_t previous = joypad_cold_devices[port].previous;
    const uint64_t pressed = current.digital & ~previous.digital;
    const joypad_data_t result = {.digital = pressed, .analog = current.analog};
    return result.inputs;
}

joypad_inputs_t joypad_released(joypad_port_t port)
{
    const joypad_data_t current = joypad_cold_devices[port].current;
    const joypad_data_t previous = joypad_cold_devices[port].previous;
    const uint64_t released = ~(current.digital & previous.digital);
    const joypad_data_t result = {.digital = released, .analog = current.analog};
    return result.inputs;
}

joypad_inputs_t joypad_held(joypad_port_t port)
{
    const joypad_data_t current = joypad_cold_devices[port].current;
    const joypad_data_t previous = joypad_cold_devices[port].previous;
    const uint64_t held = current.digital & previous.digital;
    const joypad_data_t result = {.digital = held, .analog = current.analog};
    return result.inputs;
}

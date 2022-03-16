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
    uint8_t stick_x;
    uint8_t stick_y;
    uint8_t cstick_x;
    uint8_t cstick_y;
    uint8_t analog_l;
    uint8_t analog_r;
} joypad_gcn_origin_t;

#define JOYPAD_GCN_ORIGIN_INIT ((joypad_gcn_origin_t){ 127, 127, 127, 127, 0, 0 })

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

typedef struct joypad_device_cold_s
{
    joypad_style_t style;
    joypad_data_t current;
    joypad_data_t previous;
} joypad_device_cold_t;

#define JOYPAD_DEVICE_COLD_INIT ((joypad_device_cold_t){ \
    .style = JOYPAD_STYLE_NONE, \
    .current = JOYPAD_DATA_INIT, \
    .previous = JOYPAD_DATA_INIT, \
})

typedef struct joypad_device_hot_s
{
    // Identification fields
    const joypad_port_t port;
    uint16_t identifier;
    joypad_style_t style;
    // Rumble-related fields
    bool rumble_supported;
    bool rumble_active;
    // N64-specific fields
    joypad_n64_accessory_state_t accessory_state;
} joypad_device_hot_t;

#define JOYPAD_DEVICES_HOT_INIT {{JOYPAD_PORT_1}, {JOYPAD_PORT_2}, {JOYPAD_PORT_3}, {JOYPAD_PORT_4}}

// "Hot" (interrupt-driven) global state
static volatile int64_t joypad_identify_last_ticks = 0;
static volatile bool joypad_identify_pending = false;
static volatile uint8_t joypad_identify_input_valid = false;
static volatile uint8_t joypad_identify_input[JOYBUS_BLOCK_SIZE] = {0};

static volatile bool joypad_read_pending = false;
static volatile bool joypad_read_input_valid = false;
static volatile size_t joypad_read_input_offsets[JOYPAD_PORT_COUNT] = {0};
static volatile uint8_t joypad_read_input[JOYBUS_BLOCK_SIZE] = {0};
static volatile uint8_t joypad_read_output[JOYBUS_BLOCK_SIZE] = {0};

static volatile bool joypad_gcn_origin_pending = false;
static volatile bool joypad_gcn_origin_input_valid = false;
static volatile uint8_t joypad_gcn_origin_input[JOYBUS_BLOCK_SIZE] = {0};

static volatile joypad_device_hot_t joypad_devices_hot[JOYPAD_PORT_COUNT] = JOYPAD_DEVICES_HOT_INIT;
static volatile joypad_gcn_origin_t joypad_origins_hot[JOYPAD_PORT_COUNT] = {0};

// "Cold" (stable) global state
static joypad_device_cold_t joypad_devices_cold[JOYPAD_PORT_COUNT] = {0};

static void joypad_device_reset(joypad_port_t port, uint16_t identifier)
{
    volatile joypad_device_hot_t *device = &joypad_devices_hot[port];
    device->identifier = identifier;
    device->style = JOYPAD_STYLE_NONE;
    device->rumble_supported = false;
    device->rumble_active = false;
    device->accessory_state = JOYPAD_N64_ACCESSORY_STATE_IDLE;
    joypad_origins_hot[port] = JOYPAD_GCN_ORIGIN_INIT;
    joypad_devices_cold[port] = JOYPAD_DEVICE_COLD_INIT;
}

static uint16_t __calc_addr_checksum(uint16_t address)
{
    static const uint16_t xor_table[16] = { 
        0x00, 0x00, 0x00, 0x00,
        0x00, 0x15, 0x1F, 0x0B,
        0x16, 0x19, 0x07, 0x0E,
        0x1C, 0x0D, 0x1A, 0x01
    };
    uint16_t checksum = 0;
    address &= ~0x1F;
    for (int i = 15; i >= 5; i--)
        if ((address >> i) & 0x1)
            checksum ^= xor_table[i];
    checksum &= 0x1F;
    return address | checksum;
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
    uint8_t input[JOYBUS_BLOCK_SIZE] = {0};
    size_t i = port;

    const joybus_cmd_n64_accessory_read_port_t send_cmd = {
        .send_len = sizeof(send_cmd.send_bytes),
        .recv_len = sizeof(send_cmd.recv_bytes),
        .command = JOYBUS_COMMAND_ID_N64_ACCESSORY_READ,
        .addr_checksum = __calc_addr_checksum(addr),
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
    uint8_t input[JOYBUS_BLOCK_SIZE] = {0};
    size_t i = port;

    const joybus_cmd_n64_accessory_write_port_t send_cmd = {
        .send_len = sizeof(send_cmd.send_bytes),
        .recv_len = sizeof(send_cmd.recv_bytes),
        .command = JOYBUS_COMMAND_ID_N64_ACCESSORY_WRITE,
        .addr_checksum = __calc_addr_checksum(addr),
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
    volatile joypad_device_hot_t *device = ctx;
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
            joypad_devices_hot[port].rumble_supported = true;
        }
        else
        {
            joypad_devices_hot[port].rumble_supported = false;
            joypad_devices_hot[port].rumble_active = false;
        }
        device->accessory_state = JOYPAD_N64_ACCESSORY_STATE_IDLE;
    }
}

static void joypad_n64_rumble_detect_write_callback(uint64_t *out_dwords, void *ctx)
{
    const uint8_t *out_bytes = (void *)out_dwords;
    volatile joypad_device_hot_t *device = ctx;
    joypad_n64_accessory_state_t state = device->accessory_state;
    joypad_port_t port = device->port;

    const joybus_cmd_n64_accessory_write_port_t *recv_cmd = (void *)&out_bytes[port];
    uint8_t data_crc = __calc_data_crc(recv_cmd->data);
    joypad_n64_accessory_crc_status_t crc_status = __check_data_crc(recv_cmd->data_crc, data_crc);

    if (crc_status != JOYPAD_N64_ACCESSORY_CRC_STATUS_OK)
    {
        // Accessory write failed: no accessory or bad connection
        joypad_devices_hot[port].rumble_supported = false;
        joypad_devices_hot[port].rumble_active = false;
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
    volatile joypad_device_hot_t *device = &joypad_devices_hot[port];
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
    volatile joypad_device_hot_t *device = ctx;
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
    volatile joypad_device_hot_t *device = &joypad_devices_hot[port];
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

static void joypad_gcn_rumble_toggle(joypad_port_t port, bool active)
{
    volatile joypad_device_hot_t *device = &joypad_devices_hot[port];
    device->rumble_active = active;
    if (joypad_read_input_valid)
    {
        // Set rumble active flag on cached GameCube controller read command
        size_t cmd_offset = joypad_read_input_offsets[port];
        joybus_cmd_gcn_controller_read_port_t *read_cmd;
        read_cmd = (void *)&joypad_read_input[cmd_offset];
        assert(read_cmd->send_len == sizeof(read_cmd->send_bytes));
        assert(read_cmd->command == JOYBUS_COMMAND_ID_GCN_CONTROLLER_READ);
        read_cmd->rumble = active;
    }
}

static void joypad_gcn_origin_callback(uint64_t *out_dwords, void *ctx)
{
    const uint8_t *out_bytes = (void *)out_dwords;
    const joybus_cmd_gcn_controller_origin_port_t *recv_cmd;
    size_t i = 0;

    JOYPAD_PORT_FOR_EACH (port)
    {
        // Check send_len to figure out if this port has a command on it
        if (out_bytes[i + JOYBUS_COMMAND_OFFSET_SEND_LEN] == 0)
        {
            // Skip this port
            i += JOYBUS_COMMAND_SKIP_SIZE;
        }
        else if (joypad_devices_hot[port].style != JOYPAD_STYLE_GCN)
        {
            // Skip this port
            i += sizeof(*recv_cmd);
        }
        else
        {
            recv_cmd = (void *)&out_bytes[i];
            i += sizeof(*recv_cmd);

            joypad_origins_hot[port] = (joypad_gcn_origin_t){
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
    joypad_gcn_origin_pending = true;

    uint8_t * const input = (void *)joypad_gcn_origin_input;
    if (!joypad_gcn_origin_input_valid)
    {
        const joybus_cmd_gcn_controller_origin_port_t send_cmd = {
            .send_len = sizeof(send_cmd.send_bytes),
            .recv_len = sizeof(send_cmd.recv_bytes),
            .command = JOYBUS_COMMAND_ID_GCN_CONTROLLER_ORIGIN,
        };
        const size_t recv_offset = offsetof(typeof(send_cmd), recv_bytes);
        size_t i = 0;

        // Populate the Joybus commands on each port
        memset(input, 0, JOYBUS_BLOCK_SIZE);
        JOYPAD_PORT_FOR_EACH (port)
        {
            // Micro-optimization: Minimize copy length
            memcpy(&input[i], &send_cmd, recv_offset);
            i += sizeof(send_cmd);
        }

        // Close out the Joybus operation block
        input[i] = 0xFE;
        input[JOYBUS_BLOCK_SIZE - 1] = 0x01;

        joypad_gcn_origin_input_valid = true;
    }

    joybus_exec_async(input, joypad_gcn_origin_callback, NULL);
}

static void joypad_identify_callback(uint64_t *out_dwords, void *ctx)
{
    const uint8_t *out_bytes = (void *)out_dwords;
    const joybus_cmd_identify_port_t *recv_cmd;
    volatile joypad_device_hot_t *device;
    bool devices_changed = false;
    size_t i = 0;

    JOYPAD_PORT_FOR_EACH (port)
    {
        recv_cmd = (void *)&out_bytes[i];
        i += sizeof(*recv_cmd);

        device = &joypad_devices_hot[port];
        const uint16_t identifier = recv_cmd->identifier;

        if (device->identifier != identifier)
        {
            // The identifier has changed; reset device state
            joypad_device_reset(port, identifier);
            devices_changed = true;
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

    if (devices_changed) joypad_read_input_valid = false;
    joypad_identify_last_ticks = timer_ticks();
    joypad_identify_pending = false;
}

static void joypad_identify_async(bool reset)
{
    // Bail if this operation is already in-progress
    if (joypad_identify_pending) return;
    joypad_identify_pending = true;

    uint8_t * const input = (void *)joypad_identify_input;
    // Reset invalidates the cached input block
    if (!joypad_identify_input_valid || reset)
    {
        const joybus_cmd_identify_port_t send_cmd = {
            .send_len = sizeof(send_cmd.send_bytes),
            .recv_len = sizeof(send_cmd.recv_bytes),
            .command = reset ? JOYBUS_COMMAND_ID_RESET : JOYBUS_COMMAND_ID_IDENTIFY,
        };
        const size_t recv_offset = offsetof(typeof(send_cmd), recv_bytes);
        size_t i = 0;

        // Populate the Joybus commands on each port
        memset(input, 0, JOYBUS_BLOCK_SIZE);
        JOYPAD_PORT_FOR_EACH (port)
        {
            // Micro-optimization: Minimize copy length
            memcpy(&input[i], &send_cmd, recv_offset);
            i += sizeof(send_cmd);
        }

        // Close out the Joybus operation block
        input[i] = 0xFE;
        input[JOYBUS_BLOCK_SIZE - 1] = 0x01;

        // Identify is more common than reset, so don't cache resets
        joypad_identify_input_valid = !reset;
    }

    joybus_exec_async(input, joypad_identify_callback, NULL);
}

static void joypad_read_callback(uint64_t *out_dwords, void *ctx)
{
    memcpy((void *)joypad_read_output, out_dwords, JOYBUS_BLOCK_SIZE);
    joypad_read_pending = false;
}

static void joypad_read_async(void)
{
    // Bail if this operation is already in-progress
    if (joypad_read_pending) return;
    joypad_read_pending = true;

    uint8_t * const input = (void *)joypad_read_input;
    if (!joypad_read_input_valid)
    {
        volatile joypad_device_hot_t *device;
        joypad_style_t style;
        size_t i = 0;

        // Populate Joybus controller read commands on each port
        memset(input, 0, JOYBUS_BLOCK_SIZE);
        JOYPAD_PORT_FOR_EACH (port)
        {
            joypad_read_input_offsets[port] = i;
            device = &joypad_devices_hot[port];
            style = device->style;

            if (style == JOYPAD_STYLE_N64)
            {
                const joybus_cmd_n64_controller_read_port_t send_cmd = {
                    .send_len = sizeof(send_cmd.send_bytes),
                    .recv_len = sizeof(send_cmd.recv_bytes),
                    .command = JOYBUS_COMMAND_ID_N64_CONTROLLER_READ,
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
                    .command = JOYBUS_COMMAND_ID_GCN_CONTROLLER_READ,
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
                i += JOYBUS_COMMAND_SKIP_SIZE;
            }
        }

        // Close out the Joybus operation block
        input[i] = 0xFE;
        input[JOYBUS_BLOCK_SIZE - 1] = 0x01;

        joypad_read_input_valid = true;
    }

    joybus_exec_async(input, joypad_read_callback, NULL);
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
    JOYPAD_PORT_FOR_EACH (port)
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
    // Wait for pending identify/reset operation to resolve
    while (joypad_identify_pending) { /* Spinlock */ }
    // Enqueue this identify/reset operation
    disable_interrupts();
    joypad_identify_async(reset);
    enable_interrupts();
    // Wait for the operation to finish
    while (joypad_identify_pending) { /* Spinlock */ }
}

void joypad_scan(void)
{
    uint8_t output[JOYBUS_BLOCK_SIZE];
    joypad_gcn_origin_t origins[JOYPAD_PORT_COUNT];

    disable_interrupts();
    memcpy(output, (void *)joypad_read_output, sizeof(output));
    memcpy(origins, (void *)joypad_origins_hot, sizeof(origins));
    enable_interrupts();
    
    uint8_t send_len, recv_len, command_id, command_len;
    joypad_data_t current, previous;
    bool check_origins = false;
    size_t i = 0;

    JOYPAD_PORT_FOR_EACH (port)
    {
        // Check send_len to figure out if this port has a command on it
        send_len = output[i + JOYBUS_COMMAND_OFFSET_SEND_LEN];
        if (send_len == 0)
        {
            // Commands with send_len of 0 have no recv_len or command_id 
            recv_len = 0;
            command_id = JOYBUS_COMMAND_ID_RESET;
            command_len = JOYBUS_COMMAND_SKIP_SIZE;
        }
        else
        {
            recv_len = output[i + JOYBUS_COMMAND_OFFSET_RECV_LEN];
            command_id = output[i + JOYBUS_COMMAND_OFFSET_COMMAND_ID];
            command_len = JOYBUS_COMMAND_METADATA_SIZE + send_len + recv_len;
        }

        if (command_id == JOYBUS_COMMAND_ID_N64_CONTROLLER_READ)
        {
            const joybus_cmd_n64_controller_read_port_t *recv_cmd;
            recv_cmd = (void *)&output[i];
            i += sizeof(*recv_cmd);

            int cstick_x = 0;
            if (recv_cmd->c_left && !recv_cmd->c_right) cstick_x = -JOYBUS_RANGE_GCN_CSTICK_MAX;
            if (!recv_cmd->c_left && recv_cmd->c_right) cstick_x = +JOYBUS_RANGE_GCN_CSTICK_MAX;

            int cstick_y = 0;
            if (recv_cmd->c_up && !recv_cmd->c_down) cstick_y = -JOYBUS_RANGE_GCN_CSTICK_MAX;
            if (!recv_cmd->c_up && recv_cmd->c_down) cstick_y = +JOYBUS_RANGE_GCN_CSTICK_MAX;

            previous = joypad_devices_cold[port].current;
            current.inputs = (joypad_inputs_t){
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

            joypad_devices_cold[port] = (joypad_device_cold_t){
                .style = JOYPAD_STYLE_N64,
                .current = current,
                .previous = previous,
            };
        }
        else if (command_id == JOYBUS_COMMAND_ID_GCN_CONTROLLER_READ)
        {
            const joybus_cmd_gcn_controller_read_port_t *recv_cmd;
            recv_cmd = (void *)&output[i];
            i += sizeof(*recv_cmd);

            if (recv_cmd->check_origin) check_origins = true;

            // Bias the analog values with the corresponding origin
            const int stick_x = CLAMP_ANALOG_AXIS(recv_cmd->stick_x - origins[port].stick_x);
            const int stick_y = CLAMP_ANALOG_AXIS(recv_cmd->stick_y - origins[port].stick_y);
            const int cstick_x = CLAMP_ANALOG_AXIS(recv_cmd->cstick_x - origins[port].cstick_x);
            const int cstick_y = CLAMP_ANALOG_AXIS(recv_cmd->cstick_y - origins[port].cstick_y);
            const int analog_l = CLAMP_ANALOG_TRIGGER(recv_cmd->analog_l - origins[port].analog_l);
            const int analog_r = CLAMP_ANALOG_TRIGGER(recv_cmd->analog_r - origins[port].analog_r);

            static const int cstick_threshold = JOYBUS_RANGE_GCN_CSTICK_MAX / 2;

            previous = joypad_devices_cold[port].current;
            current.inputs = (joypad_inputs_t){
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

            joypad_devices_cold[port] = (joypad_device_cold_t){
                .style = JOYPAD_STYLE_GCN,
                .current = current,
                .previous = previous,
            };
        }
        else
        {
            // Skip this port
            joypad_devices_cold[port] = JOYPAD_DEVICE_COLD_INIT;
            i += command_len;
        }
    }

    if (check_origins) joypad_gcn_origin_check_async();
}

bool joypad_get_rumble_supported(joypad_port_t port)
{
    return joypad_devices_hot[port].rumble_supported;
}

bool joypad_get_rumble_active(joypad_port_t port)
{
    return joypad_devices_hot[port].rumble_active;
}

void joypad_set_rumble_active(joypad_port_t port, bool active)
{
    disable_interrupts();
    volatile joypad_device_hot_t *device = &joypad_devices_hot[port];
    if (device->rumble_supported)
    {
        joypad_style_t style = device->style;
        if (style == JOYPAD_STYLE_N64)
        {
            joypad_n64_rumble_toggle_async(port, active);
        }
        else if (style == JOYPAD_STYLE_GCN)
        {
            joypad_gcn_rumble_toggle(port, active);
        }
        else
        {
            device->rumble_active = active;
        }
    }
    enable_interrupts();
}

joypad_style_t joypad_get_style(joypad_port_t port)
{
    return joypad_devices_cold[port].style;
}

joypad_inputs_t joypad_inputs(joypad_port_t port)
{
    return joypad_devices_cold[port].current.inputs;
}

joypad_inputs_t joypad_pressed(joypad_port_t port)
{
    const joypad_data_t current = joypad_devices_cold[port].current;
    const joypad_data_t previous = joypad_devices_cold[port].previous;
    const uint64_t pressed = current.digital & ~previous.digital;
    const joypad_data_t result = {.digital = pressed, .analog = current.analog};
    return result.inputs;
}

joypad_inputs_t joypad_released(joypad_port_t port)
{
    const joypad_data_t current = joypad_devices_cold[port].current;
    const joypad_data_t previous = joypad_devices_cold[port].previous;
    const uint64_t released = ~(current.digital & previous.digital);
    const joypad_data_t result = {.digital = released, .analog = current.analog};
    return result.inputs;
}

joypad_inputs_t joypad_held(joypad_port_t port)
{
    const joypad_data_t current = joypad_devices_cold[port].current;
    const joypad_data_t previous = joypad_devices_cold[port].previous;
    const uint64_t held = current.digital & previous.digital;
    const joypad_data_t result = {.digital = held, .analog = current.analog};
    return result.inputs;
}

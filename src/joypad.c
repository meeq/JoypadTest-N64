/**
 * @file joypad.c
 * @author Christopher Bonhage (me@christopherbonhage.com)
 * @brief Joypad Subsystem
 * @ingroup joypad
 */

#include <stdint.h>
#include <string.h>
#include <libdragon.h>

#include "joybus_commands.h"
#include "joybus_n64_accessory.h"
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
    JOYPAD_N64_ACCESSORY_STATE_RUMBLE_PAK_WRITE1_PENDING,
    JOYPAD_N64_ACCESSORY_STATE_RUMBLE_PAK_WRITE2_PENDING,
    JOYPAD_N64_ACCESSORY_STATE_RUMBLE_PAK_WRITE3_PENDING,
} joypad_n64_accessory_state_t;

typedef enum
{
    JOYPAD_RUMBLE_METHOD_NONE = 0,
    JOYPAD_RUMBLE_METHOD_N64_RUMBLE_PAK,
    JOYPAD_RUMBLE_METHOD_N64_TRANSFER_PAK_MBC5, // Not yet implemented
    JOYPAD_RUMBLE_METHOD_GCN_CONTROLLER,
} joypad_rumble_method_t;

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

typedef union joypad_buttons_raw_u
{
    uint16_t value;
    joypad_buttons_t buttons;
} joypad_buttons_raw_t;

typedef struct joypad_device_cold_s
{
    joypad_style_t style;
    joypad_inputs_t current;
    joypad_inputs_t previous;
} joypad_device_cold_t;

typedef struct joypad_device_hot_s
{
    joypad_style_t style;
    joypad_rumble_method_t rumble_method;
    bool rumble_active;
    // N64-specific fields
    joypad_n64_accessory_type_t accessory_type;
    joypad_n64_accessory_state_t accessory_state;
} joypad_device_hot_t;

// "Hot" (interrupt-driven) global state
static volatile int64_t joypad_identify_last_ticks = 0;
static volatile bool joypad_identify_pending = false;
static volatile uint8_t joypad_identify_input_valid = false;
static volatile uint8_t joypad_identify_input[JOYBUS_BLOCK_SIZE] = {0};

static volatile uint64_t joypad_read_count = 0;
static volatile bool joypad_read_pending = false;
static volatile bool joypad_read_input_valid = false;
static volatile size_t joypad_read_input_offsets[JOYPAD_PORT_COUNT] = {0};
static volatile uint8_t joypad_read_input[JOYBUS_BLOCK_SIZE] = {0};
static volatile uint8_t joypad_read_output[JOYBUS_BLOCK_SIZE] = {0};

static volatile bool joypad_gcn_origin_pending = false;
static volatile bool joypad_gcn_origin_input_valid = false;
static volatile uint8_t joypad_gcn_origin_input[JOYBUS_BLOCK_SIZE] = {0};

static volatile joybus_identifier_t joypad_identifiers_hot[JOYPAD_PORT_COUNT] = {0};
static volatile joypad_device_hot_t joypad_devices_hot[JOYPAD_PORT_COUNT] = {0};
static volatile joypad_gcn_origin_t joypad_origins_hot[JOYPAD_PORT_COUNT] = {0};

// "Cold" (stable) global state
static joypad_device_cold_t joypad_devices_cold[JOYPAD_PORT_COUNT] = {0};

static void joypad_device_reset(joypad_port_t port, joybus_identifier_t identifier)
{
    joypad_identifiers_hot[port] = identifier;
    joypad_origins_hot[port] = JOYPAD_GCN_ORIGIN_INIT;
    memset((void *)&joypad_devices_cold[port], 0, sizeof(joypad_devices_cold[port]));
    memset((void *)&joypad_devices_hot[port], 0, sizeof(joypad_devices_hot[port]));
}

static void joypad_n64_accessory_detect_read_callback(uint64_t *out_dwords, void *ctx)
{
    const uint8_t *out_bytes = (void *)out_dwords;
    joypad_port_t port = (joypad_port_t)ctx;
    volatile joypad_device_hot_t *device = &joypad_devices_hot[port];
    joypad_n64_accessory_state_t state = device->accessory_state;

    const joybus_cmd_n64_accessory_read_port_t *recv_cmd = (void *)&out_bytes[port];
    int crc_status = joybus_n64_accessory_data_crc_compare(recv_cmd->data, recv_cmd->data_crc);

    if (state == JOYPAD_N64_ACCESSORY_STATE_DETECT_READ_PENDING)
    {
        if (
            crc_status == JOYBUS_N64_ACCESSORY_DATA_CRC_STATUS_OK && 
            recv_cmd->data[0] == JOYBUS_N64_ACCESSORY_PROBE_RUMBLE_PAK
        )
        {
            device->accessory_type = JOYPAD_N64_ACCESSORY_TYPE_RUMBLE_PAK;
            device->rumble_method = JOYPAD_RUMBLE_METHOD_N64_RUMBLE_PAK;
        }
        else
        {
            // TODO Detect other accessory types
            device->accessory_type = JOYPAD_N64_ACCESSORY_TYPE_UNKNOWN;
            device->rumble_method = JOYPAD_RUMBLE_METHOD_NONE;
            device->rumble_active = false;
        }
        device->accessory_state = JOYPAD_N64_ACCESSORY_STATE_IDLE;
    }
}

static void joypad_n64_accessory_detect_write_callback(uint64_t *out_dwords, void *ctx)
{
    const uint8_t *out_bytes = (void *)out_dwords;
    joypad_port_t port = (joypad_port_t)ctx;
    volatile joypad_device_hot_t *device = &joypad_devices_hot[port];
    joypad_n64_accessory_state_t state = device->accessory_state;

    const joybus_cmd_n64_accessory_write_port_t *recv_cmd = (void *)&out_bytes[port];
    int crc_status = joybus_n64_accessory_data_crc_compare(recv_cmd->data, recv_cmd->data_crc);

    if (crc_status != JOYBUS_N64_ACCESSORY_DATA_CRC_STATUS_OK)
    {
        // Accessory write failed: no accessory or bad connection
        device->accessory_type = JOYPAD_N64_ACCESSORY_TYPE_NONE;
        device->rumble_method = JOYPAD_RUMBLE_METHOD_NONE;
        device->rumble_active = false;
        device->accessory_state = JOYPAD_N64_ACCESSORY_STATE_IDLE;
    }
    else if (state == JOYPAD_N64_ACCESSORY_STATE_DETECT_WRITE_PENDING)
    {
        // Step 2: Read back the "accessory probe sector"
        device->accessory_state = JOYPAD_N64_ACCESSORY_STATE_DETECT_READ_PENDING;
        joybus_n64_accessory_read_async(
            port,
            JOYBUS_N64_ACCESSORY_SECTOR_PROBE,
            joypad_n64_accessory_detect_read_callback,
            ctx
        );
    }
}

static void joypad_n64_accessory_detect_async(joypad_port_t port)
{
    volatile joypad_device_hot_t *device = &joypad_devices_hot[port];
    if (device->accessory_state == JOYPAD_N64_ACCESSORY_STATE_IDLE)
    {
        // Step 1: Write the Rumble Pak identifier to the "accessory probe sector"
        device->accessory_state = JOYPAD_N64_ACCESSORY_STATE_DETECT_WRITE_PENDING;
        uint8_t probe_data[JOYBUS_N64_ACCESSORY_DATA_SIZE];
        memset(probe_data, JOYBUS_N64_ACCESSORY_PROBE_RUMBLE_PAK, sizeof(probe_data));
        joybus_n64_accessory_write_async(
            port,
            JOYBUS_N64_ACCESSORY_SECTOR_PROBE,
            probe_data,
            joypad_n64_accessory_detect_write_callback,
            (void *)port
        );
    }
}

static void joypad_n64_rumble_pak_toggle_write_callback(uint64_t *out_dwords, void *ctx)
{
    const uint8_t *out_bytes = (void *)out_dwords;
    joypad_port_t port = (joypad_port_t)ctx;
    volatile joypad_device_hot_t *device = &joypad_devices_hot[port];
    joypad_n64_accessory_state_t state = device->accessory_state;

    const joybus_cmd_n64_accessory_write_port_t *recv_cmd = (void *)&out_bytes[port];
    int crc_status = joybus_n64_accessory_data_crc_compare(recv_cmd->data, recv_cmd->data_crc);

    if (crc_status != JOYBUS_N64_ACCESSORY_DATA_CRC_STATUS_OK)
    {
        device->accessory_type = JOYPAD_N64_ACCESSORY_TYPE_NONE;
        device->rumble_method = JOYPAD_RUMBLE_METHOD_NONE;
        device->rumble_active = false;
        device->accessory_state = JOYPAD_N64_ACCESSORY_STATE_IDLE;
    }
    else if (state == JOYPAD_N64_ACCESSORY_STATE_RUMBLE_PAK_WRITE3_PENDING)
    {
        device->accessory_state = JOYPAD_N64_ACCESSORY_STATE_IDLE;
    }
    else
    {
        // Proceed to the next state
        if (state == JOYPAD_N64_ACCESSORY_STATE_RUMBLE_PAK_WRITE1_PENDING)
        {
            device->accessory_state = JOYPAD_N64_ACCESSORY_STATE_RUMBLE_PAK_WRITE2_PENDING;
        }
        else if (state == JOYPAD_N64_ACCESSORY_STATE_RUMBLE_PAK_WRITE2_PENDING)
        {
            device->accessory_state = JOYPAD_N64_ACCESSORY_STATE_RUMBLE_PAK_WRITE3_PENDING;
        }
        else return;
        // For best results, rumble start/stop should be sent three times
        uint8_t motor_data[JOYBUS_N64_ACCESSORY_DATA_SIZE];
        memset(motor_data, device->rumble_active, sizeof(motor_data));
        joybus_n64_accessory_write_async(
            port,
            JOYBUS_N64_ACCESSORY_SECTOR_RUMBLE,
            motor_data,
            joypad_n64_rumble_pak_toggle_write_callback,
            ctx
        );
    }
}

static void joypad_n64_rumble_pak_toggle_async(joypad_port_t port, bool active)
{
    volatile joypad_device_hot_t *device = &joypad_devices_hot[port];
    device->accessory_state = JOYPAD_N64_ACCESSORY_STATE_RUMBLE_PAK_WRITE1_PENDING;
    device->rumble_active = active;
    uint8_t motor_data[JOYBUS_N64_ACCESSORY_DATA_SIZE];
    memset(motor_data, active, sizeof(motor_data));
    joybus_n64_accessory_write_async(
        port,
        JOYBUS_N64_ACCESSORY_SECTOR_RUMBLE,
        motor_data,
        joypad_n64_rumble_pak_toggle_write_callback,
        (void *)port
    );
}

static void joypad_gcn_controller_rumble_toggle(joypad_port_t port, bool active)
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
        device = &joypad_devices_hot[port];
        recv_cmd = (void *)&out_bytes[i];
        i += sizeof(*recv_cmd);

        joybus_identifier_t identifier = recv_cmd->identifier;
        if (joypad_identifiers_hot[port] != identifier)
        {
            // The identifier has changed; reset device state
            joypad_device_reset(port, identifier);
            devices_changed = true;
        }

        if (
            (identifier & JOYBUS_ID_TYPE_MASK) == JOYBUS_ID_TYPE_GCN && 
            (identifier & JOYBUS_IDENTIFIER_MASK_GCN_CONTROLLER)
        )
        {
            device->style = JOYPAD_STYLE_GCN;
            bool has_rumble = !(identifier & JOYBUS_IDENTIFIER_MASK_GCN_NORUMBLE);
            device->rumble_method = has_rumble
              ? JOYPAD_RUMBLE_METHOD_GCN_CONTROLLER
              : JOYPAD_RUMBLE_METHOD_NONE;
            device->rumble_active = has_rumble && device->rumble_active;
        }
        else if (identifier == JOYBUS_IDENTIFIER_N64_CONTROLLER)
        {
            device->style = JOYPAD_STYLE_N64;
            uint8_t accessory_status = recv_cmd->status & JOYBUS_IDENTIFY_STATUS_N64_ACCESSORY_MASK;
            if (accessory_status != JOYBUS_IDENTIFY_STATUS_N64_ACCESSORY_PRESENT)
            {
                device->accessory_state = JOYPAD_N64_ACCESSORY_STATE_IDLE;
                device->accessory_type = JOYPAD_N64_ACCESSORY_TYPE_NONE;
                device->rumble_method = JOYPAD_RUMBLE_METHOD_NONE;
                device->rumble_active = false;
            }
            if (accessory_status == JOYBUS_IDENTIFY_STATUS_N64_ACCESSORY_CHANGED)
            {
                device->accessory_type = JOYPAD_N64_ACCESSORY_TYPE_UNKNOWN;
                joypad_n64_accessory_detect_async(port);
            }
        }
        else if (identifier == JOYBUS_IDENTIFIER_N64_MOUSE)
        {
            device->style = JOYPAD_STYLE_MOUSE;
        }
        else if (identifier == JOYBUS_IDENTIFIER_GBA_VIA_LINK_CABLE)
        {
            // TODO Support GBA as a controller
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
    joypad_read_count += 1;
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
        joybus_identifier_t identifier;
        size_t i = 0;

        // Populate Joybus controller read commands on each port
        memset(input, 0, JOYBUS_BLOCK_SIZE);
        JOYPAD_PORT_FOR_EACH (port)
        {
            joypad_read_input_offsets[port] = i;
            device = &joypad_devices_hot[port];
            identifier = joypad_identifiers_hot[port];

             if (device->style == JOYPAD_STYLE_GCN)
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
            else if (
                identifier == JOYBUS_IDENTIFIER_N64_CONTROLLER || 
                identifier == JOYBUS_IDENTIFIER_N64_MOUSE
            )
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
    joypad_read_async();
    while (joypad_read_pending) { /* Spinlock */ }
    joypad_scan();
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
    joypad_identify_async(reset);
    // Wait for the operation to finish
    while (joypad_identify_pending) { /* Spinlock */ }
}

void joypad_scan(void)
{
    // Bail early if the joypads have not been read since last call
    static uint64_t prev_read_count = 0;
    uint64_t read_count = joypad_read_count;
    if (prev_read_count == read_count) return;
    prev_read_count = read_count;

    uint8_t output[JOYBUS_BLOCK_SIZE];
    joypad_gcn_origin_t origins[JOYPAD_PORT_COUNT];
    joybus_identifier_t identifiers[JOYPAD_PORT_COUNT];

    // Take a snapshot of the current "hot" state
    disable_interrupts();
    memcpy(output, (void *)joypad_read_output, sizeof(output));
    memcpy(origins, (void *)joypad_origins_hot, sizeof(origins));
    memcpy(identifiers, (void *)joypad_identifiers_hot, sizeof(identifiers));
    enable_interrupts();
    
    uint8_t send_len, recv_len, command_id, command_len;
    joypad_device_cold_t *device;
    bool check_origins = false;
    size_t i = 0;

    JOYPAD_PORT_FOR_EACH (port)
    {
        device = &joypad_devices_cold[port];
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

            // N64 Mouse uses the same read command as a Controller
            if (identifiers[port] == JOYBUS_IDENTIFIER_N64_MOUSE)
            {
                device->style = JOYPAD_STYLE_MOUSE;
            }
            else
            {
                device->style = JOYPAD_STYLE_N64;
            }

            // Emulate analog C-stick based on digital C-buttons
            int c_x_direction = recv_cmd->c_right - recv_cmd->c_left;
            int c_y_direction = recv_cmd->c_down - recv_cmd->c_up;
            int cstick_x = c_x_direction * JOYBUS_RANGE_GCN_CSTICK_MAX;
            int cstick_y = c_y_direction * JOYBUS_RANGE_GCN_CSTICK_MAX;
            int analog_l = recv_cmd->l ? JOYBUS_RANGE_GCN_TRIGGER_MAX : 0;
            int analog_r = recv_cmd->r ? JOYBUS_RANGE_GCN_TRIGGER_MAX : 0;

            device->previous = device->current;
            device->current = (joypad_inputs_t){
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
                .analog_l = analog_l,
                .analog_r = analog_r,
            };
        }
        else if (command_id == JOYBUS_COMMAND_ID_GCN_CONTROLLER_READ)
        {
            const joybus_cmd_gcn_controller_read_port_t *recv_cmd;
            recv_cmd = (void *)&output[i];
            i += sizeof(*recv_cmd);

            if (recv_cmd->check_origin) check_origins = true;

            // Bias the analog values with the corresponding origin
            int stick_x = CLAMP_ANALOG_AXIS(recv_cmd->stick_x - origins[port].stick_x);
            int stick_y = CLAMP_ANALOG_AXIS(recv_cmd->stick_y - origins[port].stick_y);
            int cstick_x = CLAMP_ANALOG_AXIS(recv_cmd->cstick_x - origins[port].cstick_x);
            int cstick_y = CLAMP_ANALOG_AXIS(recv_cmd->cstick_y - origins[port].cstick_y);
            int analog_l = CLAMP_ANALOG_TRIGGER(recv_cmd->analog_l - origins[port].analog_l);
            int analog_r = CLAMP_ANALOG_TRIGGER(recv_cmd->analog_r - origins[port].analog_r);

            static const int cstick_threshold = JOYBUS_RANGE_GCN_CSTICK_MAX / 2;

            device->style = JOYPAD_STYLE_GCN;
            device->previous = device->current;
            device->current = (joypad_inputs_t){
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
            // No recognized joypad on this port
            memset(device, 0, sizeof(*device));
            i += command_len;
        }
    }

    if (check_origins) joypad_gcn_origin_check_async();
}

bool joypad_get_rumble_supported(joypad_port_t port)
{
    return joypad_devices_hot[port].rumble_method != JOYPAD_RUMBLE_METHOD_NONE;
}

bool joypad_get_rumble_active(joypad_port_t port)
{
    return joypad_devices_hot[port].rumble_active;
}

void joypad_set_rumble_active(joypad_port_t port, bool active)
{
    disable_interrupts();
    volatile joypad_device_hot_t *device = &joypad_devices_hot[port];
    joypad_rumble_method_t rumble_method = device->rumble_method;
    if (rumble_method == JOYPAD_RUMBLE_METHOD_N64_RUMBLE_PAK)
    {
        joypad_n64_rumble_pak_toggle_async(port, active);
    }
    else if (rumble_method == JOYPAD_RUMBLE_METHOD_GCN_CONTROLLER)
    {
        joypad_gcn_controller_rumble_toggle(port, active);
    }
    enable_interrupts();
}

joypad_style_t joypad_get_style(joypad_port_t port)
{
    return joypad_devices_cold[port].style;
}

joypad_inputs_t joypad_get_inputs(joypad_port_t port)
{
    return joypad_devices_cold[port].current;
}

joypad_buttons_t joypad_get_buttons(joypad_port_t port)
{
    return joypad_devices_cold[port].current.buttons;
}

joypad_buttons_t joypad_get_buttons_pressed(joypad_port_t port)
{
    const joypad_buttons_raw_t current = {
        .buttons = joypad_devices_cold[port].current.buttons,
    };
    const joypad_buttons_raw_t previous = {
        .buttons = joypad_devices_cold[port].previous.buttons,
    };
    const joypad_buttons_raw_t pressed = {
        .value = current.value & ~previous.value,
    };
    return pressed.buttons;
}

joypad_buttons_t joypad_get_buttons_released(joypad_port_t port)
{
    const joypad_buttons_raw_t current = {
        .buttons = joypad_devices_cold[port].current.buttons,
    };
    const joypad_buttons_raw_t previous = {
        .buttons = joypad_devices_cold[port].previous.buttons,
    };
    const joypad_buttons_raw_t released = {
        .value = ~(current.value & previous.value),
    };
    return released.buttons;
}

joypad_buttons_t joypad_get_buttons_held(joypad_port_t port)
{
    const joypad_buttons_raw_t current = {
        .buttons = joypad_devices_cold[port].current.buttons,
    };
    const joypad_buttons_raw_t previous = {
        .buttons = joypad_devices_cold[port].previous.buttons,
    };
    const joypad_buttons_raw_t held = {
        .value = current.value & previous.value,
    };
    return held.buttons;
}
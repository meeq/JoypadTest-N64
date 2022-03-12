/**
 * @file joypad.c
 * @author Christopher Bonhage (me@christopherbonhage.com)
 * @brief Joypad Subsystem
 * @ingroup joypad
 */

#include <stdint.h>
#include <string.h>
#include <libdragon.h>

// Secret internal API
void joybus_exec_async(const void * input, void (*callback)(uint64_t *output, void *ctx), void *ctx);

#include "joybus_commands.h"
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

typedef struct joypad_device_s
{
    joybus_identifier_t identifier;
    joypad_style_t style;

    bool rumble_supported;
    bool rumble_active;

    joypad_data_t current;
    joypad_data_t previous;
} joypad_device_t;

typedef enum
{
    JOYPAD_N64_ACCESSORY_STATE_IDLE = 0,
    JOYPAD_N64_ACCESSORY_STATE_DETECT_WRITE1_PENDING,
    JOYPAD_N64_ACCESSORY_STATE_DETECT_WRITE2_PENDING,
    JOYPAD_N64_ACCESSORY_STATE_DETECT_READ_PENDING,
    JOYPAD_N64_ACCESSORY_STATE_RUMBLE_WRITE1_PENDING,
    JOYPAD_N64_ACCESSORY_STATE_RUMBLE_WRITE2_PENDING,
    JOYPAD_N64_ACCESSORY_STATE_RUMBLE_WRITE3_PENDING,
} joypad_n64_accessory_state_t;

typedef struct joypad_n64_accessory_context_s
{
    const joypad_port_t port;
    joypad_n64_accessory_state_t state;
    bool rumble_active;
} joypad_n64_accessory_context_t;

static joypad_device_t joypad_scan_devices[JOYPAD_PORT_COUNT] = { 0 };

// Interrupt-driven data
static volatile bool joypad_identify_pending = false;
static volatile int64_t joypad_identify_last_ticks = 0;
static volatile bool joypad_read_pending = false;
static volatile joypad_device_t joypad_read_devices[JOYPAD_PORT_COUNT] = { 0 };
static volatile joypad_n64_accessory_context_t joypad_n64_accessory_contexts[JOYPAD_PORT_COUNT] = {
    { JOYPAD_PORT_1 }, { JOYPAD_PORT_2 }, { JOYPAD_PORT_3 }, { JOYPAD_PORT_4 }
};

static uint16_t __calc_addr_crc( uint16_t address )
{
    static const uint16_t xor_table[16] = { 0x0, 0x0, 0x0, 0x0, 0x0, 0x15, 0x1F, 0x0B, 0x16, 0x19, 0x07, 0x0E, 0x1C, 0x0D, 0x1A, 0x01 };
    uint16_t crc = 0;
    address &= ~0x1F;
    for (int i = 15; i >= 5; i--)
        if ((address >> i) & 0x1)
            crc ^= xor_table[i];
    crc &= 0x1F;
    return address | crc;
}

static uint8_t __calc_data_crc(uint8_t *data)
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

static int __check_data_crc(uint8_t actual, uint8_t expected)
{
    if (expected == actual) return JOYBUS_N64_ACCESSORY_STATUS_OK;
    else if (expected == (actual ^ 0xFF)) return JOYBUS_N64_ACCESSORY_STATUS_ABSENT;
    else return JOYBUS_N64_ACCESSORY_STATUS_BADCRC;
}

static void joypad_n64_accessory_read(joypad_port_t port, uint16_t addr, void callback(uint64_t *out_dwords, void *ctx), void *ctx)
{
    uint16_t addr_crc = __calc_addr_crc(addr);
    uint8_t input[JOYBUS_BLOCK_SIZE] = {0};
    size_t i = port;

    const joybus_cmd_n64_accessory_read_port_t send_cmd = {
        .send_len = 0x03,
        .recv_len = 0x21,
        .command = 0x02,
        .addr_crc = addr_crc,
    };
    memcpy(&input[i], &send_cmd, sizeof(send_cmd));
    i += sizeof(send_cmd);

    // Close out the Joybus operation block
    input[i] = 0xFE;
    input[sizeof(input) - 1] = 0x01;

    joybus_exec_async(input, callback, ctx);
}

static void joypad_n64_accessory_write(joypad_port_t port, uint16_t addr, uint8_t *data, void callback(uint64_t *out_dwords, void *ctx), void *ctx)
{
    uint16_t addr_crc = __calc_addr_crc(addr);
    uint8_t input[JOYBUS_BLOCK_SIZE] = {0};
    size_t i = port;

    joybus_cmd_n64_accessory_write_port_t send_cmd = {
        .send_len = 0x23,
        .recv_len = 0x01,
        .command = 0x03,
        .addr_crc = addr_crc,
    };
    memcpy(&send_cmd.data, data, sizeof(send_cmd.data));
    memcpy(&input[i], &send_cmd, sizeof(send_cmd));
    i += sizeof(send_cmd);

    // Close out the Joybus operation block
    input[i] = 0xFE;
    input[sizeof(input) - 1] = 0x01;

    joybus_exec_async(input, callback, ctx);
}

static void joypad_n64_rumble_detect_read_callback(uint64_t *out_dwords, void *ctx)
{
    volatile joypad_n64_accessory_context_t *context = ctx;
    joypad_n64_accessory_state_t state = context->state;
    joypad_port_t port = context->port;

    uint8_t *out_bytes = (void *)out_dwords;
    joybus_cmd_n64_accessory_read_port_t recv_cmd;
    memcpy(&recv_cmd, &out_bytes[port], sizeof(recv_cmd));
    int status = __check_data_crc(recv_cmd.data_crc, __calc_data_crc(recv_cmd.data));

    if (state == JOYPAD_N64_ACCESSORY_STATE_DETECT_READ_PENDING)
    {
        if (status == JOYBUS_N64_ACCESSORY_STATUS_OK && recv_cmd.data[0] == 0x80)
        {
            joypad_read_devices[port].rumble_supported = true;
        }
        else
        {
            joypad_read_devices[port].rumble_supported = false;
            joypad_read_devices[port].rumble_active = false;
        }
        context->state = JOYPAD_N64_ACCESSORY_STATE_IDLE;
    }
}

static void joypad_n64_rumble_detect_write_callback(uint64_t *out_dwords, void *ctx)
{
    volatile joypad_n64_accessory_context_t *context = ctx;
    joypad_n64_accessory_state_t state = context->state;
    joypad_port_t port = context->port;

    uint8_t *out_bytes = (void *)out_dwords;
    joybus_cmd_n64_accessory_write_port_t recv_cmd;
    memcpy(&recv_cmd, &out_bytes[port], sizeof(recv_cmd));
    int status = __check_data_crc(recv_cmd.data_crc, __calc_data_crc(recv_cmd.data));

    if (status != JOYBUS_N64_ACCESSORY_STATUS_OK)
    {
        joypad_read_devices[port].rumble_supported = false;
        joypad_read_devices[port].rumble_active = false;
        context->state = JOYPAD_N64_ACCESSORY_STATE_IDLE;
    }
    else if (state == JOYPAD_N64_ACCESSORY_STATE_DETECT_WRITE1_PENDING)
    {
        context->state = JOYPAD_N64_ACCESSORY_STATE_DETECT_WRITE2_PENDING;
        uint8_t data2[JOYBUS_N64_ACCESSORY_DATA_SIZE];
        memset(data2, 0x80, sizeof(data2));
        joypad_n64_accessory_write(port, 0x8000, data2, joypad_n64_rumble_detect_write_callback, ctx);
    }
    else if (state == JOYPAD_N64_ACCESSORY_STATE_DETECT_WRITE2_PENDING)
    {
        context->state = JOYPAD_N64_ACCESSORY_STATE_DETECT_READ_PENDING;
        joypad_n64_accessory_read(port, 0x8000, joypad_n64_rumble_detect_read_callback, ctx);
    }
}

static void joypad_n64_rumble_detect(joypad_port_t port)
{
    volatile joypad_n64_accessory_context_t *context = &joypad_n64_accessory_contexts[port];
    if (context->state == JOYPAD_N64_ACCESSORY_STATE_IDLE)
    {
        context->state = JOYPAD_N64_ACCESSORY_STATE_DETECT_WRITE1_PENDING;
        uint8_t data[JOYBUS_N64_ACCESSORY_DATA_SIZE];
        memset(data, 0xFE, sizeof(data));
        joypad_n64_accessory_write(port, 0x8000, data, joypad_n64_rumble_detect_write_callback, (void *)context);
    }
}

static void joypad_n64_rumble_toggle_write_callback(uint64_t *out_dwords, void *ctx)
{
    volatile joypad_n64_accessory_context_t *context = ctx;
    joypad_n64_accessory_state_t state = context->state;
    joypad_port_t port = context->port;

    uint8_t *out_bytes = (void *)out_dwords;
    joybus_cmd_n64_accessory_write_port_t recv_cmd;
    memcpy(&recv_cmd, &out_bytes[port], sizeof(recv_cmd));
    int status = __check_data_crc(recv_cmd.data_crc, __calc_data_crc(recv_cmd.data));

    if (state == JOYPAD_N64_ACCESSORY_STATE_RUMBLE_WRITE3_PENDING || status != JOYBUS_N64_ACCESSORY_STATUS_OK)
    {
        context->state = JOYPAD_N64_ACCESSORY_STATE_IDLE;
    }
    else
    {
        if (state == JOYPAD_N64_ACCESSORY_STATE_RUMBLE_WRITE1_PENDING)
        {
            context->state = JOYPAD_N64_ACCESSORY_STATE_RUMBLE_WRITE2_PENDING;
        }
        else if (state == JOYPAD_N64_ACCESSORY_STATE_RUMBLE_WRITE2_PENDING)
        {
            context->state = JOYPAD_N64_ACCESSORY_STATE_RUMBLE_WRITE3_PENDING;
        }
        uint8_t data[JOYBUS_N64_ACCESSORY_DATA_SIZE];
        memset(data, context->rumble_active ? 0x01 : 0x00, sizeof(data));
        joypad_n64_accessory_write(context->port, 0xC000, data, joypad_n64_rumble_toggle_write_callback, ctx);
    }
}

static void joypad_n64_rumble_toggle(joypad_port_t port, bool active)
{
    volatile joypad_n64_accessory_context_t *context = &joypad_n64_accessory_contexts[port];
    disable_interrupts();
    context->state = JOYPAD_N64_ACCESSORY_STATE_RUMBLE_WRITE1_PENDING;
    context->rumble_active = active;
    enable_interrupts();
    uint8_t data[JOYBUS_N64_ACCESSORY_DATA_SIZE];
    memset(data, active ? 0x01 : 0x00, sizeof(data));
    joypad_n64_accessory_write(port, 0xC000, data, joypad_n64_rumble_toggle_write_callback, (void *)context);
}

static void joypad_identify_callback(uint64_t *out_dwords, void *ctx)
{
    uint8_t *out_bytes = (void *)out_dwords;
    joybus_cmd_identify_port_t recv_cmd;
    size_t i = 0;

    for (joypad_port_t port = JOYPAD_PORT_1; port < JOYPAD_PORT_COUNT; ++port)
    {
        memcpy(&recv_cmd, &out_bytes[i], sizeof(recv_cmd));
        i += sizeof(recv_cmd);

        const joybus_identifier_t identifier = recv_cmd.identifier;
        const joybus_identify_status_t status = recv_cmd.status;
        volatile joypad_device_t *device = &joypad_read_devices[port];

        if (device->identifier != identifier)
        {
            memset((void *)device, 0, sizeof(*device));
            device->identifier = identifier;
        }

        if (identifier == JOYBUS_IDENTIFIER_N64_CONTROLLER)
        {
            device->style = JOYPAD_STYLE_N64;
            if (status == JOYBUS_IDENTIFY_STATUS_N64_ACCESSORY)
            {
                joypad_n64_rumble_detect(port);
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

static void joypad_identify(bool reset)
{
    const joybus_cmd_identify_port_t send_cmd = {
        .send_len = 1,
        .recv_len = 3,
        .command = reset ? JOYBUS_COMMAND_RESET : JOYBUS_COMMAND_IDENTIFY,
    };

    uint8_t input[JOYBUS_BLOCK_SIZE] = {0};
    size_t i = 0;

    // Populate the Joybus commands on each port
    for (joypad_port_t port = JOYPAD_PORT_1; port < JOYPAD_PORT_COUNT; ++port)
    {
        memcpy(&input[i], &send_cmd, sizeof(send_cmd));
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
    uint8_t *out_bytes = (void *)out_dwords;
    volatile joypad_device_t *device;
    joypad_style_t style;
    size_t i = 0;
    uint8_t command;
    uint8_t command_len;

    for (joypad_port_t port = JOYPAD_PORT_1; port < JOYPAD_PORT_COUNT; ++port)
    {
        device = &joypad_read_devices[port];
        style = device->style;

        // Check send_len to figure out if this port has a command on it
        if (out_bytes[i] == 0)
        {
            // Skip this port
            i++;
            continue;
        }

        command_len = 2 + out_bytes[i] + out_bytes[i + 1];
        command = out_bytes[i + 2];

        if (style == JOYPAD_STYLE_N64)
        {
            // Ensure this command matches the identified controller
            if (command != JOYBUS_COMMAND_N64_CONTROLLER_READ)
            {
                // Skip this port
                i += command_len;
                continue;
            }

            joybus_cmd_n64_controller_read_port_t recv_cmd;
            memcpy(&recv_cmd, &out_bytes[i], sizeof(recv_cmd));
            i += sizeof(recv_cmd);

            int cstick_x = 0;
            if (recv_cmd.c_left && !recv_cmd.c_right) cstick_x = -127;
            if (!recv_cmd.c_left && recv_cmd.c_right) cstick_x = +127;

            int cstick_y = 0;
            if (recv_cmd.c_up && !recv_cmd.c_down) cstick_y = -127;
            if (!recv_cmd.c_up && recv_cmd.c_down) cstick_y = +127;

            device->previous = device->current;
            device->current.inputs = (joypad_inputs_t){
                .a = recv_cmd.a,
                .b = recv_cmd.b,
                .z = recv_cmd.z,
                .start = recv_cmd.start,
                .d_up = recv_cmd.d_up,
                .d_down = recv_cmd.d_down,
                .d_left = recv_cmd.d_left,
                .d_right = recv_cmd.d_right,
                .y = 0,
                .x = 0,
                .l = recv_cmd.l,
                .r = recv_cmd.r,
                .c_up = recv_cmd.c_up,
                .c_down = recv_cmd.c_down,
                .c_left = recv_cmd.c_left,
                .c_right = recv_cmd.c_right,
                .stick_x = recv_cmd.stick_x,
                .stick_y = recv_cmd.stick_y,
                .cstick_x = cstick_x,
                .cstick_y = cstick_y,
                .analog_l = recv_cmd.l ? 255 : 0,
                .analog_r = recv_cmd.r ? 255 : 0,
            };
        }
        else if (style == JOYPAD_STYLE_GCN)
        {
            // Ensure this command matches the identified controller
            if (command != JOYBUS_COMMAND_GCN_CONTROLLER_READ)
            {
                // Skip this port
                i += command_len;
                continue;
            }

            joybus_cmd_gcn_controller_read_port_t recv_cmd;
            memcpy(&recv_cmd, &out_bytes[i], sizeof(recv_cmd));
            i += sizeof(recv_cmd);

            // TODO Handle origins and dead-zones
            const int stick_x = ((int)recv_cmd.stick_x) - 128;
            const int stick_y = ((int)recv_cmd.stick_y) - 128;
            const int cstick_x = ((int)recv_cmd.cstick_x) - 128;
            const int cstick_y = ((int)recv_cmd.cstick_y) - 128;

            device->previous = device->current;
            device->current.inputs = (joypad_inputs_t){
                .a = recv_cmd.a,
                .b = recv_cmd.b,
                .z = recv_cmd.z,
                .start = recv_cmd.start,
                .d_up    = recv_cmd.d_up,
                .d_down  = recv_cmd.d_down,
                .d_left  = recv_cmd.d_left,
                .d_right = recv_cmd.d_right,
                .y = recv_cmd.y,
                .x = recv_cmd.x,
                .l = recv_cmd.l,
                .r = recv_cmd.r,
                .c_up    = cstick_y > +64,
                .c_down  = cstick_y < -64,
                .c_left  = cstick_x < -64,
                .c_right = cstick_x > +64,
                .stick_x = stick_x,
                .stick_y = stick_y,
                .cstick_x = cstick_x,
                .cstick_y = cstick_y,
                .analog_l = recv_cmd.analog_l,
                .analog_r = recv_cmd.analog_r,
            };
        }
        else
        {
            // Skip this port
            i += command_len;
        }
    }

    joypad_read_pending = false;
}

static void joypad_read(void)
{
    uint8_t input[JOYBUS_BLOCK_SIZE] = {0};
    volatile joypad_device_t *device;
    joypad_style_t style;
    size_t i = 0;
    
    // Populate the Joybus commands on each port
    for (joypad_port_t port = JOYPAD_PORT_1; port < JOYPAD_PORT_COUNT; ++port)
    {
        device = &joypad_read_devices[port];
        style = device->style;

        if (style == JOYPAD_STYLE_N64)
        {
            const joybus_cmd_n64_controller_read_port_t send_cmd = {
                .send_len = 1,
                .recv_len = 4,
                .command = JOYBUS_COMMAND_N64_CONTROLLER_READ,
            };
            memcpy(&input[i], &send_cmd, sizeof(send_cmd));
            i += sizeof(send_cmd);
        }
        else if (style == JOYPAD_STYLE_GCN)
        {
            const joybus_cmd_gcn_controller_read_port_t send_cmd = {
                .send_len = 3,
                .recv_len = 8,
                .command = JOYBUS_COMMAND_GCN_CONTROLLER_READ,
                .mode = 0x03,    // TODO: Dispel magic
                .rumble = device->rumble_active,
            };
            memcpy(&input[i], &send_cmd, sizeof(send_cmd));
            i += sizeof(send_cmd);
        }
        else
        {
            // send_len of 0 means "skip this port"
            input[i++] = 0;
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

void joypad_vi_interrupt_callback(void)
{
    if (joypad_identify_last_ticks + JOYPAD_IDENTIFY_INTERVAL_TICKS < timer_ticks())
    {
        joypad_identify(false);
    }
    joypad_read();
}

void joypad_init(void)
{
    joypad_identify(true);
    register_VI_handler(joypad_vi_interrupt_callback);
}

void joypad_close(void)
{
    unregister_VI_handler(joypad_vi_interrupt_callback);
}

void joypad_scan(void)
{
    assert(sizeof(joypad_scan_devices) == sizeof(joypad_read_devices));
    disable_interrupts();
    memcpy(&joypad_scan_devices, (void*)&joypad_read_devices, sizeof(joypad_scan_devices));
    enable_interrupts();
}

void joypad_set_rumble_active(joypad_port_t port, bool active)
{
    if (joypad_scan_devices[port].rumble_supported)
    {
        if (joypad_scan_devices[port].style == JOYPAD_STYLE_N64)
        {
            joypad_n64_rumble_toggle(port, active);
        }
        disable_interrupts();
        joypad_read_devices[port].rumble_active = active;
        enable_interrupts();
    }
}

joybus_identifier_t joypad_identifier(joypad_port_t port)
{
    return joypad_scan_devices[port].identifier;
}

joypad_style_t joypad_style(joypad_port_t port)
{
    return joypad_scan_devices[port].style;
}

bool joypad_is_rumble_supported(joypad_port_t port)
{
    return joypad_scan_devices[port].rumble_supported;
}

bool joypad_get_rumble_active(joypad_port_t port)
{
    return joypad_scan_devices[port].rumble_active;
}

joypad_inputs_t joypad_inputs(joypad_port_t port)
{
    return joypad_scan_devices[port].current.inputs;
}

joypad_inputs_t joypad_pressed(joypad_port_t port)
{
    const joypad_data_t current = joypad_scan_devices[port].current;
    const joypad_data_t previous = joypad_scan_devices[port].previous;
    const uint64_t pressed = current.digital & ~previous.digital;
    const joypad_data_t result = {.digital = pressed, .analog = current.analog};
    return result.inputs;
}

joypad_inputs_t joypad_released(joypad_port_t port)
{
    const joypad_data_t current = joypad_scan_devices[port].current;
    const joypad_data_t previous = joypad_scan_devices[port].previous;
    const uint64_t released = ~(current.digital & previous.digital);
    const joypad_data_t result = {.digital = released, .analog = current.analog};
    return result.inputs;
}

joypad_inputs_t joypad_held(joypad_port_t port)
{
    const joypad_data_t current = joypad_scan_devices[port].current;
    const joypad_data_t previous = joypad_scan_devices[port].previous;
    const uint64_t held = current.digital & previous.digital;
    const joypad_data_t result = {.digital = held, .analog = current.analog};
    return result.inputs;
}

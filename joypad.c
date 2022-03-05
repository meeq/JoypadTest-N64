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
void joybus_exec_async(const void * input, void (*callback)(uint64_t *output));

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

typedef uint16_t joypad_identifier_t;

#define JOYPAD_IDENTIFIER_UNKNOWN               0x0000
#define JOYPAD_IDENTIFIER_NONE                  0xFFFF
#define JOYPAD_IDENTIFIER_N64_CONTROLLER        0x0500
#define JOYPAD_IDENTIFIER_MASK_GCN_CONTROLLER   0x0900
#define JOYPAD_IDENTIFIER_MASK_GCN_NORUMBLE     0x2000

#define JOYPAD_IDENTIFY_INTERVAL_TICKS TICKS_PER_SECOND

typedef uint8_t joypad_status_t;

#define JOYPAD_STATUS_FLAG_GCN_RUMBLING  0x08
#define JOYPAD_STATUS_FLAG_N64_ACCESSORY 0x01

typedef struct __attribute__((packed)) joybus_cmd_identify_port_s
{
    /* metadata */
    uint8_t send_len;
    uint8_t recv_len;
    /* send data */
    uint8_t command;
    /* recv data */
    joypad_identifier_t identifier;
    joypad_status_t status;
} joybus_cmd_identify_port_t;

typedef struct __attribute__((packed)) joybus_cmd_n64_controller_read_port_s
{
    /* metadata */
    uint8_t send_len;
    uint8_t recv_len;
    /* send data */
    uint8_t command;
    /* recv data */
    unsigned a : 1;
    unsigned b : 1;
    unsigned z : 1;
    unsigned start : 1;
    unsigned d_up : 1;
    unsigned d_down : 1;
    unsigned d_left : 1;
    unsigned d_right : 1;
    unsigned reset : 1;
    unsigned : 1;
    unsigned l : 1;
    unsigned r : 1;
    unsigned c_up : 1;
    unsigned c_down : 1;
    unsigned c_left : 1;
    unsigned c_right : 1;
    signed stick_x : 8;
    signed stick_y : 8;
} joybus_cmd_n64_controller_read_port_t;

typedef struct __attribute__((packed)) joybus_cmd_gcn_controller_origin_port_s
{
    /* metadata */
    uint8_t send_len;
    uint8_t recv_len;
    /* send data */
    uint8_t command;
    /* recv data */
    unsigned err : 2;
    unsigned origin_unchecked : 1;
    unsigned start : 1;
    unsigned y : 1;
    unsigned x : 1;
    unsigned b : 1;
    unsigned a : 1;
    unsigned : 1;
    unsigned l : 1;
    unsigned r : 1;
    unsigned z : 1;
    unsigned d_up : 1;
    unsigned d_down : 1;
    unsigned d_right : 1;
    unsigned d_left : 1;
    unsigned stick_x : 8;
    unsigned stick_y : 8;
    unsigned cstick_x : 8;
    unsigned cstick_y : 8;
    unsigned analog_l : 8;
    unsigned analog_r : 8;
    unsigned deadzone_0 : 8;
    unsigned deadzone_1 : 8;
} joybus_cmd_gcn_controller_origin_port_t;

typedef struct __attribute__((packed)) joybus_cmd_gcn_controller_read_port_s
{
    /* metadata */
    uint8_t send_len;
    uint8_t recv_len;
    /* send data */
    uint8_t command;
    uint8_t mode;
    uint8_t rumble;
    /* recv data */
    unsigned err : 2;
    unsigned origin_unchecked : 1;
    unsigned start : 1;
    unsigned y : 1;
    unsigned x : 1;
    unsigned b : 1;
    unsigned a : 1;
    unsigned : 1;
    unsigned l : 1;
    unsigned r : 1;
    unsigned z : 1;
    unsigned d_up : 1;
    unsigned d_down : 1;
    unsigned d_right : 1;
    unsigned d_left : 1;
    unsigned stick_x : 8;
    unsigned stick_y : 8;
    unsigned cstick_x : 8;
    unsigned cstick_y : 8;
    unsigned analog_l : 8;
    unsigned analog_r : 8;
} joybus_cmd_gcn_controller_read_port_t;

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
    joypad_identifier_t identifier;
    joypad_style_t style;

    bool rumble_supported;
    bool rumble_state;

    joypad_data_t current;
    joypad_data_t previous;
} joypad_device_t;

static timer_link_t *joypad_identify_timer = NULL;
static joypad_device_t joypad_scan_devices[JOYPAD_PORT_COUNT];
static volatile joypad_device_t joypad_read_devices[JOYPAD_PORT_COUNT];
static volatile bool joypad_identify_pending = false;
static volatile bool joypad_read_pending = false;

#define JOYBUS_N64_ACCESSORY_DATA_SIZE 32

static void joypad_n64_controller_rumble_toggle(joypad_port_t port, bool enabled)
{
    uint8_t data[JOYBUS_N64_ACCESSORY_DATA_SIZE];
    memset(data, enabled ? 0x01 : 0x00, sizeof(data));
    // Magic alert: rumble state must be written thrice to behave consistently
    for (int i = 0; i < 3; ++i) write_mempak_address(port, 0xC000, data);
}

static void joypad_identify_callback(uint64_t *out_dwords)
{
    uint8_t *out_bytes = (void *)out_dwords;
    joybus_cmd_identify_port_t recv_cmd;
    size_t i = 0;

    for (joypad_port_t port = JOYPAD_PORT_1; port < JOYPAD_PORT_COUNT; ++port)
    {
        memcpy(&recv_cmd, &out_bytes[i], sizeof(recv_cmd));
        i += sizeof(recv_cmd);

        const joypad_identifier_t identifier = recv_cmd.identifier;
        // const joypad_status_t status = recv_cmd.status;
        volatile joypad_device_t *device = &joypad_read_devices[port];
        bool has_rumble = false;

        if (device->identifier != identifier)
        {
            memset((void *)device, 0, sizeof(*device));
            device->identifier = identifier;
        }

        if (identifier == JOYPAD_IDENTIFIER_N64_CONTROLLER)
        {
            device->style = JOYPAD_STYLE_N64;
            // has_rumble = (
            //     status == JOYPAD_STATUS_FLAG_N64_ACCESSORY &&
            //     joypad_n64_controller_rumble_supported( port )
            // );
        }
        else if (identifier & JOYPAD_IDENTIFIER_MASK_GCN_CONTROLLER)
        {
            device->style = JOYPAD_STYLE_GCN;
            has_rumble = !(identifier & JOYPAD_IDENTIFIER_MASK_GCN_NORUMBLE);
        }
        device->rumble_supported = has_rumble;
        device->rumble_state = has_rumble && device->rumble_state;
    }

    joypad_identify_pending = false;
}

static void joypad_identify(bool reset)
{
    const joybus_cmd_identify_port_t send_cmd = {
        .send_len = 1,
        .recv_len = 3,
        .command = reset ? 0xFF : 0x00,
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
        joybus_exec_async(input, joypad_identify_callback);
    }
}

static void joypad_read_callback(uint64_t *out_dwords)
{
    uint8_t *out_bytes = (void *)out_dwords;
    volatile joypad_device_t *device;
    joypad_style_t style;
    size_t i = 0;

    for (joypad_port_t port = JOYPAD_PORT_1; port < JOYPAD_PORT_COUNT; ++port)
    {
        device = &joypad_read_devices[port];
        style = device->style;

        if (style == JOYPAD_STYLE_N64)
        {
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
            assert(out_bytes[i] == 0);
            i++;
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
                .command = 0x01, // TODO Command ID enum
            };
            memcpy(&input[i], &send_cmd, sizeof(send_cmd));
            i += sizeof(send_cmd);
        }
        else if (style == JOYPAD_STYLE_GCN)
        {
            const joybus_cmd_gcn_controller_read_port_t send_cmd = {
                .send_len = 3,
                .recv_len = 8,
                .command = 0x40, // TODO: Command ID enum
                .mode = 0x03,    // TODO: Dispel magic
                .rumble = device->rumble_state,
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
        joybus_exec_async(input, joypad_read_callback);
    }
}

void joypad_identify_timer_callback(int ovfl)
{
    joypad_identify(false);
}

void joypad_init(void)
{
    memset(&joypad_scan_devices, 0, sizeof(joypad_scan_devices));
    memset((void *)&joypad_read_devices, 0, sizeof(joypad_read_devices));
    joypad_read_devices[0].style = JOYPAD_STYLE_N64;
    joypad_identify(true);
    joypad_identify_timer = new_timer(JOYPAD_IDENTIFY_INTERVAL_TICKS, TF_CONTINUOUS, joypad_identify_timer_callback);
    register_VI_handler(joypad_read);
}

void joypad_close(void)
{
    delete_timer(joypad_identify_timer);
    unregister_VI_handler(joypad_read);
}

void joypad_scan(void)
{
    assert(sizeof(joypad_scan_devices) == sizeof(joypad_read_devices));
    disable_interrupts();
    memcpy(&joypad_scan_devices, (void*)&joypad_read_devices, sizeof(joypad_scan_devices));
    enable_interrupts();
}

void joypad_set_rumble_state(joypad_port_t port, bool enabled)
{
    if (joypad_scan_devices[port].rumble_supported)
    {
        if (joypad_scan_devices[port].style == JOYPAD_STYLE_N64)
        {
            joypad_n64_controller_rumble_toggle(port, enabled);
        }
        disable_interrupts();
        joypad_read_devices[port].rumble_state = enabled;
        enable_interrupts();
    }
}

joypad_identifier_t joypad_identifier(joypad_port_t port)
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

bool joypad_get_rumble_state(joypad_port_t port)
{
    return joypad_scan_devices[port].rumble_state;
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

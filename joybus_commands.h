/**
 * @file joybus_commands.h
 * @author Christopher Bonhage (me@christopherbonhage.com)
 * @brief Joybus command helpers
 * @ingroup joypad 
 */

#ifndef __JOYBUS_COMMANDS_H
#define __JOYBUS_COMMANDS_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Expose libdragon joybus internal API
typedef void (*joybus_callback_t)(uint64_t *out_dwords, void *ctx);
void joybus_exec_async(const void * input, joybus_callback_t callback, void *ctx);

typedef uint16_t joybus_identifier_t;

#define JOYBUS_IDENTIFIER_UNKNOWN               0x0000
#define JOYBUS_IDENTIFIER_NONE                  0xFFFF
#define JOYBUS_IDENTIFIER_N64_CONTROLLER        0x0500
#define JOYBUS_IDENTIFIER_N64_MOUSE             0x0200
#define JOYBUS_IDENTIFIER_N64_VRU               0x0001
#define JOYBUS_IDENTIFIER_N64_KEYBOARD          0x0002
#define JOYBUS_IDENTIFIER_MASK_GCN_CONTROLLER   0x0900
#define JOYBUS_IDENTIFIER_MASK_GCN_NORUMBLE     0x2000

#define JOYBUS_IDENTIFY_STATUS_N64_ACCESSORY 0x01

#define JOYBUS_RANGE_N64_STICK_MAX    90
#define JOYBUS_RANGE_GCN_STICK_MAX    100
#define JOYBUS_RANGE_GCN_CSTICK_MAX   76
#define JOYBUS_RANGE_GCN_TRIGGER_MAX  200

#define JOYBUS_N64_ACCESSORY_DATA_SIZE 32

#define JOYBUS_N64_ACCESSORY_SECTOR_PROBE    0x8000
#define JOYBUS_N64_ACCESSORY_SECTOR_RUMBLE   0xC000

#define JOYBUS_N64_ACCESSORY_PROBE_RUMBLE_PAK       0x80
#define JOYBUS_N64_ACCESSORY_PROBE_TRANSFER_PAK_ON  0x84
#define JOYBUS_N64_ACCESSORY_PROBE_TRANSFER_PAK_OFF 0xFE

#define JOYBUS_COMMAND_SKIP_SIZE 1
#define JOYBUS_COMMAND_METADATA_SIZE 2
#define JOYBUS_COMMAND_OFFSET_SEND_LEN 0
#define JOYBUS_COMMAND_OFFSET_RECV_LEN 1
#define JOYBUS_COMMAND_OFFSET_COMMAND_ID 2

#define JOYBUS_COMMAND_ID_RESET                       0xFF
#define JOYBUS_COMMAND_ID_IDENTIFY                    0x00
#define JOYBUS_COMMAND_ID_N64_CONTROLLER_READ         0x01
#define JOYBUS_COMMAND_ID_N64_ACCESSORY_READ          0x02
#define JOYBUS_COMMAND_ID_N64_ACCESSORY_WRITE         0x03
#define JOYBUS_COMMAND_ID_GCN_CONTROLLER_READ         0x40
#define JOYBUS_COMMAND_ID_GCN_CONTROLLER_ORIGIN       0x41
#define JOYBUS_COMMAND_ID_GCN_CONTROLLER_RECALIBRATE  0x42

typedef struct __attribute__((packed)) joybus_cmd_n64_accessory_read_port_s
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
        uint8_t recv_bytes[0x21];
        struct __attribute__((__packed__))
        {
            uint8_t data[JOYBUS_N64_ACCESSORY_DATA_SIZE];
            uint8_t data_crc;
        };
    };
} joybus_cmd_n64_accessory_read_port_t;

typedef struct __attribute__((packed)) joybus_cmd_n64_accessory_write_port_s
{
    /* metadata */
    uint8_t send_len;
    uint8_t recv_len;
    /* send data */
    union
    {
        uint8_t send_bytes[0x23];
        struct __attribute__((__packed__))
        {
            uint8_t command;
            uint16_t addr_checksum;
            uint8_t data[JOYBUS_N64_ACCESSORY_DATA_SIZE];
        };
    };
    /* recv_data */
    union
    {
        uint8_t recv_bytes[0x01];
        uint8_t data_crc;
    };
} joybus_cmd_n64_accessory_write_port_t;

typedef struct __attribute__((packed)) joybus_cmd_identify_port_s
{
    /* metadata */
    uint8_t send_len;
    uint8_t recv_len;
    /* send data */
    union
    {
        uint8_t send_bytes[0x01];
        uint8_t command;
    };
    /* recv data */
    union
    {
        uint8_t recv_bytes[0x03];
        struct __attribute__((__packed__))
        {
            joybus_identifier_t identifier;
            uint8_t status;
        };
    };
} joybus_cmd_identify_port_t;

typedef struct __attribute__((packed)) joybus_cmd_n64_controller_read_port_s
{
    /* metadata */
    uint8_t send_len;
    uint8_t recv_len;
    /* send data */
    union
    {
        uint8_t send_bytes[0x01];
        uint8_t command;
    };
    /* recv data */
    union
    {
        uint8_t recv_bytes[0x04];
        struct __attribute__((__packed__))
        {
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
        };
    };
} joybus_cmd_n64_controller_read_port_t;

typedef struct __attribute__((packed)) joybus_cmd_gcn_controller_origin_port_s
{
    /* metadata */
    uint8_t send_len;
    uint8_t recv_len;
    /* send data */
     union
    {
        uint8_t send_bytes[0x01];
        uint8_t command;
    };
    /* recv data */
    union
    {
        uint8_t recv_bytes[0x0A];
        struct __attribute__((__packed__))
        {
            unsigned : 2;
            unsigned get_origin : 1;
            unsigned start : 1;
            unsigned y : 1;
            unsigned x : 1;
            unsigned b : 1;
            unsigned a : 1;
            unsigned use_origin : 1;
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
            unsigned analog_a : 8;
            unsigned analog_b : 8;
        };
    };
} joybus_cmd_gcn_controller_origin_port_t;

typedef struct __attribute__((packed)) joybus_cmd_gcn_controller_read_port_s
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
            uint8_t mode;
            uint8_t rumble;
        };
    };
    /* recv data */
    union
    {
        uint8_t recv_bytes[0x08];
        struct __attribute__((__packed__))
        {
            unsigned : 2;
            unsigned check_origin : 1;
            unsigned start : 1;
            unsigned y : 1;
            unsigned x : 1;
            unsigned b : 1;
            unsigned a : 1;
            unsigned use_origin : 1;
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
        };
    };
} joybus_cmd_gcn_controller_read_port_t;

#ifdef __cplusplus
}
#endif

#endif

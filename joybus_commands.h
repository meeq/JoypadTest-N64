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

typedef uint16_t joybus_identifier_t;

#define JOYBUS_IDENTIFIER_UNKNOWN               0x0000
#define JOYBUS_IDENTIFIER_NONE                  0xFFFF
#define JOYBUS_IDENTIFIER_N64_CONTROLLER        0x0500
#define JOYBUS_IDENTIFIER_MASK_GCN_CONTROLLER   0x0900
#define JOYBUS_IDENTIFIER_MASK_GCN_NORUMBLE     0x2000

typedef uint8_t joybus_identify_status_t;

#define JOYBUS_IDENTIFY_STATUS_N64_ACCESSORY 0x01

#define JOYBUS_N64_ACCESSORY_DATA_SIZE 32
#define JOYBUS_N64_ACCESSORY_STATUS_OK 0
#define JOYBUS_N64_ACCESSORY_STATUS_ABSENT -1
#define JOYBUS_N64_ACCESSORY_STATUS_BADCRC -2

typedef struct __attribute__((packed)) joybus_cmd_n64_accessory_read_port_s
{
    /* metadata */
    uint8_t send_len;
    uint8_t recv_len;
    /* send data */
    uint8_t command;
    uint16_t addr_crc;
    /* recv_data */
    uint8_t data[JOYBUS_N64_ACCESSORY_DATA_SIZE];
    uint8_t data_crc;
} joybus_cmd_n64_accessory_read_port_t;

typedef struct __attribute__((packed)) joybus_cmd_n64_accessory_write_port_s
{
    /* metadata */
    uint8_t send_len;
    uint8_t recv_len;
    /* send data */
    uint8_t command;
    uint16_t addr_crc;
    uint8_t data[JOYBUS_N64_ACCESSORY_DATA_SIZE];
    /* recv_data */
    uint8_t data_crc;
} joybus_cmd_n64_accessory_write_port_t;

typedef struct __attribute__((packed)) joybus_cmd_identify_port_s
{
    /* metadata */
    uint8_t send_len;
    uint8_t recv_len;
    /* send data */
    uint8_t command;
    /* recv data */
    joybus_identifier_t identifier;
    joybus_identify_status_t status;
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

#ifdef __cplusplus
}
#endif

#endif

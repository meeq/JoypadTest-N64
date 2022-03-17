/**
 * @file joybus_n64_accessory.h
 * @author Christopher Bonhage (me@christopherbonhage.com)
 * @brief Joybus N64 Accessory utilities
 * @ingroup joypad 
 */

#ifndef __JOYBUS_N64_ACCESSORY_H
#define __JOYBUS_N64_ACCESSORY_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define JOYBUS_N64_ACCESSORY_DATA_SIZE 32

#define JOYBUS_N64_ACCESSORY_SECTOR_PROBE    0x8000
#define JOYBUS_N64_ACCESSORY_SECTOR_RUMBLE   0xC000

#define JOYBUS_N64_ACCESSORY_PROBE_RUMBLE_PAK       0x80
#define JOYBUS_N64_ACCESSORY_PROBE_TRANSFER_PAK_ON  0x84
#define JOYBUS_N64_ACCESSORY_PROBE_TRANSFER_PAK_OFF 0xFE

#define JOYBUS_N64_ACCESSORY_DATA_CRC_STATUS_OK    0
#define JOYBUS_N64_ACCESSORY_DATA_CRC_STATUS_NULL  1
#define JOYBUS_N64_ACCESSORY_DATA_CRC_STATUS_BAD   2

typedef enum
{
    JOYBUS_N64_ACCESSORY_TYPE_NONE = 0,
    JOYBUS_N64_ACCESSORY_TYPE_CONTROLLER_PAK, // TODO
    JOYBUS_N64_ACCESSORY_TYPE_RUMBLE_PAK,
    JOYBUS_N64_ACCESSORY_TYPE_TRANSFER_PAK, // TODO
    JOYBUS_N64_ACCESSORY_TYPE_BIO_SENSOR, // TODO
} joybus_n64_accessory_type_t;

uint16_t joybus_n64_accessory_addr_checksum(uint16_t addr);
uint8_t joybus_n64_accessory_data_checksum(const uint8_t *data);
int joybus_n64_accessory_data_crc_compare(const uint8_t *data, uint8_t data_crc);

void joybus_n64_accessory_read_async(int port, uint16_t addr, joybus_callback_t callback, void *ctx);
void joybus_n64_accessory_write_async(int port, uint16_t addr, uint8_t *data, joybus_callback_t callback, void *ctx);

#ifdef __cplusplus
}
#endif

#endif

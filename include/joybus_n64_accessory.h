/**
 * @file joybus_n64_accessory.h
 * @author Christopher Bonhage (me@christopherbonhage.com)
 * @brief Joybus N64 Accessory utilities
 * @ingroup joypad 
 */

#ifndef __JOYBUS_N64_ACCESSORY_H
#define __JOYBUS_N64_ACCESSORY_H

#include <stdint.h>

#include "joybus_commands.h"

#ifdef __cplusplus
extern "C" {
#endif

#define JOYBUS_N64_ACCESSORY_DATA_SIZE 32
#define JOYBUS_N64_ACCESSORY_TRANSFER_BANK_SIZE 0x4000

/**
 * @anchor JOYBUS_N64_ACCESSORY_ADDR_MASK
 * @name Joybus N64 accessory address masks
 * @{
 */
#define JOYBUS_N64_ACCESSORY_ADDR_MASK_OFFSET    0xFFE0
#define JOYBUS_N64_ACCESSORY_ADDR_MASK_CHECKSUM  0x001F
/** @} */

/**
 * @anchor JOYBUS_N64_ACCESSORY_ADDR
 * @name Joybus N64 accessory addresses
 * @{
 */
#define JOYBUS_N64_ACCESSORY_ADDR_LABEL            0x0000
#define JOYBUS_N64_ACCESSORY_ADDR_PROBE            0x8000
#define JOYBUS_N64_ACCESSORY_ADDR_RUMBLE           0xC000
#define JOYBUS_N64_ACCESSORY_ADDR_BIO_PULSE        0xC000
#define JOYBUS_N64_ACCESSORY_ADDR_TRANSFER_BANK    0xA000
#define JOYBUS_N64_ACCESSORY_ADDR_TRANSFER_STATUS  0xB000
#define JOYBUS_N64_ACCESSORY_ADDR_TRANSFER_CART    0xC000
/** @} */

/**
 * @anchor JOYBUS_N64_ACCESSORY_PROBE
 * @name Joybus N64 accessory probe values
 * @{
 */
#define JOYBUS_N64_ACCESSORY_PROBE_RUMBLE_PAK       0x80
#define JOYBUS_N64_ACCESSORY_PROBE_BIO_SENSOR       0x81
#define JOYBUS_N64_ACCESSORY_PROBE_TRANSFER_PAK_ON  0x84
#define JOYBUS_N64_ACCESSORY_PROBE_SNAP_STATION     0x85
#define JOYBUS_N64_ACCESSORY_PROBE_TRANSFER_PAK_OFF 0xFE
/** @} */

/** @brief Joybus N64 accessory data CRC status values */
typedef enum
{
    JOYBUS_N64_ACCESSORY_DATA_CRC_STATUS_OK = 0,
    JOYBUS_N64_ACCESSORY_DATA_CRC_STATUS_NO_PAK,
    JOYBUS_N64_ACCESSORY_DATA_CRC_STATUS_MISMATCH,
} joybus_n64_accessory_data_crc_status_t;

uint16_t joybus_n64_accessory_addr_checksum(uint16_t addr);
uint8_t joybus_n64_accessory_data_checksum(const uint8_t *data);
int joybus_n64_accessory_data_crc_compare(const uint8_t *data, uint8_t data_crc);

void joybus_n64_accessory_read_async(int port, uint16_t addr, joybus_callback_t callback, void *ctx);
void joybus_n64_accessory_write_async(int port, uint16_t addr, const uint8_t *data, joybus_callback_t callback, void *ctx);

int joybus_n64_accessory_read_sync(int port, uint16_t addr, uint8_t *data);
int joybus_n64_accessory_write_sync(int port, uint16_t addr, const uint8_t *data);

#ifdef __cplusplus
}
#endif

#endif

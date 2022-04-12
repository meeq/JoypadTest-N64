/**
 * @file joypad_accessory.h
 * @author Christopher Bonhage (me@christopherbonhage.com)
 * @brief Joypad accessory
 * @ingroup joypad 
 */

#ifndef __JOYPAD_ACCESSORY_H
#define __JOYPAD_ACCESSORY_H

#include <stddef.h>
#include <stdint.h>
#include <libdragon.h>

#include "joypad.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @addtogroup joypad
 * @{
 */

#define JOYPAD_ACCESSORY_RETRY_LIMIT 2

typedef enum
{
    JOYPAD_ACCESSORY_STATE_IDLE = 0,
    // Accessory detection routine states
    JOYPAD_ACCESSORY_STATE_DETECT_INIT,
    JOYPAD_ACCESSORY_STATE_DETECT_LABEL_WRITE,
    JOYPAD_ACCESSORY_STATE_DETECT_LABEL_READ,
    JOYPAD_ACCESSORY_STATE_DETECT_RUMBLE_WRITE,
    JOYPAD_ACCESSORY_STATE_DETECT_RUMBLE_READ,
    JOYPAD_ACCESSORY_STATE_DETECT_TRANSFER_ON,
    JOYPAD_ACCESSORY_STATE_DETECT_TRANSFER_READ,
    JOYPAD_ACCESSORY_STATE_DETECT_TRANSFER_OFF,
    // Rumble Pak motor control states
    JOYPAD_ACCESSORY_STATE_RUMBLE_WRITE,
    // Transfer Pak power control states
    JOYPAD_ACCESSORY_STATE_TRANSFER_ENABLE_PROBE_WRITE,
    JOYPAD_ACCESSORY_STATE_TRANSFER_ENABLE_PROBE_WAIT,
    JOYPAD_ACCESSORY_STATE_TRANSFER_ENABLE_STATUS_WRITE,
    JOYPAD_ACCESSORY_STATE_TRANSFER_ENABLE_STATUS_WAIT,
    JOYPAD_ACCESSORY_STATE_TRANSFER_ENABLE_STATUS_READ,
    // Transfer Pak cartridge bank read states
    JOYPAD_ACCESSORY_STATE_TRANSFER_LOAD_BANK_WRITE,
    JOYPAD_ACCESSORY_STATE_TRANSFER_LOAD_DATA_READ,
    // Transfer Pak cartridge bank write states
    JOYPAD_ACCESSORY_STATE_TRANSFER_STORE_BANK_WRITE,
    JOYPAD_ACCESSORY_STATE_TRANSFER_STORE_DATA_WRITE,
} joypad_accessory_state_t;

#define joypad_accessory_state_is_detecting(state) \
    ((state) >= JOYPAD_ACCESSORY_STATE_DETECT_INIT && \
     (state) <= JOYPAD_ACCESSORY_STATE_DETECT_TRANSFER_OFF)

#define joypad_accessory_state_is_transfer_enabling(state) \
    ((state) >= JOYPAD_ACCESSORY_STATE_TRANSFER_ENABLE_PROBE_WRITE && \
     (state) <= JOYPAD_ACCESSORY_STATE_TRANSFER_ENABLE_STATUS_READ)

#define joypad_accessory_state_is_transfer_loading(state) \
    ((state) >= JOYPAD_ACCESSORY_STATE_TRANSFER_LOAD_BANK_WRITE && \
     (state) <= JOYPAD_ACCESSORY_STATE_TRANSFER_LOAD_DATA_READ)

#define joypad_accessory_state_is_transfer_storing(state) \
    ((state) >= JOYPAD_ACCESSORY_STATE_TRANSFER_STORE_BANK_WRITE && \
     (state) <= JOYPAD_ACCESSORY_STATE_TRANSFER_STORE_DATA_WRITE)

typedef enum
{
    JOYPAD_ACCESSORY_ERROR_NONE = 0,
    JOYPAD_ACCESSORY_ERROR_ABSENT,
    JOYPAD_ACCESSORY_ERROR_CHECKSUM,
    JOYPAD_ACCESSORY_ERROR_PENDING,
} joypad_accessory_error_t;

typedef union
{
    uint8_t raw;
    struct __attribute__((packed))
    {
        unsigned access_mode : 1;
        unsigned : 1;
        unsigned reset_status : 1;
        unsigned reset_detect : 1;
        unsigned : 2;
        unsigned cart_removed : 1;
        unsigned power : 1;
    };
} joypad_n64_transfer_pak_status_t;

typedef struct
{
    uint8_t *start;
    uint8_t *end;
    uint8_t *cursor;
    uint8_t bank;
    uint16_t cart_addr;
    uint16_t tpak_addr;
} joypad_n64_transfer_pak_io_t;

typedef struct joypad_accessory_s
{
    uint8_t status;
    joypad_accessory_type_t type;
    joypad_accessory_state_t state;
    joypad_accessory_error_t error;
    size_t retries;
    timer_link_t *transfer_pak_enable_timer;
    joypad_n64_transfer_pak_status_t transfer_pak_status;
    joypad_n64_transfer_pak_io_t transfer_pak_io;
} joypad_accessory_t;

joypad_accessory_type_t joypad_get_accessory_type(joypad_port_t port);
int joypad_get_accessory_state(joypad_port_t port);
int joypad_get_accessory_error(joypad_port_t port);
uint8_t joypad_get_accessory_transfer_pak_status(joypad_port_t port);

void joypad_accessory_detect_async(joypad_port_t port);
void joypad_n64_rumble_pak_toggle_async(joypad_port_t port, bool active);
void joypad_n64_transfer_pak_enable_async(joypad_port_t port, bool enabled);
void joypad_n64_transfer_pak_load_async(joypad_port_t port, uint16_t cart_addr, void *dst, size_t len);
void joypad_n64_transfer_pak_store_async(joypad_port_t port, uint16_t cart_addr, void *src, size_t len);

#ifdef __cplusplus
}
#endif

/** @} */ /* joypad */

#endif

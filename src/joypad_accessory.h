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
    JOYPAD_ACCESSORY_STATE_DETECT_INIT,
    JOYPAD_ACCESSORY_STATE_DETECT_LABEL_WRITE,
    JOYPAD_ACCESSORY_STATE_DETECT_LABEL_READ,
    JOYPAD_ACCESSORY_STATE_DETECT_RUMBLE_WRITE,
    JOYPAD_ACCESSORY_STATE_DETECT_RUMBLE_READ,
    JOYPAD_ACCESSORY_STATE_DETECT_TRANSFER_ON,
    JOYPAD_ACCESSORY_STATE_DETECT_TRANSFER_READ,
    JOYPAD_ACCESSORY_STATE_DETECT_TRANSFER_OFF,
    JOYPAD_ACCESSORY_STATE_DETECT_SNAP_WRITE,
    JOYPAD_ACCESSORY_STATE_DETECT_SNAP_READ,
    JOYPAD_ACCESSORY_STATE_RUMBLE_WRITE,
} joypad_accessory_state_t;

#define joypad_accessory_state_is_detecting(state) \
    ((state) >= JOYPAD_ACCESSORY_STATE_DETECT_INIT && \
     (state) <= JOYPAD_ACCESSORY_STATE_DETECT_SNAP_READ)

typedef enum
{
    JOYPAD_ACCESSORY_ERROR_NONE = 0,
    JOYPAD_ACCESSORY_ERROR_ABSENT,
    JOYPAD_ACCESSORY_ERROR_CHECKSUM,
    JOYPAD_ACCESSORY_ERROR_PENDING,
} joypad_accessory_error_t;

typedef struct joypad_accessory_s
{
    uint8_t status;
    joypad_accessory_type_t type;
    joypad_accessory_state_t state;
    joypad_accessory_error_t error;
    size_t retries;
} joypad_accessory_t;

void joypad_accessory_detect_async(joypad_port_t port);
void joypad_n64_rumble_pak_toggle_async(joypad_port_t port, bool active);

#ifdef __cplusplus
}
#endif

/** @} */ /* joypad */

#endif

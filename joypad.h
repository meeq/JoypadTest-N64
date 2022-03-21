/**
 * @file joypad.h
 * @author Christopher Bonhage (me@christopherbonhage.com)
 * @brief Joypad Subsystem
 * @ingroup joypad 
 */

#ifndef __JOYPAD_H
#define __JOYPAD_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @addtogroup joypad
 * @{
 */

/** @brief Joypad Port Numbers */
typedef enum
{
    /** @brief Joypad Port 1 */
    JOYPAD_PORT_1     = 0,
    /** @brief Joypad Port 2 */
    JOYPAD_PORT_2     = 1,
    /** @brief Joypad Port 3 */
    JOYPAD_PORT_3     = 2,
    /** @brief Joypad Port 4 */
    JOYPAD_PORT_4     = 3,
    /** @brief Joypad Port Count */
    JOYPAD_PORT_COUNT = 4,
} joypad_port_t;

/** @brief Convenience macro to iterate through all Joypad ports */
#define JOYPAD_PORT_FOR_EACH(iterator_token) for (\
    joypad_port_t iterator_token = JOYPAD_PORT_1; \
    iterator_token < JOYPAD_PORT_COUNT; \
    iterator_token += 1 \
)

/** @brief Joypad Style Types */
typedef enum
{
    /** @brief Unsupported Joypad Style */
    JOYPAD_STYLE_NONE = 0,
    /** @brief Nintendo 64 Controller Style */
    JOYPAD_STYLE_N64,
    /** @brief GameCube Controller Style */
    JOYPAD_STYLE_GCN,
    /** @brief Mouse Style */
    JOYPAD_STYLE_MOUSE,
} joypad_style_t;

/** @brief Common Joypad Inputs State Structure */
typedef struct __attribute__((packed)) joypad_inputs_s
{
    /** @brief State of the A button */
    unsigned a : 1;
    /** @brief State of the B button */
    unsigned b : 1;
    /** @brief State of the Z button */
    unsigned z : 1;
    /** @brief State of the Start button */
    unsigned start : 1;
    /** @brief State of the D-Pad Up button */
    unsigned d_up : 1;
    /** @brief State of the D-Pad Down button */
    unsigned d_down : 1;
    /** @brief State of the D-Pad Left button */
    unsigned d_left : 1;
    /** @brief State of the D-Pad Right button */
    unsigned d_right : 1;
    /**
     * @brief State of the Y button.
     * This input only exists on GCN controllers.
     */
    unsigned y : 1;
    /**
     * @brief State of the X button.
     * This input only exists on GCN controllers.
     */
    unsigned x : 1;
    /** @brief State of the digital L trigger */
    unsigned l : 1;
    /** @brief State of the digital R trigger */
    unsigned r : 1;
    /**
     * @brief State of the C-Up button.
     * For GameCube controllers, the value will be
     * emulated based on the C-Stick Y axis position.
     */
    unsigned c_up : 1;
    /**
     * @brief State of the C-Down button.
     * For GameCube controllers, the value will be
     * emulated based on the C-Stick Y axis position.
     */
    unsigned c_down : 1;
    /**
     * @brief State of the C-Left button.
     * For GameCube controllers, the value will be
     * emulated based on the C-Stick X axis position.
     */
    unsigned c_left : 1;
    /**
     * @brief State of the C-Right button.
     * For GameCube controllers, the value will be
     * emulated based on the C-Stick X axis position.
     */
    unsigned c_right : 1;
    /**
     * @brief State of the joystick X axis. (-127, +127)
     * On real controllers the range of this axis is roughly (-100, +100).
     * For well-worn N64 controllers, the range may be as low as (-60, +60).
     * For GCN controllers, this value will be relative to its origin.
     */
    signed stick_x : 8;
    /**
     * @brief State of the joystick Y axis. (-127, +127)
     * On real controllers the range of this axis is roughly (-100, +100).
     * For well-worn N64 controllers, the range may be as low as (-60, +60).
     * For GCN controllers, this value will be relative to its origin.
     */
    signed stick_y : 8;
    /**
     * @brief State of the "C-Stick" X axis. (-127, +127)
     * On real controllers the range of this axis is roughly (-76, +76).
     * For GCN controllers, this value will be relative to its origin.
     * For N64 controllers, this value will be emulated based on the
     * digital C-Left and C-Right button values (-76=C-Left, +76=C-Right).
     */
    signed cstick_x: 8;
    /**
     * @brief State of the "C-Stick" Y axis. (-127, +127)
     * On real controllers the range of this axis is roughly (-76, +76).
     * The value will be relative to the corresponding origin.
     * For N64 controllers, this value will be emulated based on the
     * digital C-Up and C-Down button values (-76=C-Down, +76=C-Up).
     */
    signed cstick_y: 8;
    /**
     * @brief State of the analog L trigger. (0, 255)
     * This value will be close to zero when no pressure is applied,
     * and close to 200 when full pressure is applied.
     * For GCN controllers, this value will be relative to its origin.
     * For N64 controllers, this value will be emulated based on the 
     * digital L trigger button value (0=unpressed, 200=pressed).
     */
    unsigned analog_l : 8;
    /**
     * @brief State of the analog R trigger. (0, 255)
     * This value will be close to zero when no pressure is applied,
     * and close to 200 when full pressure is applied.
     * For GCN controllers, this value will be relative to its origin.
     * For N64 controllers, this value will be emulated based on the
     * digital R trigger button value (0=unpressed, 200=pressed).
     */
    unsigned analog_r : 8;
} joypad_inputs_t;

typedef struct joypad_mouse_s
{
    unsigned a : 1;
    unsigned b : 1;
    unsigned   : 14;
    int8_t rel_x;
    int8_t rel_y;
    uint16_t abs_x;
    uint16_t abs_y;
} joypad_mouse_t;

typedef struct joypad_rect_s
{
    uint16_t min_x;
    uint16_t min_y;
    uint16_t max_x;
    uint16_t max_y;
} joypad_rect_t;

void joypad_init(void);
void joypad_close(void);
void joypad_identify(bool reset);
void joypad_scan(void);

joypad_style_t joypad_get_style(joypad_port_t port);
bool joypad_get_rumble_supported(joypad_port_t port);
bool joypad_get_rumble_active(joypad_port_t port);
void joypad_set_rumble_active(joypad_port_t port, bool enabled);

joypad_inputs_t joypad_inputs(joypad_port_t port);
joypad_inputs_t joypad_pressed(joypad_port_t port);
joypad_inputs_t joypad_released(joypad_port_t port);
joypad_inputs_t joypad_held(joypad_port_t port);

joypad_mouse_t joypad_mouse(joypad_port_t port);
void joypad_mouse_move(joypad_port_t port, unsigned x, unsigned y);
void joypad_mouse_limit(joypad_port_t port, joypad_rect_t rect);

#ifdef __cplusplus
}
#endif

/** @} */ /* joypad */

#endif

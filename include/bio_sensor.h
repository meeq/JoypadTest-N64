/**
 * @file joybus_n64_accessory.h
 * @author Christopher Bonhage (me@christopherbonhage.com)
 * @brief Joybus N64 Accessory utilities
 * @ingroup joypad 
 */

#ifndef __BIO_SENSOR_H
#define __BIO_SENSOR_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

void bio_sensor_init(void);
void bio_sensor_close(void);

void bio_sensor_read_start(int port);
void bio_sensor_read_stop(int port);

bool bio_sensor_get_active(int port);
bool bio_sensor_get_pulsing(int port);
int bio_sensor_get_bpm(int port);

#ifdef __cplusplus
}
#endif

#endif

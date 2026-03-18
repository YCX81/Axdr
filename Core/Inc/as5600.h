#ifndef __AS5600_H
#define __AS5600_H

#include <stdint.h>
#include <stdbool.h>

#define AS5600_ADDR     0x36    /* 7-bit I2C address */

typedef struct {
    uint16_t raw_angle;         /* 12-bit raw value [0, 4095] */
    float    angle_rad;         /* Electrical angle in radians [0, 2*PI) */
    float    mechanical_rad;    /* Mechanical angle in radians [0, 2*PI) */
    uint8_t  pole_pairs;        /* Motor pole pairs */
    uint16_t zero_offset;       /* Raw angle at electrical zero */
    bool     ready;             /* True if read succeeded */
} AS5600_t;

/* Initialize AS5600 (call after soft_i2c_init) */
void as5600_init(AS5600_t *enc, uint8_t pole_pairs);

/* Set current position as electrical zero */
void as5600_set_zero(AS5600_t *enc);

/* Read raw angle from sensor, update all fields.
 * Call this from main loop, NOT from ISR. */
bool as5600_update(AS5600_t *enc);

/* Get latest electrical angle (safe to call from ISR) */
float as5600_get_electrical_angle(const AS5600_t *enc);

/* Get latest mechanical angle (safe to call from ISR) */
float as5600_get_mechanical_angle(const AS5600_t *enc);

/* Global instance */
extern AS5600_t g_encoder;

#endif /* __AS5600_H */

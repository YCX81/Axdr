#ifndef __MT6835_H
#define __MT6835_H

#include <stdint.h>
#include <stdbool.h>

typedef struct {
    uint32_t raw_angle;         /* 21-bit raw value [0, 2097151] */
    float    angle_rad;         /* Electrical angle in radians [0, 2*PI) */
    float    mechanical_rad;    /* Mechanical angle in radians [0, 2*PI) */
    uint8_t  pole_pairs;        /* Motor pole pairs */
    uint32_t zero_offset;       /* Raw angle at electrical zero */
    bool     ready;             /* True if read succeeded */
} MT6835_t;

/* Initialize MT6835 (call after MX_SPI1_Init) */
void mt6835_init(MT6835_t *enc, uint8_t pole_pairs);

/* Set current position as electrical zero */
void mt6835_set_zero(MT6835_t *enc);

/* Read raw angle from sensor, update all fields.
 * Call this from main loop, NOT from ISR. */
bool mt6835_update(MT6835_t *enc);

/* Get latest electrical angle (safe to call from ISR) */
float mt6835_get_electrical_angle(const MT6835_t *enc);

/* Get latest mechanical angle (safe to call from ISR) */
float mt6835_get_mechanical_angle(const MT6835_t *enc);

/* Global instance */
extern MT6835_t g_encoder;

#endif /* __MT6835_H */

#include "as5600.h"
#include "soft_i2c.h"
#include "foc_config.h"

#define AS5600_REG_RAW_ANGLE_H  0x0C
#define AS5600_REG_STATUS       0x0B
#define AS5600_RAW_MAX          4096.0f

AS5600_t g_encoder = { 0 };

void as5600_init(AS5600_t *enc, uint8_t pole_pairs)
{
    enc->pole_pairs = pole_pairs;
    enc->zero_offset = 0;
    enc->raw_angle = 0;
    enc->angle_rad = 0.0f;
    enc->mechanical_rad = 0.0f;
    enc->ready = false;
}

void as5600_set_zero(AS5600_t *enc)
{
    if (enc->ready) {
        enc->zero_offset = enc->raw_angle;
    }
}

bool as5600_update(AS5600_t *enc)
{
    uint8_t buf[2];

    if (!soft_i2c_read_regs(AS5600_ADDR, AS5600_REG_RAW_ANGLE_H, buf, 2)) {
        /* Keep previous angle values on read failure — don't reset ready */
        return false;
    }

    /* Raw angle: 12-bit, buf[0] is high nibble, buf[1] is low byte */
    uint16_t raw = ((uint16_t)(buf[0] & 0x0F) << 8) | buf[1];
    enc->raw_angle = raw;

    /* Mechanical angle */
    enc->mechanical_rad = (float)raw * (FOC_2PI / AS5600_RAW_MAX);

    /* Electrical angle = (zero_offset - mechanical_angle) * pole_pairs  (reversed) */
    int32_t offset_raw = (int32_t)enc->zero_offset - (int32_t)raw;
    if (offset_raw < 0) {
        offset_raw += 4096;
    }

    float e_angle = (float)offset_raw * (FOC_2PI / AS5600_RAW_MAX) * (float)enc->pole_pairs;

    /* Wrap to [0, 2*PI) */
    while (e_angle >= FOC_2PI) e_angle -= FOC_2PI;
    while (e_angle < 0.0f)    e_angle += FOC_2PI;

    enc->angle_rad = e_angle;
    enc->ready = true;

    return true;
}

float as5600_get_electrical_angle(const AS5600_t *enc)
{
    return enc->angle_rad;
}

float as5600_get_mechanical_angle(const AS5600_t *enc)
{
    return enc->mechanical_rad;
}

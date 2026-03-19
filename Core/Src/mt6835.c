#include "mt6835.h"
#include "spi.h"
#include "main.h"
#include "foc_config.h"

/* MT6835 SPI registers (12-bit address space) */
#define MT6835_REG_ANGLE_H   0x003   /* ANGLE[20:13] */
#define MT6835_REG_ANGLE_M   0x004   /* ANGLE[12:5]  */
#define MT6835_REG_ANGLE_L   0x005   /* ANGLE[4:0] in bits [7:3] */

#define MT6835_CMD_READ      0x3U    /* 4-bit read opcode */
#define MT6835_RAW_MAX       2097152.0f  /* 2^21 */

#define MT6835_CS_LOW()   HAL_GPIO_WritePin(SPI1_CSN_GPIO_Port, SPI1_CSN_Pin, GPIO_PIN_RESET)
#define MT6835_CS_HIGH()  HAL_GPIO_WritePin(SPI1_CSN_GPIO_Port, SPI1_CSN_Pin, GPIO_PIN_SET)

MT6835_t g_encoder = { 0 };

/* Read one register via SPI (24-bit frame: 4-bit cmd | 12-bit addr | 8-bit data) */
static uint8_t mt6835_read_reg(uint16_t reg)
{
    uint8_t tx[3] = {
        (uint8_t)((MT6835_CMD_READ << 4) | ((reg >> 8) & 0x0F)),
        (uint8_t)(reg & 0xFF),
        0x00
    };
    uint8_t rx[3] = { 0 };

    MT6835_CS_LOW();
    HAL_SPI_TransmitReceive(&hspi1, tx, rx, 3, 10);
    MT6835_CS_HIGH();

    return rx[2];
}

void mt6835_init(MT6835_t *enc, uint8_t pole_pairs)
{
    enc->pole_pairs = pole_pairs;
    enc->zero_offset = 0;
    enc->raw_angle = 0;
    enc->angle_rad = 0.0f;
    enc->mechanical_rad = 0.0f;
    enc->ready = false;

    /* Ensure CS starts deasserted */
    MT6835_CS_HIGH();
}

void mt6835_set_zero(MT6835_t *enc)
{
    if (enc->ready) {
        enc->zero_offset = enc->raw_angle;
    }
}

bool mt6835_update(MT6835_t *enc)
{
    uint8_t h = mt6835_read_reg(MT6835_REG_ANGLE_H);
    uint8_t m = mt6835_read_reg(MT6835_REG_ANGLE_M);
    uint8_t l = mt6835_read_reg(MT6835_REG_ANGLE_L);

    /* Assemble 21-bit raw angle: H[7:0]=bit20..13, M[7:0]=bit12..5, L[7:3]=bit4..0 */
    uint32_t raw = ((uint32_t)h << 13) | ((uint32_t)m << 5) | (l >> 3);
    enc->raw_angle = raw;

    /* Mechanical angle */
    enc->mechanical_rad = (float)raw * (FOC_2PI / MT6835_RAW_MAX);

    /* Electrical angle = (zero_offset - raw) * pole_pairs (reversed direction) */
    int32_t offset_raw = (int32_t)enc->zero_offset - (int32_t)raw;
    if (offset_raw < 0) {
        offset_raw += 2097152;
    }

    float e_angle = (float)offset_raw * (FOC_2PI / MT6835_RAW_MAX) * (float)enc->pole_pairs;

    /* Wrap to [0, 2*PI) */
    while (e_angle >= FOC_2PI) e_angle -= FOC_2PI;
    while (e_angle < 0.0f)    e_angle += FOC_2PI;

    enc->angle_rad = e_angle;
    enc->ready = true;

    return true;
}

float mt6835_get_electrical_angle(const MT6835_t *enc)
{
    return enc->angle_rad;
}

float mt6835_get_mechanical_angle(const MT6835_t *enc)
{
    return enc->mechanical_rad;
}

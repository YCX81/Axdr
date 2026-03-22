#include "mt6835.h"
#include "spi.h"
#include "main.h"
#include "foc_config.h"
#include <math.h>

/* MT6835 SPI registers (12-bit address space) */
#define MT6835_REG_ANGLE_H   0x003   /* ANGLE[20:13] */
#define MT6835_REG_ANGLE_M   0x004   /* ANGLE[12:5]  */
#define MT6835_REG_ANGLE_L   0x005   /* ANGLE[4:0] in bits [7:3] */

#define MT6835_CMD_READ      0x3U    /* 4-bit read opcode */
#define MT6835_RAW_MAX       2097152.0f  /* 2^21 */

/* Direct GPIO for CS — single BSRR write instead of HAL function call */
#define MT6835_CS_LOW()   (GPIOD->BSRR = (uint32_t)SPI1_CSN_Pin << 16)
#define MT6835_CS_HIGH()  (GPIOD->BSRR = SPI1_CSN_Pin)

/* Pre-computed command bytes for the 3 angle registers (never change at runtime) */
#define CMD_ANGLE_H_B0  ((uint8_t)((MT6835_CMD_READ << 4) | ((MT6835_REG_ANGLE_H >> 8) & 0x0F)))
#define CMD_ANGLE_H_B1  ((uint8_t)(MT6835_REG_ANGLE_H & 0xFF))
#define CMD_ANGLE_M_B0  ((uint8_t)((MT6835_CMD_READ << 4) | ((MT6835_REG_ANGLE_M >> 8) & 0x0F)))
#define CMD_ANGLE_M_B1  ((uint8_t)(MT6835_REG_ANGLE_M & 0xFF))
#define CMD_ANGLE_L_B0  ((uint8_t)((MT6835_CMD_READ << 4) | ((MT6835_REG_ANGLE_L >> 8) & 0x0F)))
#define CMD_ANGLE_L_B1  ((uint8_t)(MT6835_REG_ANGLE_L & 0xFF))

MT6835_t g_encoder = { 0 };

/* Direct SPI byte transfer — no HAL overhead */
static inline uint8_t spi1_txrx_byte(uint8_t tx)
{
    /* Wait for TX buffer empty */
    while (!(SPI1->SR & SPI_SR_TXE)) {}
    /* Write as 8-bit to avoid 16-bit packing issues */
    *(volatile uint8_t *)&SPI1->DR = tx;
    /* Wait for RX data ready */
    while (!(SPI1->SR & SPI_SR_RXNE)) {}
    return *(volatile uint8_t *)&SPI1->DR;
}

/* Read one register: 3-byte SPI frame (cmd+addr | data) */
static inline uint8_t mt6835_read_reg_fast(uint8_t b0, uint8_t b1)
{
    MT6835_CS_LOW();
    (void)spi1_txrx_byte(b0);
    (void)spi1_txrx_byte(b1);
    uint8_t val = spi1_txrx_byte(0x00);
    /* Wait for SPI idle before releasing CS */
    while (SPI1->SR & SPI_SR_BSY) {}
    MT6835_CS_HIGH();
    return val;
}

void mt6835_init(MT6835_t *enc, uint8_t pole_pairs)
{
    enc->pole_pairs = pole_pairs;
    enc->zero_offset = 0;
    enc->raw_angle = 0;
    enc->angle_rad = 0.0f;
    enc->mechanical_rad = 0.0f;
    enc->ready = false;

    /* Pre-compute raw-to-electrical-angle scale factor */
    enc->raw_to_elec = (FOC_2PI / MT6835_RAW_MAX) * (float)pole_pairs;

    /* Ensure CS starts deasserted */
    MT6835_CS_HIGH();

    /* Enable SPI1 peripheral (HAL_SPI_Init does NOT set SPE) */
    SPI1->CR1 |= SPI_CR1_SPE;

    /* Drain any stale RX data to prevent RXNE stuck */
    while (SPI1->SR & SPI_SR_RXNE) {
        (void)*(volatile uint8_t *)&SPI1->DR;
    }
}

void mt6835_set_zero(MT6835_t *enc)
{
    if (enc->ready) {
        enc->zero_offset = enc->raw_angle;
    }
}

bool mt6835_update(MT6835_t *enc)
{
    uint8_t h = mt6835_read_reg_fast(CMD_ANGLE_H_B0, CMD_ANGLE_H_B1);
    uint8_t m = mt6835_read_reg_fast(CMD_ANGLE_M_B0, CMD_ANGLE_M_B1);
    uint8_t l = mt6835_read_reg_fast(CMD_ANGLE_L_B0, CMD_ANGLE_L_B1);

    /* Assemble 21-bit raw angle: H[7:0]=bit20..13, M[7:0]=bit12..5, L[7:3]=bit4..0 */
    uint32_t raw = ((uint32_t)h << 13) | ((uint32_t)m << 5) | (l >> 3);
    enc->raw_angle = raw;

    /* Mechanical angle */
    enc->mechanical_rad = (float)raw * (FOC_2PI / MT6835_RAW_MAX);

    /* Electrical angle = (zero_offset - raw) * pole_pairs */
     int32_t offset_raw = (int32_t)enc->zero_offset - (int32_t)raw;
    if (offset_raw < 0) {
        offset_raw += 2097152;
    }

    float e_angle = (float)offset_raw * enc->raw_to_elec;

    /* Wrap to [0, 2*PI) — fmodf avoids multi-iteration while loop */
    e_angle = fmodf(e_angle, FOC_2PI);
    if (e_angle < 0.0f) e_angle += FOC_2PI;

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

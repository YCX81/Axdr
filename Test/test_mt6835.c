#include "test_mt6835.h"
#include "test_runner.h"
#include "mt6835.h"
#include "spi.h"
#include "main.h"
#include <stdio.h>
#include <stdint.h>

#define MT6835_RAW_MAX_VAL  2097151U   /* 2^21 - 1 */
#define STABILITY_READS     10
#define STABILITY_THRESHOLD 50         /* counts (~0.008 deg) */

/* Re-declare register addresses (same as mt6835.c) */
#define REG_ANGLE_H  0x003
#define REG_ANGLE_M  0x004
#define REG_ANGLE_L  0x005
#define CMD_READ     0x3U

static uint8_t read_reg_raw(uint16_t reg)
{
    uint8_t tx[3] = {
        (uint8_t)((CMD_READ << 4) | ((reg >> 8) & 0x0F)),
        (uint8_t)(reg & 0xFF),
        0x00
    };
    uint8_t rx[3] = { 0 };

    HAL_GPIO_WritePin(SPI1_CSN_GPIO_Port, SPI1_CSN_Pin, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(&hspi1, tx, rx, 3, 10);
    HAL_GPIO_WritePin(SPI1_CSN_GPIO_Port, SPI1_CSN_Pin, GPIO_PIN_SET);

    return rx[2];
}

int test_mt6835_range(void)
{
    mt6835_update(&g_encoder);
    uint32_t raw = g_encoder.raw_angle;

    int ok = (raw > 0) && (raw < MT6835_RAW_MAX_VAL);

    char buf[80];
    snprintf(buf, sizeof(buf), "  [%s] Range: raw=%lu (0x%06lX)%s\r\n",
             ok ? "PASS" : "FAIL",
             (unsigned long)raw, (unsigned long)raw,
             (raw == 0)              ? " <- all zeros, MISO stuck low?" :
             (raw == MT6835_RAW_MAX_VAL) ? " <- all ones, MISO stuck high?" : "");
    test_print(buf);
    return ok;
}

int test_mt6835_stability(void)
{
    uint32_t values[STABILITY_READS];

    for (int i = 0; i < STABILITY_READS; i++) {
        mt6835_update(&g_encoder);
        values[i] = g_encoder.raw_angle;
        HAL_Delay(5);
    }

    uint32_t mn = values[0], mx = values[0];
    for (int i = 1; i < STABILITY_READS; i++) {
        if (values[i] < mn) mn = values[i];
        if (values[i] > mx) mx = values[i];
    }
    uint32_t delta = mx - mn;

    int ok = (delta < STABILITY_THRESHOLD);

    char buf[72];
    snprintf(buf, sizeof(buf),
             "  [%s] Stability: delta=%lu counts (%lu reads, threshold=%d)\r\n",
             ok ? "PASS" : "FAIL",
             (unsigned long)delta,
             (unsigned long)STABILITY_READS,
             STABILITY_THRESHOLD);
    test_print(buf);
    return ok;
}

void test_mt6835_dump(void)
{
    uint8_t h = read_reg_raw(REG_ANGLE_H);
    uint8_t m = read_reg_raw(REG_ANGLE_M);
    uint8_t l = read_reg_raw(REG_ANGLE_L);
    uint32_t raw = ((uint32_t)h << 13) | ((uint32_t)m << 5) | (l >> 3);

    char buf[80];
    snprintf(buf, sizeof(buf),
             "  [INFO] Raw bytes: H=0x%02X M=0x%02X L=0x%02X -> raw=%lu\r\n",
             h, m, l, (unsigned long)raw);
    test_print(buf);
}

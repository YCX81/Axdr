#include "vofa.h"
#include "foc_ctrl.h"
#include "mt6835.h"
#include "usbd_cdc_if.h"
#include <string.h>

/*
 * USB CDC FS throughput: ~800 KB/s practical
 *   Frame size = 20 channels * 4 bytes + 4 tail = 84 bytes
 *   Max frame rate = 800000 / 84 ≈ 9500 fps
 *
 * Decimation = 4  -> 5kHz output  -> 420 KB/s -> safe
 * Decimation = 2  -> 10kHz output -> 840 KB/s -> near limit
 */
#define VOFA_DECIMATION  4   /* 5kHz output, ~360 KB/s (safe for USB FS) */

/* JustFloat tail bytes: IEEE 754 representation of +Inf (0x7F800000) */
static const uint8_t vofa_tail[4] = { 0x00, 0x00, 0x80, 0x7F };

/* Double-buffer: fill one while USB sends the other */
static uint8_t vofa_buf[2][VOFA_FRAME_SIZE] __attribute__((aligned(4)));
static volatile uint8_t vofa_buf_idx = 0;
static uint16_t vofa_counter = 0;

void vofa_send_from_isr(void)
{
    vofa_counter++;
    if (vofa_counter < VOFA_DECIMATION) {
        return;
    }
    vofa_counter = 0;

    /* Fill the idle buffer */
    uint8_t fill_idx = vofa_buf_idx;
    float *p = (float *)vofa_buf[fill_idx];

    p[0] = g_foc.theta_elec;
    p[1] = (float)g_foc.raw_adc_a;
    p[2] = (float)g_foc.raw_adc_b;
    p[3] = (float)g_foc.raw_adc_c;
    p[4] = (float)g_foc.raw_adc_bus;
    p[5] = (float)g_foc.adc_offset_a;
    p[6] = (float)g_foc.adc_offset_b;
    p[7] = (float)g_foc.adc_offset_c;
    p[8] = (float)g_foc.adc_offset_bus;
    p[9] = g_foc.ia;
    p[10] = g_foc.ib;
    p[11] = g_foc.ic;
    p[12] = g_foc.ibus;
    p[13] = g_foc.id;
    p[14] = g_foc.iq;
    p[15] = g_foc.duty_a;
    p[16] = g_foc.duty_b;
    p[17] = g_foc.duty_c;
    p[18] = g_encoder.mechanical_rad;
    p[19] = (float)g_encoder.raw_angle;

    /* Append JustFloat tail */
    memcpy(&vofa_buf[fill_idx][VOFA_NUM_CHANNELS * 4], vofa_tail, 4);

    /* Swap buffer */
    vofa_buf_idx = 1 - fill_idx;

    /* Send via USB CDC (non-blocking, returns USBD_BUSY if previous TX pending) */
    CDC_Transmit_FS(vofa_buf[fill_idx], VOFA_FRAME_SIZE);
}

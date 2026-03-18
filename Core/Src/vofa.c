#include "vofa.h"
#include "foc_ctrl.h"
#include "usbd_cdc_if.h"
#include <string.h>

/*
 * USB CDC FS throughput: ~800 KB/s practical
 *   Frame size = 8 channels * 4 bytes + 4 tail = 36 bytes
 *   Max frame rate = 800000 / 36 ≈ 22000 fps
 *
 * Decimation = 2  -> 10kHz output -> 360 KB/s -> comfortable
 * Decimation = 4  -> 5kHz output  -> 180 KB/s -> very safe
 * Decimation = 1  -> 20kHz output -> 720 KB/s -> near limit, may drop frames
 */
#define VOFA_DECIMATION  1   /* 20kHz output, ~720 KB/s (near USB FS limit) */

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
    p[1] = g_foc.ia;
    p[2] = g_foc.ib;
    p[3] = g_foc.ic;
    p[4] = g_foc.id;
    p[5] = g_foc.iq;
    p[6] = g_foc.duty_a;
    p[7] = g_foc.duty_b;
    p[8] = g_foc.duty_c;

    /* Append JustFloat tail */
    memcpy(&vofa_buf[fill_idx][VOFA_NUM_CHANNELS * 4], vofa_tail, 4);

    /* Swap buffer */
    vofa_buf_idx = 1 - fill_idx;

    /* Send via USB CDC (non-blocking, returns USBD_BUSY if previous TX pending) */
    CDC_Transmit_FS(vofa_buf[fill_idx], VOFA_FRAME_SIZE);
}

#ifndef __VOFA_H
#define __VOFA_H

#include <stdint.h>

/* VOFA+ JustFloat protocol debug output via USB CDC
 *
 * JustFloat format: [float ch0][float ch1]...[float chN][0x00 0x00 0x80 0x7f]
 * Each frame = (N_channels * 4) + 4 bytes
 *
 * Channels:
 *   0: theta_elec    (electrical angle, rad)
 *   1: raw_adc_a     (phase A raw ADC code)
 *   2: raw_adc_b     (phase B raw ADC code)
 *   3: raw_adc_c     (phase C raw ADC code)
 *   4: raw_adc_bus   (bus current raw ADC code)
 *   5: offset_a      (phase A zero-current offset code)
 *   6: offset_b      (phase B zero-current offset code)
 *   7: offset_c      (phase C zero-current offset code)
 *   8: offset_bus    (bus current zero-current offset code)
 *   9: ia            (filtered phase A current, A)
 *  10: ib            (filtered phase B current, A)
 *  11: ic            (filtered phase C current, A)
 *  12: ibus          (filtered bus current, A)
 *  13: id            (filtered d-axis current, A)
 *  14: iq            (filtered q-axis current, A)
 *  15: duty_a        (phase A duty cycle, 0~1)
 *  16: duty_b        (phase B duty cycle, 0~1)
 *  17: duty_c        (phase C duty cycle, 0~1)
 *  18: mech_angle    (MT6835 mechanical angle, rad)
 *  19: enc_raw       (MT6835 raw count, 0~2097151)
 */

#define VOFA_NUM_CHANNELS  20
#define VOFA_FRAME_SIZE    ((VOFA_NUM_CHANNELS * 4) + 4)  /* floats + tail */

/* Call from TIM1 ISR (20kHz). Decimates internally to fit UART bandwidth. */
void vofa_send_from_isr(void);

#endif /* __VOFA_H */

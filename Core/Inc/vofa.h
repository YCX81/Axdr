#ifndef __VOFA_H
#define __VOFA_H

#include <stdint.h>

/* VOFA+ JustFloat protocol debug output via USB CDC
 *
 * JustFloat format: [float ch0][float ch1]...[float chN][0x00 0x00 0x80 0x7f]
 * Each frame = (N_channels * 4) + 4 bytes
 *
 * Channels:
 *   0: theta_elec  (electrical angle, rad)
 *   1: ia          (phase A current, A)
 *   2: ib          (phase B current, A)
 *   3: ic          (phase C current, A)
 *   4: id          (d-axis current, A)
 *   5: iq          (q-axis current, A)
 *   6: duty_a      (phase A duty cycle, 0~1)
 *   7: duty_b      (phase B duty cycle, 0~1)
 *   8: duty_c      (phase C duty cycle, 0~1)
 */

#define VOFA_NUM_CHANNELS  9
#define VOFA_FRAME_SIZE    ((VOFA_NUM_CHANNELS * 4) + 4)  /* floats + tail */

/* Call from TIM1 ISR (20kHz). Decimates internally to fit UART bandwidth. */
void vofa_send_from_isr(void);

#endif /* __VOFA_H */

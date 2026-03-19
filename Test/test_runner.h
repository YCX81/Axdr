#ifndef __TEST_RUNNER_H
#define __TEST_RUNNER_H

#include <stdint.h>

/* Print a string over USB CDC */
void test_print(const char *str);

/* Run all tests, print results via USB CDC.
 * Call after MX_USB_Device_Init() + HAL_Delay(500) to let USB enumerate. */
void test_run_all(void);

#endif /* __TEST_RUNNER_H */

#ifndef __TEST_MT6835_H
#define __TEST_MT6835_H

/* Range check: raw_angle must be in (0, 2097151) — all-zero or all-one means SPI failure */
int test_mt6835_range(void);

/* Stability: 10 reads at rest, max-min must be < 50 counts (~0.008 deg) */
int test_mt6835_stability(void);

/* Dump: print raw SPI bytes for visual inspection */
void test_mt6835_dump(void);

#endif /* __TEST_MT6835_H */

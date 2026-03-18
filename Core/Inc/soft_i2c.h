#ifndef __SOFT_I2C_H
#define __SOFT_I2C_H

#include <stdint.h>
#include <stdbool.h>

/* Initialize GPIO pins for software I2C (PA3=SCL, PA4=SDA) */
void soft_i2c_init(void);

/* Read len bytes from device at 7-bit addr, starting at reg_addr */
bool soft_i2c_read_regs(uint8_t dev_addr, uint8_t reg_addr, uint8_t *buf, uint8_t len);

/* Write len bytes to device at 7-bit addr, starting at reg_addr */
bool soft_i2c_write_regs(uint8_t dev_addr, uint8_t reg_addr, const uint8_t *buf, uint8_t len);

#endif /* __SOFT_I2C_H */

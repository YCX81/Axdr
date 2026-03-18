#include "soft_i2c.h"
#include "stm32g4xx_hal.h"

/* ---- Pin definitions: PB3 = SCL, PB4 = SDA ---- */
#define I2C_SCL_PORT    GPIOB
#define I2C_SCL_PIN     GPIO_PIN_3
#define I2C_SDA_PORT    GPIOB
#define I2C_SDA_PIN     GPIO_PIN_4

/* Half-period delay for ~200kHz (conservative, reliable in ISR context) */
static void i2c_delay(void)
{
    /* ~0.5us at 160MHz, short wires + internal pull-up */
    volatile uint32_t n = 4;
    while (n--) { __NOP(); }
}

static void scl_high(void) { I2C_SCL_PORT->BSRR = I2C_SCL_PIN; }
static void scl_low(void)  { I2C_SCL_PORT->BSRR = (uint32_t)I2C_SCL_PIN << 16U; }
static void sda_high(void) { I2C_SDA_PORT->BSRR = I2C_SDA_PIN; }  /* open-drain: release */
static void sda_low(void)  { I2C_SDA_PORT->BSRR = (uint32_t)I2C_SDA_PIN << 16U; }

static uint8_t sda_read(void)
{
    return (I2C_SDA_PORT->IDR & I2C_SDA_PIN) ? 1U : 0U;
}

void soft_i2c_init(void)
{
    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitTypeDef gpio = {0};
    /* Open-drain output with pull-up for both SCL and SDA */
    gpio.Pin   = I2C_SCL_PIN | I2C_SDA_PIN;
    gpio.Mode  = GPIO_MODE_OUTPUT_OD;
    gpio.Pull  = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &gpio);

    /* Idle: both lines high */
    scl_high();
    sda_high();
}

static void i2c_start(void)
{
    sda_high(); i2c_delay();
    scl_high(); i2c_delay();
    sda_low();  i2c_delay();
    scl_low();  i2c_delay();
}

static void i2c_stop(void)
{
    sda_low();  i2c_delay();
    scl_high(); i2c_delay();
    sda_high(); i2c_delay();
}

/* Send 8 bits MSB first, return ACK (0=ACK, 1=NACK) */
static uint8_t i2c_write_byte(uint8_t byte)
{
    for (uint8_t i = 0; i < 8; i++) {
        if (byte & 0x80) {
            sda_high();
        } else {
            sda_low();
        }
        byte <<= 1;
        i2c_delay();
        scl_high(); i2c_delay();
        scl_low();  i2c_delay();
    }

    /* Read ACK */
    sda_high(); i2c_delay();
    scl_high(); i2c_delay();
    uint8_t ack = sda_read();
    scl_low();  i2c_delay();

    return ack;
}

/* Read 8 bits MSB first, send ACK or NACK */
static uint8_t i2c_read_byte(uint8_t send_ack)
{
    uint8_t byte = 0;
    sda_high(); /* Release SDA for slave to drive */

    for (uint8_t i = 0; i < 8; i++) {
        byte <<= 1;
        scl_high(); i2c_delay();
        if (sda_read()) {
            byte |= 1;
        }
        scl_low(); i2c_delay();
    }

    /* Send ACK (0) or NACK (1) */
    if (send_ack) {
        sda_low();
    } else {
        sda_high();
    }
    i2c_delay();
    scl_high(); i2c_delay();
    scl_low();  i2c_delay();
    sda_high();

    return byte;
}

bool soft_i2c_read_regs(uint8_t dev_addr, uint8_t reg_addr, uint8_t *buf, uint8_t len)
{
    /* Write phase: send device address + register */
    i2c_start();
    if (i2c_write_byte((dev_addr << 1) | 0)) { /* Write mode */
        i2c_stop();
        return false;
    }
    if (i2c_write_byte(reg_addr)) {
        i2c_stop();
        return false;
    }

    /* Repeated start + read phase */
    i2c_start();
    if (i2c_write_byte((dev_addr << 1) | 1)) { /* Read mode */
        i2c_stop();
        return false;
    }

    for (uint8_t i = 0; i < len; i++) {
        uint8_t ack = (i < (len - 1)) ? 1 : 0;  /* ACK all except last byte */
        buf[i] = i2c_read_byte(ack);
    }

    i2c_stop();
    return true;
}

bool soft_i2c_write_regs(uint8_t dev_addr, uint8_t reg_addr, const uint8_t *buf, uint8_t len)
{
    i2c_start();
    if (i2c_write_byte((dev_addr << 1) | 0)) {
        i2c_stop();
        return false;
    }
    if (i2c_write_byte(reg_addr)) {
        i2c_stop();
        return false;
    }

    for (uint8_t i = 0; i < len; i++) {
        if (i2c_write_byte(buf[i])) {
            i2c_stop();
            return false;
        }
    }

    i2c_stop();
    return true;
}

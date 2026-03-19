#ifndef __TEST_SPI_H
#define __TEST_SPI_H

/* SPI1 state check: returns 1 if hspi1 is READY */
int test_spi_state(void);

/* CS GPIO toggle: drive low then high, verify via ODR */
int test_spi_cs_gpio(void);

/* Raw SPI transaction: send 3 zero bytes, expect HAL_OK */
int test_spi_transaction(void);

#endif /* __TEST_SPI_H */

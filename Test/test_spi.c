#include "test_spi.h"
#include "test_runner.h"
#include "spi.h"
#include "main.h"
#include <stdio.h>
#include <string.h>

int test_spi_state(void)
{
    if (hspi1.State == HAL_SPI_STATE_READY) {
        test_print("  [PASS] SPI1 state: READY\r\n");
        return 1;
    } else {
        char buf[64];
        snprintf(buf, sizeof(buf), "  [FAIL] SPI1 state: %d (expected READY=1)\r\n",
                 (int)hspi1.State);
        test_print(buf);
        return 0;
    }
}

int test_spi_cs_gpio(void)
{
    /* Drive CS low — ODR bit must be 0 */
    HAL_GPIO_WritePin(SPI1_CSN_GPIO_Port, SPI1_CSN_Pin, GPIO_PIN_RESET);
    int low_ok = (SPI1_CSN_GPIO_Port->ODR & SPI1_CSN_Pin) == 0;

    /* Drive CS high — ODR bit must be 1 */
    HAL_GPIO_WritePin(SPI1_CSN_GPIO_Port, SPI1_CSN_Pin, GPIO_PIN_SET);
    int high_ok = (SPI1_CSN_GPIO_Port->ODR & SPI1_CSN_Pin) != 0;

    if (low_ok && high_ok) {
        test_print("  [PASS] CS GPIO: LOW=0 HIGH=1\r\n");
        return 1;
    } else {
        char buf[64];
        snprintf(buf, sizeof(buf), "  [FAIL] CS GPIO: low_ok=%d high_ok=%d\r\n",
                 low_ok, high_ok);
        test_print(buf);
        return 0;
    }
}

int test_spi_transaction(void)
{
    uint8_t tx[3] = { 0x30, 0x03, 0x00 }; /* READ cmd, ANGLE_H reg, dummy */
    uint8_t rx[3] = { 0 };

    HAL_GPIO_WritePin(SPI1_CSN_GPIO_Port, SPI1_CSN_Pin, GPIO_PIN_RESET);
    HAL_StatusTypeDef ret = HAL_SPI_TransmitReceive(&hspi1, tx, rx, 3, 10);
    HAL_GPIO_WritePin(SPI1_CSN_GPIO_Port, SPI1_CSN_Pin, GPIO_PIN_SET);

    if (ret == HAL_OK) {
        char buf[64];
        snprintf(buf, sizeof(buf),
                 "  [PASS] SPI transaction: rx=[0x%02X 0x%02X 0x%02X]\r\n",
                 rx[0], rx[1], rx[2]);
        test_print(buf);
        return 1;
    } else {
        char buf[48];
        snprintf(buf, sizeof(buf), "  [FAIL] SPI transaction: HAL error %d\r\n", (int)ret);
        test_print(buf);
        return 0;
    }
}

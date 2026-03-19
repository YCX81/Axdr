#include "test_runner.h"
#include "test_spi.h"
#include "test_mt6835.h"
#include "usart.h"
#include <string.h>

/* Blocking UART print via USART3 (JLink CDC UART, COM4) */
void test_print(const char *str)
{
    uint16_t len = (uint16_t)strlen(str);
    HAL_UART_Transmit(&huart3, (uint8_t *)str, len, 100);
}

void test_run_all(void)
{
    int passed = 0, total = 0;

    test_print("\r\n=== SPI Tests ===\r\n");

    total++; if (test_spi_state())       passed++;
    total++; if (test_spi_cs_gpio())     passed++;
    total++; if (test_spi_transaction()) passed++;

    test_print("\r\n=== MT6835 Tests ===\r\n");

    total++; if (test_mt6835_range())     passed++;
    total++; if (test_mt6835_stability()) passed++;
    test_mt6835_dump();   /* informational only, not counted */

    char summary[48];
    int len = snprintf(summary, sizeof(summary),
                       "\r\n=== Results: %d/%d PASSED ===\r\n\r\n",
                       passed, total);
    (void)len;
    test_print(summary);
}

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"

#define PIN_CS 5
#define PIN_SCK 6
#define PIN_MOSI 7
#define LDAC 8
#define LED 25
#define SPI_PORT spi0

int main()
{
    stdio_init_all();
    sleep_ms(3000);

    printf("\n=== MCP4822 Gain Test ===\n");

    spi_init(SPI_PORT, 1000000);
    spi_set_format(SPI_PORT, 16, 0, 0, 0);

    gpio_set_function(PIN_SCK, GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);

    gpio_init(PIN_CS);
    gpio_set_dir(PIN_CS, GPIO_OUT);
    gpio_put(PIN_CS, 1);

    gpio_init(LDAC);
    gpio_set_dir(LDAC, GPIO_OUT);
    gpio_put(LDAC, 0);

    gpio_init(LED);
    gpio_set_dir(LED, GPIO_OUT);

    while (1)
    {
        // Test 1: Gain = 1x, DAC = 2048 (should be ~1.024V)
        uint16_t cmd1 = 0b1011000000000000 | 2048;
        gpio_put(LED, 1);
        gpio_put(PIN_CS, 0);
        spi_write16_blocking(SPI_PORT, &cmd1, 1);
        gpio_put(PIN_CS, 1);

        printf("Test 1: Gain=1x, Value=2048, Expected=~1.024V\n");
        printf("Measure Pin 6 NOW!\n\n");
        sleep_ms(5000);

        // Test 2: Gain = 2x, DAC = 2048 (should be ~2.048V)
        uint16_t cmd2 = 0b1001000000000000 | 2048;
        gpio_put(LED, 0);
        gpio_put(PIN_CS, 0);
        spi_write16_blocking(SPI_PORT, &cmd2, 1);
        gpio_put(PIN_CS, 1);

        printf("Test 2: Gain=2x, Value=2048, Expected=~2.048V\n");
        printf("Measure Pin 6 NOW!\n\n");
        sleep_ms(5000);

        // Test 3: Gain = 2x, DAC = 4095 (should be ~4.096V but limited by VDD)
        uint16_t cmd3 = 0b1001000000000000 | 4095;
        gpio_put(LED, 1);
        gpio_put(PIN_CS, 0);
        spi_write16_blocking(SPI_PORT, &cmd3, 1);
        gpio_put(PIN_CS, 1);

        printf("Test 3: Gain=2x, Value=4095, Expected=~3.3V (limited by VDD)\n");
        printf("Measure Pin 6 NOW!\n\n");
        sleep_ms(5000);
    }
}
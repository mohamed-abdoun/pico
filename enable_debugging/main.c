#include <stdio.h>
#include "pico/stdlib.h"

int main()
{
    // Initialize USB serial
    stdio_init_all();

    // Wait for USB connection (optional but helpful)
    sleep_ms(2000); // Give time for USB to enumerate

    printf("Hello from Pico!\n");
    printf("Debug: Program started\n");

    while (true)
    {
        printf("Running... %d\n", time_us_32());
        sleep_ms(1000);
    }

    return 0;
}
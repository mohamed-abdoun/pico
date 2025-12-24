/**
 *  V. Hunter Adams (vha3@cornell.edu)

    A timer interrupt on core 0 generates a 400Hz beep
    thru an SPI DAC, once per second. A single protothread
    blinks the LED.

    === PICO TO MCP4822 DAC CONNECTIONS ===

    Pico Pin | Pico GPIO | MCP4822 Pin | MCP4822 Function
    ---------|-----------|-------------|------------------
    Pin 7    | GPIO 5    | Pin 2       | CS (Chip Select)
    Pin 9    | GPIO 6    | Pin 3       | SCK (SPI Clock)
    Pin 10   | GPIO 7    | Pin 4       | SDI (SPI Data In)
    Pin 11   | GPIO 8    | Pin 5       | LDAC (Latch DAC)
    Pin 36   | 3.3V      | Pin 1       | VDD (Power)
    Pin 3    | GND       | Pin 8       | VSS (Ground)

    Audio Output:
    MCP4822 Pin 6 (VOUTB) -> 3.5mm Audio Jack TIP & RING (direct connection)
    MCP4822 Pin 8 (VSS)   -> 3.5mm Audio Jack SLEEVE (ground)

    Debug/Timing:
    Pin 4    | GPIO 2    | (Optional)  | ISR timing output

    === DAC CONFIGURATION ===
    - MCP4822: 12-bit dual DAC with internal 2.048V reference
    - Gain: 2x (for 0-4.096V output range)
    - Channel B used for audio output
    - Sample rate: 50kHz (20µs period)
    - Frequency: 400Hz beep

 */

// Include necessary libraries
#include <stdio.h>
#include <math.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/sync.h"
// Include protothreads
#include "pt_cornell_rp2040_v1_4.h"

// Low-level alarm infrastructure we'll be using
#define ALARM_NUM 0
#define ALARM_IRQ TIMER_IRQ_0

// Macros for fixed-point arithmetic (faster than floating point)
typedef signed int fix15;
#define multfix15(a, b) ((fix15)((((signed long long)(a)) * ((signed long long)(b))) >> 15))
#define float2fix15(a) ((fix15)((a) * 32768.0))
#define fix2float15(a) ((float)(a) / 32768.0)
#define absfix15(a) abs(a)
#define int2fix15(a) ((fix15)(a << 15))
#define fix2int15(a) ((int)(a >> 15))
#define char2fix15(a) (fix15)(((fix15)(a)) << 15)
#define divfix(a, b) (fix15)((((signed long long)(a)) << 15) / (b))

// Direct Digital Synthesis (DDS) parameters
#define two32 4294967296.0 // 2^32 (a constant)
#define Fs 50000
#define DELAY 20 // 1/Fs (in microseconds)

// the DDS units - core 0
volatile float current_freq = 400.0; // frequency to be generated
volatile unsigned int phase_incr_main_0;
// Phase accumulator and phase increment. Increment sets output frequency.
volatile unsigned int phase_accum_main_0;
// volatile unsigned int phase_incr_main_0 = (int)((current_freq * two32) / Fs);

// DDS sine table (populated in main())
#define sine_table_size 256
fix15 sin_table[sine_table_size];

// Values output to DAC
int DAC_output_0;
int DAC_output_1;

// Amplitude modulation parameters and variables
fix15 max_amplitude = int2fix15(1); // maximum amplitude
fix15 attack_inc;                   // rate at which sound ramps up
fix15 decay_inc;                    // rate at which sound ramps down
fix15 current_amplitude_0 = 0;      // current amplitude (modified in ISR)
fix15 current_amplitude_1 = 0;      // current amplitude (modified in ISR)

// Timing parameters for beeps (units of interrupts)
#define ATTACK_TIME 250            // Extended attack for smoother start
#define DECAY_TIME 250             // Extended decay for smoother end
#define SUSTAIN_TIME 48000         // Very long sustain
#define BEEP_DURATION 6000         // Total: 1000ms beep (at 50kHz)
#define BEEP_REPEAT_INTERVAL 15000 // 2 seconds between beep starts

// State machine variables
volatile unsigned int STATE_0 = 0;
volatile unsigned int count_0 = 0;

// === DEBUGGING: Added for tracking ===
volatile unsigned int beep_count = 0; // Track number of completed beeps
volatile unsigned int last_state = 0; // Track state changes

// SPI data
uint16_t DAC_data_1; // output value
uint16_t DAC_data_0; // output value

// DAC parameters - *** FIXED FOR MCP4822 WITH INTERNAL 2.048V VREF ***
// *** CHANGED: Use 2x gain (bit 13 = 0) for full 0-4.096V range ***
#define DAC_config_chan_A 0b0001000000000000 // Channel A, 2x gain, active
#define DAC_config_chan_B 0b1001000000000000 // Channel B, 2x gain, active

// SPI configurations (note these represent GPIO number, NOT pin number)
#define PIN_MISO 4
#define PIN_CS 5
#define PIN_SCK 6
#define PIN_MOSI 7
#define LDAC 8
#define LED 25
#define SPI_PORT spi0

// GPIO for timing the ISR
#define ISR_GPIO 2

// This timer ISR is called on core 0
static void alarm_irq(void)
{
    // Debug timing pin HIGH
    gpio_put(ISR_GPIO, 1);

    // Clear interrupt
    hw_clear_bits(&timer_hw->intr, 1u << ALARM_NUM);

    // Re-arm alarm
    timer_hw->alarm[ALARM_NUM] = timer_hw->timerawl + DELAY;

    if (STATE_0 == 0)
    {
        // ======================================================
        // 1) UPDATE FREQUENCY → PHASE INCREMENT (DDS CORE CHANGE)
        // ======================================================
        phase_incr_main_0 = (uint32_t)((current_freq * two32) / Fs);

        // ======================================================
        // 2) DDS PHASE ACCUMULATION + SINE LOOKUP
        // ======================================================
        phase_accum_main_0 += phase_incr_main_0;

        DAC_output_0 =
            fix2int15(multfix15(current_amplitude_0,
                                sin_table[phase_accum_main_0 >> 24])) +
            2048;

        // ======================================================
        // 3) AMPLITUDE ENVELOPE (ATTACK / DECAY)
        // ======================================================
        if (count_0 < ATTACK_TIME)
        {
            current_amplitude_0 += attack_inc;
        }
        else if (count_0 > (BEEP_DURATION - DECAY_TIME))
        {
            current_amplitude_0 -= decay_inc;
        }

        // ======================================================
        // 4) SEND SAMPLE TO DAC
        // ======================================================
        DAC_data_0 = (DAC_config_chan_B | (DAC_output_0 & 0x0FFF));

        gpio_put(PIN_CS, 0);
        spi_write16_blocking(SPI_PORT, &DAC_data_0, 1);
        gpio_put(PIN_CS, 1);

        // ======================================================
        // 5) FREQUENCY SWEEP (BEEP → WHISTLE)
        // ======================================================
        current_freq += 0.83f; // 0.05f; // Hz per sample (smooth upward sweep)

        // ======================================================
        // 6) TIMEKEEPING
        // ======================================================
        count_0++;

        if (count_0 >= BEEP_DURATION)
        {
            STATE_0 = 1;
            count_0 = 0;
            beep_count++;
        }
    }
    else
    {
        // Silence between chirps
        count_0++;

        if (count_0 >= BEEP_REPEAT_INTERVAL)
        {
            STATE_0 = 0;
            count_0 = 0;

            // Reset for next chirp
            current_freq = 400.0f;
            current_amplitude_0 = 0;
        }
    }

    // Debug timing pin LOW
    gpio_put(ISR_GPIO, 0);
}

// This thread runs on core 0
static PT_THREAD(protothread_core_0(struct pt *pt))
{
    // Indicate thread beginning
    PT_BEGIN(pt);
    while (1)
    {
        // Toggle on LED
        gpio_put(LED, !gpio_get(LED));

        // Yield for 500 ms
        PT_YIELD_usec(500000);
    }
    // Indicate thread end
    PT_END(pt);
}

// ========================================================================
// === DEBUGGING: Debug thread - prints status information ===
// ========================================================================
static PT_THREAD(protothread_debug(struct pt *pt))
{
    PT_BEGIN(pt);
    printf("\n=== Debug Thread Started ===\n");
    while (1)
    {
        // Print status information
        printf("State: %d | Beeps: %d | Amplitude: %.3f | Count: %d | DAC: %d\n",
               STATE_0, beep_count, fix2float15(current_amplitude_0), count_0, DAC_output_0);

        // Check for state transitions
        if (STATE_0 != last_state)
        {
            if (STATE_0 == 0)
            {
                printf(">>> STATE CHANGE: Starting beep #%d\n", beep_count + 1);
            }
            else
            {
                printf(">>> STATE CHANGE: Beep complete, entering silence period\n");
            }
            last_state = STATE_0;
        }

        // Yield for 1 second
        PT_YIELD_usec(1000000);
    }
    PT_END(pt);
}
// ========================================================================

// Core 0 entry point
int main()
{
    // Initialize stdio/uart (printf won't work unless you do this!)
    stdio_init_all();

    // === DEBUGGING: Wait for USB serial connection ===
    sleep_ms(2000);

    // === DEBUGGING: Startup banner ===
    printf("\n");
    printf("================================================\n");
    printf("  Audio Beep Synthesis - CORRECTED VERSION\n");
    printf("================================================\n");
    printf("Hello, friends!\n");

    // Initialize SPI channel (channel, baud rate set to 20MHz)
    spi_init(SPI_PORT, 20000000);
    // Format (channel, data bits per transfer, polarity, phase, order)
    spi_set_format(SPI_PORT, 16, 0, 0, 0);

    // Map SPI signals to GPIO ports
    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
    gpio_set_function(PIN_SCK, GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);

    // *** CHANGED: Manual CS control instead of hardware SPI CS ***
    gpio_init(PIN_CS);
    gpio_set_dir(PIN_CS, GPIO_OUT);
    gpio_put(PIN_CS, 1); // CS starts HIGH (inactive)

    // Map LDAC pin to GPIO port, hold it low (could alternatively tie to GND)
    gpio_init(LDAC);
    gpio_set_dir(LDAC, GPIO_OUT);
    gpio_put(LDAC, 0);

    // Setup the ISR-timing GPIO
    gpio_init(ISR_GPIO);
    gpio_set_dir(ISR_GPIO, GPIO_OUT);
    gpio_put(ISR_GPIO, 0);

    // Map LED to GPIO port, make it low
    gpio_init(LED);
    gpio_set_dir(LED, GPIO_OUT);
    gpio_put(LED, 0);

    // set up increments for calculating bow envelope
    attack_inc = divfix(max_amplitude, int2fix15(ATTACK_TIME));
    decay_inc = divfix(max_amplitude, int2fix15(DECAY_TIME));

    // Build the sine lookup table
    // scaled to produce values between 0 and 4096 (for 12-bit DAC)
    int ii;
    for (ii = 0; ii < sine_table_size; ii++)
    {
        sin_table[ii] = float2fix15(2047 * sin((float)ii * 6.283 / (float)sine_table_size));
    }

    // === DEBUGGING: Print configuration parameters ===
    printf("\n=== Configuration Parameters ===\n");
    printf("Sample Rate (Fs): %d Hz\n", Fs);
    printf("Beep Frequency: 400 Hz\n");
    printf("ISR Period: %d us\n", DELAY);
    printf("Attack Time: %d samples (%.1f ms)\n", ATTACK_TIME, (float)ATTACK_TIME * 1000.0 / Fs);
    printf("Decay Time: %d samples (%.1f ms)\n", DECAY_TIME, (float)DECAY_TIME * 1000.0 / Fs);
    printf("Sustain Time: %d samples (%.1f ms)\n", SUSTAIN_TIME, (float)SUSTAIN_TIME * 1000.0 / Fs);
    printf("Beep Duration: %d samples (%.1f ms)\n", BEEP_DURATION, (float)BEEP_DURATION * 1000.0 / Fs);
    printf("Beep Interval: %d samples (%.2f sec)\n", BEEP_REPEAT_INTERVAL, (float)BEEP_REPEAT_INTERVAL / Fs);
    printf("Attack Increment: %.6f\n", fix2float15(attack_inc));
    printf("Decay Increment: %.6f\n", fix2float15(decay_inc));
    printf("Sine Table Size: %d entries\n", sine_table_size);

    printf("\n=== GPIO Pin Configuration ===\n");
    printf("SPI CS: GPIO %d\n", PIN_CS);
    printf("SPI SCK: GPIO %d\n", PIN_SCK);
    printf("SPI MOSI: GPIO %d\n", PIN_MOSI);
    printf("LDAC: GPIO %d\n", LDAC);
    printf("LED: GPIO %d\n", LED);
    printf("ISR Timing: GPIO %d\n", ISR_GPIO);

    printf("\n=== DAC Configuration ===\n");
    printf("*** CORRECTED: Using 2x gain for MCP4822 internal VREF ***\n");
    printf("Internal VREF: 2.048V\n");
    printf("Gain: 2x (bit 13 = 0)\n");
    printf("Output Range: 0V to 4.096V (or VDD-0.04V)\n");
    printf("DAC Command (Channel B): 0x%04X\n", DAC_config_chan_B);

    printf("\n=== Starting System ===\n");

    // Enable the interrupt for the alarm (we're using Alarm 0)
    hw_set_bits(&timer_hw->inte, 1u << ALARM_NUM);
    // Associate an interrupt handler with the ALARM_IRQ
    irq_set_exclusive_handler(ALARM_IRQ, alarm_irq);
    // Enable the alarm interrupt
    irq_set_enabled(ALARM_IRQ, true);
    // Write the lower 32 bits of the target time to the alarm register, arming it.
    timer_hw->alarm[ALARM_NUM] = timer_hw->timerawl + DELAY;

    // Add core 0 threads
    pt_add_thread(protothread_core_0);
    pt_add_thread(protothread_debug); // === DEBUGGING: Add debug thread ===

    // === DEBUGGING: Thread start message ===
    printf("Threads initialized. Starting scheduler...\n");
    printf("Monitor output below for real-time status:\n\n");

    // Start scheduling core 0 threads
    pt_schedule_start;
}
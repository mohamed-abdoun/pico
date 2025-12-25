/**
 * ==============================================================================
 *  beep_beep.c - Direct Digital Synthesis (DDS) Audio Chirp Generator
 * ==============================================================================
 *
 *  OVERVIEW:
 *  This program demonstrates DDS-based audio synthesis on the Raspberry Pi Pico.
 *  A timer interrupt running at 50 kHz generates audio samples using DDS, which
 *  are output through an SPI-connected 12-bit DAC (MCP4822). The result is a
 *  rising frequency "chirp" sound that sweeps from 400 Hz to ~5.4 kHz.
 *
 * ==============================================================================
 *  WHAT IS DIRECT DIGITAL SYNTHESIS (DDS)?
 * ==============================================================================
 *
 *  DDS is a method for generating analog waveforms digitally. Instead of storing
 *  entire waveforms in memory or using analog oscillators, DDS uses:
 *
 *  1. PHASE ACCUMULATOR (32-bit counter)
 *     - Increments by a calculated amount each sample
 *     - Wraps around at 2^32, creating continuous phase rotation
 *
 *  2. PHASE-TO-AMPLITUDE CONVERTER (lookup table)
 *     - Top 8 bits of accumulator index into 256-entry sine table
 *     - Returns instantaneous sine amplitude for current phase
 *
 *  3. FREQUENCY CONTROL (phase increment)
 *     - Equation: phase_increment = (desired_freq × 2^32) / sample_rate
 *     - Larger increment = faster phase rotation = higher frequency
 *
 *  DDS ADVANTAGES:
 *  ✓ Instant frequency changes (just update phase_increment)
 *  ✓ Extremely precise frequency control (32-bit resolution)
 *  ✓ No analog components needed (pure digital)
 *  ✓ Memory efficient (256 samples vs. full waveform storage)
 *  ✓ Mathematically perfect phase continuity
 *
 * ==============================================================================
 *  WHY THESE SPECIFIC VALUES?
 * ==============================================================================
 *
 *  SAMPLE RATE (Fs = 50 kHz):
 *  - Nyquist requirement: Must be > 2× highest frequency (50k > 2×5.4k ✓)
 *  - DAC settling time: MCP4822 needs ~4.5µs, we give it 20µs (safe margin)
 *  - CPU overhead: RP2040 @ 125 MHz has 2500 cycles per sample (plenty)
 *  - Audio quality: 50 kHz gives clean reproduction of audio frequencies
 *
 *  SINE TABLE SIZE (256 entries):
 *  - Power of 2: Enables fast indexing (phase >> 24 extracts top 8 bits)
 *  - Memory cost: 256 × 2 bytes = 512 bytes (0.2% of RP2040's 264KB RAM)
 *  - Resolution: 256 points around circle gives smooth audio waveform
 *  - Performance: Single array lookup per sample (very fast)
 *
 *  PHASE ACCUMULATOR (32 bits):
 *  - Frequency resolution: Fs/2^32 = 50000/4294967296 ≈ 0.000012 Hz
 *  - Dynamic range: Can represent 0 to Fs/2 (0 to 25 kHz)
 *  - Automatic wraparound: 32-bit overflow creates continuous phase rotation
 *
 *  DAC AMPLITUDE (±2047):
 *  - 12-bit DAC range: 0 to 4095
 *  - Bipolar signal: -2047 to +2047 (centered at 2048)
 *  - Output voltage: 0V to 4.096V (using 2x gain on 2.048V reference)
 *  - Full swing: Maximum volume without clipping distortion
 *
 * ==============================================================================
 *  HARDWARE CONNECTIONS
 * ==============================================================================
 *
 *    Pico Pin | Pico GPIO | MCP4822 Pin | MCP4822 Function
 *    ---------|-----------|-------------|------------------
 *    Pin 7    | GPIO 5    | Pin 2       | CS (Chip Select)
 *    Pin 9    | GPIO 6    | Pin 3       | SCK (SPI Clock)
 *    Pin 10   | GPIO 7    | Pin 4       | SDI (SPI Data In)
 *    Pin 11   | GPIO 8    | Pin 5       | LDAC (Latch DAC)
 *    Pin 36   | 3.3V      | Pin 1       | VDD (Power)
 *    Pin 3    | GND       | Pin 8       | VSS (Ground)
 *
 * ==============================================================================
 *  DAC CONFIGURATION (MCP4822)
 * ==============================================================================
 *
 *  - Model: MCP4822 (12-bit dual DAC)
 *  - Reference: Internal 2.048V (no external voltage needed)
 *  - Gain: 2x (bit 13 = 0) → Output range 0 to 4.096V
 *  - Channel: B (bit 15 = 1)
 *  - Buffered: Yes (bit 14 = 1) for stable output
 *  - Sample rate: 50 kHz (one sample every 20µs)
 *
 *  Output Voltage Equation:
 *    V_out = (DAC_value / 4096) × V_ref × Gain
 *    V_out = (DAC_value / 4096) × 2.048V × 2
 *    V_out ranges from 0V (DAC=0) to 4.096V (DAC=4095)
 *
 * ==============================================================================
 *  SIGNAL FLOW
 * ==============================================================================
 *
 *  Timer ISR (50 kHz) → Calculate Phase Increment → Advance Phase Accumulator
 *       ↓                                                       ↓
 *  Apply Envelope ← Multiply by Amplitude ← Lookup Sine Table
 *       ↓
 *  Add DC Offset (2048) → Send to DAC via SPI → Analog Audio Output
 *
 * ==============================================================================
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

// ==============================================================================
// TIMER ALARM CONFIGURATION
// ==============================================================================
// Low-level alarm infrastructure we'll be using
#define ALARM_NUM 0
#define ALARM_IRQ TIMER_IRQ_0

// ==============================================================================
// FIXED-POINT ARITHMETIC
// ==============================================================================
// Fixed-point arithmetic is used instead of floating-point for performance.
// The RP2040 doesn't have hardware FPU, so fixed-point is ~10x faster.
// Format: Q15.15 (1 sign bit, 15 integer bits, 15 fractional bits)
// Range: -32768.0 to +32767.99997 with precision of 1/32768 ≈ 0.00003

typedef signed int fix15;

// Multiply two Q15.15 numbers: (a * b) >> 15
// Equation: result = (a × b) / 2^15
#define multfix15(a, b) ((fix15)((((signed long long)(a)) * ((signed long long)(b))) >> 15))

// Convert float to Q15.15: multiply by 2^15
#define float2fix15(a) ((fix15)((a) * 32768.0))

// Convert Q15.15 to float: divide by 2^15
#define fix2float15(a) ((float)(a) / 32768.0)

// Absolute value
#define absfix15(a) abs(a)

// Convert integer to Q15.15: shift left by 15
#define int2fix15(a) ((fix15)(a << 15))

// Convert Q15.15 to integer: shift right by 15
#define fix2int15(a) ((int)(a >> 15))

// Convert char to Q15.15
#define char2fix15(a) (fix15)(((fix15)(a)) << 15)

// Divide two Q15.15 numbers: (a << 15) / b
// Equation: result = (a × 2^15) / b
#define divfix(a, b) (fix15)((((signed long long)(a)) << 15) / (b))

// ==============================================================================
// DIRECT DIGITAL SYNTHESIS (DDS) PARAMETERS
// ==============================================================================
// DDS generates sine waves by accumulating phase and looking up sine values.
//
// ═══════════════════════════════════════════════════════════════════════════
// ORIGINAL DDS ALGORITHM EQUATIONS (Standard Implementation):
// ═══════════════════════════════════════════════════════════════════════════
//
// 1. PHASE INCREMENT CALCULATION (done once at initialization):
//    phase_incr = (f_out × 2^N) / Fs
//    where: N = accumulator bit width (typically 32)
//           f_out = desired output frequency (Hz)
//           Fs = sample rate (Hz)
//
// 2. PHASE ACCUMULATION (done every sample):
//    phase[n] = phase[n-1] + phase_incr (mod 2^N)
//
// 3. PHASE-TO-AMPLITUDE CONVERSION (done every sample):
//    index = phase[n] >> (N - M)  [where M = log2(table_size)]
//    output = sin_table[index]
//
// 4. OUTPUT EQUATION:
//    y[n] = A × sin(2π × f_out × n / Fs)
//    where: A = amplitude, n = sample number
//
// ═══════════════════════════════════════════════════════════════════════════
// MODIFICATIONS IN THIS IMPLEMENTATION:
// ═══════════════════════════════════════════════════════════════════════════
//
// CHANGE 1: DYNAMIC FREQUENCY SWEEP (Major modification)
// ────────────────────────────────────────────────────────────
// ORIGINAL:  phase_incr calculated ONCE at initialization (fixed frequency)
//            phase_incr_main_0 = (400.0 × 2^32) / 50000  [constant]
//
// MODIFIED:  phase_incr recalculated EVERY SAMPLE (variable frequency)
//            current_freq += 0.83 Hz per sample
//            phase_incr_main_0 = (current_freq × 2^32) / Fs  [in ISR]
//
// IMPACT:    Creates rising "chirp" from 400 Hz → 5380 Hz over 120 ms
//            Adds ~1 division per sample (acceptable at 50 kHz)
//
// CHANGE 2: MANUAL CHIP SELECT CONTROL
// ────────────────────────────────────────────────────────────
// ORIGINAL:  Hardware SPI CS control
//            gpio_set_function(PIN_CS, GPIO_FUNC_SPI)  [automatic]
//
// MODIFIED:  Manual GPIO CS control
//            gpio_put(PIN_CS, 0); spi_write(); gpio_put(PIN_CS, 1);
//
// REASON:    Better control over SPI timing, explicit CS assertion
//
// CHANGE 3: DAC GAIN CONFIGURATION
// ────────────────────────────────────────────────────────────
// ORIGINAL:  1x gain (0-2.048V output range)
//            DAC_config = 0b1011000000000000  [bit 13 = 1]
//
// MODIFIED:  2x gain (0-4.096V output range)
//            DAC_config = 0b1001000000000000  [bit 13 = 0]
//
// REASON:    Full voltage swing for louder audio output
//
// CHANGE 4: TIMING PARAMETERS
// ────────────────────────────────────────────────────────────
// ORIGINAL:  BEEP_DURATION = 10500 (210 ms beep)
//            BEEP_REPEAT_INTERVAL = 50000 (1 second between beeps)
//
// MODIFIED:  BEEP_DURATION = 6000 (120 ms chirp)
//            BEEP_REPEAT_INTERVAL = 15000 (300 ms between chirps)
//
// REASON:    Faster, more frequent chirps for different audio character
//
// CHANGE 5: ADDED DEBUGGING
// ────────────────────────────────────────────────────────────
// ADDED:     beep_count, last_state tracking
//            protothread_debug() for status monitoring
//
// ═══════════════════════════════════════════════════════════════════════════
//
// WHY DDS?
// - Memory efficient: Only stores 256 sine samples (not continuous waveform)
// - Frequency agile: Change frequency instantly by changing phase increment
// - Precise: 32-bit phase accumulator gives frequency resolution of Fs/2^32
// - No drift: Digital phase accumulation eliminates analog oscillator drift
//
// HOW IT WORKS:
// 1. Phase accumulator (32-bit) increments by phase_incr each sample
// 2. Top 8 bits of accumulator index into 256-entry sine lookup table
// 3. Output sine value is scaled by amplitude envelope
//
// SAMPLE RATE SELECTION (Fs = 50 kHz):
// - Nyquist: Can generate clean signals up to 25 kHz (well above human hearing)
// - DAC settling: 20µs period gives MCP4822 time to settle (~4.5µs typical)
// - ISR overhead: 50 kHz manageable on RP2040 @ 125 MHz (~2500 cycles/sample)
// - Audio quality: More than adequate for 400 Hz beep (125× oversampling)

#define two32 4294967296.0  // 2^32: Full range of 32-bit phase accumulator
#define Fs 50000            // Sample rate: 50 kHz (chosen for reasons above)
#define DELAY 20            // Sample period: 1/Fs = 20 µs

// DDS state variables for core 0
volatile float current_freq = 400.0;          // Output frequency in Hz (sweeps upward)
volatile unsigned int phase_incr_main_0;      // Phase increment (calculated from frequency)
volatile unsigned int phase_accum_main_0;     // 32-bit phase accumulator (wraps at 2^32)

// Sine lookup table (256 entries covering 0 to 2π)
// WHY 256 ENTRIES?
// - Power of 2: Top 8 bits of 32-bit accumulator directly index the table
// - Memory: 256 × 2 bytes = 512 bytes (reasonable for RP2040's 264KB RAM)
// - Quality: Adequate for audio; linear interpolation could improve but adds cost
#define sine_table_size 256
fix15 sin_table[sine_table_size];

// ==============================================================================
// DAC OUTPUT VARIABLES
// ==============================================================================
int DAC_output_0;   // Final 12-bit value sent to DAC (0-4095)
int DAC_output_1;   // Reserved for potential second channel

// ==============================================================================
// AMPLITUDE ENVELOPE (ADSR-style)
// ==============================================================================
// Creates smooth attack/decay to eliminate clicks and pops.
// Without envelope, instant on/off creates harsh transients (broadband noise).
//
// ENVELOPE SHAPE:
//    Attack    Sustain      Decay
//    /|‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾\
//   / |                   \
//  /  |                    \
// 0   |                     0
//
// ATTACK/DECAY INCREMENT CALCULATION:
// attack_inc = max_amplitude / ATTACK_TIME
// Equation: Δamplitude = A_max / N_samples
// This creates linear ramp from 0 to max over ATTACK_TIME samples

fix15 max_amplitude = int2fix15(1);     // Maximum amplitude (1.0 in fixed-point)
fix15 attack_inc;                       // Amplitude increment during attack phase
fix15 decay_inc;                        // Amplitude decrement during decay phase
fix15 current_amplitude_0 = 0;          // Current envelope amplitude (modified in ISR)
fix15 current_amplitude_1 = 0;          // Reserved for second channel

// ==============================================================================
// TIMING PARAMETERS (all in units of sample interrupts @ 50 kHz)
// ==============================================================================
// TIME CALCULATION: time_seconds = SAMPLES / Fs
// Example: BEEP_DURATION = 6000 samples / 50000 Hz = 0.12 seconds

#define ATTACK_TIME 250             // Attack: 250/50000 = 5 ms (smooth turn-on)
#define DECAY_TIME 250              // Decay: 250/50000 = 5 ms (smooth turn-off)
#define SUSTAIN_TIME 48000          // Sustain: 48000/50000 = 0.96 s (constant amplitude)
#define BEEP_DURATION 6000          // Total beep: 6000/50000 = 0.12 s (120 ms chirp)
#define BEEP_REPEAT_INTERVAL 15000  // Silence: 15000/50000 = 0.3 s between beeps

// ==============================================================================
// STATE MACHINE VARIABLES
// ==============================================================================
// STATE_0 = 0: Active beep (generating sound)
// STATE_0 = 1: Silent interval between beeps
volatile unsigned int STATE_0 = 0;   // Current state (0=beep, 1=silence)
volatile unsigned int count_0 = 0;   // Sample counter within current state

// Debugging: Track beep count and state transitions
volatile unsigned int beep_count = 0;   // Number of completed beeps
volatile unsigned int last_state = 0;   // Previous state (for transition detection)

// ==============================================================================
// SPI/DAC COMMUNICATION
// ==============================================================================
uint16_t DAC_data_1;    // 16-bit SPI word for DAC channel A
uint16_t DAC_data_0;    // 16-bit SPI word for DAC channel B

// ==============================================================================
// MCP4822 DAC CONFIGURATION
// ==============================================================================
// The MCP4822 uses a 16-bit control word:
// Bit 15:    A/B̅ (0=Channel A, 1=Channel B)
// Bit 14:    BUF (1=Buffered, 0=Unbuffered) - we use buffered
// Bit 13:    G̅A̅ (0=2x gain, 1=1x gain)
// Bit 12:    S̅H̅D̅N̅ (0=Shutdown, 1=Active)
// Bits 11-0: Data (12-bit DAC value, 0-4095)
//
// WHY 2x GAIN?
// - MCP4822 has internal 2.048V reference
// - With 1x gain: Output range = 0 to 2.048V (limited)
// - With 2x gain: Output range = 0 to 4.096V (full swing for audio)
// - Equation: V_out = (DAC_value / 4096) × V_ref × Gain
// - Example: DAC=2048, V_ref=2.048V, Gain=2 → V_out = 2.048V (midpoint)
//
// CONFIGURATION BREAKDOWN:
// Channel A: 0b0001000000000000
//            ││││└──────────── Data bits (0x000)
//            │││└───────────── Active (SHDN=1)
//            ││└────────────── 2x gain (GA=0)
//            │└─────────────── Buffered (BUF=1)
//            └──────────────── Channel A (A/B=0)
//
// Channel B: 0b1001000000000000 (same as A, but bit 15 = 1 for Channel B)

#define DAC_config_chan_A 0b0001000000000000 // Channel A, buffered, 2x gain, active
#define DAC_config_chan_B 0b1001000000000000 // Channel B, buffered, 2x gain, active

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

// ==============================================================================
// TIMER INTERRUPT SERVICE ROUTINE (ISR)
// ==============================================================================
// This ISR fires every 20µs (50 kHz sample rate) to generate audio samples.
// Total execution time must be < 20µs to avoid missing samples.
//
// ISR PIPELINE (executed 50,000 times per second):
// 1. Calculate phase increment from current frequency
// 2. Advance phase accumulator and lookup sine value
// 3. Apply amplitude envelope (attack/sustain/decay)
// 4. Send 12-bit sample to DAC via SPI
// 5. Update frequency (for chirp sweep effect)
// 6. Manage timing and state transitions

static void alarm_irq(void)
{
    // Debug: Set timing pin HIGH (for oscilloscope measurement of ISR duration)
    gpio_put(ISR_GPIO, 1);

    // Clear the interrupt flag for this alarm
    hw_clear_bits(&timer_hw->intr, 1u << ALARM_NUM);

    // Re-arm the alarm for next interrupt (+20µs from now)
    timer_hw->alarm[ALARM_NUM] = timer_hw->timerawl + DELAY;

    // ===========================================================================
    // STATE 0: ACTIVE BEEP (Generate and output audio samples)
    // ===========================================================================
    if (STATE_0 == 0)
    {
        // -----------------------------------------------------------------------
        // STEP 1: CALCULATE PHASE INCREMENT FROM FREQUENCY
        // -----------------------------------------------------------------------
        // DDS Core Equation: phase_incr = (f_out × 2^32) / Fs
        //
        // *** MODIFIED FROM ORIGINAL DDS: ***
        // Standard DDS: Calculate phase_incr ONCE at init (constant frequency)
        // This implementation: Recalculate EVERY SAMPLE (allows frequency sweep)
        //
        // Example: f_out = 400 Hz, Fs = 50000 Hz
        //   phase_incr = (400 × 4294967296) / 50000 = 34,359,738
        //
        // Physical meaning: This is how much the 32-bit phase accumulator
        // advances each sample to generate the desired output frequency.
        phase_incr_main_0 = (uint32_t)((current_freq * two32) / Fs);

        // -----------------------------------------------------------------------
        // STEP 2: DDS PHASE ACCUMULATION + SINE TABLE LOOKUP
        // -----------------------------------------------------------------------
        // Advance the 32-bit phase accumulator (wraps automatically at 2^32)
        phase_accum_main_0 += phase_incr_main_0;

        // Extract top 8 bits as index into 256-entry sine table
        // Equation: index = phase_accum >> 24 (right-shift by 24 bits)
        // The bottom 24 bits provide sub-sample phase resolution
        //
        // Lookup sine value, scale by current amplitude envelope, convert to int
        // Then add 2048 to center the bipolar signal around the DAC midpoint
        // Equation: DAC_out = (amplitude × sin[phase]) + 2048
        // Range: sin_table values are ±2047 (in fixed-point), so output is 0-4095
        DAC_output_0 =
            fix2int15(multfix15(current_amplitude_0,
                                sin_table[phase_accum_main_0 >> 24])) +
            2048;

        // -----------------------------------------------------------------------
        // STEP 3: AMPLITUDE ENVELOPE (ATTACK / SUSTAIN / DECAY)
        // -----------------------------------------------------------------------
        // Attack phase: Ramp amplitude from 0 to max over ATTACK_TIME samples
        if (count_0 < ATTACK_TIME)
        {
            current_amplitude_0 += attack_inc;
        }
        // Decay phase: Ramp amplitude from max to 0 over DECAY_TIME samples
        else if (count_0 > (BEEP_DURATION - DECAY_TIME))
        {
            current_amplitude_0 -= decay_inc;
        }
        // Sustain phase: Hold amplitude at max (no change needed)

        // -----------------------------------------------------------------------
        // STEP 4: TRANSMIT SAMPLE TO DAC VIA SPI
        // -----------------------------------------------------------------------
        // Combine DAC configuration bits with 12-bit sample value
        // Mask with 0x0FFF to ensure only bottom 12 bits are used
        //
        // NOTE: Original code used 0xFFFF mask (incorrect - would pass all bits)
        // This version correctly uses 0x0FFF to mask to 12 bits (0-4095)
        DAC_data_0 = (DAC_config_chan_B | (DAC_output_0 & 0x0FFF));

        // SPI transaction: Assert CS low, send 16-bit word, deassert CS high
        gpio_put(PIN_CS, 0);
        spi_write16_blocking(SPI_PORT, &DAC_data_0, 1);
        gpio_put(PIN_CS, 1);

        // -----------------------------------------------------------------------
        // STEP 5: FREQUENCY SWEEP (Creates rising "chirp" effect)
        // -----------------------------------------------------------------------
        // Increment frequency each sample for upward pitch sweep
        //
        // WHY 0.83 Hz PER SAMPLE?
        // This value was chosen to create an audible, pleasing chirp effect.
        //
        // CALCULATION:
        //   Sweep rate = Δf / N_samples
        //   where: Δf = desired frequency change
        //          N_samples = BEEP_DURATION
        //
        //   If we want to sweep from 400 Hz to ~5400 Hz over 6000 samples:
        //   Sweep rate = (5400 - 400) / 6000 = 5000 / 6000 ≈ 0.83 Hz/sample
        //
        // RESULT:
        //   Start frequency: 400 Hz
        //   End frequency: 400 + (0.83 × 6000) = 400 + 4980 = 5380 Hz
        //   Duration: 6000 samples / 50000 Hz = 120 ms
        //   Sweep range: ~13.5× frequency multiplication (over 3 octaves)
        //
        // WHY THIS RANGE?
        // - 400 Hz: Low enough to be clearly audible, warm tone
        // - 5380 Hz: High enough for bright, attention-getting sound
        // - 120 ms: Fast enough to sound like a "chirp", not a slow sweep
        // - Creates distinctive "rising whistle" character
        //
        // ALTERNATIVE VALUES:
        //   0.05 Hz/sample → 400-700 Hz (subtle, mellow)
        //   0.42 Hz/sample → 400-2920 Hz (moderate chirp)
        //   0.83 Hz/sample → 400-5380 Hz (dramatic chirp) ← CURRENT
        //   1.67 Hz/sample → 400-10420 Hz (extreme, may alias above Nyquist!)
        current_freq += 0.83f;

        // -----------------------------------------------------------------------
        // STEP 6: TIMING AND STATE MANAGEMENT
        // -----------------------------------------------------------------------
        count_0++;  // Increment sample counter

        // Check if beep duration completed
        if (count_0 >= BEEP_DURATION)
        {
            STATE_0 = 1;            // Transition to silence state
            count_0 = 0;            // Reset counter
            beep_count++;           // Increment completed beep counter
        }
    }
    // ===========================================================================
    // STATE 1: SILENT INTERVAL (Wait between beeps)
    // ===========================================================================
    else
    {
        count_0++;  // Increment silence counter

        // Check if silence interval completed
        if (count_0 >= BEEP_REPEAT_INTERVAL)
        {
            STATE_0 = 0;                // Transition back to beep state
            count_0 = 0;                // Reset counter

            // Reset synthesis parameters for next beep
            current_freq = 400.0f;      // Start frequency back at 400 Hz
            current_amplitude_0 = 0;    // Start envelope from zero
        }
    }

    // Debug: Set timing pin LOW (ISR complete)
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
// === DEBUGGING: Debug thread - prints DDS parameters and status ===
// ========================================================================
// This thread displays real-time DDS equation values to help understand
// how the algorithm works:
//
// DDS EQUATION VALUES DISPLAYED:
// - f_out:         Current output frequency (changes with chirp sweep)
// - phase_incr:    Calculated from: phase_incr = (f_out × 2^32) / Fs
// - phase_accum:   32-bit accumulator that wraps at 2^32
// - Table Index:   Top 8 bits of phase_accum (phase_accum >> 24)
// - Phase (°):     Current position in sine wave (0-360°)
//
// SIGNAL VALUES DISPLAYED:
// - Amplitude:     Current envelope value (0.0 to 1.0)
// - DAC Output:    Final 12-bit value sent to DAC (0-4095)
// - DAC Voltage:   Calculated output voltage using V_out equation
// - Sine Table:    Raw fixed-point value from lookup table
//
// This allows verification that the DDS equations are working correctly!
// ========================================================================
static PT_THREAD(protothread_debug(struct pt *pt))
{
    PT_BEGIN(pt);
    printf("\n=== Debug Thread Started ===\n");
    printf("\n=== DDS CONFIGURATION ===\n");
    printf("Sample Rate (Fs):           %d Hz\n", Fs);
    printf("Sample Period:              %d µs\n", DELAY);
    printf("Sine Table Size:            %d entries\n", sine_table_size);
    printf("Phase Accumulator Width:    32 bits\n");
    printf("Frequency Resolution:       %.6f Hz\n", (float)Fs / two32);
    printf("Beep Duration:              %d samples (%.1f ms)\n",
           BEEP_DURATION, (float)BEEP_DURATION * 1000.0f / Fs);
    printf("Frequency Sweep Rate:       0.83 Hz/sample\n");
    printf("Starting Frequency:         400 Hz\n");
    printf("Ending Frequency:           ~5380 Hz\n");
    printf("Attack Time:                %d samples (%.1f ms)\n",
           ATTACK_TIME, (float)ATTACK_TIME * 1000.0f / Fs);
    printf("Decay Time:                 %d samples (%.1f ms)\n",
           DECAY_TIME, (float)DECAY_TIME * 1000.0f / Fs);
    printf("Beep Interval:              %d samples (%.1f ms)\n\n",
           BEEP_REPEAT_INTERVAL, (float)BEEP_REPEAT_INTERVAL * 1000.0f / Fs);

    while (1)
    {
        // Calculate current DDS parameters
        uint32_t current_phase_incr = (uint32_t)((current_freq * two32) / Fs);
        uint8_t table_index = phase_accum_main_0 >> 24;
        float instantaneous_phase_deg = (phase_accum_main_0 / two32) * 360.0f;

        // Print comprehensive status information
        printf("\n─────────────────────────────────────────────────────────────\n");
        printf("State: %d | Beeps Completed: %d | Sample Count: %d\n",
               STATE_0, beep_count, count_0);

        // DDS Core Parameters (Equation values)
        printf("\n[DDS EQUATION VALUES]\n");
        printf("  f_out (current_freq):     %.2f Hz\n", current_freq);
        printf("  phase_incr:               %u (0x%08X)\n",
               current_phase_incr, current_phase_incr);
        printf("  phase_accum:              %u (0x%08X)\n",
               phase_accum_main_0, phase_accum_main_0);
        printf("  Table Index:              %u / 256\n", table_index);
        printf("  Instantaneous Phase:      %.2f°\n", instantaneous_phase_deg);

        // Signal Parameters
        printf("\n[SIGNAL VALUES]\n");
        printf("  Amplitude (envelope):     %.4f\n", fix2float15(current_amplitude_0));
        printf("  DAC Output Value:         %d (0x%03X)\n", DAC_output_0, DAC_output_0);
        printf("  DAC Voltage:              %.3f V\n",
               (DAC_output_0 / 4096.0f) * 2.048f * 2.0f);
        printf("  Sine Table Value:         %d (fixed-point)\n",
               sin_table[table_index]);

        // Performance Metrics
        if (STATE_0 == 0 && count_0 > 0)
        {
            float elapsed_ms = (count_0 * 1000.0f) / Fs;
            float progress_pct = (count_0 * 100.0f) / BEEP_DURATION;
            printf("\n[BEEP PROGRESS]\n");
            printf("  Elapsed Time:             %.1f ms\n", elapsed_ms);
            printf("  Progress:                 %.1f%%\n", progress_pct);

            // Determine envelope phase
            if (count_0 < ATTACK_TIME)
                printf("  Envelope Phase:           ATTACK\n");
            else if (count_0 > (BEEP_DURATION - DECAY_TIME))
                printf("  Envelope Phase:           DECAY\n");
            else
                printf("  Envelope Phase:           SUSTAIN\n");
        }

        // Check for state transitions
        if (STATE_0 != last_state)
        {
            printf("\n>>> STATE CHANGE: ");
            if (STATE_0 == 0)
            {
                printf("Starting beep #%d <<<\n", beep_count + 1);
            }
            else
            {
                printf("Beep complete, entering silence period <<<\n");
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
    printf("  Audio Beep Synthesis n");
    printf("================================================\n");


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

    // ===========================================================================
    // CALCULATE ATTACK/DECAY ENVELOPE INCREMENTS
    // ===========================================================================
    // These determine how fast the amplitude ramps up and down.
    // Equation: increment = max_amplitude / time_samples
    //
    // Example: max_amplitude = 1.0, ATTACK_TIME = 250 samples
    //   attack_inc = 1.0 / 250 = 0.004 per sample
    //   Result: Amplitude increases from 0 to 1.0 over 250 samples (5 ms)
    attack_inc = divfix(max_amplitude, int2fix15(ATTACK_TIME));
    decay_inc = divfix(max_amplitude, int2fix15(DECAY_TIME));

    // ===========================================================================
    // BUILD SINE LOOKUP TABLE
    // ===========================================================================
    // Precompute 256 samples of one complete sine wave cycle (0 to 2π).
    // This table is used by the DDS algorithm for fast sine generation.
    //
    // Equation: sin_table[i] = 2047 × sin(2π × i / 256)
    //
    // WHY 2047?
    // - DAC is 12-bit (0-4095 range)
    // - We want bipolar signal: -2047 to +2047
    // - Adding 2048 offset centers it: 1 to 4095
    // - Full swing gives maximum volume without clipping
    //
    // INDEXING: phase_accumulator >> 24 converts 32-bit phase to 8-bit index
    int ii;
    for (ii = 0; ii < sine_table_size; ii++)
    {
        // Compute sine value: 2047 × sin(2πi/256)
        // 6.283 ≈ 2π radians (full circle)
        sin_table[ii] = float2fix15(2047 * sin((float)ii * 6.283 / (float)sine_table_size));
    }



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


    // Start scheduling core 0 threads
    pt_schedule_start;
}
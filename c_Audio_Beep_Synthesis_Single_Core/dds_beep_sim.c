// dds_beep_sim.c
// Simulates your RP2040 DDS + ADS envelope + state machine on a PC
// and writes the output to a WAV file (mono, 16-bit PCM).
//
// "Northern Cardinal"-like version:
// - Higher frequency (1.8 kHz -> 2.6 kHz upward sweep)
// - Short chirp (~60 ms) repeated ~6 times/sec with small jitter
// - Sharper envelope
//
// Build on Ubuntu:
//   gcc dds_beep_sim.c -o dds_beep_sim -lm
// Run:
//   ./dds_beep_sim
// Output:
//   cardinal_chirp.wav

#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include <stdlib.h>

// ================= CONFIG =================
#define FS 50000                 // sample rate
#define DURATION_SEC 10          // output duration
#define TOTAL_SAMPLES (FS * DURATION_SEC)

#define SINE_TABLE_SIZE 256
#define TWO32 4294967296.0

// Cardinal-ish whistle sweep (Hz)
#define FREQ_START 1800.0
#define FREQ_END   2600.0

// Envelope + timing (in samples @ 50kHz)
#define ATTACK_TIME 80           // ~1.6 ms
#define DECAY_TIME  120          // ~2.4 ms

#define BEEP_DURATION 3000       // ~60 ms chirp
#define BASE_REPEAT_INTERVAL 8000 // ~160 ms between chirp starts (~6.25/sec)

// Random jitter (in samples) applied to repeat interval
#define JITTER_RANGE_SAMPLES 200 // 0..199 samples (~0..4 ms)

// Output gain (0.0 to 1.0). Keep < 1.0 to avoid clipping
#define OUTPUT_GAIN 0.9f
// ==========================================


// -------- fixed-point helpers (same style as your Pico)
typedef int32_t fix15;
#define float2fix15(a) ((fix15)((a) * 32768.0f))
#define fix2float15(a) ((float)(a) / 32768.0f)
#define multfix15(a,b) ((fix15)((((int64_t)(a)) * ((int64_t)(b))) >> 15))
#define int2fix15(a) ((fix15)((a) << 15))
#define divfix(a,b) ( (fix15)((((int64_t)(a)) << 15) / (b)) )

// ================= GLOBALS =================
static fix15 sin_table[SINE_TABLE_SIZE];

static uint32_t phase_accum = 0;

// Instead of a single fixed phase increment, we sweep from start to end
static uint32_t phase_incr_start = 0;
static uint32_t phase_incr_end   = 0;
static uint32_t phase_incr       = 0;

static fix15 max_amplitude;
static fix15 attack_inc;
static fix15 decay_inc;
static fix15 current_amplitude = 0;

// State machine like your firmware
// STATE = 0 => chirp active, STATE = 1 => silence
static uint32_t STATE = 0;
static uint32_t count = 0;
static uint32_t repeat_interval = BASE_REPEAT_INTERVAL;

// Simple deterministic RNG for jitter (no libc rand dependency)
static uint32_t rng_state = 1u;
static uint32_t lcg_rand_u32(void) {
    // LCG constants (Numerical Recipes)
    rng_state = (1664525u * rng_state) + 1013904223u;
    return rng_state;
}
static uint32_t jitter_samples(void) {
    // 0..JITTER_RANGE_SAMPLES-1
    return (JITTER_RANGE_SAMPLES == 0) ? 0u : (lcg_rand_u32() % JITTER_RANGE_SAMPLES);
}

// ============== WAV HEADER =================
static void write_wav_header(FILE *f, int sample_rate, int total_samples) {
    // Mono, 16-bit PCM
    uint32_t data_size = (uint32_t)total_samples * 2u; // 2 bytes per sample
    uint32_t riff_size = 36u + data_size;

    // RIFF
    fwrite("RIFF", 1, 4, f);
    fwrite(&riff_size, 4, 1, f);
    fwrite("WAVE", 1, 4, f);

    // fmt chunk
    fwrite("fmt ", 1, 4, f);
    uint32_t subchunk1_size = 16u;
    uint16_t audio_format = 1u;     // PCM
    uint16_t num_channels = 1u;     // mono
    uint16_t bits_per_sample = 16u; // 16-bit
    uint32_t byte_rate = (uint32_t)sample_rate * (uint32_t)num_channels * (bits_per_sample / 8u);
    uint16_t block_align = (uint16_t)(num_channels * (bits_per_sample / 8u));

    fwrite(&subchunk1_size, 4, 1, f);
    fwrite(&audio_format, 2, 1, f);
    fwrite(&num_channels, 2, 1, f);
    fwrite(&sample_rate, 4, 1, f);
    fwrite(&byte_rate, 4, 1, f);
    fwrite(&block_align, 2, 1, f);
    fwrite(&bits_per_sample, 2, 1, f);

    // data chunk
    fwrite("data", 1, 4, f);
    fwrite(&data_size, 4, 1, f);
}

// Clamp float to [-1, 1]
static float clamp1(float x) {
    if (x > 1.0f) return 1.0f;
    if (x < -1.0f) return -1.0f;
    return x;
}

// ================= MAIN ====================
int main(void) {
    const char *out_name = "cardinal_chirp.wav";
    FILE *f = fopen(out_name, "wb");
    if (!f) {
        perror("Failed to open output WAV");
        return 1;
    }

    write_wav_header(f, FS, TOTAL_SAMPLES);

    // Build sine lookup table in fix15 [-1, 1)
    for (int i = 0; i < SINE_TABLE_SIZE; i++) {
        float s = sinf(2.0f * (float)M_PI * (float)i / (float)SINE_TABLE_SIZE);
        sin_table[i] = float2fix15(s);
    }

    // Phase increment endpoints for sweep
    phase_incr_start = (uint32_t)((FREQ_START * TWO32) / (double)FS);
    phase_incr_end   = (uint32_t)((FREQ_END   * TWO32) / (double)FS);

    // Envelope increments
    max_amplitude = int2fix15(1);
    attack_inc = divfix(max_amplitude, int2fix15(ATTACK_TIME));
    decay_inc  = divfix(max_amplitude, int2fix15(DECAY_TIME));

    // Start with chirp active
    STATE = 0;
    count = 0;
    current_amplitude = 0;

    // Seed RNG so output is repeatable; change for different "bird personality"
    rng_state = 1234567u;

    // Main sample loop (this replaces your ISR timing)
    for (int n = 0; n < TOTAL_SAMPLES; n++) {
        int16_t pcm = 0;

        if (STATE == 0) {
            // ===== Chirp active =====
            // Linear frequency sweep across the chirp duration
            // phase_incr = start + (end-start) * count / BEEP_DURATION
            uint32_t delta = (phase_incr_end >= phase_incr_start)
                               ? (phase_incr_end - phase_incr_start)
                               : (phase_incr_start - phase_incr_end);

            if (count < (uint32_t)BEEP_DURATION) {
                uint32_t step = (uint32_t)(((uint64_t)delta * (uint64_t)count) / (uint64_t)BEEP_DURATION);
                phase_incr = (phase_incr_end >= phase_incr_start)
                               ? (phase_incr_start + step)
                               : (phase_incr_start - step);
            } else {
                phase_incr = phase_incr_end;
            }

            // DDS oscillator
            phase_accum += phase_incr;
            fix15 s = sin_table[phase_accum >> 24];

            // Envelope: attack then decay near the end
            if (count < (uint32_t)ATTACK_TIME) {
                current_amplitude += attack_inc;
                if (current_amplitude > max_amplitude) current_amplitude = max_amplitude;
            } else if (count > (uint32_t)(BEEP_DURATION - DECAY_TIME)) {
                // avoid underflow if count goes beyond
                if (current_amplitude > decay_inc) current_amplitude -= decay_inc;
                else current_amplitude = 0;
            }

            // Apply amplitude and output gain
            fix15 val_fix = multfix15(current_amplitude, s);
            float val = fix2float15(val_fix) * OUTPUT_GAIN;

            val = clamp1(val);
            pcm = (int16_t)lrintf(val * 32767.0f);

            count++;

            // End chirp?
            if (count >= (uint32_t)BEEP_DURATION) {
                STATE = 1;
                count = 0;

                // Choose next repeat interval with slight jitter (more natural)
                repeat_interval = BASE_REPEAT_INTERVAL + jitter_samples();
            }
        } else {
            // ===== Silence =====
            pcm = 0;
            count++;

            // End silence -> start next chirp
            if (count >= repeat_interval) {
                STATE = 0;
                count = 0;
                current_amplitude = 0;
                // keep phase_accum continuous; that’s fine (sounds natural)
            }
        }

        fwrite(&pcm, sizeof(int16_t), 1, f);
    }

    fclose(f);
    printf("Generated %s (%d seconds, %d Hz)\n", out_name, DURATION_SEC, FS);
    printf("Chirp: %.0f->%.0f Hz, %d samples (~%.1f ms)\n",
           FREQ_START, FREQ_END, BEEP_DURATION, (1000.0f * (float)BEEP_DURATION) / (float)FS);
    printf("Repeat interval: ~%d samples (~%.1f ms) + jitter up to %d samples (~%.1f ms)\n",
           BASE_REPEAT_INTERVAL,
           (1000.0f * (float)BASE_REPEAT_INTERVAL) / (float)FS,
           JITTER_RANGE_SAMPLES,
           (1000.0f * (float)JITTER_RANGE_SAMPLES) / (float)FS);
    return 0;
}

